 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.2
 *
 * @version Package Version: 3.1.2
*/

/*
© [2026] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/
#include <avr/io.h>
#include "mcc_generated_files/system/system.h"
#include <usb_core.h>
#include <util/delay.h>
#include <string.h>
// Number of consecutive equal AC measurements before power is seen as stable
#define AC_MEASUREMENT_STABLE_COUNT 5U


/* ------------------------------------------------------------------ */
/*  Controller state                                                   */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t  id;                /* 0x0F == disconnected */
    uint8_t  index;
    uint16_t buttons;           /* 1 = released, 0 = pressed (PSX convention) */
    uint8_t  axes[6];           /* RX, RY, LX, LY (+ 2 spare for pressure modes) */
    uint8_t  psx_buffer[21];
    uint8_t  rumble_active;
    volatile uint8_t report_dirty;
} controller_t;

#define NUM_CONTROLLERS 1

static controller_t controllers[NUM_CONTROLLERS];

static const uint8_t readMode_template[]     = { 0x01, 0x42, 0x00, 0x00, 0x00 };
static const uint8_t setAnalogMode[]         = { 0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t enterConfigMode[]       = { 0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t enableRumbleMode[]      = { 0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF };
static const uint8_t exitConfigMode[]        = { 0x01, 0x43, 0x00, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A };

/* Mutable copy of read command so the rumble bytes can be patched per-frame. */
static uint8_t readMode[sizeof(readMode_template)];

/* Idle inter-byte gap (microseconds), matching the original sketch. */
#define INTER_CMD_BYTE_DELAY_US     15

// Interrupt variables
volatile bool vbusFlag = true;
bool sendMouseMovement = false;
volatile bool controllerAcked = false;

// USB status variable
RETURN_CODE_t status;

/* ------------------------------------------------------------------ */
/*  Forward declarations                                               */
/* ------------------------------------------------------------------ */

static bool   wait_ack(void);
static bool   send_string(controller_t *port, const uint8_t *str, uint8_t len);
static bool   send_string_retry(controller_t *port, const uint8_t *str, uint8_t len);
static void   setup_controller(controller_t *port);
static void   poll_controllers(void);

void USB_ConnectionHandler();
void ACKPin_OnRisingEdge();

/* ------------------------------------------------------------------ */
/*  Pin helpers                                                        */
/* ------------------------------------------------------------------ */

static inline void attn_low(controller_t *port)
{
    ATT_SetLow();
    SPI0_Open(0);
    //if (port->index == 0) SPI_PORT.OUTCLR     = SPI_SS_bm;
    //else                  P2_ATTN_PORT.OUTCLR = P2_ATTN_bm;
}

static inline void attn_high(controller_t *port)
{
    SPI0_Close();
    ATT_SetHigh();
    //if (port->index == 0) SPI_PORT.OUTSET     = SPI_SS_bm;
    //else                  P2_ATTN_PORT.OUTSET = P2_ATTN_bm;
}

/*
    Main application
*/

int main(void)
{
    SYSTEM_Initialize();
    ACK_SetInterruptHandler();
    
    for (uint8_t i = 0; i < NUM_CONTROLLERS; i++) {
        controllers[i].id            = 0x0F;
        controllers[i].index         = i;
        controllers[i].buttons       = 0xFFFF;
        controllers[i].rumble_active = 0;
        memset(controllers[i].axes, 0x80, sizeof(controllers[i].axes));
    }
    
    while(1)
    {
        poll_controllers();
        
        USB_ConnectionHandler();
    }    
}


/**
 * Routine that checks the connectivity of the USB peripheral and start/stops the USB driver when connected/disconnected
 */
void USB_ConnectionHandler()
{
    static volatile bool prevVbusState = false;
    // Check if VBUS was true last check, indicating that USB was connected
    if (prevVbusState == true)
    {
        // Handle USB Transfers
        status = USBDevice_StatusGet();
    }
    // Get current status of VBUS
    bool currentVbusState = vbusFlag;
    // If changes to USB VBUS state
    if (currentVbusState != prevVbusState)
    {
        // If USB has been connected
        if (currentVbusState == true)
        {
            // Start USB operations
            status = USB_Start();
            //LED_SetHigh();
        }
        else
        {
            // Stop USB operations
            status = USB_Stop();
            //LED_SetLow();
        }
        // Save state
        prevVbusState = currentVbusState;
    }
}

/* ------------------------------------------------------------------ */
/*  PSX bus protocol                                                   */
/* ------------------------------------------------------------------ */

static bool wait_ack(void)
{
    /*
     * Wait for INTER_CMD_BYTE_DELAY_US microseconds and see if the ACK pin interrupt fires in that time. 
     *  if not, no controller.
     */
    controllerAcked = false;
    _delay_us(INTER_CMD_BYTE_DELAY_US);
    return controllerAcked;
}

static bool send_string(controller_t *port, const uint8_t *str, uint8_t len)
{
    uint8_t new_len = len;
    uint8_t mode    = str[1];
    bool    found   = false;

    attn_low(port);
    _delay_us(12);  /* CLOCK_SPEED * 3 from the original Arduino code */

    for (uint8_t i = 0; i < new_len; i++) {
        uint8_t cmd = (i < len) ? str[i] : 0x5A;
        uint8_t rx  = SPI0_ByteExchange(cmd);
        if (i < sizeof(port->psx_buffer)) port->psx_buffer[i] = rx;

        if (i == 1) {
            /* second byte tells us how many words of payload follow */
            new_len = ((rx & 0x0F) << 1) + 3;
            if (new_len > sizeof(port->psx_buffer)) new_len = sizeof(port->psx_buffer);
        }

        if (i != (new_len - 1) && !wait_ack()) {
            attn_high(port);
            _delay_us(100);
            return false;
        }
    }

    if ((mode == 0x42 || mode == 0x43) && port->psx_buffer[1] != 0xF3) {
        uint8_t new_id = port->psx_buffer[1] >> 4;
        found = (new_id != port->id && port->id == 0x0F);
        port->id = new_id;

        port->buttons = ((uint16_t)port->psx_buffer[3] << 8) | port->psx_buffer[4];

        for (uint8_t axis = 5; axis < new_len && (axis - 5) < sizeof(port->axes); axis++) {
            port->axes[axis - 5] = port->psx_buffer[axis];
        }
        port->report_dirty = 1;
    }

    attn_high(port);

    if (found) {
        setup_controller(port);
    }
    return true;
}

static bool send_string_retry(controller_t *port, const uint8_t *str, uint8_t len)
{
    for (uint8_t i = 0; i < 6; i++) {
        if (send_string(port, str, len)) return true;
        _delay_us(25);
    }
    return false;
}

static void setup_controller(controller_t *port)
{
    _delay_us(50);
    send_string_retry(port, enterConfigMode,  sizeof(enterConfigMode));
    _delay_us(50);
    send_string_retry(port, setAnalogMode,    sizeof(setAnalogMode));
    _delay_us(50);
    send_string_retry(port, enableRumbleMode, sizeof(enableRumbleMode));
    _delay_us(50);
    send_string_retry(port, exitConfigMode,   sizeof(exitConfigMode));
}

static void poll_controllers(void)
{
    for (uint8_t i = 0; i < NUM_CONTROLLERS; i++) {
        controller_t *port = &controllers[i];

        memcpy(readMode, readMode_template, sizeof(readMode));
        if (port->rumble_active) {
            readMode[3] = 0xFF;
            readMode[4] = 0xFF;
        }

        bool ok = send_string_retry(port, readMode, sizeof(readMode));
        if (!ok && port->id != 0x0F) {
            port->id = 0x0F;
            port->buttons = 0xFFFF;
            memset(port->axes, 0x80, sizeof(port->axes));
            port->report_dirty = 1;
        }
    }
}

void ACKPin_OnRisingEdge()
{
    controllerAcked = true;
}