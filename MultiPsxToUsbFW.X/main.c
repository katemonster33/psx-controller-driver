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
(c) [2026] Microchip Technology Inc. and its subsidiaries.

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
#include <usb_core_transfer.h>
#include <usb_config.h>
#include <util/delay.h>
#include <string.h>
#include <avr/sleep.h>
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

#define USB_GAMEPAD_COUNT 4U
#define NUM_CONTROLLERS 1

typedef struct __attribute__((packed)) {
    uint8_t left_x;
    uint8_t left_y;
    uint8_t right_x;
    uint8_t right_y;
    uint8_t left_trigger;
    uint8_t right_trigger;
    uint16_t hat_buttons;
} usb_gamepad_report_t;
typedef char usb_gamepad_report_size_must_be_8_bytes[(sizeof(usb_gamepad_report_t) == 8U) ? 1 : -1];

#define PSX_BTN_SELECT   (1U << 8)
#define PSX_BTN_L3       (1U << 9)
#define PSX_BTN_R3       (1U << 10)
#define PSX_BTN_START    (1U << 11)
#define PSX_BTN_UP       (1U << 12)
#define PSX_BTN_RIGHT    (1U << 13)
#define PSX_BTN_DOWN     (1U << 14)
#define PSX_BTN_LEFT     (1U << 15)
#define PSX_BTN_L2       (1U << 0)
#define PSX_BTN_R2       (1U << 1)
#define PSX_BTN_L1       (1U << 2)
#define PSX_BTN_R1       (1U << 3)
#define PSX_BTN_TRIANGLE (1U << 4)
#define PSX_BTN_CIRCLE   (1U << 5)
#define PSX_BTN_CROSS    (1U << 6)
#define PSX_BTN_SQUARE   (1U << 7)


static controller_t controllers[NUM_CONTROLLERS];
static volatile uint8_t usb_gamepad_dirty[USB_GAMEPAD_COUNT];
static volatile bool usb_gamepad_report_busy[USB_GAMEPAD_COUNT];
static usb_gamepad_report_t usb_gamepad_report_buffer[USB_GAMEPAD_COUNT];
static USB_PIPE_t usb_gamepad_pipes[USB_GAMEPAD_COUNT] = {
    {.address = INTERFACE0ALTERNATE0_INTERRUPT_EP1_IN, .direction = USB_EP_DIR_IN},
    {.address = INTERFACE1ALTERNATE0_INTERRUPT_EP2_IN, .direction = USB_EP_DIR_IN},
    {.address = INTERFACE2ALTERNATE0_INTERRUPT_EP3_IN, .direction = USB_EP_DIR_IN},
    {.address = INTERFACE3ALTERNATE0_INTERRUPT_EP4_IN, .direction = USB_EP_DIR_IN},
};

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
static void   mark_all_gamepads_dirty(void);
static uint8_t psx_hat_get(uint16_t buttons);
static uint16_t psx_hid_buttons_get(uint16_t buttons);
static void   build_gamepad_report(uint8_t gamepad, usb_gamepad_report_t *report);
static void   send_usb_gamepad_reports(void);
static void   gamepad_report_sent_callback(USB_PIPE_t pipe, USB_TRANSFER_STATUS_t transferStatus, uint16_t bytesTransferred);

void USB_ConnectionHandler();
void ACKPin_OnRisingEdge();

/* ------------------------------------------------------------------ */
/*  Pin helpers                                                        */
/* ------------------------------------------------------------------ */

static inline void attn_low(controller_t *port)
{
    ATT_SetLow();
    //SPI0_Open(0);
    
    // { 0x25, 0xc7 },


    SPI0.CTRLB = SPI_MODE_3_gc | SPI_SSD_bm;    /* mode 3, disable SS slave-fault */
    SPI0.CTRLA = SPI_MASTER_bm                  /* master mode */
               | SPI_DORD_bm                    /* LSB first */
              | SPI_PRESC_DIV64_gc            /* /64 -> 250 kHz */
               | SPI_ENABLE_bm;
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

    ACK_SetInterruptHandler(ACKPin_OnRisingEdge);

    ATT_SetHigh();
    for (uint8_t i = 0; i < NUM_CONTROLLERS; i++) {
        controllers[i].id            = 0x0F;
        controllers[i].index         = i;
        controllers[i].buttons       = 0xFFFF;
        controllers[i].rumble_active = 0;
        memset(controllers[i].axes, 0x80, sizeof(controllers[i].axes));
    }
    mark_all_gamepads_dirty();

    while(1)
    {
        poll_controllers();

        USB_ConnectionHandler();
        send_usb_gamepad_reports();

        _delay_ms(10);
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
            if (status == SUCCESS) {
                for (uint8_t i = 0; i < USB_GAMEPAD_COUNT; i++) {
                    usb_gamepad_report_busy[i] = false;
                }
                mark_all_gamepads_dirty();
            }
            //LED_SetHigh();
        }
        else
        {
            // Stop USB operations
            status = USB_Stop();
            for (uint8_t i = 0; i < USB_GAMEPAD_COUNT; i++) {
                usb_gamepad_report_busy[i] = false;
            }
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
        SPI0.DATA = cmd;
        while (!(SPI0.INTFLAGS & SPI_RXCIF_bm)) { }
        uint8_t rx = SPI0.DATA;
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

static void mark_all_gamepads_dirty(void)
{
    for (uint8_t i = 0; i < USB_GAMEPAD_COUNT; i++) {
        usb_gamepad_dirty[i] = 1;
    }
}

static uint8_t psx_hat_get(uint16_t buttons)
{
    bool up    = ((buttons & PSX_BTN_UP) == 0U);
    bool right = ((buttons & PSX_BTN_RIGHT) == 0U);
    bool down  = ((buttons & PSX_BTN_DOWN) == 0U);
    bool left  = ((buttons & PSX_BTN_LEFT) == 0U);

    if (up && !down) {
        if (right && !left) return 1U;
        if (left && !right) return 7U;
        return 0U;
    }
    if (down && !up) {
        if (right && !left) return 3U;
        if (left && !right) return 5U;
        return 4U;
    }
    if (right && !left) return 2U;
    if (left && !right) return 6U;
    return 8U;
}

static uint16_t psx_hid_buttons_get(uint16_t buttons)
{
    static const uint16_t button_map[9] = {
        PSX_BTN_CROSS,
        PSX_BTN_CIRCLE,
        PSX_BTN_SQUARE,
        PSX_BTN_TRIANGLE,
        PSX_BTN_L1,
        PSX_BTN_R1,
        PSX_BTN_SELECT,
        PSX_BTN_START,
        PSX_BTN_L3,
    };

    uint16_t hid_buttons = 0U;
    for (uint8_t i = 0; i < 9U; i++) {
        if ((buttons & button_map[i]) == 0U) {
            hid_buttons |= (1U << i);
        }
    }
    return hid_buttons;
}

static void build_gamepad_report(uint8_t gamepad, usb_gamepad_report_t *report)
{
    report->left_x = 0x80U;
    report->left_y = 0x80U;
    report->right_x = 0x80U;
    report->right_y = 0x80U;
    report->left_trigger = 0U;
    report->right_trigger = 0U;
    report->hat_buttons = 8U;

    if (gamepad >= NUM_CONTROLLERS) {
        return;
    }

    controller_t *port = &controllers[gamepad];
    if (port->id == 0x0F) {
        return;
    }

    report->left_x = port->axes[2];
    report->left_y = port->axes[3];
    report->right_x = port->axes[0];
    report->right_y = port->axes[1];
    report->left_trigger = ((port->buttons & PSX_BTN_L2) == 0U) ? 0xFFU : 0U;
    report->right_trigger = ((port->buttons & PSX_BTN_R2) == 0U) ? 0xFFU : 0U;
    report->hat_buttons = psx_hat_get(port->buttons) | (psx_hid_buttons_get(port->buttons) << 4);
}

static void send_usb_gamepad_reports(void)
{
    if (vbusFlag == false) {
        return;
    }

    for (uint8_t i = 0; i < USB_GAMEPAD_COUNT; i++) {
        if (usb_gamepad_report_busy[i] == true) {
            continue;
        }

        bool dirty = (usb_gamepad_dirty[i] != 0U);
        if (i < NUM_CONTROLLERS) {
            dirty = dirty || (controllers[i].report_dirty != 0U);
        }

        if (dirty) {
            build_gamepad_report(i, &usb_gamepad_report_buffer[i]);
            RETURN_CODE_t report_status = USB_TransferWriteStart(usb_gamepad_pipes[i],
                                                                  (uint8_t *)&usb_gamepad_report_buffer[i],
                                                                  sizeof(usb_gamepad_report_buffer[i]),
                                                                  false,
                                                                  gamepad_report_sent_callback);
            if (report_status == SUCCESS) {
                usb_gamepad_report_busy[i] = true;
                usb_gamepad_dirty[i] = 0U;
                if (i < NUM_CONTROLLERS) {
                    controllers[i].report_dirty = 0U;
                }
            }
        }
    }
}

static void gamepad_report_sent_callback(USB_PIPE_t pipe, USB_TRANSFER_STATUS_t transferStatus, uint16_t bytesTransferred)
{
    (void)bytesTransferred;

    for (uint8_t i = 0; i < USB_GAMEPAD_COUNT; i++) {
        if (usb_gamepad_pipes[i].address == pipe.address) {
            usb_gamepad_report_busy[i] = false;
            if (transferStatus != USB_PIPE_TRANSFER_OK) {
                usb_gamepad_dirty[i] = 1U;
            }
            return;
        }
    }
}
