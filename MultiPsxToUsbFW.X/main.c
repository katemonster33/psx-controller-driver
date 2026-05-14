/*
 * File:   main.c
 * Author: pn1711
 *
 * Multi-PSX-to-USB firmware for the AVR16DU14.
 *
 * Ports the protocol logic originally written for the Arduino sketch
 * arduino-psx/ps1_bus/ps1_bus.ino over to the AVR16DU14:
 *
 *   - SPI is driven by the SPI0 hardware peripheral on its default pin
 *     mapping (PORTMUX.SPIROUTEA = default):
 *
 *         PA4 = MOSI  (COMMAND / TX  to controller)
 *         PA5 = MISO  (DATA    / RX  from controller, with internal pull-up)
 *         PA6 = SCK   (CLOCK)
 *         PA7 = SS    (ATTN / chip-select for Player 1, hardware SS pin)
 *
 *     The DU's SS pin is *not* used in hardware-driven mode; the SPI
 *     peripheral is set up as master with the slave-fault path disabled
 *     so we can bit-bang SS and hold it low for the entire multi-byte
 *     PSX command sequence.
 *
 *   - PD4 = ATTN / chip-select for Player 2 (generic GPIO).
 *   - PC3 = ACK input from the PSX controllers (active-low, open-drain).
 *
 *   - All UART debug / host-control traffic has been removed.  Instead
 *     the built-in USB peripheral enumerates as a composite HID gamepad
 *     device with two report IDs (one per player).  Rumble is received
 *     from the host through HID OUT reports.
 *
 * Targeted toolchain: AVR-GCC / XC8 for AVR (project is XC8 3.10).
 */

#define F_CPU 24000000UL    /* 24 MHz internal HF oscillator */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Pin definitions                                                    */
/* ------------------------------------------------------------------ */

/* SPI0 (default PORTMUX) */
#define SPI_PORT        PORTA
#define SPI_MOSI_bm     PIN4_bm
#define SPI_MISO_bm     PIN5_bm
#define SPI_SCK_bm      PIN6_bm
#define SPI_SS_bm       PIN7_bm     /* Player 1 attention */

/* Player 2 attention */
#define P2_ATTN_PORT    PORTD
#define P2_ATTN_bm      PIN4_bm

/* ACK from the PSX bus */
#define ACK_PORT        PORTC
#define ACK_bm          PIN3_bm

#define NUM_CONTROLLERS 2

/* Idle inter-byte gap (microseconds), matching the original sketch. */
#define INTER_CMD_BYTE_DELAY_US     15

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

static controller_t controllers[NUM_CONTROLLERS];

static const uint8_t readMode_template[]     = { 0x01, 0x42, 0x00, 0x00, 0x00 };
static const uint8_t setAnalogMode[]         = { 0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t enterConfigMode[]       = { 0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t enableRumbleMode[]      = { 0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF };
static const uint8_t exitConfigMode[]        = { 0x01, 0x43, 0x00, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A };

/* Mutable copy of read command so the rumble bytes can be patched per-frame. */
static uint8_t readMode[sizeof(readMode_template)];

/* ------------------------------------------------------------------ */
/*  Forward declarations                                               */
/* ------------------------------------------------------------------ */

static void clock_init(void);
static void pins_init(void);
static void spi_init(void);
static uint8_t spi_transfer(uint8_t v);
static bool   wait_ack(void);
static void   cleanup_psx(void);
static bool   send_string(controller_t *port, const uint8_t *str, uint8_t len);
static bool   send_string_retry(controller_t *port, const uint8_t *str, uint8_t len);
static void   setup_controller(controller_t *port);
static void   poll_controllers(void);

static void usb_init(void);
static void usb_task(void);
static void hid_publish_reports(void);

/* ------------------------------------------------------------------ */
/*  Pin helpers                                                        */
/* ------------------------------------------------------------------ */

static inline void attn_low(controller_t *port)
{
    if (port->index == 0) SPI_PORT.OUTCLR     = SPI_SS_bm;
    else                  P2_ATTN_PORT.OUTCLR = P2_ATTN_bm;
}

static inline void attn_high(controller_t *port)
{
    if (port->index == 0) SPI_PORT.OUTSET     = SPI_SS_bm;
    else                  P2_ATTN_PORT.OUTSET = P2_ATTN_bm;
}

/* ------------------------------------------------------------------ */
/*  Clock                                                              */
/* ------------------------------------------------------------------ */

static void clock_init(void)
{
    /* Run the main clock from the internal 24 MHz HF oscillator with the
     * USB-SOF auto-tune option enabled so the oscillator is calibrated by
     * USB host frame timing while the device is enumerated. */
    ccp_write_io((void *)&CLKCTRL.OSCHFCTRLA,
                 CLKCTRL_FRQSEL_24M_gc | CLKCTRL_AUTOTUNE_bm);
    ccp_write_io((void *)&CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCHF_gc);
    ccp_write_io((void *)&CLKCTRL.MCLKCTRLB, 0); /* No prescaler -> 24 MHz */
}

/* ------------------------------------------------------------------ */
/*  GPIO                                                               */
/* ------------------------------------------------------------------ */

static void pins_init(void)
{
    /* MOSI, SCK, SS outputs; idle high (PSX bus idles high). */
    SPI_PORT.DIRSET = SPI_MOSI_bm | SPI_SCK_bm | SPI_SS_bm;
    SPI_PORT.OUTSET = SPI_MOSI_bm | SPI_SCK_bm | SPI_SS_bm;

    /* MISO as input with internal pull-up (controllers drive open-drain). */
    SPI_PORT.DIRCLR = SPI_MISO_bm;
    PORTA.PIN5CTRL  = PORT_PULLUPEN_bm;

    /* Player-2 ATTN */
    P2_ATTN_PORT.DIRSET = P2_ATTN_bm;
    P2_ATTN_PORT.OUTSET = P2_ATTN_bm;

    /* ACK input with pull-up */
    ACK_PORT.DIRCLR = ACK_bm;
    PORTC.PIN3CTRL  = PORT_PULLUPEN_bm;
}

/* ------------------------------------------------------------------ */
/*  SPI peripheral                                                     */
/* ------------------------------------------------------------------ */

static void spi_init(void)
{
    /* Default PORTMUX (SPI0 on PA4..PA7) - explicit assignment for clarity. */
    PORTMUX.SPIROUTEA = PORTMUX_SPI0_DEFAULT_gc;

    /*
     * PSX bus requires:
     *   - SPI mode 3 (CPOL=1, CPHA=1)
     *   - LSB first
     *   - ~250 kHz bit clock
     *
     * With F_CPU = 24 MHz, the slowest prescaler available is CLK/128 ->
     * 187.5 kHz.  That is well under the 250 kHz target and within the
     * PSX controller's tolerance, so we use it.
     */
    SPI0.CTRLB = SPI_MODE_3_gc | SPI_SSD_bm;    /* mode 3, disable SS slave-fault */
    SPI0.CTRLA = SPI_MASTER_bm                  /* master mode */
               | SPI_DORD_bm                    /* LSB first */
               | SPI_PRESC_DIV128_gc            /* /128 -> 187.5 kHz */
               | SPI_ENABLE_bm;
}

static uint8_t spi_transfer(uint8_t v)
{
    SPI0.DATA = v;
    while (!(SPI0.INTFLAGS & SPI_IF_bm)) { }
    return SPI0.DATA;
}

/* ------------------------------------------------------------------ */
/*  PSX bus protocol                                                   */
/* ------------------------------------------------------------------ */

static bool wait_ack(void)
{
    /*
     * The original sketch simply waited a fixed inter-byte gap instead
     * of actually sampling ACK.  Keep that behaviour for compatibility -
     * it works with every controller tested in practice.  PC3 is still
     * wired up so callers can switch to true ACK polling later if
     * desired.
     */
    _delay_us(INTER_CMD_BYTE_DELAY_US);
    return true;
}

static void cleanup_psx(void)
{
    /* Release the bus: COMMAND/CLOCK idle high, all ATTN lines high. */
    SPI_PORT.OUTSET     = SPI_MOSI_bm | SPI_SCK_bm | SPI_SS_bm;
    P2_ATTN_PORT.OUTSET = P2_ATTN_bm;
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
        uint8_t rx  = spi_transfer(cmd);
        if (i < sizeof(port->psx_buffer)) port->psx_buffer[i] = rx;

        if (i == 1) {
            /* second byte tells us how many words of payload follow */
            new_len = ((rx & 0x0F) << 1) + 3;
            if (new_len > sizeof(port->psx_buffer)) new_len = sizeof(port->psx_buffer);
        }

        if (i != (new_len - 1) && !wait_ack()) {
            cleanup_psx();
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

    cleanup_psx();

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

/* ================================================================== */
/*  USB HID composite gamepad device                                  */
/* ================================================================== */
/*
 * The AVR16DU14 has a Full-Speed USB device peripheral whose endpoint
 * configuration is described by a buffer-descriptor table pointed to by
 * USB0.EPPTR.  We implement only the subset needed for a HID device with
 * one IN interrupt endpoint (for input reports) plus EP0 control traffic
 * for SET_REPORT (rumble) and standard enumeration.
 *
 * The HID report descriptor exposes two gamepads (one report ID per
 * player) so the host sees two distinct game controllers behind a single
 * USB interface.
 */

/* ---- Buffer-descriptor table -------------------------------------- */

typedef struct {
    uint16_t CNT;       /* byte count / configuration */
    uint8_t *DATAPTR;
    uint8_t  STATUS;
    uint8_t  CTRL;
    uint16_t reserved;
} usb_ep_desc_t;

typedef struct {
    usb_ep_desc_t OUT;
    usb_ep_desc_t IN;
} usb_ep_pair_t;

#define USB_NUM_ENDPOINTS   2   /* EP0 + EP1 */

static volatile usb_ep_pair_t usb_endpoints[USB_NUM_ENDPOINTS]
        __attribute__((aligned(2)));

/* Endpoint FIFO buffers */
#define EP0_SIZE    64
static uint8_t ep0_out_buf[EP0_SIZE];
static uint8_t ep0_in_buf [EP0_SIZE];

/* HID IN report buffer */
typedef struct __attribute__((packed)) {
    uint8_t  report_id;
    uint8_t  lx, ly, rx, ry;
    uint16_t buttons;       /* active-high: 1 = pressed */
} hid_gamepad_report_t;

static hid_gamepad_report_t ep1_in_buf;

#define HID_IN_REPORT_SIZE  ((uint8_t)sizeof(hid_gamepad_report_t))

/* ---- USB descriptors --------------------------------------------- */

static const uint8_t device_descriptor[] PROGMEM = {
    18,         /* bLength */
    0x01,       /* bDescriptorType = DEVICE */
    0x00, 0x02, /* bcdUSB 2.00 */
    0x00,       /* bDeviceClass (defined at interface level) */
    0x00,       /* bDeviceSubClass */
    0x00,       /* bDeviceProtocol */
    EP0_SIZE,   /* bMaxPacketSize0 */
    0xEB, 0x03, /* idVendor  = 0x03EB (Microchip/Atmel) */
    0x01, 0x21, /* idProduct = 0x2101 */
    0x00, 0x01, /* bcdDevice */
    0x01,       /* iManufacturer */
    0x02,       /* iProduct */
    0x00,       /* iSerialNumber */
    0x01        /* bNumConfigurations */
};

/*
 * HID report descriptor: two report IDs, each a "Game Pad" with four
 * 8-bit axes and 16 buttons.  Axes are reported as 8-bit unsigned
 * (0..0xFF), matching the PSX analog stick range.
 */
static const uint8_t hid_report_descriptor[] PROGMEM = {
    /* Player 1 */
    0x05, 0x01,         /* Usage Page (Generic Desktop) */
    0x09, 0x05,         /* Usage (Game Pad)             */
    0xA1, 0x01,         /* Collection (Application)     */
    0x85, 0x01,         /*  Report ID (1)               */
    0x09, 0x01,         /*  Usage (Pointer)             */
    0xA1, 0x00,         /*  Collection (Physical)       */
    0x09, 0x30,         /*   Usage (X)                  */
    0x09, 0x31,         /*   Usage (Y)                  */
    0x09, 0x32,         /*   Usage (Z)                  */
    0x09, 0x35,         /*   Usage (Rz)                 */
    0x15, 0x00,         /*   Logical Min 0              */
    0x26, 0xFF, 0x00,   /*   Logical Max 255            */
    0x75, 0x08,         /*   Report Size 8              */
    0x95, 0x04,         /*   Report Count 4             */
    0x81, 0x02,         /*   Input (Data,Var,Abs)       */
    0xC0,               /*  End Collection              */
    0x05, 0x09,         /*  Usage Page (Button)         */
    0x19, 0x01,         /*  Usage Min (1)               */
    0x29, 0x10,         /*  Usage Max (16)              */
    0x15, 0x00,         /*  Logical Min 0               */
    0x25, 0x01,         /*  Logical Max 1               */
    0x75, 0x01,         /*  Report Size 1               */
    0x95, 0x10,         /*  Report Count 16             */
    0x81, 0x02,         /*  Input (Data,Var,Abs)        */
    /* Rumble OUT (1 byte, vendor-defined) */
    0x06, 0x00, 0xFF,   /*  Usage Page (Vendor-defined) */
    0x09, 0x01,         /*  Usage (vendor)              */
    0x75, 0x08,         /*  Report Size 8               */
    0x95, 0x01,         /*  Report Count 1              */
    0x91, 0x02,         /*  Output (Data,Var,Abs)       */
    0xC0,               /* End Collection               */

    /* Player 2 (identical layout, different Report ID) */
    0x05, 0x01,
    0x09, 0x05,
    0xA1, 0x01,
    0x85, 0x02,
    0x09, 0x01,
    0xA1, 0x00,
    0x09, 0x30,
    0x09, 0x31,
    0x09, 0x32,
    0x09, 0x35,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x04,
    0x81, 0x02,
    0xC0,
    0x05, 0x09,
    0x19, 0x01,
    0x29, 0x10,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x10,
    0x81, 0x02,
    0x06, 0x00, 0xFF,
    0x09, 0x01,
    0x75, 0x08,
    0x95, 0x01,
    0x91, 0x02,
    0xC0
};

#define HID_REPORT_DESC_SIZE  (sizeof(hid_report_descriptor))

static const uint8_t config_descriptor[] PROGMEM = {
    /* Configuration */
    9, 0x02,
    9 + 9 + 9 + 7, 0x00,    /* wTotalLength */
    1,                      /* bNumInterfaces */
    1,                      /* bConfigurationValue */
    0,                      /* iConfiguration */
    0x80,                   /* bmAttributes (bus-powered) */
    50,                     /* bMaxPower = 100 mA */

    /* Interface */
    9, 0x04,
    0,      /* bInterfaceNumber */
    0,      /* bAlternateSetting */
    1,      /* bNumEndpoints */
    0x03,   /* bInterfaceClass = HID */
    0x00,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0,      /* iInterface */

    /* HID class descriptor */
    9, 0x21,
    0x11, 0x01,                                     /* bcdHID 1.11 */
    0x00,                                           /* bCountryCode */
    1,                                              /* bNumDescriptors */
    0x22,                                           /* bDescriptorType = Report */
    (uint8_t)(HID_REPORT_DESC_SIZE & 0xFF),
    (uint8_t)((HID_REPORT_DESC_SIZE >> 8) & 0xFF),

    /* Endpoint - EP1 IN interrupt */
    7, 0x05,
    0x81,                       /* bEndpointAddress = EP1 IN */
    0x03,                       /* bmAttributes = Interrupt */
    HID_IN_REPORT_SIZE, 0x00,   /* wMaxPacketSize */
    5                           /* bInterval = 5 ms */
};

static const uint8_t string_lang[] PROGMEM = {
    4, 0x03, 0x09, 0x04         /* English (US) */
};
static const uint8_t string_mfr[] PROGMEM = {
    2 + 6*2, 0x03,
    'p',0,'n',0,'1',0,'7',0,'1',0,'1',0
};
static const uint8_t string_prod[] PROGMEM = {
    2 + 15*2, 0x03,
    'M',0,'u',0,'l',0,'t',0,'i',0,'P',0,'S',0,'X',0,'-',0,'t',0,'o',0,'-',0,'U',0,'S',0,'B',0
};

/* ---- USB state ---------------------------------------------------- */

static volatile uint8_t  usb_address_pending = 0;
static volatile uint8_t  usb_config_value    = 0;
static volatile uint8_t  ep1_in_busy         = 0;

/* Multi-packet IN data state for EP0. */
static const uint8_t *ep0_in_src      = NULL;
static uint16_t       ep0_in_len      = 0;
static uint8_t        ep0_in_progmem  = 0;

/* ---- USB low-level helpers --------------------------------------- */

static void usb_clock_init(void)
{
    /* Feed the USB peripheral from the auto-tuned internal OSCHF.  The
     * peripheral derives its 48 MHz timing from this and is calibrated
     * by USB SOF reception once enumerated. */
    ccp_write_io((void *)&CLKCTRL.USBCTRLA,
                 CLKCTRL_USBSEL_OSCHF_gc | CLKCTRL_USBEN_bm);
}

static void usb_load_ep0_in_chunk(void)
{
    uint8_t n = (ep0_in_len > EP0_SIZE) ? EP0_SIZE : (uint8_t)ep0_in_len;
    for (uint8_t i = 0; i < n; i++) {
        ep0_in_buf[i] = ep0_in_progmem
                      ? pgm_read_byte(ep0_in_src + i)
                      : ep0_in_src[i];
    }
    ep0_in_src += n;
    ep0_in_len -= n;

    usb_endpoints[0].IN.DATAPTR = ep0_in_buf;
    usb_endpoints[0].IN.CNT     = n;
    usb_endpoints[0].IN.STATUS &= ~(USB_BUSNAK0_bm | USB_TRNCOMPL0_bm);
}

static void usb_ep0_send(const uint8_t *src, uint16_t len, uint8_t from_progmem)
{
    ep0_in_src     = src;
    ep0_in_len     = len;
    ep0_in_progmem = from_progmem;
    usb_load_ep0_in_chunk();
}

static void usb_ep0_send_zlp(void)
{
    ep0_in_src     = NULL;
    ep0_in_len     = 0;
    ep0_in_progmem = 0;
    usb_endpoints[0].IN.DATAPTR = ep0_in_buf;
    usb_endpoints[0].IN.CNT     = 0;
    usb_endpoints[0].IN.STATUS &= ~(USB_BUSNAK0_bm | USB_TRNCOMPL0_bm);
}

static void usb_ep0_stall(void)
{
    usb_endpoints[0].IN.CTRL  |= USB_DOSTALL_bm;
    usb_endpoints[0].OUT.CTRL |= USB_DOSTALL_bm;
}

/* ---- Endpoint configuration -------------------------------------- */

static void usb_endpoints_configure(void)
{
    memset((void *)usb_endpoints, 0, sizeof(usb_endpoints));

    /* EP0 OUT: 64-byte control */
    usb_endpoints[0].OUT.CTRL    = USB_TYPE_CONTROL_gc | USB_BUFSIZE_DEFAULT_BUF64_gc;
    usb_endpoints[0].OUT.DATAPTR = ep0_out_buf;
    usb_endpoints[0].OUT.CNT     = 0;
    usb_endpoints[0].OUT.STATUS  = 0;

    /* EP0 IN: 64-byte control, start out NAKing until we have data. */
    usb_endpoints[0].IN.CTRL     = USB_TYPE_CONTROL_gc | USB_BUFSIZE_DEFAULT_BUF64_gc;
    usb_endpoints[0].IN.DATAPTR  = ep0_in_buf;
    usb_endpoints[0].IN.CNT      = 0;
    usb_endpoints[0].IN.STATUS   = USB_BUSNAK0_bm;

    /* EP1 IN: interrupt endpoint, HID report sized */
    usb_endpoints[1].IN.CTRL     = USB_TYPE_BULKINT_gc | USB_BUFSIZE_DEFAULT_BUF64_gc;
    usb_endpoints[1].IN.DATAPTR  = (uint8_t *)&ep1_in_buf;
    usb_endpoints[1].IN.CNT      = 0;
    usb_endpoints[1].IN.STATUS   = USB_BUSNAK0_bm;
    ep1_in_busy = 0;

    USB0.EPPTR = (uint16_t)(uintptr_t)&usb_endpoints[0];
}

/* ---- USB init ----------------------------------------------------- */

static void usb_init(void)
{
    usb_clock_init();
    usb_endpoints_configure();

    /* Attach as Full-Speed device. */
    USB0.CTRLB = USB_ATTACH_bm;
    USB0.CTRLA = USB_ENABLE_bm | (uint8_t)(USB_NUM_ENDPOINTS - 1); /* MAXEP */
}

/* ---- Standard request handling ----------------------------------- */

typedef struct __attribute__((packed)) {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_t;

static void usb_handle_setup(const usb_setup_t *s)
{
    uint8_t  type = (s->bmRequestType >> 5) & 0x03;     /* 0=std,1=class */
    uint16_t len  = s->wLength;

    if (type == 0) {  /* Standard */
        switch (s->bRequest) {
        case 0x05: /* SET_ADDRESS */
            usb_address_pending = (uint8_t)s->wValue;
            usb_ep0_send_zlp();
            return;
        case 0x06: { /* GET_DESCRIPTOR */
            uint8_t dtype  = s->wValue >> 8;
            uint8_t dindex = s->wValue & 0xFF;
            const uint8_t *src = NULL;
            uint16_t       n   = 0;

            if (dtype == 0x01) {                /* DEVICE */
                src = device_descriptor;
                n   = sizeof(device_descriptor);
            } else if (dtype == 0x02) {         /* CONFIGURATION */
                src = config_descriptor;
                n   = sizeof(config_descriptor);
            } else if (dtype == 0x03) {         /* STRING */
                switch (dindex) {
                case 0: src = string_lang; n = sizeof(string_lang); break;
                case 1: src = string_mfr;  n = sizeof(string_mfr);  break;
                case 2: src = string_prod; n = sizeof(string_prod); break;
                default: usb_ep0_stall(); return;
                }
            } else if (dtype == 0x22) {         /* HID Report */
                src = hid_report_descriptor;
                n   = HID_REPORT_DESC_SIZE;
            } else {
                usb_ep0_stall();
                return;
            }
            if (n > len) n = len;
            usb_ep0_send(src, n, 1 /* from progmem */);
            return;
        }
        case 0x09: /* SET_CONFIGURATION */
            usb_config_value = (uint8_t)s->wValue;
            usb_ep0_send_zlp();
            return;
        case 0x08: /* GET_CONFIGURATION */
            ep0_in_buf[0] = usb_config_value;
            usb_ep0_send(ep0_in_buf, 1, 0);
            return;
        case 0x00: /* GET_STATUS */
            ep0_in_buf[0] = 0;
            ep0_in_buf[1] = 0;
            usb_ep0_send(ep0_in_buf, 2, 0);
            return;
        default: break;
        }
    } else if (type == 1) {  /* Class (HID) */
        switch (s->bRequest) {
        case 0x0A: /* SET_IDLE */
        case 0x0B: /* SET_PROTOCOL */
            usb_ep0_send_zlp();
            return;
        case 0x01: /* GET_REPORT */
            ep0_in_buf[0] = ep1_in_buf.report_id;
            memcpy(ep0_in_buf + 1,
                   ((uint8_t *)&ep1_in_buf) + 1,
                   HID_IN_REPORT_SIZE - 1);
            usb_ep0_send(ep0_in_buf, HID_IN_REPORT_SIZE, 0);
            return;
        case 0x09: /* SET_REPORT - used here for rumble (data on EP0 OUT) */
            usb_ep0_send_zlp();
            return;
        default: break;
        }
    }

    usb_ep0_stall();
}

static void usb_apply_pending_address(void)
{
    if (usb_address_pending) {
        USB0.ADDR = usb_address_pending;
        usb_address_pending = 0;
    }
}

/* ---- USB poll task (call from main loop) ------------------------- */

static void usb_handle_setup_packet(void)
{
    usb_setup_t s;
    memcpy(&s, ep0_out_buf, sizeof(s));
    usb_handle_setup(&s);

    /* Re-arm OUT for the next setup/data */
    usb_endpoints[0].OUT.CNT    = 0;
    usb_endpoints[0].OUT.STATUS &= ~(USB_BUSNAK0_bm | USB_TRNCOMPL0_bm | USB_SETUP_bm);
}

static void usb_task(void)
{
    /* Bus reset */
    if (USB0.INTFLAGSA & USB_RESET_bm) {
        USB0.INTFLAGSA = USB_RESET_bm;
        USB0.ADDR = 0;
        usb_address_pending = 0;
        usb_config_value = 0;
        usb_endpoints_configure();
    }

    /* SETUP received on EP0 OUT? */
    if (usb_endpoints[0].OUT.STATUS & USB_SETUP_bm) {
        usb_handle_setup_packet();
    }

    /* EP0 IN transfer complete? */
    if (usb_endpoints[0].IN.STATUS & USB_TRNCOMPL0_bm) {
        usb_endpoints[0].IN.STATUS &= ~USB_TRNCOMPL0_bm;
        if (ep0_in_len > 0) {
            usb_load_ep0_in_chunk();
        } else {
            usb_endpoints[0].IN.STATUS |= USB_BUSNAK0_bm;
            usb_apply_pending_address();
        }
    }

    /* EP0 OUT data stage (e.g. SET_REPORT payload for rumble). */
    if (usb_endpoints[0].OUT.STATUS & USB_TRNCOMPL0_bm) {
        uint16_t n = usb_endpoints[0].OUT.CNT;
        if (n >= 2) {
            /* Convention: [report_id, rumble_byte] */
            uint8_t rid    = ep0_out_buf[0];
            uint8_t rumble = ep0_out_buf[1];
            if (rid >= 1 && rid <= NUM_CONTROLLERS) {
                controllers[rid - 1].rumble_active = (rumble != 0);
            }
        }
        usb_endpoints[0].OUT.CNT    = 0;
        usb_endpoints[0].OUT.STATUS &= ~(USB_TRNCOMPL0_bm | USB_BUSNAK0_bm);
    }

    /* EP1 IN transfer complete? */
    if (usb_endpoints[1].IN.STATUS & USB_TRNCOMPL0_bm) {
        usb_endpoints[1].IN.STATUS &= ~USB_TRNCOMPL0_bm;
        usb_endpoints[1].IN.STATUS |=  USB_BUSNAK0_bm;
        ep1_in_busy = 0;
    }
}

/* ---- HID input-report publication -------------------------------- */

static void hid_publish_reports(void)
{
    if (usb_config_value == 0) return;     /* Not configured yet */
    if (ep1_in_busy)          return;      /* Previous transfer still queued */

    for (uint8_t i = 0; i < NUM_CONTROLLERS; i++) {
        controller_t *port = &controllers[i];
        if (!port->report_dirty) continue;

        ep1_in_buf.report_id = i + 1;
        ep1_in_buf.rx = port->axes[0];     /* Right stick X */
        ep1_in_buf.ry = port->axes[1];     /* Right stick Y */
        ep1_in_buf.lx = port->axes[2];     /* Left  stick X */
        ep1_in_buf.ly = port->axes[3];     /* Left  stick Y */

        /* PSX reports buttons active-LOW; HID convention is active-HIGH. */
        ep1_in_buf.buttons = (uint16_t)(~port->buttons);

        port->report_dirty = 0;

        usb_endpoints[1].IN.DATAPTR = (uint8_t *)&ep1_in_buf;
        usb_endpoints[1].IN.CNT     = HID_IN_REPORT_SIZE;
        usb_endpoints[1].IN.STATUS &= ~(USB_BUSNAK0_bm | USB_TRNCOMPL0_bm);
        ep1_in_busy = 1;
        return;     /* one report per call - next call drains the other */
    }
}

/* ================================================================== */
/*  main                                                              */
/* ================================================================== */

int main(void)
{
    clock_init();
    pins_init();
    spi_init();

    for (uint8_t i = 0; i < NUM_CONTROLLERS; i++) {
        controllers[i].id            = 0x0F;
        controllers[i].index         = i;
        controllers[i].buttons       = 0xFFFF;
        controllers[i].rumble_active = 0;
        memset(controllers[i].axes, 0x80, sizeof(controllers[i].axes));
    }
    cleanup_psx();

    usb_init();
    sei();

    while (1) {
        poll_controllers();
        usb_task();
        hid_publish_reports();
        _delay_ms(1);
    }
}
