
//
// Sketch Settings:
//   Board: ESP32 Dev Module
//   (rest are default settings)
//
// Requires the following URL to be in the Additional Board Managers URL list:
//   https://dl.espressif.com/dl/package_esp32_index.json
//   (install the esp32 board set from this repository)
//

#include <WiFi.h>
#include <WiFiUdp.h>

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

#include <rom/ets_sys.h>
#include <sys/time.h>

static esp_err_t spi_setup(void);
static void spi_poll_task(void *arg);
static void spi_pre_cb(spi_transaction_t *trans);
static void spi_post_cb(spi_transaction_t *trans);
static void spi_srdy_pin_isr(void *arg);

/* Slave TX state machine related functions. */
static void fspi_handle_slave_tx_req(void);
static bool fspi_perform_slave_header_read(void);
static bool fspi_perform_slave_payload_read(void);

/* Master TX state machine related functions. */
static void fspi_handle_srdy_ack(void);
static bool fspi_handle_master_tx_req(const uint8_t *buf, size_t len);
static bool fspi_make_frame(const uint8_t *buf, size_t len);
static void fspi_transmit_master_frame(void);

/* Helper functions to interact with the SPI thread. */
static void send_master_tx_req(const uint8_t *buf, size_t len);

// Secrets file to define WIFI_NAME, WIFI_PASS, UDP_DEST, and optionally UDP_PORT.
#include "local_wifi_config.h"

#ifndef UDP_PORT
#define UDP_PORT 37250
#endif

#define SPI_PIN_SCK         GPIO_NUM_16
#define SPI_PIN_MOSI        GPIO_NUM_17
#define SPI_PIN_MISO        GPIO_NUM_5
#define SPI_PIN_MRDY        GPIO_NUM_18
#define SPI_PIN_SRDY        GPIO_NUM_19

/* Returns 0 or 1 depending on SRDY's actual signal state. */
#define READ_SRDY()         ((GPIO.in >> SPI_PIN_SRDY) & 1)

/* Sets MRDY low. */
#define ASSERT_MRDY()       do { GPIO.out_w1tc = (1 << SPI_PIN_MRDY); } while(0)

/* Sets MRDY high. */
#define DEASSERT_MRDY()     do { GPIO.out_w1ts = (1 << SPI_PIN_MRDY); } while(0)

#define MAX_SPI_FRAME_PAYLOAD 253
#define SPI_FRAME_OVERHEAD 3

#define SPI_CMD_QUEUE_LEN 1

#define FSPI_START_OF_FRAME 0xFE

/* This is the agreed-upon delay (in usec) that the master should wait before clocking bytes from the slave. */
#define MRDY_TO_SCK_SLAVE_TX_DELAY  30

/* If the slave holds SRDY low longer than this period (in usec), it's considered a request to send. */
#define MAX_SLAVE_SRDY_ACK_PULSE    1000

/* Defines the interval at which the SRDY line is checked while waiting for an ack to complete. */
#define SRDY_ACK_PULSE_CHECK        5

const char *wifiName = WIFI_NAME;
const char *wifiPass = WIFI_PASS;

const char *udpAddr = UDP_DEST;
const int   udpPort = UDP_PORT;

#define SPI_THREAD_EVENT_SRDY_LOW 0x00000001
#define SPI_THREAD_EVENT_MSG_RECV 0x00000002

const EventBits_t SPI_THREAD_EVENT_ALL =
    SPI_THREAD_EVENT_SRDY_LOW |
    SPI_THREAD_EVENT_MSG_RECV ;

typedef enum {
    FSPI_STATE_IDLE,            /**< Waiting for either local TX request or slave to lower SRDY. */
    FSPI_STATE_MASTER_TX_REQ,   /**< Master has lowered MRDY, waiting for SRDY to lower. */
    FSPI_STATE_MASTER_TX,       /**< Master is actively transmitting entire frame. */
    FSPI_STATE_SLAVE_TX_HDR,    /**< Currently clocking in bytes to read header from slave   */
    FSPI_STATE_SLAVE_TX_PYLD,   /**< Currently clocking in bytes to read payload from slave. */
} FramedSpiState_t;

typedef enum {
    SPI_MSG_TYPE_TX_BUF,    /**< Request to send a message. */
} SpiCmdMsgType_t;

typedef struct {
    size_t   len;
    uint8_t *data;
} SpiCmdTxBuf_t;

typedef struct {
    SpiCmdMsgType_t type;
    union {
        SpiCmdTxBuf_t txBuf;
    } data;
} SpiCmdMsg_t;

static spi_device_handle_t spiDevice;
static QueueHandle_t spiCmdQueue;
static EventGroupHandle_t spiEvtGroup;

static volatile FramedSpiState_t spiState = FSPI_STATE_IDLE;

const uint8_t spiTestFrame[] = {
    0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x12, 0x34, 0x56, 0x78
};

static uint8_t           spi_frame_buf[256];
static size_t            spi_frame_buf_len;
static uint8_t           spi_rx_buf[256];
static spi_transaction_t spi_next_txn;

void setup() {

    esp_err_t err;

    Serial.begin(115200);

    // Allow us to use gpio_isr_handler_add() for individual pins.
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    spiCmdQueue = xQueueCreate(SPI_CMD_QUEUE_LEN, sizeof(SpiCmdMsg_t));
    if (spiCmdQueue == NULL) {
        Serial.printf("Could not create SPI cmd queue.\n");
        return;
    }

    spiEvtGroup = xEventGroupCreate();
    if (spiEvtGroup == NULL) {
        vQueueDelete(spiCmdQueue);

        Serial.printf("Could not create SPI event group.\n");
        return;
    }

    err = spi_setup();
    if (err != ESP_OK) {
        vQueueDelete(spiCmdQueue);
        vEventGroupDelete(spiEvtGroup);

        Serial.printf("Could not initialize SPI\n");
        return;
    }

    // Create a separate thread to handle waiting for SPI transactions.
    xTaskCreate(spi_poll_task, "spi_task", 1024*32, NULL, 4, NULL);
}

void loop()
{
    delay(1000);
    send_master_tx_req(spiTestFrame, sizeof(spiTestFrame));
}

static void spi_poll_task(void *arg)
{
    EventBits_t evt;
    esp_err_t err;
    int txn = 0;

    while (true)
    {
        // Wait for something to happen, and clear the event when it happens.
        evt = xEventGroupWaitBits(spiEvtGroup, SPI_THREAD_EVENT_ALL, pdTRUE, pdFALSE, portMAX_DELAY);

        if (evt)
        {
            /* Handle asynchronous SRDY signalling. */
            if (evt & SPI_THREAD_EVENT_SRDY_LOW)
            {
                switch (spiState)
                {
                case FSPI_STATE_IDLE:
                    /* We have not yet lowered MRDY, so this is a request to send from the slave. */
                    fspi_handle_slave_tx_req();
                    break;

                case FSPI_STATE_MASTER_TX_REQ:
                    /* Handle the slave ack state, while also double-checking that it's actually
                       an ack (pulse) vs. a request-to-send (stays low). */
                    fspi_handle_srdy_ack();
                    break;

                default:
                    Serial.printf("spur isr\n");
                    break;
                }
            }

            /* FIXME: we shouldn't rely on an event being set here. Just peek the queue if we're idle to see
                      if there is anything left to send. */

            /* Handle messages received to our command queue. */
            if (evt & SPI_THREAD_EVENT_MSG_RECV)
            {
                SpiCmdMsg_t msg;

                /* The event is also set after writing to the queue. Don't block here because
                   there should already be a message in the queue to retrieve. */
                if (xQueueReceive(spiCmdQueue, &msg, 0) == pdFALSE)
                {
                    /* This shouldn't happen. */
                    Serial.printf("queue rx fail\n");
                    continue;
                }

                switch (msg.type)
                {
                case SPI_MSG_TYPE_TX_BUF:

                    if (spiState == FSPI_STATE_IDLE)
                    {
                        if (fspi_handle_master_tx_req(msg.data.txBuf.data, msg.data.txBuf.len))
                        {
                            /* This buffer was malloced earlier. */
                            free(msg.data.txBuf.data);
                        }
                        else
                        {
                            // FIXME: add a tx queue or something that's checked when a transaction completes
                            Serial.printf("discarding tx msg %d\n", msg.data.txBuf.len);
                            free(msg.data.txBuf.data);
                        }
                    }

                    break;
                }
            }
        }
    }
}

const gpio_config_t gpioCfg_srdy = {
    .pin_bit_mask = (1 << SPI_PIN_SRDY),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_NEGEDGE,
};

const gpio_config_t gpioCfg_mrdy = {
    .pin_bit_mask = (1 << SPI_PIN_MRDY),
    .mode         = GPIO_MODE_OUTPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE,
};

const spi_bus_config_t spiCfg = {
    .mosi_io_num     = SPI_PIN_MOSI,
    .miso_io_num     = SPI_PIN_MISO,
    .sclk_io_num     = SPI_PIN_SCK,
    .quadwp_io_num   = -1,
    .quadhd_io_num   = -1,
    .max_transfer_sz = 256,
    .flags           = SPICOMMON_BUSFLAG_MASTER,
    .intr_flags      = 0,
};

const spi_device_interface_config_t spiDevCfg = {
    .command_bits     = 0,
    .address_bits     = 0,
    .dummy_bits       = 0,
    .mode             = 1, // CPOL = 0, CPHA = 1
    .duty_cycle_pos   = 0,
    .cs_ena_pretrans  = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz   = 1000000,
    .input_delay_ns   = 0,
    .spics_io_num     = -1,
    .flags            = 0,
    .queue_size       = 1,
    .pre_cb           = spi_pre_cb,
    .post_cb          = spi_post_cb,
};

// Sets up HSPI in master mode.
esp_err_t spi_setup(void)
{
    esp_err_t err;

    //
    // Set up pin muxing using the GPIO matrix.
    //

    // Configure our GPIOs.
    gpio_config(&gpioCfg_mrdy);
    gpio_set_level(SPI_PIN_MRDY, 1);

    gpio_config(&gpioCfg_srdy);
    gpio_isr_handler_add(SPI_PIN_SRDY, spi_srdy_pin_isr, NULL);

    // TODO: check if SRDY is already low?

    // Configure HSPI / SPI2 as our master SPI bus, no DMA
    err = spi_bus_initialize(HSPI_HOST, &spiCfg, 0);
    if (err != ESP_OK)
    {
        Serial.printf("spi_bus_initialize err\n");
        return err;
    }

    err = spi_bus_add_device(HSPI_HOST, &spiDevCfg, &spiDevice);
    if (err != ESP_OK)
    {

        spi_bus_free(HSPI_HOST);

        Serial.printf("spi_bus_add_device\n");
        return err;
    }

    Serial.printf("SPI initialization success\n");
    return ESP_OK;
}

static void IRAM_ATTR spi_pre_cb(spi_transaction_t *trans)
{

}

static void IRAM_ATTR spi_post_cb(spi_transaction_t *trans)
{
    //GPIO.out_w1ts = (1 << SPI_PIN_MRDY);
}

static bool fspi_make_frame(const uint8_t *buf, size_t len)
{

    uint8_t fcs;

    if (len > MAX_SPI_FRAME_PAYLOAD) {
        return false;
    }

    /* Calculate the FCS which is just XOR of len and all buf bytes. */
    fcs = len;
    for (size_t i = 0; i < len; i++)
    {
        fcs ^= buf[i];
    }

    /* Fill out the entire buffer. */
    spi_frame_buf[0] = FSPI_START_OF_FRAME;
    spi_frame_buf[1] = (uint8_t)len;
    memcpy(&spi_frame_buf[2], buf, len);
    spi_frame_buf[len + 2] = fcs;

    /* Record the length of the buffer. */
    spi_frame_buf_len = SPI_FRAME_OVERHEAD + len;

    return true;
}

static void IRAM_ATTR spi_srdy_pin_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken;
    uint32_t state = READ_SRDY();

    if (!state)
    {
        xEventGroupSetBitsFromISR(spiEvtGroup, SPI_THREAD_EVENT_SRDY_LOW, &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR();
        }
    }
}

static bool fspi_perform_slave_header_read(void)
{
    int attempts = 10;
    spiState = FSPI_STATE_SLAVE_TX_HDR;

    memset(spi_frame_buf, 0, sizeof(spi_frame_buf));

    while (attempts > 0)
    {
        attempts--;

        spi_next_txn.flags = 0;
        spi_next_txn.cmd = 0;
        spi_next_txn.addr = 0;
        spi_next_txn.length = 2 * 8;
        spi_next_txn.rxlength = 0;
        spi_next_txn.user = 0;
        spi_next_txn.tx_buffer = spi_frame_buf;
        spi_next_txn.rx_buffer = spi_rx_buf;

        if (spi_device_polling_transmit(spiDevice, &spi_next_txn) != ESP_OK)
        {
            Serial.printf("slave header fail\n");
            return false;
        }

        if (spi_rx_buf[0] == FSPI_START_OF_FRAME)
        {
            if (spi_rx_buf[1] <= MAX_SPI_FRAME_PAYLOAD)
            {
                /* Complete header received. */
                return true;
            }
        }
        else if (spi_rx_buf[1] == FSPI_START_OF_FRAME)
        {
            /* Off by one byte, read the length byte. */

            spi_rx_buf[0] = spi_rx_buf[1];

            spi_next_txn.flags = 0;
            spi_next_txn.cmd = 0;
            spi_next_txn.addr = 0;
            spi_next_txn.length = 1 * 8;
            spi_next_txn.rxlength = 0;
            spi_next_txn.user = 0;
            spi_next_txn.tx_buffer = spi_frame_buf;
            spi_next_txn.rx_buffer = &spi_rx_buf[1];

            if (spi_device_polling_transmit(spiDevice, &spi_next_txn) != ESP_OK)
            {
                Serial.printf("slave prtl header fail\n");
                return false;
            }

            return true;
        }
    }

    return false;
}

static bool fspi_perform_slave_payload_read(void)
{
    /* We will need to read both the payload and final FCS byte. */
    uint8_t payload_size = spi_rx_buf[1];

    spi_next_txn.flags = 0;
    spi_next_txn.cmd = 0;
    spi_next_txn.addr = 0;
    spi_next_txn.length = (payload_size + 1) * 8;
    spi_next_txn.rxlength = 0;
    spi_next_txn.user = 0;
    spi_next_txn.tx_buffer = spi_frame_buf;
    spi_next_txn.rx_buffer = &spi_rx_buf[2];

    if (spi_device_polling_transmit(spiDevice, &spi_next_txn) != ESP_OK)
    {
        Serial.printf("slave payload fail\n");
        return false;
    }

    return true;
}

static void fspi_handle_slave_tx_req(void)
{
    /* Assert MRDY. */
    ASSERT_MRDY();

    /* Wait for the appropriate time before initiating the transaction. */
    ets_delay_us(MRDY_TO_SCK_SLAVE_TX_DELAY);

    if (fspi_perform_slave_header_read())
    {
        if (fspi_perform_slave_payload_read())
        {
            // TODO: verify checksum

            Serial.printf("Slave TX'd %d bytes\n", spi_rx_buf[1]);
        }
    }

    DEASSERT_MRDY();
}

static void fspi_handle_srdy_ack(void)
{
    bool slave_tx = false;
    uint32_t totalWait = 0;

    /* Wait for SRDY to go back high... */

    while (READ_SRDY() == 0)
    {
#if 0
        if (totalWait >= MAX_SLAVE_SRDY_ACK_PULSE)
        {
            Serial.printf("slave race 2\n");
            slave_tx = true;

            break;
        }

        totalWait += SRDY_ACK_PULSE_CHECK;
        ets_delay_us(SRDY_ACK_PULSE_CHECK);
#endif
    }

    /* Did we lose a signalling race here? */
    if (slave_tx)
    {
        /* Allow the slave to TX. */
        spiState = FSPI_STATE_IDLE;
        fspi_handle_slave_tx_req();
    }
    else
    {
        /* Proceed with the transmit as planned. */
        fspi_transmit_master_frame();
    }
}

static bool fspi_handle_master_tx_req(const uint8_t *buf, size_t len)
{
    if (!fspi_make_frame(buf, len))
    {
        return false;
    }

    memset(&spi_next_txn, 0, sizeof(spi_next_txn));

    spi_next_txn.flags = 0;
    spi_next_txn.cmd = 0;
    spi_next_txn.addr = 0;
    spi_next_txn.length = (sizeof(spiTestFrame) + SPI_FRAME_OVERHEAD) * 8;
    spi_next_txn.rxlength = 0;
    spi_next_txn.user = 0;
    spi_next_txn.tx_buffer = spi_frame_buf;
    spi_next_txn.rx_buffer = spi_rx_buf;

    /* Last chance to bail out of this transaction. */
    if (READ_SRDY() != 0)
    {
        /* SRDY is still valid. */
        ASSERT_MRDY();
        spiState = FSPI_STATE_MASTER_TX_REQ;
    }
    else
    {
        /* Set the event just in case (probably unnecessary). */
        xEventGroupSetBits(spiEvtGroup, SPI_THREAD_EVENT_SRDY_LOW);

        Serial.printf("slave race 1\n");
        return false;
    }
}

/** Main handler for the MASTER_TX state. Assumes spi_frame_buf[_len] are already filled. */
static void fspi_transmit_master_frame(void)
{
    /* spi_next_txn should have already been filled out in
       preparation by fspi_handle_master_tx_req. */

    if (spi_device_polling_transmit(spiDevice, &spi_next_txn) != ESP_OK)
    {
        Serial.printf("master tx fail\n");
    }

    spiState = FSPI_STATE_IDLE;
    DEASSERT_MRDY();
}

static void send_master_tx_req(const uint8_t *buf, size_t len)
{
    SpiCmdMsg_t msg;

    uint8_t *newBuf = (uint8_t *)malloc(len);

    if (newBuf == NULL)
    {
        Serial.printf("Failed to alloc buf\n");
        return;
    }

    memcpy(newBuf, buf, len);

    msg.type            = SPI_MSG_TYPE_TX_BUF;
    msg.data.txBuf.data = newBuf;
    msg.data.txBuf.len  = len;

    if(xQueueSend(spiCmdQueue, &msg, 0) == pdTRUE)
    {
        xEventGroupSetBits(spiEvtGroup, SPI_THREAD_EVENT_MSG_RECV);
    }
    else
    {
        Serial.printf("failed to queue tx\n");
    }
}
