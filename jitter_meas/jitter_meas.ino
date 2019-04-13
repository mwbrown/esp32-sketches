
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

// Uncomment this to enable printing out stats every second.
// #define ENABLE_SERIAL_STATS

const int measPin = 22;

static void http_task(void *arg);

// Secrets file to define WIFI_NAME, WIFI_PASS, UDP_DEST, and optionally UDP_PORT.
#include "local_wifi_config.h"

#ifndef UDP_PORT
#define UDP_PORT 37250
#endif

const char *wifiName = WIFI_NAME;
const char *wifiPass = WIFI_PASS;

const char *udpAddr = UDP_DEST;
const int   udpPort = UDP_PORT;

//
// Copied in from timer.h to access timer structures directly.
// Note we cannot call timer APIs not marked as IRAM_ATTR from inside the GPIO ISR.
//

typedef struct {
    union {
        struct {
            uint32_t reserved0:   10;
            uint32_t alarm_en:     1;             /*When set  alarm is enabled*/
            uint32_t level_int_en: 1;             /*When set  level type interrupt will be generated during alarm*/
            uint32_t edge_int_en:  1;             /*When set  edge type interrupt will be generated during alarm*/
            uint32_t divider:     16;             /*Timer clock (T0/1_clk) pre-scale value.*/
            uint32_t autoreload:   1;             /*When set  timer 0/1 auto-reload at alarming is enabled*/
            uint32_t increase:     1;             /*When set  timer 0/1 time-base counter increment. When cleared timer 0 time-base counter decrement.*/
            uint32_t enable:       1;             /*When set  timer 0/1 time-base counter is enabled*/
        };
        uint32_t val;
    } config;
    uint32_t cnt_low;                             /*Register to store timer 0/1 time-base counter current value lower 32 bits.*/
    uint32_t cnt_high;                            /*Register to store timer 0 time-base counter current value higher 32 bits.*/
    uint32_t update;                              /*Write any value will trigger a timer 0 time-base counter value update (timer 0 current value will be stored in registers above)*/
    uint32_t alarm_low;                           /*Timer 0 time-base counter value lower 32 bits that will trigger the alarm*/
    uint32_t alarm_high;                          /*Timer 0 time-base counter value higher 32 bits that will trigger the alarm*/
    uint32_t load_low;                            /*Lower 32 bits of the value that will load into timer 0 time-base counter*/
    uint32_t load_high;                           /*higher 32 bits of the value that will load into timer 0 time-base counter*/
    uint32_t reload;                              /*Write any value will trigger timer 0 time-base counter reload*/
} hw_timer_reg_t;

typedef struct hw_timer_s {
        hw_timer_reg_t * dev;
        uint8_t num;
        uint8_t group;
        uint8_t timer;
        portMUX_TYPE lock;
} hw_timer_t;

//
// End timer.h copy
//

#define RINGBUF_LEN           (1024*8)  /**< The number of 32-bit samples in the sample buffer. */
#define RINGBUF_MAX_PKT_SIZE  256       /**< The sample size at which a message MUST be sent. */
#define RINGBUF_MAX_WAIT_MS   500       /**< The maximum wait interval before a packet is forced to be sent. */

typedef struct ringbuf_s {
  size_t idx_r;
  size_t idx_w;
  volatile size_t count;
  uint32_t data[RINGBUF_LEN];
} ringbuf_t;

static hw_timer_t *timer;
static ringbuf_t ringbuf;
static portMUX_TYPE ringbuf_lock = portMUX_INITIALIZER_UNLOCKED;

static bool ticked;
static uint64_t lastTick;

static TaskHandle_t httpTaskHandle;

#ifdef ENABLE_SERIAL_STATS
volatile uint32_t num_interrupts;
volatile uint32_t num_dropped;
#endif

void IRAM_ATTR clockInputIsr()
{
  portENTER_CRITICAL_ISR(&ringbuf_lock);

#ifdef ENABLE_SERIAL_STATS
  num_interrupts++;
#endif


  // The next four lines are the contents of timerRead(), copied here because
  // timerRead() is not marked IRAM_ATTR and thus crashes if called from an ISR.
  timer->dev->update = 1;
  uint64_t timerHi = timer->dev->cnt_high;
  uint64_t timerLo = timer->dev->cnt_low;
  uint64_t timerVal = (timerHi << 32) | timerLo;

  // The first clock pulse will be bogus, so ignore it.
  if (ticked)
  {
    // Put the sample into the ring buffer.
    if (ringbuf.count < RINGBUF_LEN)
    {
      ringbuf.data[ringbuf.idx_w++] = (uint32_t)(timerVal - lastTick);
      ringbuf.count++;

      if (ringbuf.idx_w == RINGBUF_LEN)
      {
        ringbuf.idx_w = 0;
      }
    }
    else
    {
#ifdef ENABLE_SERIAL_STATS
      num_dropped++;
#endif
    }
  }

  ticked = true;
  lastTick = timerVal;

  if ((ringbuf.count >= RINGBUF_MAX_PKT_SIZE) && (httpTaskHandle != 0))
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(httpTaskHandle, &xHigherPriorityTaskWoken);
  }

  portEXIT_CRITICAL_ISR(&ringbuf_lock);
  portYIELD_FROM_ISR();
}

void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(measPin, INPUT_PULLUP);

  timer = timerBegin(0, 8, true); // Count-up timer @ 10MHz (base clock is 80MHz)
  timerStart(timer);

  Serial.printf("Timer value: %08X\n", (uint8_t *)timer);
  if (timer != NULL)
  {
    Serial.printf("Timer dev:   %08X\n", (uint8_t *)(timer->dev));
  }

  attachInterrupt(digitalPinToInterrupt(measPin), clockInputIsr, CHANGE);

  // Create a separate thread to handle HTTP stuff.
  xTaskCreate(http_task, "http_task", 1024*32, NULL, 4, NULL);
}

void loop() {

  // Should probably disable this portion for real data collection.
#ifdef ENABLE_SERIAL_STATS

  uint32_t nisr;
  uint32_t ndrp;
  uint32_t count;

  portENTER_CRITICAL(&ringbuf_lock);
  nisr = num_interrupts;
  ndrp = num_dropped;
  count = ringbuf.count;
  portEXIT_CRITICAL(&ringbuf_lock);

  Serial.printf("%d [%d] (%d)\n", nisr, ndrp, count);

#endif

  delay(1000);
}

static void http_task(void *arg) {
  WiFiUDP udp;
  uint32_t sampleBuf[RINGBUF_MAX_PKT_SIZE];
  TickType_t waitTime = pdMS_TO_TICKS(RINGBUF_MAX_WAIT_MS);

  memset(sampleBuf, 0, sizeof(sampleBuf));

  httpTaskHandle = xTaskGetCurrentTaskHandle();

  WiFi.begin(wifiName, wifiPass);

  Serial.println("Awaiting WiFi connection");
  while (WiFi.status() != WL_CONNECTED)  {
    delay(100);
  }

  Serial.println("Connected!");

  while (1) {

    size_t bytes = 0;
    size_t to_read = 0;
    size_t count;

    if (ulTaskNotifyTake(pdTRUE, waitTime) != 1)
    {
      /* Wait timed out. Check for the number of samples anyway. */
    }

    /* Check for the number of samples. */
    portENTER_CRITICAL(&ringbuf_lock);
    count = ringbuf.count;
    portEXIT_CRITICAL(&ringbuf_lock);

    if (count == 0)
    {
      continue;
    }

    /* Calculate our pointers, and copy the data. */
    to_read = count < RINGBUF_MAX_PKT_SIZE ? count : RINGBUF_MAX_PKT_SIZE;
    if ((RINGBUF_LEN - ringbuf.idx_r) < to_read)
    {
      const size_t first_copy = RINGBUF_LEN - ringbuf.idx_r;
      const size_t second_copy = to_read - first_copy;

      /* Two-phase copy. */
      memcpy(sampleBuf, &ringbuf.data[ringbuf.idx_r], first_copy * sizeof(uint32_t));
      memcpy(&sampleBuf[first_copy], ringbuf.data, second_copy * sizeof(uint32_t));

      /* Update the pointer, knowing it wrapped. */
      ringbuf.idx_r = second_copy;
    }
    else
    {
      /* One-phase copy. */
      memcpy(sampleBuf, &ringbuf.data[ringbuf.idx_r], to_read * sizeof(uint32_t));
      ringbuf.idx_r += to_read;

      /* Wrap the pointer, if necessary. */
      if (ringbuf.idx_r == RINGBUF_LEN) {
        ringbuf.idx_r = 0;
      }
    }

    /* Update the ring buffer with the amount we read out of it. */
    portENTER_CRITICAL(&ringbuf_lock);
    ringbuf.count -= to_read;
    portEXIT_CRITICAL(&ringbuf_lock);

    udp.beginPacket(udpAddr, udpPort);
    udp.write((uint8_t *)sampleBuf, sizeof(uint32_t) * to_read);
    udp.endPacket();
  }
}
