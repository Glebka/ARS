#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <SSD1306.h>

#define SCK GPIO_NUM_5   // GPIO5  -- SX1278's SCK
#define MISO GPIO_NUM_19 // GPIO19 -- SX1278's MISO
#define MOSI GPIO_NUM_27 // GPIO27 -- SX1278's MOSI
#define SS GPIO_NUM_18   // GPIO18 -- SX1278's CS
#define RST GPIO_NUM_23  // GPIO14 -- SX1278's RESET
#define DI0 GPIO_NUM_26  // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define LED GPIO_NUM_25
#define SDA GPIO_NUM_21
#define SCL GPIO_NUM_22
#define BAND 433E6
#define DEFAULT_HEADER 0x77
#define LED_BLINK_INTERVAL 1000
#define ENTER_SLEEP_TIMEOUT 10000

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

static void init_ulp_program(void);
static void start_ulp_program(void);

typedef struct LoRaMessage_t
{
    uint8_t header;
    uint8_t cmd;
    uint8_t cnt;
    int32_t seed;
    uint8_t reserved;
} LoRaMessage;

enum CommandType
{
    CMD_PING,
    CMD_ALERT,
};

SSD1306 display(0x3c, SDA, SCL);

volatile bool packet_available = false;
int32_t last_seed = 0;
LoRaMessage last_message;

TimerHandle_t SleepTimerHandle = NULL;

void on_lora_received(int _)
{
    packet_available = true;
}

void init_lora()
{
    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DI0);
    if (!LoRa.begin(BAND))
    {
        printf("Failed to init LoRa\n");
        while (1)
            ;
    }
    LoRa.onReceive(on_lora_received);
    gpio_wakeup_enable(DI0, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    LoRa.receive();
}

void init_display()
{
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.clear();
}

void update_display()
{
    display.displayOn();
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Received " + String(ulp_alerts_received, DEC) + " alerts");
    display.display();
}

void enter_light_sleep_mode(TimerHandle_t timer)
{
    display.clear();
    display.displayOff();
    xTimerDelete(SleepTimerHandle, 0);
    SleepTimerHandle = NULL;
    esp_light_sleep_start();
}

void schedule_sleep()
{
    if (SleepTimerHandle == NULL)
    {
        SleepTimerHandle = xTimerCreate("SLEEP_TIMER",
                                        (ENTER_SLEEP_TIMEOUT),
                                        pdFALSE,
                                        NULL,                                           /* The ID is used, as the LED number so set it to that. */
                                        (TimerCallbackFunction_t)enter_light_sleep_mode /* The callback function that inspects the status of all the other tasks. */
        );
        if (SleepTimerHandle == NULL)
        {
            printf("Failed to create sleep timer!\n");
            return;
        }
        if (xTimerStart(SleepTimerHandle, 0) != pdPASS)
        {
            printf("The timer could not be set into the Active state.\n");
        }
    }
}

void reset_sleep_timer()
{
    if (SleepTimerHandle != NULL)
    {
        xTimerReset(SleepTimerHandle, 0);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println();

    rtc_gpio_init(LED);
    rtc_gpio_set_direction(LED, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(LED, 0);

    init_lora();
    init_display();
    init_ulp_program();
    start_ulp_program();
    update_display();
}

void loop()
{
    if (packet_available)
    {
        packet_available = false;
        while (LoRa.available() > 0)
        {
            printf("Available to read from LoRa: %d bytes\n", LoRa.available());
            size_t bytes_read = LoRa.readBytes((uint8_t *)&last_message, sizeof(LoRaMessage));
            if (bytes_read == sizeof(LoRaMessage) && last_message.header == DEFAULT_HEADER && last_message.cmd == CMD_ALERT && last_message.seed != last_seed)
            {
                last_seed = last_message.seed;
                ulp_alerts_received++;
                printf("Msg: cmd=0x%x, cnt=%d, seed=0x%x\n", last_message.cmd, last_message.cnt, last_message.seed);
                update_display();
            }
        }
        reset_sleep_timer();
    }
    schedule_sleep();
    delay(100);
}

static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* Set ULP wake up period to 1s */
    ulp_set_wakeup_period(0, 1000000);
}

static void start_ulp_program(void)
{
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}