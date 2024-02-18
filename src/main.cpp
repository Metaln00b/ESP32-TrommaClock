#include <Arduino.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "soc/ledc_reg.h"

#define LEDC_TIMER_RES          LEDC_TIMER_10_BIT
#define DUTY_MAX                ((1 << LEDC_TIMER_10_BIT) -1 )
#define FREQ_MIN_Hz             1 /* Do not decrease it! */

#define ONE_PIN                 5 /* SUS PIN */
#define TWO_PIN                 16
#define THREE_PIN               13
#define FOUR_PIN                14 /* SUS PIN */
#define FIVE_PIN                15 /* SUS PIN */
#define SIX_PIN                 4
#define SEVEN_PIN               17

#define EIGHT_PIN               18
#define NINE_PIN                19
#define TEN_PIN                 21 /* 1x */
#define TEN2_PIN                22 /* x0 */
#define ELEVEN_PIN              23 /* 1x */
#define ELEVEN2_PIN             25 /* x1 */
#define TWELVE_PIN              26 /* 1x */
#define TWELVE2_PIN             27 /* x2 */

#define PIN_COUNT               15

uint8_t pins_arr[PIN_COUNT] = {
    ONE_PIN, TWO_PIN, THREE_PIN, FOUR_PIN, FIVE_PIN, SIX_PIN, SEVEN_PIN,
    EIGHT_PIN, NINE_PIN, TEN_PIN, TEN2_PIN, ELEVEN_PIN, ELEVEN2_PIN, TWELVE_PIN, TWELVE2_PIN
};

void ledc_init(uint8_t pin, float freq_Hz, ledc_channel_t channel, ledc_timer_t timer) {
    const char * ME = __func__;

    esp_err_t err;
    periph_module_enable(PERIPH_LEDC_MODULE);

    uint32_t precision = DUTY_MAX + 1;
    uint32_t div_param = ((uint64_t) LEDC_REF_CLK_HZ << 8) / freq_Hz / precision;
    if (div_param < 256 || div_param > LEDC_DIV_NUM_HSTIMER0_V)
    {
        ESP_LOGE(ME, "requested frequency and duty resolution can not be achieved, try increasing freq_hz or duty_resolution. div_param=%d", (uint32_t ) div_param);
    }

    ledc_channel_config_t ledc_channel = {
      .gpio_num   = pin,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel    = channel,
      .intr_type  = LEDC_INTR_DISABLE,
      .timer_sel  = timer,
      .duty       = DUTY_MAX,
      .hpoint     = 0         // TODO: AD 10.11.2018: new, does 0 work (0xfffff does not work)
    };
    err = ledc_channel_config(&ledc_channel);
    ESP_LOGD(ME,"ledc_channel_config returned %d",err);
    
    err = ledc_timer_set(LEDC_HIGH_SPEED_MODE, timer, div_param, LEDC_TIMER_RES, LEDC_REF_TICK);
    if (err)
    {
        ESP_LOGE(ME, "ledc_timer_set returned %d",err);
    }
    
    ledc_timer_rst(LEDC_HIGH_SPEED_MODE, timer);
    ESP_LOGD(ME, "ledc_timer_set: divider: 0x%05x duty_resolution: %d\n", (uint32_t) div_param, LEDC_TIMER_RES);
}

void setup() {
    Serial.begin(9600);

    for (size_t i = 0; i < PIN_COUNT; i++)
    {
        pinMode(pins_arr[i], OUTPUT);
        
    }

    //ledc_init(pins_arr[i], 120, LEDC_CHANNEL_0, LEDC_TIMER_0);
    //ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 320);
    //ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

void loop() {
    for (size_t i = 0; i < PIN_COUNT; i++)
    {
        digitalWrite(pins_arr[i], HIGH); 
        delay(150);
    }

    delay(1000);

    //for (size_t i = PIN_COUNT - 1; i >= 0; i--)
    for (size_t i = 0; i < PIN_COUNT; i++)
    {
        digitalWrite(pins_arr[i], LOW);
        delay(150);
    }

    delay(1000);

    for (size_t i = 0; i < PIN_COUNT; i++)
    {
        digitalWrite(pins_arr[i], HIGH); 
    }

    delay(1000);

    for (size_t i = 0; i < PIN_COUNT; i++)
    {
        digitalWrite(pins_arr[i], LOW); 
    }

    delay(1000);

    digitalWrite(pins_arr[3-1], HIGH);
    digitalWrite(pins_arr[6-1], HIGH);
    digitalWrite(pins_arr[9-1], HIGH);
    digitalWrite(pins_arr[12+1], HIGH);
    digitalWrite(pins_arr[12+2], HIGH);

    delay(2000);
}