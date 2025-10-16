#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
#include "driver/adc_types_legacy.h"
#include "driver/gpio.h"
#include "driver/gptimer_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "hal/adc_types.h"
#include "hal/gpio_types.h"
#include "hal/timer_types.h"
#include "portmacro.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_struct.h"
#include "soc/timer_group_struct.h"
#include "rom/ets_sys.h"
#include "driver/gptimer.h"
#include <driver/adc.h>
#include <math.h>


#define DEBUG 1

#if DEBUG
inline void dbg(const char *restrict format, ...){
	va_list args;
	printf(format, args);
}
#else
inline void dbg(const char *restrict format, ...){}
#endif

#define PROBE_PIN (27UL)
#define PULSE_PIN (26UL)
#define BUZZER_PIN (25UL)

#define MAXIMUM_PROBE_DELAY_TICKS (2000)


#define MEASURES_PER_CLICK (10)
#define PULSE_LENGTH_US (500)
#define CLICK_LENGTH_US (2000)

#define SENSITIVITY (0.98)

gptimer_handle_t timer;


#define ERR_LONG_DELAY ((int)(-2))
#define ERR_SHORT_DELAY ((int)(-1))


int measure() {
	int sum = 0;
	uint64_t time1;
	uint64_t time2;
	for(int i = 0; i < MEASURES_PER_CLICK; i++){
		int lim = 10000;
		gpio_set_level(PULSE_PIN,1);
		ets_delay_us(PULSE_LENGTH_US);
		GPIO.out_w1tc = (1U << PULSE_PIN);
		// wait for negedge in probe pin
		// if(gpio_get_level(PROBE_PIN) == 0){
		// 	return ERR_SHORT_DELAY;
		// }
		TIMERG0.hw_timer->update.val = 1;
		time1 = TIMERG0.hw_timer->lo.val;
		while(gpio_get_level(PROBE_PIN) != 0 && --lim != 0);
		TIMERG0.hw_timer->update.val = 1;
		time2 = TIMERG0.hw_timer->lo.val;
		if(lim == 0){
			return ERR_LONG_DELAY;
		} else if (time2 - time1 < 5) {
			return ERR_SHORT_DELAY;
		}
		sum += time2-time1;
		GPIO.status_w1tc = (1UL << 27);
	}
	return sum;
}

// int measure_adc() {
// 	int sum = 0;
// 	for(int i = 0; i < 5; i++){
// 		int out = 0;
// 		gpio_set_level(PULSE_PIN,1);
// 		ets_delay_us(500);
// 		gpio_set_level(PULSE_PIN,0);
// 		// wait for negedge in probe pin
// 		ets_delay_us(500);
// 		adc2_get_raw(ADC_CHANNEL_9, ADC_WIDTH_BIT_12, &out);
// 		sum += out;
// 		ets_delay_us(1000);
// 	}
// 	return sum/5;
// }
//


int ERROR_TASK_ERRNO = 0;

void error_task(void* arg) {

	if(ERROR_TASK_ERRNO >= 0 || ERROR_TASK_ERRNO < -5) {
		gpio_set_level(BUZZER_PIN,1);
		while(1){vTaskDelay(portMAX_DELAY);}
		vTaskDelete(NULL);
	} 


	int err_beeps = -ERROR_TASK_ERRNO;

	while (1) {
			for(int i = 0; i < err_beeps; i++){
				ets_delay_us(250000);
				gpio_set_level(BUZZER_PIN, 1);
				ets_delay_us(250000);
				gpio_set_level(BUZZER_PIN, 0);
			}
			vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

}

void measure_task() {

	measure();
	int raw_ref = 0;
	for(int i = 0; i < 5; i++){
		int measure_again = measure();
		raw_ref = measure_again > raw_ref ? measure_again : raw_ref;
	}

	int ref = raw_ref * SENSITIVITY;

	int max_measure = 0;
	int min_measure = ref;

	while(1){
		int m = measure();
		printf("%d (%d)\n", m, ref);
		if(m < ref && m > 0){
				gpio_set_level(BUZZER_PIN,1);
				ets_delay_us(CLICK_LENGTH_US);
				gpio_set_level(BUZZER_PIN,0);
				if(m - min_measure > 0) {
					ets_delay_us(30000 * (m - min_measure) / ref);
				}
				max_measure = max_measure > m ? max_measure : m;
				min_measure = min_measure < m ? min_measure : m;
		} else if(m <= 0){
			ERROR_TASK_ERRNO = m;
			xTaskCreatePinnedToCore(error_task, "error", 1024, NULL, 5, NULL, 1);
			vTaskDelete(NULL);
			break;
		}
	}
}

void app_main(void)
{


	gpio_config_t cfg_probe = {
		.intr_type = GPIO_INTR_NEGEDGE,
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = (1UL << PROBE_PIN),
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.pull_up_en = GPIO_PULLUP_DISABLE,
	};
	gpio_config_t cfg_pulse =  {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1UL << PULSE_PIN),
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.pull_up_en = GPIO_PULLUP_DISABLE,
	};
	gpio_config_t cfg_buzzer =  {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1UL << BUZZER_PIN),
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.pull_up_en = GPIO_PULLUP_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&cfg_probe));
	ESP_ERROR_CHECK(gpio_config(&cfg_pulse));
	ESP_ERROR_CHECK(gpio_config(&cfg_buzzer));
	ESP_ERROR_CHECK(gpio_intr_enable(PROBE_PIN));


	gptimer_config_t cfg_timer = {
		.clk_src = GPTIMER_CLK_SRC_APB,
		.direction = GPTIMER_COUNT_UP,
		.intr_priority = 0,
		.resolution_hz = 40000000,
	};

	ESP_ERROR_CHECK(gptimer_new_timer(&cfg_timer, &timer));
	ESP_ERROR_CHECK(gptimer_enable(timer));
	ESP_ERROR_CHECK(gptimer_start(timer));

	// ESP_ERROR_CHECK(adc2_config_channel_atten(ADC_CHANNEL_9, ADC_ATTEN_DB_12));

	xTaskCreatePinnedToCore(measure_task, "measure", 4096, NULL, 5, NULL, 1);

	// ESP_ERROR_CHECK(gpio_isr_register(isr, NULL, ESP_INTR_FLAG_LOWMED, NULL));
}
