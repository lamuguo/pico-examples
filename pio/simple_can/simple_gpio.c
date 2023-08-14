#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

const uint PIN = 19;
#define INTERVAL_US 20
#define MAX_SAMPLES 1000

volatile uint32_t data_samples[MAX_SAMPLES];
volatile int64_t time_samples[MAX_SAMPLES];
volatile uint sample_count = 0;

int64_t gpio_timer_callback(alarm_id_t id, void *user_data) {
    if (sample_count >= MAX_SAMPLES) {
        return 0;  // Cancel the timer.
    }

    time_samples[sample_count] = to_us_since_boot(get_absolute_time());
    data_samples[sample_count] = gpio_get(PIN);

    sample_count++;
    return -INTERVAL_US;  // Reschedule.
}

int main() {
    stdio_init_all();
    printf("Started reading from GPIO\n");

    gpio_init(PIN);
    gpio_set_dir(PIN, GPIO_IN);

    add_alarm_in_us(INTERVAL_US, gpio_timer_callback, NULL, true);

    while (sample_count < MAX_SAMPLES) {
        sleep_ms(10);
    }

    for (uint i = 0; i < MAX_SAMPLES; i++) {
        printf("%llu - %u\n", time_samples[i], data_samples[i]);
    }

    return 0;
}
