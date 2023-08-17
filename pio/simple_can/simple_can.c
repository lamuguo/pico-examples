#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "simple_can.pio.h" // 这是你的PIO程序的头文件，名字可能有所不同

#define SAMPLE_SIZE 1024
#define PIN_GP19 19

volatile uint32_t data_samples[SAMPLE_SIZE];
volatile int64_t time_samples[SAMPLE_SIZE];
volatile uint sample_count = 0;

static PIO pio;
static uint sm;

void pio_irq_handler() {
    if (sample_count == SAMPLE_SIZE) {
        printf("data\n");
        for (int i = 0; i < SAMPLE_SIZE; ++i) {
            printf("%llu - %u\n", time_samples[i], data_samples[i] >> 31);
        }
        printf("\n");
        sample_count = 0;
    }

    while (!pio_sm_is_rx_fifo_empty(pio, sm)) { // 检查FIFO是否有数据
        data_samples[sample_count] = pio_sm_get(pio0, 0);
        time_samples[sample_count] = to_us_since_boot(get_absolute_time());
        sample_count++;
    }
}

int main() {
    stdio_init_all();
    printf("Started reading from GPIO\n");

    pio = pio0;
    uint offset = pio_add_program(pio, &simple_can_program);

    // 初始化SM
    sm = pio_claim_unused_sm(pio, true);

    pio_sm_set_consecutive_pindirs(pio, sm, PIN_GP19, 1, false);
    pio_gpio_init(pio, PIN_GP19);
    gpio_pull_up(PIN_GP19);
    pio_sm_config config = simple_can_program_get_default_config(offset);
    sm_config_set_in_pins(&config, PIN_GP19); // for WAIT, IN
    sm_config_set_jmp_pin(&config, PIN_GP19); // for JMP
    // Shift to right, autopush disabled
    sm_config_set_in_shift(&config, true, false, 32);

    float div = (float)clock_get_hz(clk_sys) / (2 * 50000);
    sm_config_set_clkdiv(&config, div);

    // SM transmits 1 bit per 8 execution cycles.
    pio_sm_init(pio, sm, offset, &config);
    pio_sm_set_enabled(pio, sm, true);

    irq_add_shared_handler(PIO0_IRQ_0, pio_irq_handler, PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY);
    irq_set_enabled(PIO0_IRQ_0, true);

    // 确保使用正确的中断源名称。
    pio_set_irq0_source_enabled(pio, pis_sm0_rx_fifo_not_empty + sm, true);

    while (1) {
        // 主程序循环，可能包含其他代码
        sleep_ms(5);
    }
}
