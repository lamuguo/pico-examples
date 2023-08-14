#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "simple_can.pio.h"

uint irq_num;

// 为了设置PIO时钟为5MHz
void set_pio_clock(PIO pio, uint sm, int pio_freq) {
    uint32_t src_freq = clock_get_hz(clk_sys); // 获取系统时钟频率
    float div = (float)src_freq / (float)pio_freq; // 计算分频值
    pio_sm_set_clkdiv(pio, sm, div);  // 设置状态机的时钟分频
}

void simple_can_irq_handler() {
    // 处理中断，并从RX FIFO读取数据，然后可能将数据发送到其他地方
    uint data = pio_sm_get(pio0, 0); // 从状态机0的RX FIFO读取数据
    printf("Data: %u\n", data);  // 仅为示例，将数据打印到控制台
}

int main() {
    // 初始化
    stdio_init_all();
    printf("xfguo: start simple_can\n");

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &simple_can_program);

    pio_sm_clear_fifos(pio, sm);

    // PIO状态机配置
    pio_sm_config c = simple_can_program_get_default_config(offset);
    pio_sm_set_in_pins(pio, sm, 19);
    pio_sm_init(pio, sm, offset, &c);

    pio_sm_set_enabled(pio, sm, true);

    // 设置PIO时钟为5MHz
    set_pio_clock(pio, sm, 5000000);

    // 配置中断
    irq_num = PIO0_IRQ_0 + sm;
    irq_clear(irq_num);
    irq_set_exclusive_handler(irq_num, simple_can_irq_handler);
    irq_set_priority(irq_num, 1);
    irq_set_enabled(irq_num, true);
    pio_set_irq0_source_enabled(pio, PIO_INTR_SM0_RXNEMPTY_BITS + sm, true);

    while (true) {
        uint32_t res = 0;
        for (int i = 0; i < 8; ++i) {
            uint32_t data = pio_sm_get_blocking(pio, sm);
            res = (res << 1) | (data & 0x80000000 ? 1 : 0);
        }
        printf("result: %0x\n", res);

        sleep_ms(100);
    }
}