#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define UART1_ID uart1
#define BAUD_RATE 57600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define UART1_TX_PIN 5
#define UART1_RX_PIN 4

void uart_init_config(uart_inst_t* uart, uint tx_pin, uint rx_pin) {
  uart_init(uart, BAUD_RATE);

  gpio_set_function(tx_pin, GPIO_FUNC_UART);
  gpio_set_function(rx_pin, GPIO_FUNC_UART);

  uart_set_hw_flow(uart, false, false);
  uart_set_format(uart, DATA_BITS, STOP_BITS, PARITY);
  uart_set_fifo_enabled(uart, false);
}

#define GCONF_REGISTER 0x00
#define UART_BIT 25

#define STEP_PIN 7

void tmc2209_write_register(uart_inst_t* uart, uint8_t address, uint32_t data) {
  uint8_t packet[8];

  // SYNC
  packet[0] = 0x05;

  // Slave address
  packet[1] = 0x00;

  // Register address
  packet[2] = address;

  // Data, MSB first
  packet[3] = (data >> 24) & 0xFF;
  packet[4] = (data >> 16) & 0xFF;
  packet[5] = (data >> 8) & 0xFF;
  packet[6] = data & 0xFF;

  // Calculate checksum
  uint8_t checksum = 0;
  for(int i = 0; i < 7; i++) {
    checksum += packet[i];
  }
  packet[7] = checksum;

  uart_write_blocking(uart, packet, sizeof(packet));
}

void enable_tmc2209_uart_mode(uart_inst_t* uart) {
  // Read the current value of the GCONF register
  tmc2209_write_register(uart, GCONF_REGISTER, 0);
  // Add some delay here for TMC2209 to respond
  sleep_ms(1);

  uint32_t gconf = 0;
  // You should modify this part to read the real response from TMC2209
  // Here is a dummy implementation
  if (uart_is_readable(uart)) {
    gconf = uart_getc(uart);
  }

  printf("xfguo: gconf = %x\n", gconf);

  // Set the UART bit
  gconf |= (1 << UART_BIT);

  // Write back the new GCONF value
  tmc2209_write_register(uart, GCONF_REGISTER, gconf);

  if (uart_is_readable(uart)) {
    gconf = uart_getc(uart);
  }
  printf("xfguo: confirm gconf = %x\n", gconf);

  printf("xfguo: done for enable_tmc2209_uart_mode()\n");
}

#define IHOLD_IRUN_REGISTER 0x10
#define CHOPCONF_REGISTER 0x6C
#define VACTUAL_REGISTER 0x22

#define EN_SPREADCYCLE_BIT 2
#define STEALTHCHOP_BIT 14

void tmc2209_init(uart_inst_t* uart) {
  // Enable UART mode and stealthChop
  uint32_t gconf = (0 << EN_SPREADCYCLE_BIT);
  tmc2209_write_register(uart, GCONF_REGISTER, gconf);

  // Set the motor current to a low value (e.g. 10/31 of the maximum current)
  // You might need to adjust this value depending on your motor and power supply
  uint32_t ihold_irun = (10 << 0) | (10 << 8) | (10 << 16);
  tmc2209_write_register(uart, IHOLD_IRUN_REGISTER, ihold_irun);

  // Set the parameters for stealthChop (e.g. TOFF=3, HSTRT=4, HEND=1)
  // These values might need to be adjusted depending on the exact motor and operating conditions
  uint32_t chopconf = (3 << 0) | (4 << 4) | (1 << 7);
  tmc2209_write_register(uart, CHOPCONF_REGISTER, chopconf);

  uint32_t vactual = 1000;
  tmc2209_write_register(UART1_ID, VACTUAL_REGISTER, vactual);

}

void process_command(char command) {
  switch(command) {
    case 'a':
      printf("Got command 'a'\n");
      tmc2209_write_register(UART1_ID, 0x00, 0x12345678);
      break;
    default:
      printf("Unknown command\n");
  }
}

#define GSTAT_REGISTER 0x01
#define DRV_STATUS_REGISTER 0x6F
#define IOIN_REGISTER 0x06

void print_tmc2209_status(uart_inst_t* uart) {
  uint32_t gstat, drv_status, ioin;

  tmc2209_write_register(uart, GSTAT_REGISTER, 0);
  sleep_ms(1);
  if (uart_is_readable(uart)) {
    gstat = uart_getc(uart);
  }

  tmc2209_write_register(uart, DRV_STATUS_REGISTER, 0);
  sleep_ms(1);
  if (uart_is_readable(uart)) {
    drv_status = uart_getc(uart);
  }

  tmc2209_write_register(uart, IOIN_REGISTER, 0);
  sleep_ms(1);
  if (uart_is_readable(uart)) {
    ioin = uart_getc(uart);
  }

  printf("GSTAT: %x, DRV_STATUS: %x, IOIN: %x\n", gstat, drv_status, ioin);
}

int main() {
  stdio_init_all();

//  uart_init_config(UART0_ID, UART0_TX_PIN, UART0_RX_PIN);
  uart_init_config(UART1_ID, UART1_TX_PIN, UART1_RX_PIN);
  tmc2209_init(UART1_ID);

  gpio_init(STEP_PIN);
  gpio_set_dir(STEP_PIN, GPIO_OUT);

  while (true) {
    // Set a slow speed (e.g. 1000)
    // The units here depend on the motor and microstepping settings
//    printf("xfguo: tmc2209 try\n");
    gpio_put(STEP_PIN, 1);
    print_tmc2209_status(UART1_ID);
    sleep_ms(1);
    gpio_put(STEP_PIN, 0);

    sleep_ms(500);
  }

//  enable_tmc2209_uart_mode(UART1_ID);
//
//  char command;
//  while (true) {
//    if (uart_is_readable(UART0_ID)) {
//      command = uart_getc(UART0_ID);
//      process_command(command);
//    }
//
//    printf("Get response from TMC2209:\n");
//    while (uart_is_readable(UART1_ID)) {
//      char c = uart_getc(UART1_ID);
//      putchar(c);
//    }
//    printf("Finish one cycle.\n");
//  }

  return 0;
}
