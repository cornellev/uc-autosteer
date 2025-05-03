#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/uart.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1
int count = 0;
float received_value = 0;
float received_steering_angle = 0.0;
float received_throttle = 0.0;
volatile float steer = 0;
volatile float brake = 0;
volatile float throttle = 0;

volatile float last_throttle = 0;
const uint PIN = 19;

void core1_entry() {
    while (true) {
        char buffer[100];
        int index = 0;
        char curr;

        if (uart_is_readable(UART_ID)) {
            curr = uart_getc(uart0);

            if (curr != '(') {
                continue;
            }

            memset(buffer, 0, sizeof(buffer));

            while (curr != ')' && index < 100) {
                if (uart_is_readable(UART_ID)) {
                    curr = uart_getc(uart0);
                    buffer[index++] = curr;
                }
            }

            buffer[index] = '\0';

            float steer_, brake_, throttle_ = 0;
            if (sscanf(buffer, "%f,%f,%f", &steer_, &brake_, &throttle_) == 3) {
                steer = steer_;
                brake = brake_;
                throttle = throttle_;
            }
        }
    }
}

void step() {
    gpio_put(PIN, 1);
    sleep_ms(1000);
    gpio_put(PIN, 0);
    sleep_ms(1000);
}

int main() {
    multicore_launch_core1(core1_entry);
    int iters = 0;

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);
    
    gpio_init(PIN);
    gpio_set_dir(PIN, GPIO_OUT);

    while (true) {
        if (throttle > 0 && last_throttle == 0) {
            step();
        }
        last_throttle == throttle;
        printf("Throttle: %f\n", throttle);
    }
    
    return 0;
}