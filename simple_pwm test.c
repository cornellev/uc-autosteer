// to test the motors, rename the simple_pwm.c file to something else and change the name of this file to simple_pwm.c

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pt_cornell_rp2040_v1_3.h"

#define WRAPVAL 5000
#define CLKDIV 12.0f
#define LED 25

// GPIO we're using for PWM
#define PWM_OUT 9

// Variable to hold PWM slice number
uint slice_num ;

// PWM interrupt service routine
void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));
}

int main() {
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);

    stdio_init_all();
    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Mask our slice's IRQ output into the PWM block's single interrupt line, and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0.5*WRAPVAL);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0.5*WRAPVAL);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));
}