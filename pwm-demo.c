/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input
 *
 * This demonstration sets a PWM duty cycle to a
 * user-specified value.
 *
 * HARDWARE CONNECTIONS
 *   - GPIO 4 ---> PWM output
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/adc.h"

#include "pt_cornell_rp2040_v1_3.h"

// 5 kHz frequency
#define WRAPVAL 5000
#define CLKDIV 5.0f

// GPIO output for PWM
#define PWM_OUT_A 18
#define PWM_OUT_B 20

#define BRAKE_A 16
#define ENABLE_A 17

#define BRAKE_B 14
#define ENABLE_B 15

#define THROTTLE_ADC_PIN 26

#define MAX_DUTY_CYCLE 1.0f

#define MAX_THROTTLE_CHANGE_PER_SECOND 1.0f

// One PWM slice corresponds to both GPIO pins
uint slice_num_a = 1;
uint slice_num_b = 2;

volatile uint16_t last_control = 0;
volatile uint16_t control = 0;

// PWM interrupt service routine
// void on_pwm_wrap()
// {
//     // Clear the interrupt flag that brought us here
//     pwm_clear_irq(slice_num_a);
//     pwm_clear_irq(slice_num_b);

//     // Update duty cycle if control input changed
//     if (control != last_control)
//     {
//         // Both channels correspond to one GPIO pin
//         pwm_set_chan_level(slice_num_a, PWM_CHAN_A, control);
//         pwm_set_chan_level(slice_num_b, PWM_CHAN_B, control);
//         last_control = control;
//     }
// }

void on_pwm_wrap()
{
    // Check which PWM slice triggered the interrupt
    uint32_t irq_status = pwm_hw->ints;

    if (irq_status & (1 << slice_num_a))
    {
        pwm_clear_irq(slice_num_a);
    }

    if (irq_status & (1 << slice_num_b))
    {
        pwm_clear_irq(slice_num_b);
    }

    // Update duty cycle if control input changed
    if (control != last_control)
    {
        pwm_set_chan_level(slice_num_a, PWM_CHAN_A, control);
        pwm_set_chan_level(slice_num_a, PWM_CHAN_B, control);
        pwm_set_chan_level(slice_num_b, PWM_CHAN_A, control);
        pwm_set_chan_level(slice_num_b, PWM_CHAN_B, control);
        last_control = control;
    }
}

static PT_THREAD(protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    static int test_in;
    while (1)
    {
        // Read throttle
        uint16_t val = adc_read();

        if (val < 1500) // Stop if too small value (safety)
        {
            control = 0;
        }
        else
        {
            // Map to maximum 40%
            control = (int)((val - 1500) * (MAX_DUTY_CYCLE * 3));
        }
    }
    PT_END(pt);
}

int main()
{
    stdio_init_all();

    // Initialize throttle
    adc_init();
    adc_gpio_init(THROTTLE_ADC_PIN);
    adc_select_input(0);

    // GPIO Switch On Enable and Brake for both channels
    gpio_init(BRAKE_A);
    gpio_init(ENABLE_A);
    gpio_init(BRAKE_B);
    gpio_init(ENABLE_B);

    gpio_set_dir(BRAKE_A, GPIO_OUT);
    gpio_set_dir(ENABLE_A, GPIO_OUT);
    gpio_set_dir(BRAKE_B, GPIO_OUT);
    gpio_set_dir(ENABLE_B, GPIO_OUT);

    gpio_put(BRAKE_A, 1);
    gpio_put(ENABLE_A, 1);
    gpio_put(BRAKE_B, 1);
    gpio_put(ENABLE_B, 1);

    // LED
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

    gpio_set_function(PWM_OUT_A, GPIO_FUNC_PWM);
    gpio_set_function(PWM_OUT_B, GPIO_FUNC_PWM);

    // Schedule PWM setting interrupt
    pwm_clear_irq(slice_num_a);
    pwm_set_irq_enabled(slice_num_a, true);
    pwm_clear_irq(slice_num_b);
    pwm_set_irq_enabled(slice_num_b, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_set_wrap(slice_num_a, WRAPVAL);
    pwm_set_clkdiv(slice_num_a, CLKDIV);
    pwm_set_wrap(slice_num_b, WRAPVAL);
    pwm_set_clkdiv(slice_num_b, CLKDIV);

    pwm_set_chan_level(slice_num_a, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_num_a, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num_b, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_num_b, PWM_CHAN_B, 0);

    // Switch on PWM slice
    pwm_set_mask_enabled((1u << slice_num_a));
    pwm_set_mask_enabled((1u << slice_num_b));

    // start throttle thread
    pt_add_thread(protothread_serial);
    pt_schedule_start;
}