/**
 * Thank you to V. Hunter Adams (vha3@cornell.edu) for opening my eyes
 * to this beautiful program
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

#include "pt_cornell_rp2040_v1_3.h"

// 8 kHz frequency, with 5000 corresponding to 100% duty cycle
#define WRAPVAL 5000
#define CLKDIV 3.125f

// GPIO output for PWM
#define PWM_OUT_A 16
#define PWM_OUT_B 17

#define BRAKE_A 18
#define ENABLE_A 19

#define BRAKE_B 13
#define ENABLE_B 12

#define MAX_DUTY_CYCLE 1.0f

// One PWM slice corresponds to both GPIO pins
uint slice_num = 0;

volatile uint16_t last_control = 0;
volatile uint16_t control = 0;

// PWM interrupt service routine
void on_pwm_wrap() {
  // Clear the interrupt flag to restart interrupt timer
  pwm_clear_irq(slice_num);

  // Update duty cycle if control input changed
  if (control != last_control) { // TODO: Smoothen by capping duty cycle change
    // Both channels correspond to one GPIO pin
    pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    pwm_set_chan_level(slice_num, PWM_CHAN_B, control);
    last_control = control;
  }
}

static PT_THREAD(protothread_serial(struct pt *pt)) {
  PT_BEGIN(pt);
  static int test_in;
  while (1) {
    // Read throttle
    uint16_t val = adc_read();

    // This "mapping" only works for this specific sensor,
    // experimentally determined
    if (val < 1500) // Stop if too small value (safety)
    {
      control = 0;
    } else {
      // Map to maximum 40%
      control = (int)((val - 1500) * (MAX_DUTY_CYCLE * 3));
      // TODO: Add configuration for throttle lower limit, make map function
    }
  }
  PT_END(pt);
}

/**
 * @brief Set a GPIO pin high in the out direction
 *
 * @param pin
 */
void set_high(int pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_put(pin, 1);
}

int main() {
  stdio_init_all();

  // Throttle Sensor Analog Input
  adc_init();
  adc_gpio_init(26);
  adc_select_input(0);

  // ~~GPIO~~

  // GPIO Switch On Enable and Brake for both channels
  set_high(BRAKE_A);
  set_high(BRAKE_B);
  set_high(ENABLE_A);
  set_high(ENABLE_B);

  gpio_set_dir(BRAKE_A, GPIO_OUT);
  gpio_set_dir(ENABLE_A, GPIO_OUT);
  gpio_set_dir(BRAKE_B, GPIO_OUT);
  gpio_set_dir(ENABLE_B, GPIO_OUT);

  // Onboard LED
  set_high(25);

  // ~~PWM~~
  gpio_set_function(PWM_OUT_A, GPIO_FUNC_PWM);
  gpio_set_function(PWM_OUT_B, GPIO_FUNC_PWM);

  // Schedule PWM setting interrupt
  pwm_clear_irq(slice_num);
  pwm_set_irq_enabled(slice_num, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  pwm_set_wrap(slice_num, WRAPVAL);
  pwm_set_clkdiv(slice_num, CLKDIV);

  pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);

  // Switch on PWM slice
  pwm_set_mask_enabled((1u << slice_num));

  // start throttle read thread
  pt_add_thread(protothread_serial);
  pt_schedule_start;
}