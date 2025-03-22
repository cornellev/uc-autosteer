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

#define WRAPVAL 5000
#define BRAKE_CLKDIV_MAX 15.0f
#define BRAKE_CLKDIV_MIN 7.5f
#define STEER_CLKDIV_MAX 15.0f
// #define STEER_CLKDIV_MIN 7.5f
#define STEER_CLKDIV_MIN 12.0f

#define brake_gear_ratio 13.5
#define brake_max 800
#define brake_min 0
#define steer_max 3400
#define steer_min 1450
#define threshold 0.08

float BRAKE_CLKDIV = 7.5;
float STEER_CLKDIV = 7.5;

#define BRAKE_OUT 15
#define BRAKE_DIR 13

#define STEER_OUT 9
#define STEER_DIR 3

#define POT_PIN 26
#define SENSOR 16
#define brake_kP 0.008
#define steer_kP 0.0003

// Variable to hold PWM slice number
uint brake_slice_num;
uint steer_slice_num;

// other variables...
int potentiometer_value = 0;

// brake variables
float brake_output = 0;
int brake_position = 0;
int brake_position_scaled = 0;
int brake_setpoint = 0;

// steer variables
float steer_output = 0;
int steer_position = 0;
int steer_setpoint = 0;

// Mapping of PWM slices to GPIO pins (adjust based on your setup)
uint slice;

void pwm_irq_handler() {
    uint32_t mask = pwm_get_irq_status_mask(); // Get active interrupt mask

    pwm_clear_irq(steer_slice_num);
    pwm_clear_irq(brake_slice_num);

    // if (mask & (1u << steer_slice_num)) {
    //   if (steer_output > threshold) {
    //     steer_position++;
    //   } else if (steer_output < -threshold) {
    //     steer_position--;
    //   }
    // }

    
    adc_select_input(1); 
    steer_position = adc_read();

    if (mask & (1u << brake_slice_num)) {
      if (brake_output > threshold) {
        brake_position++;
      } else if (brake_output < -threshold) {
        brake_position--;
      }
    }
}

void initialize() {
  stdio_init_all(); 

  gpio_init(BRAKE_DIR);
  gpio_init(STEER_DIR);
  gpio_set_function(BRAKE_OUT, GPIO_FUNC_PWM);
  gpio_set_function(STEER_OUT, GPIO_FUNC_PWM);

  brake_slice_num = pwm_gpio_to_slice_num(BRAKE_OUT);
  steer_slice_num = pwm_gpio_to_slice_num(STEER_OUT);

  gpio_set_dir(BRAKE_DIR, GPIO_OUT);
  gpio_set_dir(STEER_DIR, GPIO_OUT);
  gpio_put(BRAKE_DIR, 0);
  gpio_put(STEER_DIR, 0);

  adc_init();
  adc_gpio_init(POT_PIN);
  adc_gpio_init(SENSOR);
  
  // Mask our slice's IRQ output into the PWM block's single interrupt line, and register our interrupt handler
  pwm_clear_irq(brake_slice_num);
  pwm_clear_irq(steer_slice_num);
  pwm_set_irq_enabled(brake_slice_num, true);
  pwm_set_irq_enabled(steer_slice_num, true);
  
  irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  // This section configures the period of the PWM signals
  pwm_set_wrap(brake_slice_num, WRAPVAL);
  pwm_set_wrap(steer_slice_num, WRAPVAL);
  pwm_set_clkdiv(brake_slice_num, BRAKE_CLKDIV);
  pwm_set_clkdiv(steer_slice_num, STEER_CLKDIV);

  // Start the channel
  pwm_set_mask_enabled((1u << brake_slice_num) | (1u << steer_slice_num));

  adc_select_input(1); 
  steer_position = adc_read();
}

float read() {
    adc_select_input(0); 
    potentiometer_value = adc_read();

    // velocity control
    // if (potentiometer_value < 1365) // lowest third of the potentiometer's range (0-1365)
    // {                  
    //   return -( 1.0 - (float)potentiometer_value / 1365.0 ); // normalize to percent, negative for CCW
    // }
    // else if (potentiometer_value < 2730) // middle third of potentiometer's range (1365-2730)
    // {
    //   return 0.0; // no movement when in center
    // }
    // else  // upper third of potentiometer"s range (2730-4095)
    // {
    //   return ((float)potentiometer_value-2730.0) / 1365.0; // normalize to percent, positive for CW
    // }

    // positional control
    return potentiometer_value;
}

void writePercent(uint dir, uint pwm, float value) {
  // constraint to value between -1 and 1
  value = (value > 1 ? 1.0 : value);
  value = (value < -1 ? -1.0 : value);

  // set direction
  int direction = (value > 0 ? 0 : 1);
  gpio_put(dir, direction);

  // make value positive, map to speed
  value = (value > 0 ? value : -value);
  float CLKDIV_MAX = pwm == brake_slice_num ? BRAKE_CLKDIV_MAX : STEER_CLKDIV_MAX;
  float CLKDIV_MIN = pwm == brake_slice_num ? BRAKE_CLKDIV_MIN : STEER_CLKDIV_MIN;
  float CLKDIV = CLKDIV_MAX - (CLKDIV_MAX - CLKDIV_MIN) * value;
  bool running = (value > threshold);
  float duty_cycle = (running ? 0.5 * WRAPVAL : 0);
  pwm_set_clkdiv(pwm, CLKDIV);
  pwm_set_chan_level(pwm, PWM_CHAN_B, duty_cycle);   
}

float calculate(float kP, int setpoint, int position) {
  return kP * (position - setpoint);
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main() {
    initialize();    

    int iters = 0;

    while (true) {
        brake_output = read();
        brake_position_scaled = brake_position / brake_gear_ratio;
        brake_setpoint = map(read(), 0, 4095, brake_min, brake_max);
        steer_setpoint = map(read(), 0, 4095, steer_min, steer_max);

        steer_setpoint = (steer_setpoint > steer_max ? steer_max : steer_setpoint);
        steer_setpoint = (steer_setpoint < steer_min ? steer_min : steer_setpoint);

        brake_output = calculate(brake_kP, brake_setpoint, brake_position_scaled);
        steer_output = calculate(steer_kP, steer_setpoint, steer_position);
        // writePercent(BRAKE_DIR, brake_slice_num, brake_output);

        if (iters > 10000) {
          printf("Steer Position: %d\tSteer Setpoint: %d\tSteer Output: %f\n", steer_position, steer_setpoint, steer_output);
          iters = 0;
        } else {
          iters += 1;
        }

        if (steer_position < 1450 && steer_output > 0) {
          steer_output = 0;
          // printf("ERROR\n");
        } else if (steer_position > 3400 && steer_output < 0) {
          steer_output = 0;
          // printf("ERROR\n");
        }


        writePercent(STEER_DIR, steer_slice_num, steer_output);

        
        // printf("Direction: %d\tOutput: %f\n", direction, output);
        // printf("Position: %d\tOutput: %f\n", brake_position, brake_output); 
        // sleep_ms(1000);
    }
}