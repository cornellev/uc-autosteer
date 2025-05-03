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

#define WRAPVAL 5000
#define CLKDIV 5.0f // only for throttle
#define STEER_CLKDIV_MAX 15.0f
#define STEER_CLKDIV_MIN 7.5f
#define BRAKE_CLKDIV_MAX 15.0f
#define BRAKE_CLKDIV_MIN 7.5f
#define LED 25
#define THRESHOLD 0.08f

// steer variables
#define STEER_GPIO 21
#define STEER_DIR_GPIO 20
#define STEER_SENSOR 28
uint STEER_SLICE;
const int STEER_MIN_POSITION = 800;
const int STEER_MAX_POSITION = 2200;
volatile int steer_position;

// brake variables
#define BRAKE_GPIO 19
#define BRAKE_DIR_GPIO 18
#define BRAKE_SENSOR 27;
uint BRAKE_SLICE;
volatile int brake_state;
volatile int brake_position;
volatile float brake_output;

// throttle variables
#define THROTTLE_L 16
#define THROTTLE_R 17
uint THROTTLE_L_SLICE;
uint THROTTLE_R_SLICE;
const float MAX_THROTTLE_CHANGE_PER_SECOND_UP = 0.5;
const float MAX_THROTTLE_CHANGE_PER_SECOND_DOWN = 2.0;
volatile float last_throttle = 0;
volatile uint16_t control = 0;
volatile uint32_t last_time_us = 0;
int min_duty_cycle = 825;

// serial communication
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

void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(STEER_GPIO));
    pwm_clear_irq(pwm_gpio_to_slice_num(BRAKE_GPIO));
    pwm_clear_irq(pwm_gpio_to_slice_num(THROTTLE_L));
    pwm_clear_irq(pwm_gpio_to_slice_num(THROTTLE_R));

    uint32_t mask = pwm_get_irq_status_mask();
    if (mask & (1u << BRAKE_SLICE)) {
        if (brake_output > THRESHOLD) {
          brake_position++;
        } else if (brake_output < -THRESHOLD) {
          brake_position--;
        }
      }
}

void initialize() {
    stdio_init_all();

    // led
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);

    // steer pwm
    gpio_set_function(STEER_GPIO, GPIO_FUNC_PWM);
    STEER_SLICE = pwm_gpio_to_slice_num(STEER_GPIO);
    pwm_set_wrap(STEER_SLICE, WRAPVAL);
    pwm_clear_irq(STEER_SLICE);
    pwm_set_irq_enabled(STEER_SLICE, true);

    // steer direction
    gpio_init(STEER_DIR_GPIO);
    gpio_set_dir(STEER_DIR_GPIO, GPIO_OUT);
    gpio_put(STEER_DIR_GPIO, 0);

    // steer sensor
    adc_init();
    adc_gpio_init(STEER_SENSOR);
    adc_select_input(2);
    steer_position = adc_read();

    // brake pwm
    gpio_set_function(BRAKE_GPIO, GPIO_FUNC_PWM);
    BRAKE_SLICE = pwm_gpio_to_slice_num(BRAKE_GPIO);
    pwm_set_wrap(BRAKE_SLICE, WRAPVAL);
    pwm_clear_irq(BRAKE_SLICE);
    pwm_set_irq_enabled(BRAKE_SLICE, true);
    // gpio_init(BRAKE_GPIO);
    // gpio_set_dir(BRAKE_GPIO, GPIO_OUT);
    // gpio_put(BRAKE_GPIO, 0);

    // brake direction
    gpio_init(BRAKE_DIR_GPIO);
    gpio_set_dir(BRAKE_DIR_GPIO, GPIO_OUT);
    gpio_put(BRAKE_DIR_GPIO, 0);

    // throttle
    gpio_set_function(THROTTLE_L, GPIO_FUNC_PWM);
    THROTTLE_L_SLICE = pwm_gpio_to_slice_num(THROTTLE_L);
    pwm_set_clkdiv(THROTTLE_L_SLICE, CLKDIV);
    pwm_set_wrap(THROTTLE_L_SLICE, WRAPVAL);
    pwm_clear_irq(THROTTLE_L_SLICE);
    pwm_set_irq_enabled(THROTTLE_L_SLICE, true);
    
    gpio_set_function(THROTTLE_R, GPIO_FUNC_PWM);
    THROTTLE_R_SLICE = pwm_gpio_to_slice_num(THROTTLE_R);
    pwm_set_clkdiv(THROTTLE_R_SLICE, CLKDIV);
    pwm_set_wrap(THROTTLE_R_SLICE, WRAPVAL);
    pwm_clear_irq(THROTTLE_R_SLICE);
    pwm_set_irq_enabled(THROTTLE_R_SLICE, true);

    // enable pwm
    pwm_set_mask_enabled(   (1u << BRAKE_SLICE) | 
                            (1u << STEER_SLICE) | 
                            (1u << THROTTLE_L_SLICE) | 
                            (1u << THROTTLE_R_SLICE)
                        );
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // serial communication
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

float constrain(float x, float min, float max) {
    float value = x;
    value = (value > max ? max : value);
    value = (value < min ? min : value);
    return value;
}

void writePercentSteer(float value) {
    float percent = constrain(value, -1, 1);
    int direction = (percent < 0); // set direction pin to 0 for positive / clockwise
    gpio_put(STEER_DIR_GPIO, direction);

    if (steer_position > STEER_MAX_POSITION || steer_position < STEER_MIN_POSITION) {
        percent = 0;
    }
    
    percent = (percent > 0 ? percent : -percent);
    float STEER_CLKDIV = STEER_CLKDIV_MAX - (STEER_CLKDIV_MAX - STEER_CLKDIV_MIN) * percent;
    float level = (percent > THRESHOLD) ? 0.5 * WRAPVAL : 0;
    pwm_set_clkdiv(STEER_SLICE, STEER_CLKDIV);
    pwm_set_chan_level(STEER_SLICE, PWM_CHAN_B, level);
}

void writePercentBrake(float value) {
    float percent = constrain(value, -1, 1);
    int direction = (percent > 0); // set direction pin to 0 for positive / clockwise
    gpio_put(BRAKE_DIR_GPIO, direction);
    
    percent = (percent > 0 ? percent : -percent);
    float BRAKE_CLKDIV = BRAKE_CLKDIV_MAX - (BRAKE_CLKDIV_MAX - BRAKE_CLKDIV_MIN) * percent;
    float level = (percent > THRESHOLD) ? 0.5 * WRAPVAL : 0;
    pwm_set_clkdiv(BRAKE_SLICE, BRAKE_CLKDIV);
    pwm_set_chan_level(BRAKE_SLICE, PWM_CHAN_B, level);
}

float calculate(float kP, int setpoint, int position) {
    if (abs(setpoint - position) > 50)
        return kP * (position - setpoint);
    else
        return 0;
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

void writePercentThrottle(float value) {
    float percent = constrain(value, 0, 1);
    float delta_time_us = time_us_32() - last_time_us;
    last_time_us += delta_time_us;

    float max_duty_cycle = last_throttle + MAX_THROTTLE_CHANGE_PER_SECOND_UP * delta_time_us / 1e6;
    float min_duty_cycle = last_throttle - MAX_THROTTLE_CHANGE_PER_SECOND_DOWN * delta_time_us / 1e6;
    max_duty_cycle = constrain(max_duty_cycle, 0.0, 1.0);
    min_duty_cycle = constrain(min_duty_cycle, 0.0, 1.0);
    percent = constrain(throttle, min_duty_cycle, max_duty_cycle);
    last_throttle = percent;

    pwm_set_chan_level(THROTTLE_L_SLICE, PWM_CHAN_A, (int)(percent * WRAPVAL));
    pwm_set_chan_level(THROTTLE_R_SLICE, PWM_CHAN_B, (int)(percent * WRAPVAL));
}

void read() {
    adc_select_input(2);
    steer_position = adc_read();

    adc_select_input(1);
    brake_state = adc_read();
    // if (brake_state == 0) { brake_position == 0; }
    brake_position = 5;
}

void step() {
    gpio_put(BRAKE_GPIO, 1);
    sleep_ms(1000);
    gpio_put(BRAKE_GPIO, 0);
}

void setBrake(int goal_state) {
    if (goal_state == 1 && brake_position < 10) {
        brake_output = 0.5;
    } else if (goal_state == 0 && brake_state == 1) {
        brake_output = -0.5;
    } else {
        brake_output = 0;
    }
    writePercentBrake(brake_output);
}

int main() {
    initialize();
    multicore_launch_core1(core1_entry);
    int iters = 0;

    while (true) {        
        read();

        int steer_setpoint = (int) map(steer, -1.0, 1.0, STEER_MIN_POSITION, STEER_MAX_POSITION);
        float steer_output = calculate(0.0003, steer_setpoint, steer_position);
        writePercentSteer(steer_output);

        // if (brake == 0) {
        //     setBrake(0);
        // } else {
        //     setBrake(1);
        // }
        // setBrake(0);

        brake_output = brake; // for testing
        writePercentBrake(brake_output);
        
        // writePercentThrottle(throttle);

        // if (throttle > 0 && last_throttle == 0) {
        //     step();
        // }
        // last_throttle == throttle;

        if (iters > 1000) {
            // printf("Steer Position: %d\tSteer Setpoint: %d\tSteer Output: %f\n", steer_position, steer_setpoint, steer_output);
            // printf("Steer Position: %d\tSteer Output: %f\n", adc_read(), steer_output);
            // printf("Steer Position: %d \n", adc_read());
            // printf("Throttle: %f\n", throttle);
            printf("Brake Output: %f\tBrake Position: %d\n", brake_output, brake_position);
            iters = 0;
        } else {
            iters += 1;
        }
    }
}