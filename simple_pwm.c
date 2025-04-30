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
#include "pt_cornell_rp2040_v1_3.h"

#define WRAPVAL 5000
#define STEER_CLKDIV_MAX 15.0f
#define STEER_CLKDIV_MIN 7.5f
#define BRAKE_CLKDIV_MAX 15.0f
#define BRAKE_CLKDIV_MIN 7.5f

#define LED 25

#define STEER_GPIO 9
#define STEER_DIR_GPIO 11
uint STEER_SLICE;
volatile int steer_position;

#define BRAKE_GPIO 4
#define BRAKE_DIR_GPIO 2
uint BRAKE_SLICE;

#define SENSOR 28

// serial communication
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1
int count = 0;
float received_value = 0;

float received_steering_angle = 0.0;
float received_throttle = 0.0;

void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(STEER_GPIO));
    pwm_clear_irq(pwm_gpio_to_slice_num(BRAKE_GPIO));

    adc_select_input(2); 
    steer_position = adc_read();
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
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // steer direction
    gpio_init(STEER_DIR_GPIO);
    gpio_set_dir(STEER_DIR_GPIO, GPIO_OUT);
    gpio_put(STEER_DIR_GPIO, 0);

    // steer sensor
    adc_init();
    adc_gpio_init(SENSOR);
    adc_select_input(2); 
    steer_position = adc_read();

    // brake pwm
    gpio_set_function(BRAKE_GPIO, GPIO_FUNC_PWM);
    BRAKE_SLICE = pwm_gpio_to_slice_num(BRAKE_GPIO);
    pwm_set_wrap(BRAKE_SLICE, WRAPVAL);
    pwm_clear_irq(BRAKE_SLICE);
    pwm_set_irq_enabled(BRAKE_SLICE, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // brake direction
    gpio_init(BRAKE_DIR_GPIO);
    gpio_set_dir(BRAKE_DIR_GPIO, GPIO_OUT);
    gpio_put(BRAKE_DIR_GPIO, 0);

    // enable pwm
    pwm_set_mask_enabled((1u << BRAKE_SLICE) | (1u << STEER_SLICE));

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

    percent = (percent > 0 ? percent : -percent);
    float CLKDIV = STEER_CLKDIV_MAX - (STEER_CLKDIV_MAX - STEER_CLKDIV_MIN) * percent;
    float level = (percent > 0.08) ? 0.5 * WRAPVAL : 0; 

    pwm_set_clkdiv(STEER_SLICE, CLKDIV);
    pwm_set_chan_level(STEER_SLICE, PWM_CHAN_B, level);  
}

void writePercentBrake(float value) {
    float percent = constrain(value, -1, 1);

    int direction = (percent < 0); // set direction pin to 0 for positive / clockwise
    gpio_put(BRAKE_DIR_GPIO, direction);

    percent = (percent > 0 ? percent : -percent);
    float CLKDIV = BRAKE_CLKDIV_MAX - (BRAKE_CLKDIV_MAX - BRAKE_CLKDIV_MIN) * percent;
    float level = (percent > 0.08) ? 0.5 * WRAPVAL : 0; 

    pwm_set_clkdiv(BRAKE_SLICE, CLKDIV);
    pwm_set_chan_level(BRAKE_SLICE, PWM_CHAN_A, level);  
}

float calculate(float kP, int setpoint, int position) {
    if (abs(setpoint - position) > 50)
      return kP * (position - setpoint);
    else
      return 0;
}

char buffer[64];
int curr_index = 0;

bool serial_communication(float *steer, float *brake, float *throttle) {
    if (uart_is_readable(UART_ID)) {
        char curr_char = uart_getc(uart0);

        int searches = 0; // Limit checking 30 to not block
        while (curr_char != '(') { // Wait for '('

            // printf("\nCURR: %c\n", curr_char);

            if (uart_is_readable(UART_ID)) {
                curr_char = uart_getc(uart0);
            } else {
                return false;
            }

            if (searches > 30) {
                return false;
            }

            searches++;
        }

        // printf("FOUND (");

        while (curr_char != ')' && curr_index < 64) { // Wait for end, limit searching
            if (uart_is_readable(UART_ID)) {
                curr_char = uart_getc(uart0);
                // printf("%c", curr_char);
                buffer[curr_index] = curr_char; // Update buffer
                curr_index++;
            } else {
                return false;
            }
        }

        buffer[curr_index - 1] = '\0'; // Null-terminate, overwrite ')'

        if (curr_index >= 63) { // Didn't actually find a ')'
            memset(buffer, 0, sizeof(buffer));
            curr_index = 0;
            return false;
        } 

        // printf("\nSCANNING");

        float steer_, brake_, throttle_ = 0;

        if (sscanf(buffer, "%f,%f,%f", &steer_, &brake_, &throttle_) != 3) {
            return false;
        }

        // printf("SCANNED SUCCESSFULLY: %f, %f, %f", steer_, brake_, throttle_);

        *steer = steer_;
        *brake = brake_;
        *throttle = throttle_;

        memset(buffer, 0, sizeof(buffer));
        curr_index = 0;
        return true;

    } else {
        return false;
    }
  }

int main() {
    initialize(); 
    int iters = 0;

    float steer = 0;
    float brake = 0;
    float throttle = 0;
    
    while (true) {
        char meow[64];
        // serial_communication();

        int steer_setpoint = 1500;
        float steer_output = calculate(0.0003, 1500, steer_position);
        writePercentSteer(steer_output);

        if (iters > 1000) {
            // printf("Steer Position: %d\tSteer Setpoint: %d\tSteer Output: %f\n", steer_position, steer_setpoint, steer_output);
            bool read_successfully = serial_communication(&steer, &brake, &throttle);
            
            // if (read_successfully) {
                printf("New Steer: %f, Brake: %f, Throttle: %f\n", steer, brake, throttle);
            // }


            // sprintf(meow, "Received Steering Angle: %.4f\tReceived Throttle: %.4f\n", received_steering_angle, received_throttle);
            // uart_puts(UART_ID, meow);
            iters = 0;
        } else {
            iters += 1;
        }

        // writePercentSteer(0.5);
        // writePercentBrake(0.5);
    }
}