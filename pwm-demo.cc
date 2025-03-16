/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/divider.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

#include "hardware/i2c.h"

#include "mpu6050.h"

#include "pt_cornell_rp2040_v1_3.h"

// Midpoint of potentiometer
#define WRAPVAL 25000
#define CLKDIV 5.0f

// I/O Pins
#define PWM_OUTPUT_PIN 5
#define POT_INPUT_PIN 31

fix15 divfixn(fix15 a, fix15 b)
{
    return (fix15)(div_s64s64((((signed long long)(a)) << 16), ((signed long long)(b))));
}

// Variable to hold PWM slice number
uint slice_num;

// PWM duty cycle
const fix15 p = int2fix15(1000);
const fix15 i = int2fix15(5);
const fix15 d = 0;

volatile int control;
fix15 last_error;
fix15 integral = 0;

uint32_t last_time = 0;

// IMU
// Arrays in which raw measurements will be stored
// When the fan is at 0 degrees to the table, Z is up, Y is toward the table
fix15 acceleration[3], gyro[3];
fix15 accel_angle = 0;

fix15 get_accelerometer_angle()
{
    return float2fix15(atan(fix2float15(divfixn(acceleration[1], acceleration[2]))));
}

void on_pwm_wrap()
{

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    if (control < 0)
    {
        control = 0;
    }
    pwm_set_chan_level(slice_num, PWM_CHAN_B, control);
}

float rad_to_deg(float angle)
{
    return angle * 180.0 / 3.14159653585;
}

float deg_to_rad(float angle)
{
    return angle * 3.14159653585 / 180;
}

static PT_THREAD(protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    static int test_in;
    while (1)
    {
        // Read potentiometer
        // Scale 0-4095 to 0-25000
        // control = adc_read() * 6.105006;

        mpu6050_read_raw(acceleration, gyro);

        accel_angle = get_accelerometer_angle();

        // printf("Fix: %f \n", rad_to_deg(accel_angle));

        // printf("Gyro: %f, %f, %f \n", fix2float15(gyro[0]), fix2float15(gyro[1]), fix2float15(gyro[2]));

        fix15 setpoint = float2fix15(deg_to_rad(-40));
        fix15 error = setpoint - accel_angle;

        fix15 derivative = 0;
        if (last_time != 0)
        {
            derivative = divfixn(error - last_error, int2fix15(time_us_32() - last_time));
            last_error = error;
        }
        last_time = time_us_32();

        integral += multfix15(error, int2fix15(time_us_32() - last_time));

        control = fix2int15(multfix15(p, error)) + fix2int15(multfix15(i, integral));

        printf("%d, %f \n", control, rad_to_deg(fix2float15(error)));
    }
    PT_END(pt);
}

int main()
{
    stdio_init_all();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO 5 that it is allocated to the PWM
    gpio_set_function(PWM_OUTPUT_PIN, GPIO_FUNC_PWM);

    // // Find out which PWM slice is connected to GPIO 5 (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(PWM_OUTPUT_PIN);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL);
    pwm_set_clkdiv(slice_num, CLKDIV);

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 3125);

    pwm_set_output_polarity(slice_num, true, true);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    /////////////////////////////// ADC SETUP //////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    adc_init();
    adc_gpio_init(POT_INPUT_PIN);
    adc_select_input(0);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // Pullup resistors on breakout board, don't need to turn on internals
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    pt_add_thread(protothread_serial);
    pt_schedule_start;
}
