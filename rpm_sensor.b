#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#define LEFT_SENSOR_PIN 10
#define RIGHT_SENSOR_PIN 9
#define M_PER_TICK 0.01           // TODO: Adjust for actual wheel velocity
#define VELOCITY_TIMEOUT_US 50000 // 50ms timeout to reset velocity
#define FILTER_SIZE 5             // Number of samples for rolling average

volatile uint32_t last_left_time = 0;
volatile uint32_t last_right_time = 0;
volatile float left_velocity = 0.0;
volatile float right_velocity = 0.0;

volatile int light_on = 0;

float left_velocity_buffer[FILTER_SIZE] = {0};
float right_velocity_buffer[FILTER_SIZE] = {0};
int left_index = 0, right_index = 0;

float rolling_average(float *buffer, int size)
{
    float sum = 0.0;
    for (int i = 0; i < size; i++)
    {
        sum += buffer[i];
    }
    return sum / size;
}

void left_wheel_isr(uint gpio, uint32_t events)
{
    gpio_put(25, 1);

    uint32_t current_time = time_us_32();
    if (last_left_time != 0)
    {
        uint32_t delta_time = current_time - last_left_time;
        float new_velocity = M_PER_TICK / (delta_time / 1e6); // Convert to meters per second
        left_velocity_buffer[left_index] = new_velocity;
        left_index = (left_index + 1) % FILTER_SIZE;

        float avg = rolling_average(left_velocity_buffer, FILTER_SIZE);
        if (avg < left_velocity + 1)
        {
            left_velocity = avg;
        }
    }
    last_left_time = current_time;
}

void right_wheel_isr(uint gpio, uint32_t events)
{
    gpio_put(25, 1);

    uint32_t current_time = time_us_32();
    if (last_right_time != 0)
    {
        uint32_t delta_time = current_time - last_right_time;
        float new_velocity = M_PER_TICK / (delta_time / 1e6); // Convert to meters per second
        right_velocity_buffer[right_index] = new_velocity;
        right_index = (right_index + 1) % FILTER_SIZE;

        float avg = rolling_average(right_velocity_buffer, FILTER_SIZE);
        if (avg < right_velocity + 1)
        {
            right_velocity = avg;
        }
    }
    last_right_time = current_time;
}

int main()
{
    stdio_init_all();

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);

    // Configure GPIO for sensors
    gpio_init(LEFT_SENSOR_PIN);
    gpio_set_dir(LEFT_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(LEFT_SENSOR_PIN);
    gpio_set_irq_enabled_with_callback(LEFT_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, &left_wheel_isr);

    gpio_init(RIGHT_SENSOR_PIN);
    gpio_set_dir(RIGHT_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_SENSOR_PIN);
    gpio_set_irq_enabled_with_callback(RIGHT_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, &right_wheel_isr);

    while (1)
    {
        uint32_t current_time = time_us_32();

        // Check for timeout on left wheel
        if ((current_time - last_left_time) > VELOCITY_TIMEOUT_US)
        {
            left_velocity = 0.0;
        }

        // Check for timeout on right wheel
        if ((current_time - last_right_time) > VELOCITY_TIMEOUT_US)
        {
            right_velocity = 0.0;
        }

        printf("Left Velocity: %.2f m/s, Right Velocity: %.2f m/s\n", left_velocity, right_velocity);
    }
}
