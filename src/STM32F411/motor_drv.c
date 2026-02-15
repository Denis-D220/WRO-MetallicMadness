#include "motor_drv.h"
#include "stm32f4xx_hal.h"

static TIM_HandleTypeDef *motor_pwm_tim;
static uint32_t motor_pwm_channel;
static GPIO_TypeDef *motor_dir_port;
static uint16_t motor_dir_pin;

static volatile int32_t encoder_count = 0;
static volatile uint32_t last_encoder_time = 0;
static volatile uint32_t last_encoder_count = 0;

#define PULSES_PER_REV 1595



void motor_init(TIM_HandleTypeDef *pwm_tim, uint32_t pwm_channel, GPIO_TypeDef *dir_port, uint16_t dir_pin)
{
    motor_pwm_tim = pwm_tim;
    motor_pwm_channel = pwm_channel;
    motor_dir_port = dir_port;
    motor_dir_pin = dir_pin;

    HAL_TIM_PWM_Start(motor_pwm_tim, motor_pwm_channel);
    motor_set_speed(0);
}

void motor_set_speed(uint8_t speed_percent)
{
    if (speed_percent > 100) speed_percent = 100;

    if (motor_pwm_tim == NULL) {
        printf("motor_pwm_tim is NULL!\n");
        Error_Handler();
    }

    printf("Timer Period: %lu\n", motor_pwm_tim->Init.Period);
    // Calculate the pulse value based on the speed percentage
    uint32_t pulse = (motor_pwm_tim->Init.Period + 1) * speed_percent / 100;

    // Ensure the pulse value is within the valid range
    if (pulse > motor_pwm_tim->Init.Period) {
        pulse = motor_pwm_tim->Init.Period;
    }

    // Set the compare value for the PWM signal
    __HAL_TIM_SET_COMPARE(motor_pwm_tim, motor_pwm_channel, pulse);
}

void motor_forward(uint8_t speed_percent)
{
    HAL_GPIO_WritePin(motor_dir_port, motor_dir_pin, GPIO_PIN_SET);
    motor_set_speed(speed_percent); // or any desired speed
}

void motor_reverse(uint8_t speed_percent)
{
    HAL_GPIO_WritePin(motor_dir_port, motor_dir_pin, GPIO_PIN_RESET);
    motor_set_speed(speed_percent); // or any desired speed
}



void motor_forward_degrees(uint16_t degrees, uint8_t speed_percent)
{
	printf("[DEBUG] motor_forward_degrees called\n");
	printf("[DEBUG] Degrees: %u\n", degrees);
    encoder_count = 0;
    int32_t target_ticks = ((degrees * PULSES_PER_REV) + 359) / 360;
    printf("[DEBUG] Target Ticks: %ld\n", target_ticks);

    motor_set_speed(speed_percent);
    HAL_GPIO_WritePin(motor_dir_port, motor_dir_pin, GPIO_PIN_SET);

    printf("before look\n");

    while (encoder_count < target_ticks) {
        printf("ENC: now=%ld, target=%ld\n", encoder_count, target_ticks);

    }

    motor_stop();
    printf("[DEBUG] motor_stop() called. Final encoder count: %ld\r\n", encoder_count);
}


void motor_reverse_degrees(uint16_t degrees, uint8_t speed_percent)
{
    printf("[DEBUG] motor_reverse_degrees called\n");
    printf("[DEBUG] Degrees: %u\n", degrees);

    encoder_count = 0;
    int32_t target_ticks = ((degrees * PULSES_PER_REV) + 359) / 360;
    printf("[DEBUG] Target Ticks: %ld\n", target_ticks);

    motor_set_speed(speed_percent);
    HAL_GPIO_WritePin(motor_dir_port, motor_dir_pin, GPIO_PIN_RESET);

    printf("before look\n");

    while (-encoder_count < target_ticks) {
        printf("ENC: now=%ld, target=%ld\n", encoder_count, target_ticks);
    }

    motor_stop();
    printf("[DEBUG] motor_stop() called. Final encoder count: %ld\r\n", encoder_count);
}



void motor_stop(void)
{
    motor_set_speed(0);
}

void motor_encoder_update(uint8_t ch_a_state, uint8_t ch_b_state)
{
    static uint8_t last_state = 0;
    uint8_t current_state = (ch_a_state << 1) | ch_b_state;

    uint8_t transition = (last_state << 2) | current_state;
    last_state = current_state;

    // Standard quadrature decoding transitions
    switch (transition)
    {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            encoder_count++;  // Forward
            break;
        case 0b0010:
        case 0b0100:
        case 0b1101:
        case 0b1011:
            encoder_count--;  // Reverse
            break;
        default:
            // Invalid transition or no movement
            break;
    }

    // Optional for debug
//     printf("[ENC UPDATE] Count = %ld\n", encoder_count);
}


uint32_t motor_get_rpm(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - last_encoder_time;
    if (elapsed == 0) return 0;

    int32_t delta = encoder_count - last_encoder_count;
    last_encoder_count = encoder_count;
    last_encoder_time = now;

    // Assume 11 pulses per revolution (PPR from Hall encoder doc)
    return (delta * 60000) / (11 * elapsed);  // RPM = (delta * 60s * 1000ms/s) / (PPR * time_ms)
}
