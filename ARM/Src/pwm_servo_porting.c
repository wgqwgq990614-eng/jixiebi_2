#include "tim.h"
#include "pwm_servos.h"
#include "global.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

PWMServoHandleTypeDef pwm_servos[USE_SERVO_NUM];

static void pwm_servo_object_init(PWMServoHandleTypeDef* handle, uint8_t id)
{
	memset(handle, 0, sizeof(PWMServoHandleTypeDef));
	handle->id = id;
	switch(id)
	{
		case 1:
			handle->current_duty = PWM_SERVO1_RESET_DUTY;
			handle->write_duty = PWM_SERVO1_RESET_DUTY;
			break;
		
		case 2:
			handle->current_duty = PWM_SERVO2_RESET_DUTY;
			handle->write_duty = PWM_SERVO2_RESET_DUTY;
			break;
		
		case 3:
			handle->current_duty = PWM_SERVO3_RESET_DUTY;
			handle->write_duty = PWM_SERVO3_RESET_DUTY;
			break;
		
		case 4:
			handle->current_duty = PWM_SERVO4_RESET_DUTY;
			handle->write_duty = PWM_SERVO4_RESET_DUTY;
			break;
		
		case 5:
			handle->current_duty = PWM_SERVO5_RESET_DUTY;
			handle->write_duty = PWM_SERVO5_RESET_DUTY;
			break;
		
		case 6:
			handle->current_duty = PWM_SERVO6_RESET_DUTY;
			handle->write_duty = PWM_SERVO6_RESET_DUTY;
			break;
	}
}
void pwm_servos_init(void)
{
     for (uint8_t i = 0; i < USE_SERVO_NUM; i++) {
        pwm_servo_object_init(&pwm_servos[i], i + 1); // id = 1..6
    }

    pwm_servos[5].htim    = &htim1;
    pwm_servos[5].channel = TIM_CHANNEL_1;

    pwm_servos[4].htim    = &htim2;
    pwm_servos[4].channel = TIM_CHANNEL_1;

    pwm_servos[3].htim    = &htim3;
    pwm_servos[3].channel = TIM_CHANNEL_1;

    pwm_servos[2].htim    = &htim4;
    pwm_servos[2].channel = TIM_CHANNEL_1;

    pwm_servos[1].htim    = &htim5;
    pwm_servos[1].channel = TIM_CHANNEL_2;

    pwm_servos[0].htim    = &htim8;
    pwm_servos[0].channel = TIM_CHANNEL_1;

    for (uint8_t i = 0; i < USE_SERVO_NUM; i++)
	{
        HAL_TIM_PWM_Stop(pwm_servos[i].htim, pwm_servos[i].channel);
		HAL_TIM_PWM_Start(pwm_servos[i].htim, pwm_servos[i].channel);
    }

}

void pwm_servos_handler()
{
	for(uint8_t i = 0; i < USE_SERVO_NUM; i++)
	{
		pwm_servo_handler(&pwm_servos[i]);
	}
}
