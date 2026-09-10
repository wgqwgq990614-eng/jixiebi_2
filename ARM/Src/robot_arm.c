#include "global.h"
#include "Kinematics.h"
#include "robot_arm.h"
#include "stdlib.h"
#include "math.h"
#include "tim.h"
KinematicsObjectTypeDef  kinematics;
RobotArmHandleTypeDef robot_arm;	


static void theta2servo(KinematicsObjectTypeDef* self, float time)
{
	float target_angle[4] = {0};

			target_angle[0] = 90.0f + self->knot[0].theta;
			target_angle[1] = 180.0f - self->knot[1].theta;
			target_angle[2] = 90.0f + self->knot[2].theta;
			target_angle[3] = 90.0f + self->knot[3].theta;
			
			for (uint8_t i = 0; i < 4; i++)
			{	
				pwm_servo_angle_set(&pwm_servos[i], target_angle[i], time);
			}

}
uint8_t robot_arm_angle_set(float target_x,
                                 float target_y,
                                 float target_z,
                                 float pitch,       
                                 float min_z,      
                                 float max_z,        
                                 uint32_t time)
{
    bool result1_state, result2_state;

    KinematicsObjectTypeDef kinematics_result1;
    KinematicsObjectTypeDef kinematics_result2;
    VectorObjectTypeDef vector;

    vector.x = target_x;
    vector.y = target_y;
    vector.z = target_z;

    result1_state = set_z_range(&kinematics_result1, &vector,  pitch, min_z);
    result2_state = set_z_range(&kinematics_result2,&vector, pitch, max_z);

    if (result1_state)
    {
        kinematics.alpha    = kinematics_result1.alpha;
        kinematics.vector.x = kinematics_result1.vector.x;
        kinematics.vector.y = kinematics_result1.vector.y;
        kinematics.vector.z = kinematics_result1.vector.z;
        for (uint8_t i = 0; i < 4; i++)
        {
            kinematics.knot[i].theta = kinematics_result1.knot[i].theta;
            kinematics.knot[i].rad   = kinematics_result1.knot[i].rad;
        }

        if (result2_state)
        {
            if (fabsf(kinematics_result2.vector.z - target_z) <
                fabsf(kinematics_result1.vector.z - target_z))
            {
                kinematics.alpha    = kinematics_result2.alpha;
                kinematics.vector.x = kinematics_result2.vector.x;
                kinematics.vector.y = kinematics_result2.vector.y;
                kinematics.vector.z = kinematics_result2.vector.z;
                for (uint8_t i = 0; i < 4; i++)
                {
                    kinematics.knot[i].theta = kinematics_result2.knot[i].theta;
                    kinematics.knot[i].rad   = kinematics_result2.knot[i].rad;
                }
            }
        }
    }
    else
    {
        if (result2_state)
        {
            kinematics.alpha    = kinematics_result2.alpha;
            kinematics.vector.x = kinematics_result2.vector.x;
            kinematics.vector.y = kinematics_result2.vector.y;
            kinematics.vector.z = kinematics_result2.vector.z;
            for (uint8_t i = 0; i < 4; i++)
            {
                kinematics.knot[i].theta = kinematics_result2.knot[i].theta;
                kinematics.knot[i].rad   = kinematics_result2.knot[i].rad;
            }
        }
        else
        {
            return false;
        }
    }
    result1_state = 0;
	  result2_state = 0;
    theta2servo(&kinematics, time);

    return true;
}


uint8_t robot_arm_coordinate_set(float target_x,
								 float target_y,
								 float target_z,
								 float pitch,
								 float min_pitch,
								 float max_pitch,
								 uint32_t time)
{
	
	bool result1_state, result2_state;
	
	KinematicsObjectTypeDef kinematics_result1;
	KinematicsObjectTypeDef kinematics_result2;	
	VectorObjectTypeDef vector;

	vector.x = target_x;
	vector.y = target_y;
	vector.z = target_z;
	
	result1_state = set_pitch_range(&kinematics_result1, &vector, pitch, min_pitch);
	result2_state = set_pitch_range(&kinematics_result2, &vector, pitch, max_pitch);
	
	
	if (result1_state)
	{
		kinematics.alpha = kinematics_result1.alpha;
		kinematics.vector.x = kinematics_result1.vector.x;
		kinematics.vector.y = kinematics_result1.vector.y;
		kinematics.vector.z = kinematics_result1.vector.z;
		for (uint8_t i = 0; i< 4; i++)
		{
			kinematics.knot[i].theta = kinematics_result1.knot[i].theta;
		}
		
		if (result2_state)
		{
			if (fabs(kinematics_result2.alpha - pitch) < fabs(kinematics_result1.alpha - pitch))
			{
				kinematics.alpha = kinematics_result2.alpha;
				kinematics.vector.x = kinematics_result2.vector.x;
				kinematics.vector.y = kinematics_result2.vector.y;
				kinematics.vector.z = kinematics_result2.vector.z;
				for (uint8_t i = 0; i< 4; i++)
				{
					kinematics.knot[i].theta = kinematics_result2.knot[i].theta;
				}			
			}
		}
	}
	else
	{
		if (result2_state)
		{
			kinematics.alpha = kinematics_result2.alpha;
			kinematics.vector.x = kinematics_result2.vector.x;
			kinematics.vector.y = kinematics_result2.vector.y;
			kinematics.vector.z = kinematics_result2.vector.z;
			for (uint8_t i = 0; i< 4; i++)
			{
				kinematics.knot[i].theta = kinematics_result2.knot[i].theta;
			}
		}
		else
		{
			return false;
		}
	}
	result1_state = 0;
	result2_state = 0;
	theta2servo(&kinematics, time);

	return true;
}

void robot_arm_reset(uint32_t time)
{
			pwm_servo_duty_set(&pwm_servos[0], PWM_SERVO6_RESET_DUTY, time);
			pwm_servo_duty_set(&pwm_servos[1], PWM_SERVO5_RESET_DUTY, time);
			pwm_servo_duty_set(&pwm_servos[2], PWM_SERVO4_RESET_DUTY, time);
			pwm_servo_duty_set(&pwm_servos[3], PWM_SERVO3_RESET_DUTY, time);
			pwm_servo_duty_set(&pwm_servos[4], PWM_SERVO2_RESET_DUTY, time);
			pwm_servo_duty_set(&pwm_servos[5], PWM_SERVO1_RESET_DUTY, time);
}

bool robot_arm_init(void)
{
	pwm_servos_init();
	
	kinematics_init(&kinematics);

	HAL_Delay(200);
	
	return true;
}
/*
void robot_arm_claw(*/