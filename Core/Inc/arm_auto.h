#ifndef __VISION_ARM_CTRL_H__
#define __VISION_ARM_CTRL_H__

#include "stdint.h"
#include "stdbool.h"

/* 机械臂当前的“极坐标状态”，全局变量 */
typedef struct
{
    float len;      // 水平长度（len）
    float yaw;      // 底座角度 θ1，单位：deg
    float z;        // 当前高度 z
    float pitch;    // 当前俯仰角 α，单位：deg
} ArmMotionStateTypeDef;

float deg2rad(float deg);

void move_with_fix_pitch_z_range(uint32_t time);

 void move_with_fix_z_pitch_range(void);

void sweep_once(void);

void track_once(uint8_t dir_lr, uint8_t dir_ud);
	
void handle_cmd(uint8_t cmd, uint8_t dir_lr, uint8_t dir_ud, uint8_t color);

void VisionArm_ScanTask(void);

void VisionArm_Cmd2_Task(void);

/* 全局运动参数（你后面要用的“记住的角度和长度变量”就是这里） */
extern ArmMotionStateTypeDef gArmMotionState;

/* 初始化：设置初始 len/yaw/z/pitch */
void VisionArm_Init(void);

/* 串口每收到一个字节就喂进来（USART2 Rx 回调里调用） */
void VisionArm_OnUartByte(uint8_t data);

uint8_t VisionArm_IsUart2Locked(void);

void VisionArm_LockUart2(void);

void VisionArm_UnlockUart2(void);

#endif /* __VISION_ARM_CTRL_H__ */
