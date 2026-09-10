#include "arm_auto.h"
#include "global.h"
#include "Kinematics.h"
#include "robot_arm.h"
#include "stdlib.h"
#include "math.h"
/*==================== 1. 运动参数全局变量 ====================*/

/* 初始姿态可以按你机械臂实际情况改 */
ArmMotionStateTypeDef gArmMotionState =
{
    .len   = 20.0f,   // 初始水平距离
    .yaw   =   0.0f,   // 初始底座角度
    .z     =  15.0f,   // 初始高度
    .pitch = 0.0f,   // 初始俯仰角（例如爪子向下）
};

/* 扫动范围 / 步长 / 时间 等参数 */
#define ARM_YAW_STEP        2.0f      // 左右每步 1 度
#define ARM_LEN_STEP        0.1f      // 上下每步 1mm
#define ARM_MOVE_TIME       50u       // 每一步动作 50ms
#define ARM_SWEEP_YAW_MIN  (-90.0f)   
#define ARM_SWEEP_YAW_MAX   (90.0f)
#define ARM_RING_LEN_STEP   2.0f    // 每扫完一圈，length + 2mm
#define ARM_LEN_MIN         0.0f
#define ARM_LEN_MAX        30.0f
#define FINAL_Z            (-1.5f)    
#define PLACE_LEFT_YAW    (-80.0f)  
#define PLACE_RIGHT_YAW    (80.0f)  
#define CMD2_WAIT_MS      2000u
#define LIFT_Z            5.0f      // 抬头后的 Z 目标高度
#define PLACE_LEFT_YAW   (-80.0f)
#define PLACE_RIGHT_YAW   (80.0f)
#define SCAN_STEP_TIME_MS   180u   // 每一步运动时间，下面会解释为什么选 180ms

static uint8_t scan_enable = 0;    // 1 = 开始持续扫描，0 = 停止
static uint8_t scan_step_busy = 0; // 当前这一步是否还在走
static uint32_t scan_step_tick = 0;

typedef enum
{
    CMD2_STATE_IDLE = 0,
    CMD2_STATE_MOVE_DOWN,
    CMD2_STATE_WAIT_AFTER_DOWN,
    CMD2_STATE_MOVE_UP,          
    CMD2_STATE_WAIT_AFTER_UP,   
    CMD2_STATE_MOVE_PLACE,
    CMD2_STATE_WAIT_AFTER_PLACE,
	  CMD2_STATE_MOVE_HOME,         
    CMD2_STATE_WAIT_AFTER_HOME    
} Cmd2StateTypeDef;


static Cmd2StateTypeDef cmd2_state = CMD2_STATE_IDLE;
static uint32_t cmd2_timestamp = 0;
static uint8_t  cmd2_color     = 0;  
 int8_t sweep_dir = 1;          

 uint8_t rx_state   = 0;      
 uint8_t rx_counter = 0;
 uint8_t rx_buf[8]  = {0};

 uint8_t place_request_flag = 0;   
 uint8_t place_color        = 0;   
 uint8_t place_step         = 0;   
 uint32_t place_timestamp   = 0;   

uint8_t stop_confirm_flag = 0;  
uint8_t flag=1;

 float deg2rad(float deg)
{
    return deg * 3.1415926f / 180.0f;
}


 void move_with_fix_pitch_z_range(uint32_t time)
{
    float x = gArmMotionState.len * cosf(deg2rad(gArmMotionState.yaw));
    float y = gArmMotionState.len * sinf(deg2rad(gArmMotionState.yaw));
    robot_arm_angle_set(x, y, gArmMotionState.z,
                             0,
                             -10.0f,
                             40.0f,
                             time);
}

 void move_with_fix_z_pitch_range(void)
{
    float x = gArmMotionState.len * cosf(deg2rad(gArmMotionState.yaw));
    float y = gArmMotionState.len * sinf(deg2rad(gArmMotionState.yaw));
    float z = gArmMotionState.z;

    float pitch0    = -50.0f;

    robot_arm_coordinate_set(x, y, z,
                             pitch0,
                             -90.0f,
                             90.0f,
                             1000);
 }

/* 左右扫动一步：每调一次转 1 度，到头换方向 */
void sweep_once(void)
{
gArmMotionState.yaw += sweep_dir * ARM_YAW_STEP;

    // 2. 到达右端：钳在最大角，并把方向改为向左
    if (gArmMotionState.yaw >= ARM_SWEEP_YAW_MAX)
    {
        gArmMotionState.yaw = ARM_SWEEP_YAW_MAX;
        sweep_dir = -1;   // 改成往左扫
    }
    // 3. 到达左端：钳在最小角，方向改为向右，并把 length 加 2
    else if (gArmMotionState.yaw <= ARM_SWEEP_YAW_MIN)
    {
        gArmMotionState.yaw = ARM_SWEEP_YAW_MIN;
        sweep_dir = +1;   // 改成往右扫

        // —— 此时认为“从左扫到右、再从右扫回左”这一圈完成 —— //
        gArmMotionState.len += ARM_RING_LEN_STEP;
        if (gArmMotionState.len > ARM_LEN_MAX)
            gArmMotionState.len = ARM_LEN_MAX;   // 做个上限保护
    }

    // 4. 用“俯仰角固定、Z 允许偏差”的函数移动到新的 (len, yaw, z, pitch)
    move_with_fix_pitch_z_range( SCAN_STEP_TIME_MS);
}

/* 上下左右纠偏*/
 void track_once(uint8_t dir_lr, uint8_t dir_ud)
{
    /* 第二位：0 偏左 → 底座向左转（角度增大）；1 偏右 → 底座向右转 */
    if (dir_lr == 1)
        gArmMotionState.yaw -= ARM_YAW_STEP;
    else if (dir_lr == 2)
        gArmMotionState.yaw += ARM_YAW_STEP;

    /* 第三位：0 偏上；1 偏下*/
    if (dir_ud == 1)
        gArmMotionState.len += ARM_LEN_STEP;  // 目标在上 → 伸短一点
    else if (dir_ud == 2)
        gArmMotionState.len -= ARM_LEN_STEP;  // 目标在下 → 伸长一点

    move_with_fix_pitch_z_range(50);
}

/*==================== 4. 处理 cmd=0/1/2 的核心函数 ====================*/
void handle_cmd(uint8_t cmd, uint8_t dir_lr, uint8_t dir_ud, uint8_t color)
{
    switch (cmd)
    {
    case 0:   // 扫描
			  stop_confirm_flag   = 0;
        place_request_flag  = 0;
		if(flag){
        scan_enable = 1;
        scan_step_busy = 0; }
        break;

    case 1:   // 纠偏
        stop_confirm_flag   = 0;
        place_request_flag  = 0;
		scan_enable = 0;
		flag=0;
        track_once(dir_lr, dir_ud);
        break;

       case 2:
    if (stop_confirm_flag == 0)
    {
        // 第一次 2：确认稳定
        stop_confirm_flag = 1;
    }
    else
    {
        // 第二次 2：启动状态机，让主循环慢慢跑动作
        stop_confirm_flag = 0;
        cmd2_color = color;     
        cmd2_state = CMD2_STATE_MOVE_DOWN;
    }
    break;

    default:
        stop_confirm_flag   = 0;
        place_request_flag  = 0;
        break;
    }
}
void VisionArm_ScanTask(void)
{
    if (!scan_enable || cmd2_state != CMD2_STATE_IDLE)
    {
        scan_step_busy = 0;
        return;
    }

    uint32_t now = HAL_GetTick();

    if (!scan_step_busy)
    {
        // 发起新的一步扫描
        sweep_once();  // 每次走一小步，时间 180ms
        scan_step_busy = 1;
        scan_step_tick = now;
    }
    else
    {
        // 等待这一小步走完，时间到了再允许下一步
        if (now - scan_step_tick >= SCAN_STEP_TIME_MS)
        {
            scan_step_busy = 0;  // 准备进入下一步
        }
    }
}

void VisionArm_Cmd2_Task(void)
{
    switch (cmd2_state)
    {
    case CMD2_STATE_IDLE:
        // 空闲，啥也不做
        break;

    case CMD2_STATE_MOVE_DOWN:
        /* 第一步：下压到 FINAL_Z（保持当前 yaw / len） */
        gArmMotionState.z = FINAL_Z;
		    gArmMotionState.len-=8.0f;
		    move_with_fix_z_pitch_range();          // 固定 Z，俯仰角允许微调
        cmd2_timestamp = HAL_GetTick();
        cmd2_state = CMD2_STATE_WAIT_AFTER_DOWN;
        break;

    case CMD2_STATE_WAIT_AFTER_DOWN:
        /* 下压完成后等 2 秒，然后合爪子，进入抬头 */
        if (HAL_GetTick() - cmd2_timestamp >= CMD2_WAIT_MS)
        { 
	          pwm_servo_angle_set(&pwm_servos[5], 60.0f, 200);   

            cmd2_timestamp = HAL_GetTick();
            cmd2_state = CMD2_STATE_MOVE_UP;
        }
        break;

    case CMD2_STATE_MOVE_UP:
			 if (HAL_GetTick() - cmd2_timestamp >= CMD2_WAIT_MS)
        { 
        gArmMotionState.z = LIFT_Z;
        move_with_fix_z_pitch_range();          
				cmd2_timestamp = HAL_GetTick();
        cmd2_state = CMD2_STATE_WAIT_AFTER_UP;
				}
        break;

    case CMD2_STATE_WAIT_AFTER_UP:
        /* 抬头完成后再等 2 秒，然后去左右 80° 放置 */
        if (HAL_GetTick() - cmd2_timestamp >= CMD2_WAIT_MS)
        {
            cmd2_state = CMD2_STATE_MOVE_PLACE;
        }
        break;

    case CMD2_STATE_MOVE_PLACE:
        /* 第三步：根据颜色转到左右 80° 放置位置 */
        if (cmd2_color == 1)
        {
            gArmMotionState.yaw = PLACE_RIGHT_YAW;   // 颜色1 → 向右
        }
        else if(cmd2_color == 2)
        {
            gArmMotionState.yaw = PLACE_LEFT_YAW;    // 颜色0/2 → 向左
        }
				else if(cmd2_color == 2){
					 gArmMotionState.yaw =0.0f;
				}

        move_with_fix_z_pitch_range();
        cmd2_timestamp = HAL_GetTick();
        cmd2_state = CMD2_STATE_WAIT_AFTER_PLACE;
        break;

    case CMD2_STATE_WAIT_AFTER_PLACE:
        /* 放置完成后再等 2 秒，松爪，结束 */
        if (HAL_GetTick() - cmd2_timestamp >= CMD2_WAIT_MS)
        {
         pwm_servo_angle_set(&pwm_servos[5], 150.0f, 200);
       cmd2_timestamp = HAL_GetTick();
            cmd2_state = CMD2_STATE_MOVE_HOME; 
			   }
        break;
   case CMD2_STATE_MOVE_HOME:
	 if (HAL_GetTick() - cmd2_timestamp >= CMD2_WAIT_MS)
        {
					VisionArm_Init();
					 cmd2_timestamp = HAL_GetTick();
					cmd2_state = CMD2_STATE_WAIT_AFTER_HOME;
        break;
				}
				case CMD2_STATE_WAIT_AFTER_HOME:
        if (HAL_GetTick() - cmd2_timestamp >= CMD2_WAIT_MS)
        {
					  flag=0;
					 VisionArm_Init();
            cmd2_state = CMD2_STATE_IDLE;
        }
        break;
    default:
        cmd2_state = CMD2_STATE_IDLE;
        break;
    }
}



/*==================== 5. 对外接口：初始化 + 串口接收 ====================*/


/* 串口每收到一个字节就调这个函数（USART2） */
void VisionArm_OnUartByte(uint8_t data)
{
    switch (rx_state)
    {
    case 0: // 等帧头1: 0x2C
        if (data == 0x2C)
        {
            rx_state   = 1;
            rx_counter = 0;
            rx_buf[rx_counter++] = data;
        }
        break;

    case 1: // 等帧头2: 0x12
        if (data == 0x12)
        {
            rx_state = 2;
            rx_buf[rx_counter++] = data;
        }
        else
        {
            rx_state   = 0;
            rx_counter = 0;
        }
        break;

case 2: // 收 cmd + dir_lr + dir_ud + color + 结束符
    rx_buf[rx_counter++] = data;
    if (rx_counter >= 7)  // 0:2C 1:12 2:cmd 3:dir_lr 4:dir_ud 5:color 6:end
    {
        if (rx_buf[6] == 0x5B)
        {
					  uint8_t cmd    = rx_buf[2];
            uint8_t dir_lr = rx_buf[3];
            uint8_t dir_ud = rx_buf[4];
            uint8_t color  = rx_buf[5];  
					
            handle_cmd(cmd, dir_lr, dir_ud, color);
        }

        rx_state   = 0;
        rx_counter = 0;
    }
    break;


    default:
        rx_state   = 0;
        rx_counter = 0;
        break;
    }
}
void VisionArm_Init(void)
{
    /* 1. 先让它动一次——完全照你在 main 里测试那句写 */

    /* 2. 再把状态变量同步成这一姿态，后面扫动/纠偏用这个 */
    gArmMotionState.len   =30.0f;
    gArmMotionState.yaw   =   8.0f;
    gArmMotionState.z     =  15.0f;
    gArmMotionState.pitch = 0.0f;
	move_with_fix_pitch_z_range(1000);
	
}

