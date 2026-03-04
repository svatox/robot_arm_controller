/**
  ******************************************************************************
  * @file    motion.c
  * @brief   运动控制实现 - 运动学计算、关节控制、夹爪PWM
  *
  * @details 本文件实现了机械臂的运动学计算和运动控制：
  *         - 关节角度与电机脉冲数的相互转换
  *         - 关节速度与电机转速的相互转换
  *         - 关节运动控制（微动，回零、位置控制）
  *         - 夹爪舵机的PWM控制
  *
  * @note 运动学公式：
  *       电机脉冲 = (关节角度 × 减速比) / 0.1125° + 零位脉冲
  *       关节角度 = (电机脉冲 - 零位脉冲) × 0.1125° / 减速比
  ******************************************************************************
 */

#include "motion.h"
#include "storage.h"
#include "tim.h"
#include <math.h>

/**
 * @brief 全局运动系统实例
 *
 * 存储所有关节和夹爪的状态，包括：
 * - 6个关节的配置（减速比、零位、角度、速度等）
 * - 夹爪配置（PWM范围、当前位置、模式等）
 */
MotionSystem g_motion;

/* ==================== 夹爪PWM定时器变量 ==================== */

/** 夹爪PWM开始时间（用于超时检测）*/
static uint32_t gripper_start_time = 0;

/** 夹爪PWM运动持续时间 */
static uint32_t gripper_duration = 0;


/* ==================== 系统初始化 ==================== */

/**
 * @brief 初始化运动系统
 *
 * 初始化所有关节参数、加载Flash中存储的配置、启动TIM4 PWM
 */
void Motion_Init(void)
{
    // 1. 初始化6个关节
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        // 设置默认值
        g_motion.joints[i].gear_ratio = 1.0f;    // 默认减速比1:1
        g_motion.joints[i].zero_pulse = 0;        // 默认零位0
        g_motion.joints[i].zero_set_flag = 0;      // 默认未设置零位
        g_motion.joints[i].limit_position = 0;     // 默认极限位置0
        g_motion.joints[i].limit_set_flag = 0;     // 默认未设置极限位置
        g_motion.joints[i].current_angle = 0.0f;  // 当前角度0°
        g_motion.joints[i].target_angle = 0.0f;   // 目标角度0°
        g_motion.joints[i].current_speed = 0.0f;  // 当前速度0°
        g_motion.joints[i].enabled = false;       // 默认禁能

        // 2. 从Flash加载配置（如果存在）
        float ratio;
        int32_t zero;
        uint8_t zero_flag;
        int32_t limit;
        uint8_t limit_flag;

        if (Storage_LoadGearRatio(i + 1, &ratio)) {
            g_motion.joints[i].gear_ratio = ratio;
        }
        if (Storage_LoadZeroPosition(i + 1, &zero)) {
            g_motion.joints[i].zero_pulse = zero;
        }
        if (Storage_LoadZeroSetFlag(i + 1, &zero_flag)) {
            g_motion.joints[i].zero_set_flag = zero_flag;
        }
        if (Storage_LoadLimitPosition(i + 1, &limit)) {
            g_motion.joints[i].limit_position = limit;
        }
        if (Storage_LoadLimitSetFlag(i + 1, &limit_flag)) {
            g_motion.joints[i].limit_set_flag = limit_flag;
        }
    }

    // 3. 初始化夹爪默认参数
    g_motion.gripper_pwm_min = 25;      // 0.5ms脉宽对应
    g_motion.gripper_pwm_max = 125;     // 2.5ms脉宽对应
    g_motion.gripper_pwm_center = 75;   // 1.5ms脉宽对应（居中）
    g_motion.gripper_current_pwm = 75;
    g_motion.gripper_mode = GRIPPER_MODE_CENTER;
    g_motion.gripper_moving = false;

    // 4. 初始化位置保护开关（默认开启）
    uint8_t protection;
    if (Storage_LoadProtectionSwitch(&protection)) {
        g_motion.position_protection_enabled = (protection != 0);
    } else {
        g_motion.position_protection_enabled = true;  // 默认开启
    }

    // 5. 启动TIM4 PWM输出 (PB8 = TIM4_CH3)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);  // 初始占空比为0
}


/* ==================== 运动学计算 ==================== */

/**
 * @brief 将关节角度转换为电机脉冲数
 *
 * 计算公式：
 * 1. 电机角度偏移 = 关节角度 × 减速比
 *    （减速比表示电机转动角度与关节转动角度的比值）
 *
 * 2. 电机脉冲偏移 = 电机角度偏移 ÷ 0.1125°
 *    （0.1125° = 360°/3200，每脉冲对应的角度）
 *
 * 3. 电机目标脉冲 = 零位脉冲 + 脉冲偏移
 *
 * @param angle 关节目标角度（度）
 * @param gear_ratio 关节减速比
 * @param zero_pulse 零位脉冲值
 * @return 电机目标脉冲数
 */
int32_t Motion_AngleToPulse(float angle, float gear_ratio, int32_t zero_pulse)
{
    /* 第1步：计算电机角度偏移
     * 例：关节转动10°，减速比5:1，则电机需要转动50° */
    float motor_angle = angle * gear_ratio;

    /* 第2步：计算电机脉冲偏移
     * 例：电机需要转动50°，每脉冲0.1125°，需要50/0.1125=444.44个脉冲
     * 使用roundf四舍五入取整 */
    float pulse_offset = motor_angle / DEG_PER_PULSE;

    /* 第3步：计算目标脉冲
     * 目标脉冲 = 零位基准 + 偏移量
     * 例：零位脉冲=1000，偏移=444，目标=1444 */
    int32_t target_pulse = zero_pulse + (int32_t)roundf(pulse_offset);

    return target_pulse;
}

/**
 * @brief 将电机脉冲数转换为关节角度
 *
 * 计算公式（上述的逆过程）：
 * 1. 脉冲偏移 = 当前脉冲 - 零位脉冲
 * 2. 电机角度偏移 = 脉冲偏移 × 0.1125°
 * 3. 关节角度 = 电机角度偏移 ÷ 减速比
 *
 * @param pulse 电机当前脉冲数
 * @param gear_ratio 关节减速比
 * @param zero_pulse 零位脉冲值
 * @return 关节当前角度（度）
 */
float Motion_PulseToAngle(int32_t pulse, float gear_ratio, int32_t zero_pulse)
{
    /* 第1步：计算相对于零位的脉冲偏移 */
    int32_t pulse_offset = pulse - zero_pulse;

    /* 第2步：计算电机角度偏移
     * 例：偏移444脉冲，电机转动了444×0.1125°=50° */
    float motor_angle = (float)pulse_offset * DEG_PER_PULSE;

    /* 第3步：计算关节角度
     * 例：电机转50°，减速比5:1，关节转动10° */
    float joint_angle = motor_angle / gear_ratio;

    return joint_angle;
}

/**
 * @brief 将关节速度转换为电机转速
 *
 * 计算公式：
 * 电机RPM = (关节速度 / 360°) × 60 × 减速比
 *
 * 解释：
 * - 关节速度/360° = 关节每秒转动的圈数
 * - 乘以60 = 每分钟转动的圈数（RPM）
 * - 乘以减速比 = 电机需要转动的RPM
 *
 * @param joint_speed_deg 关节速度（度/秒）
 * @param gear_ratio 关节减速比
 * @return 电机目标转速（RPM）
 */
uint16_t Motion_JointSpeedToMotorSpeed(float joint_speed_deg, float gear_ratio)
{
    /* 计算公式：RPM = (速度/360) × 60 × 减速比
     * 例：速度10°/s，减速比5
     *     = (10/360) × 60 × 5 = 8.33 RPM */
    float motor_rpm = (joint_speed_deg / 360.0f) * 60.0f * gear_ratio;
    return (uint16_t)roundf(motor_rpm);
}

/**
 * @brief 将电机转速转换为关节速度
 *
 * @param motor_speed_rpm 电机转速（RPM）
 * @param gear_ratio 关节减速比
 * @return 关节速度（度/秒）
 */
float Motion_MotorSpeedToJointSpeed(uint16_t motor_speed_rpm, float gear_ratio)
{
    /* 计算公式：速度 = (RPM/60) × 360 / 减速比
     * 例：电机8.33RPM，减速比5
     *     = (8.33/60) × 360 / 5 = 10°/s */
    float joint_speed = ((float)motor_speed_rpm / 60.0f) * 360.0f / gear_ratio;
    return joint_speed;
}


/* ==================== 关节配置 ==================== */

/**
 * @brief 设置关节减速比
 */
bool Motion_SetGearRatio(uint8_t joint_id, float ratio)
{
    // 检查关节ID是否有效
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX) {
        return false;
    }

    // 验证减速比是否在有效范围内
    if (!Motion_ValidateGearRatio(ratio)) {
        return false;
    }

    // 设置减速比（数组下标从0开始，所以joint_id-1）
    g_motion.joints[joint_id - 1].gear_ratio = ratio;
    return true;
}

/**
 * @brief 获取关节减速比
 */
bool Motion_GetGearRatio(uint8_t joint_id, float *ratio)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX || ratio == NULL) {
        return false;
    }
    *ratio = g_motion.joints[joint_id - 1].gear_ratio;
    return true;
}

/**
 * @brief 设置关节零位
 */
bool Motion_SetZeroPosition(uint8_t joint_id, int32_t motor_pulse)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX) {
        return false;
    }

    // 设置零位脉冲
    g_motion.joints[joint_id - 1].zero_pulse = motor_pulse;

    // 将当前角度重置为0°
    g_motion.joints[joint_id - 1].current_angle = 0.0f;
    return true;
}

/**
 * @brief 获取关节零位脉冲
 */
bool Motion_GetZeroPosition(uint8_t joint_id, int32_t *motor_pulse)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX || motor_pulse == NULL) {
        return false;
    }
    *motor_pulse = g_motion.joints[joint_id - 1].zero_pulse;
    return true;
}

/**
 * @brief 设置关节极限位置
 */
bool Motion_SetLimitPosition(uint8_t joint_id, int32_t motor_pulse)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX) {
        return false;
    }
    g_motion.joints[joint_id - 1].limit_position = motor_pulse;
    g_motion.joints[joint_id - 1].limit_set_flag = 1;
    // 保存到Flash
    Storage_SaveLimitPosition(joint_id, motor_pulse);
    Storage_SaveLimitSetFlag(joint_id, 1);
    Storage_SaveAllConfig();
    return true;
}

/**
 * @brief 获取关节极限位置
 */
bool Motion_GetLimitPosition(uint8_t joint_id, int32_t *motor_pulse)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX || motor_pulse == NULL) {
        return false;
    }
    *motor_pulse = g_motion.joints[joint_id - 1].limit_position;
    return true;
}

/**
 * @brief 设置零位设置标志
 */
bool Motion_SetZeroSetFlag(uint8_t joint_id, uint8_t flag)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX) {
        return false;
    }
    g_motion.joints[joint_id - 1].zero_set_flag = flag;
    Storage_SaveZeroSetFlag(joint_id, flag);
    Storage_SaveAllConfig();
    return true;
}

/**
 * @brief 获取零位设置标志
 */
bool Motion_GetZeroSetFlag(uint8_t joint_id, uint8_t *flag)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX || flag == NULL) {
        return false;
    }
    *flag = g_motion.joints[joint_id - 1].zero_set_flag;
    return true;
}

/**
 * @brief 设置极限位置设置标志
 */
bool Motion_SetLimitSetFlag(uint8_t joint_id, uint8_t flag)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX) {
        return false;
    }
    g_motion.joints[joint_id - 1].limit_set_flag = flag;
    Storage_SaveLimitSetFlag(joint_id, flag);
    Storage_SaveAllConfig();
    return true;
}

/**
 * @brief 获取极限位置设置标志
 */
bool Motion_GetLimitSetFlag(uint8_t joint_id, uint8_t *flag)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX || flag == NULL) {
        return false;
    }
    *flag = g_motion.joints[joint_id - 1].limit_set_flag;
    return true;
}

/**
 * @brief 设置位置保护开关
 */
void Motion_SetPositionProtection(bool enable)
{
    g_motion.position_protection_enabled = enable;
    Storage_SaveProtectionSwitch(enable ? 1 : 0);
    Storage_SaveAllConfig();
}

/**
 * @brief 获取位置保护开关状态
 */
bool Motion_GetPositionProtection(void)
{
    return g_motion.position_protection_enabled;
}

/**
 * @brief 检查目标位置是否在允许范围内
 */
bool Motion_CheckPositionLimit(uint8_t joint_id, int32_t target_pulse)
{
    // 如果位置保护关闭，则不限制
    if (!g_motion.position_protection_enabled) {
        return true;
    }

    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX) {
        return false;
    }

    JointConfig *joint = &g_motion.joints[joint_id - 1];

    // 如果零位或极限位置未设置，则不限制
    if (joint->zero_set_flag == 0 || joint->limit_set_flag == 0) {
        return true;
    }

    // 获取零位和极限位置
    int32_t zero_pos = joint->zero_pulse;
    int32_t limit_pos = joint->limit_position;

    // 确定范围的最小值和最大值
    int32_t min_pos = (zero_pos < limit_pos) ? zero_pos : limit_pos;
    int32_t max_pos = (zero_pos > limit_pos) ? zero_pos : limit_pos;

    // 检查目标位置是否在范围内
    return (target_pulse >= min_pos && target_pulse <= max_pos);
}


/* ==================== 运动控制 ==================== */

/**
 * @brief 关节微动
 *
 * 让关节向指定方向微动一个步长
 *
 * @param joint_id 关节ID (1-6)
 * @param direction 方向 (0=正向, 1=反向)
 * @param step_deg 微动步长（度）
 * @param speed_deg 运动速度（度/秒）
 */
bool Motion_JogJoint(uint8_t joint_id, uint8_t direction, float step_deg, float speed_deg)
{
    // 参数验证
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX) {
        return false;
    }
    if (!Motion_ValidateJogStep(step_deg) || !Motion_ValidateJogSpeed(speed_deg)) {
        return false;
    }

    // 获取关节配置
    JointConfig *joint = &g_motion.joints[joint_id - 1];

    /* 计算目标角度
     * direction=0: 正向，target = current + step
     * direction=1: 反向，target = current - step */
    float step = (direction == 0) ? step_deg : -step_deg;
    float target_angle = joint->current_angle + step;

    // 限幅检查
    if (target_angle < JOINT_ANGLE_MIN) target_angle = JOINT_ANGLE_MIN;
    if (target_angle > JOINT_ANGLE_MAX) target_angle = JOINT_ANGLE_MAX;

    /* 运动学转换
     * 1. 关节角度 → 电机脉冲 */
    int32_t target_pulse = Motion_AngleToPulse(target_angle, joint->gear_ratio, joint->zero_pulse);

    // 检查位置限制
    if (!Motion_CheckPositionLimit(joint_id, target_pulse)) {
        return false;  // 超出限制范围
    }

    /* 2. 关节速度 → 电机转速 */
    uint16_t motor_speed = Motion_JointSpeedToMotorSpeed(speed_deg, joint->gear_ratio);

    /* 发送命令到电机（非同步模式，sync=0）
     * EmmV5_MoveToPosition会等待电机响应 */
    if (EmmV5_MoveToPosition(joint_id, target_pulse, motor_speed, 50, 0)) {
        // 设置目标角度
        joint->target_angle = target_angle;
        return true;
    }
    return false;
}

/**
 * @brief 移动关节到指定角度
 */
bool Motion_MoveToAngle(uint8_t joint_id, float target_angle, float speed_deg)
{
    // 参数验证
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX) {
        return false;
    }
    if (!Motion_ValidateJointAngle(target_angle) || !Motion_ValidateJointSpeed(speed_deg)) {
        return false;
    }

    JointConfig *joint = &g_motion.joints[joint_id - 1];

    // 运动学转换
    int32_t target_pulse = Motion_AngleToPulse(target_angle, joint->gear_ratio, joint->zero_pulse);

    // 检查位置限制
    if (!Motion_CheckPositionLimit(joint_id, target_pulse)) {
        return false;  // 超出限制范围
    }

    uint16_t motor_speed = Motion_JointSpeedToMotorSpeed(speed_deg, joint->gear_ratio);

    // 发送命令
    if (EmmV5_MoveToPosition(joint_id, target_pulse, motor_speed, 50, 0)) {
        joint->target_angle = target_angle;
        return true;
    }
    return false;
}

/**
 * @brief 单关节回零
 *
 * 关节运动到零位位置（zero_pulse对应的位置）
 */
bool Motion_Homing(uint8_t joint_id, float speed_deg)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX) {
        return false;
    }
    if (!Motion_ValidateHomingSpeed(speed_deg)) {
        return false;
    }

    JointConfig *joint = &g_motion.joints[joint_id - 1];

    // 目标位置就是零位脉冲
    uint16_t motor_speed = Motion_JointSpeedToMotorSpeed(speed_deg, joint->gear_ratio);

    if (EmmV5_MoveToPosition(joint_id, joint->zero_pulse, motor_speed, 50, 0)) {
        joint->target_angle = 0.0f;
        return true;
    }
    return false;
}

/**
 * @brief 所有关节同时回零
 *
 * 使用同步模式，最后一个关节设置sync=1触发同步
 */
bool Motion_HomingAll(float speed_deg)
{
    if (!Motion_ValidateHomingSpeed(speed_deg)) {
        return false;
    }

    // 发送回零命令到所有关节
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        JointConfig *joint = &g_motion.joints[i];
        uint16_t motor_speed = Motion_JointSpeedToMotorSpeed(speed_deg, joint->gear_ratio);

        // 最后一个关节设置sync=1，触发同步
        uint8_t sync = (i == JOINT_COUNT - 1) ? 1 : 0;
        EmmV5_MoveToPosition(i + 1, joint->zero_pulse, motor_speed, 50, sync);
        joint->target_angle = 0.0f;
    }

    return true;
}

/**
 * @brief 紧急停止
 */
bool Motion_EmergencyStop(uint8_t address)
{
    if (address == 0x00) {
        // 停止所有电机
        for (uint8_t i = 1; i <= JOINT_COUNT; i++) {
            EmmV5_StopMotor(i);
            g_motion.joints[i - 1].target_angle = g_motion.joints[i - 1].current_angle;
        }
        return true;
    } else if (address >= JOINT_ID_MIN && address <= JOINT_ID_MAX) {
        // 停止指定电机
        EmmV5_StopMotor(address);
        g_motion.joints[address - 1].target_angle = g_motion.joints[address - 1].current_angle;
        return true;
    }
    return false;
}


/* ==================== 多关节同步控制 ==================== */

/**
 * @brief 多关节同步运动
 */
bool Motion_MoveJointsSync(const float *target_angles, const float *speeds_deg)
{
    if (target_angles == NULL || speeds_deg == NULL) {
        return false;
    }

    // 发送位置命令到所有关节
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        JointConfig *joint = &g_motion.joints[i];

        // 验证角度
        if (!Motion_ValidateJointAngle(target_angles[i])) {
            return false;
        }

        // 运动学转换
        int32_t target_pulse = Motion_AngleToPulse(target_angles[i], joint->gear_ratio, joint->zero_pulse);
        uint16_t motor_speed = Motion_JointSpeedToMotorSpeed(speeds_deg[i], joint->gear_ratio);

        // 最后一个关节触发同步
        uint8_t sync = (i == JOINT_COUNT - 1) ? 1 : 0;
        EmmV5_MoveToPosition(i + 1, target_pulse, motor_speed, 50, sync);

        joint->target_angle = target_angles[i];
    }

    return true;
}

/**
 * @brief 设置多个关节的目标角度
 */
bool Motion_SetTargetAngles(const float *angles)
{
    if (angles == NULL) {
        return false;
    }

    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        if (!Motion_ValidateJointAngle(angles[i])) {
            return false;
        }
        g_motion.joints[i].target_angle = angles[i];
    }
    return true;
}

/**
 * @brief 触发同步运动
 */
void Motion_TriggerSync(void)
{
    EmmV5_TriggerSync();
}


/* ==================== 夹爪控制 ==================== */

/**
 * @brief 设置夹爪PWM
 *
 * 根据模式设置夹爪舵机的PWM输出
 *
 * @param mode 控制模式
 *        - GRIPPER_MODE_OPEN_CLOSE: 根据PWM值开合
 *        - GRIPPER_MODE_CENTER: 回中
 *        - GRIPPER_MODE_HOLD: 保持当前位置
 * @param pwm PWM值 (0-100，仅在OPEN_CLOSE模式有效)
 * @param time_ms 运动时间(毫秒)
 */
bool Motion_SetGripperPWM(uint8_t mode, uint8_t pwm, uint8_t time_ms)
{
    uint16_t compare_value;  // TIM3比较值

    switch (mode) {
        case GRIPPER_MODE_OPEN_CLOSE:
            // 开合模式
            if (pwm > GRIPPER_PWM_MAX) {
                return false;
            }

            /* PWM映射计算
             * TIM3配置：72MHz预分频7200→10kHz，周期200→50Hz
             * 0.5ms = 10计数，2.5ms = 50计数
             * pwm=0 → 10计数(0.5ms)
             * pwm=100 → 50计数(2.5ms) */
            compare_value = 10 + (pwm * 40 / 100);  // 范围10-50
            break;

        case GRIPPER_MODE_CENTER:
            // 回中模式：1.5ms = 30计数
            compare_value = 30;
            break;

        case GRIPPER_MODE_HOLD:
            // 保持模式：保持当前PWM
            compare_value = 10 + (g_motion.gripper_current_pwm * 40 / 100);
            break;

        default:
            return false;
    }

    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, compare_value);

    // 保存当前PWM值
    g_motion.gripper_current_pwm = pwm;
    g_motion.gripper_mode = mode;
    g_motion.gripper_moving = true;

    // 记录开始时间和持续时间（用于超时自动停止）
    gripper_start_time = HAL_GetTick();
    gripper_duration = time_ms;

    return true;
}

/**
 * @brief 打开/关闭夹爪
 */
bool Motion_GripperOpenClose(uint8_t pwm, uint8_t time_ms)
{
    return Motion_SetGripperPWM(GRIPPER_MODE_OPEN_CLOSE, pwm, time_ms);
}

/**
 * @brief 夹爪回中
 */
bool Motion_GripperCenter(uint8_t time_ms)
{
    return Motion_SetGripperPWM(GRIPPER_MODE_CENTER, 0, time_ms);
}

/**
 * @brief 夹爪保持当前位置
 */
bool Motion_GripperHold(void)
{
    return Motion_SetGripperPWM(GRIPPER_MODE_HOLD, g_motion.gripper_current_pwm, 0);
}


/* ==================== 状态更新 ==================== */

/**
 * @brief 更新关节状态
 *
 * 根据电机返回的状态更新关节的当前角度和速度
 */
bool Motion_UpdateJointStatus(uint8_t joint_id, const EmmMotorStatus *motor_status)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX || motor_status == NULL) {
        return false;
    }

    JointConfig *joint = &g_motion.joints[joint_id - 1];

    /* 更新当前角度
     * 电机脉冲 → 关节角度 */
    joint->current_angle = Motion_PulseToAngle(motor_status->position,
                                                joint->gear_ratio,
                                                joint->zero_pulse);

    /* 更新当前速度
     * 电机RPM → 关节速度 */
    joint->current_speed = Motion_MotorSpeedToJointSpeed(motor_status->speed,
                                                          joint->gear_ratio);

    // 更新使能状态
    joint->enabled = (motor_status->enabled != 0);

    return true;
}

/**
 * @brief 获取关节状态
 */
bool Motion_GetJointStatus(uint8_t joint_id, float *angle, float *speed, uint16_t *current)
{
    if (joint_id < JOINT_ID_MIN || joint_id > JOINT_ID_MAX) {
        return false;
    }

    // 从电机读取状态
    EmmMotorStatus status;
    if (!EmmV5_GetStatus(joint_id, &status)) {
        return false;
    }

    // 更新关节状态
    Motion_UpdateJointStatus(joint_id, &status);

    // 返回数据
    JointConfig *joint = &g_motion.joints[joint_id - 1];
    if (angle != NULL) *angle = joint->current_angle;
    if (speed != NULL) *speed = joint->current_speed;
    if (current != NULL) *current = status.current;

    return true;
}


/* ==================== 参数验证 ==================== */

bool Motion_ValidateJointAngle(float angle)
{
    return (angle >= JOINT_ANGLE_MIN && angle <= JOINT_ANGLE_MAX);
}

bool Motion_ValidateJointSpeed(float speed)
{
    return (speed >= JOINT_SPEED_MIN && speed <= JOINT_SPEED_MAX);
}

bool Motion_ValidateGearRatio(float ratio)
{
    return (ratio >= GEAR_RATIO_MIN && ratio <= GEAR_RATIO_MAX);
}

bool Motion_ValidateJogStep(float step)
{
    return (step >= JOG_STEP_MIN && step <= JOG_STEP_MAX);
}

bool Motion_ValidateJogSpeed(float speed)
{
    return (speed >= JOG_SPEED_MIN && speed <= JOG_SPEED_MAX);
}

bool Motion_ValidateHomingSpeed(float speed)
{
    return (speed >= HOMING_SPEED_MIN && speed <= HOMING_SPEED_MAX);
}


/* ==================== 工具函数 ==================== */

uint8_t Motion_JointIdToAddress(uint8_t joint_id)
{
    if (joint_id >= JOINT_ID_MIN && joint_id <= JOINT_ID_MAX) {
        return joint_id;
    }
    return 0;
}

bool Motion_AddressToJointId(uint8_t address, uint8_t *joint_id)
{
    if (address >= JOINT_ID_MIN && address <= JOINT_ID_MAX) {
        *joint_id = address;
        return true;
    }
    return false;
}
