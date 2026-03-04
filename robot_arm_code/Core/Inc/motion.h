/**
  ******************************************************************************
  * @file    motion.h
  * @brief   运动控制 - 运动学计算、关节控制、夹爪PWM
  *
  * @details 本模块负责机械臂的运动学计算和运动控制：
  *         - 关节角度与电机脉冲数的相互转换
  *         - 关节速度与电机转速的相互转换
  *         - 关节运动控制（微动、回零、位置控制）
  *         - 夹爪舵机的PWM控制
  *
  * @note 运动学公式：
  *       电机脉冲 = (关节角度 × 减速比) / 0.1125° + 零位脉冲
  *       关节角度 = (电机脉冲 - 零位脉冲) × 0.1125° / 减速比
  ******************************************************************************
 */

#ifndef __MOTION_H
#define __MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "emm_v5.h"


/* ==================== 关节数量和ID ==================== */

/**
 * @brief 关节数量
 */
#define JOINT_COUNT         6

/**
 * @brief 关节ID有效范围
 */
#define JOINT_ID_MIN        1
#define JOINT_ID_MAX        6


/* ==================== 运动参数范围 ==================== */

/**
 * @brief 关节角度范围（度）
 * @note 允许关节在±360°范围内运动
 */
#define JOINT_ANGLE_MIN     -360.0f
#define JOINT_ANGLE_MAX     360.0f

/**
 * @brief 关节速度范围（度/秒）
 */
#define JOINT_SPEED_MIN     0.1f
#define JOINT_SPEED_MAX     50.0f

/**
 * @brief 减速比有效范围
 */
#define GEAR_RATIO_MIN      0.1f
#define GEAR_RATIO_MAX      100.0f


/* ==================== 微动参数 ==================== */

/**
 * @brief 关节微动步长范围（度）
 * @note 每次微动的角度范围
 */
#define JOG_STEP_MIN        0.01f
#define JOG_STEP_MAX        5.0f

/**
 * @brief 关节微动速度范围（度/秒）
 */
#define JOG_SPEED_MIN       0.1f
#define JOG_SPEED_MAX       10.0f


/* ==================== 回零参数 ==================== */

/**
 * @brief 回零速度范围（度/秒）
 */
#define HOMING_SPEED_MIN    0.5f
#define HOMING_SPEED_MAX    20.0f


/* ==================== 运动学常量 ==================== */

/**
 * @brief 每脉冲对应的角度（度）
 * @note 360° / 3200脉冲 = 0.1125°/脉冲
 *       假设电机为1.8°步距角，16细分
 */
#define DEG_PER_PULSE       0.1125f

/**
 * @brief 每度对应的脉冲数
 * @note 3200脉冲 / 360° = 8.8889脉冲/度
 */
#define PULSE_PER_DEG       8.8889f


/* ==================== 夹爪PWM参数 ==================== */

/**
 * @brief 夹爪PWM控制参数范围
 */
#define GRIPPER_PWM_MIN     0       /**< PWM最小值 */
#define GRIPPER_PWM_MAX     100     /**< PWM最大值 */
#define GRIPPER_TIME_MIN    10      /**< 运动时间最小值(ms) */
#define GRIPPER_TIME_MAX    255     /**< 运动时间最大值(ms) */

/**
 * @brief 夹爪控制模式
 */
#define GRIPPER_MODE_OPEN_CLOSE  0x00    /**< 开合模式 - 根据PWM值开合 */
#define GRIPPER_MODE_CENTER      0x01    /**< 回中模式 - 恢复到中间位置 */
#define GRIPPER_MODE_HOLD        0x02    /**< 保持模式 - 保持当前PWM不变 */


/* ==================== 数据结构 ==================== */

/**
 * @brief 关节配置结构体
 *
 * 存储单个关节的所有运动相关参数：
 *
 * @param gear_ratio 减速比 - 电机转动的度数与关节转动度数的比值
 *                  例如：减速比10表示电机转动10°，关节转动1°
 * @param zero_pulse 零位脉冲 - 物理零位对应的电机脉冲数
 * @param zero_set_flag 零位是否已设置
 * @param limit_position 极限位置脉冲数
 * @param limit_set_flag 极限位置是否已设置
 * @param current_angle 当前关节角度（度）- 实时更新
 * @param target_angle 目标关节角度（度）- 命令下发后设置
 * @param current_speed 当前关节速度（度/秒）
 * @param enabled 使能状态 - 电机是否使能
 */
typedef struct {
    float gear_ratio;           /**< 减速比 */
    int32_t zero_pulse;       /**< 零位脉冲 */
    uint8_t zero_set_flag;    /**< 零位是否已设置 */
    int32_t limit_position;   /**< 极限位置脉冲 */
    uint8_t limit_set_flag;   /**< 极限位置是否已设置 */
    float current_angle;       /**< 当前角度(°) */
    float target_angle;        /**< 目标角度(°) */
    float current_speed;       /**< 当前速度(°/s) */
    bool enabled;              /**< 使能状态 */
} JointConfig;


/**
 * @brief 运动系统结构体
 *
 * 存储整个机械臂的运动状态，包括6个关节和夹爪：
 *
 * @param joints 6个关节的配置和状态
 * @param gripper_pwm_min 夹爪PWM最小值（舵机脉宽最小）
 * @param gripper_pwm_max 夹爪PWM最大值（舵机脉宽最大）
 * @param gripper_pwm_center 夹爪PWM中间值（舵机居中）
 * @param gripper_current_pwm 当前夹爪PWM值(0-100)
 * @param gripper_mode 当前夹爪控制模式
 * @param gripper_moving 夹爪是否正在运动
 * @param position_protection_enabled 位置保护开关
 */
typedef struct {
    JointConfig joints[JOINT_COUNT];      /**< 6个关节 */
    uint8_t gripper_pwm_min;              /**< PWM最小值 */
    uint8_t gripper_pwm_max;              /**< PWM最大值 */
    uint8_t gripper_pwm_center;           /**< PWM中间值 */
    uint8_t gripper_current_pwm;           /**< 当前PWM值 */
    uint8_t gripper_mode;                  /**< 控制模式 */
    bool gripper_moving;                  /**< 运动状态 */
    bool position_protection_enabled;     /**< 位置保护开关 */
} MotionSystem;


/**
 * @brief 全局运动系统实例
 * @note 由motion.c中定义，存放当前运动状态
 */
extern MotionSystem g_motion;


/* ==================== 系统初始化函数 ==================== */

/**
 * @brief 初始化运动系统
 *
 * 初始化所有关节参数、加载Flash中存储的配置、启动TIM3 PWM
 */
void Motion_Init(void);


/* ==================== 运动学计算函数 ==================== */

/**
 * @brief 将关节角度转换为电机脉冲数
 * @param angle 关节目标角度（度）
 * @param gear_ratio 关节减速比
 * @param zero_pulse 零位脉冲值
 * @return 电机目标脉冲数
 *
 * @note 计算公式：
 *       1. 电机角度偏移 = 关节角度 × 减速比
 *       2. 脉冲偏移 = 电机角度偏移 / 0.1125°
 *       3. 目标脉冲 = 零位脉冲 + 脉冲偏移
 */
int32_t Motion_AngleToPulse(float angle, float gear_ratio, int32_t zero_pulse);

/**
 * @brief 将电机脉冲数转换为关节角度
 * @param pulse 电机当前脉冲数
 * @param gear_ratio 关节减速比
 * @param zero_pulse 零位脉冲值
 * @return 关节当前角度（度）
 *
 * @note 计算公式：
 *       1. 脉冲偏移 = 当前脉冲 - 零位脉冲
 *       2. 电机角度偏移 = 脉冲偏移 × 0.1125°
 *       3. 关节角度 = 电机角度偏移 / 减速比
 */
float Motion_PulseToAngle(int32_t pulse, float gear_ratio, int32_t zero_pulse);

/**
 * @brief 将关节速度转换为电机转速
 * @param joint_speed_deg 关节速度（度/秒）
 * @param gear_ratio 关节减速比
 * @return 电机目标转速（RPM）
 *
 * @note 计算公式：
 *       电机RPM = (关节速度 / 360°) × 60 × 减速比
 *       例：速度10°/s，减速比5，RPM = 10/360×60×5 = 8.33
 */
uint16_t Motion_JointSpeedToMotorSpeed(float joint_speed_deg, float gear_ratio);

/**
 * @brief 将电机转速转换为关节速度
 * @param motor_speed_rpm 电机转速（RPM）
 * @param gear_ratio 关节减速比
 * @return 关节速度（度/秒）
 *
 * @note 计算公式：
 *       关节速度 = (电机RPM / 60) × 360° / 减速比
 */
float Motion_MotorSpeedToJointSpeed(uint16_t motor_speed_rpm, float gear_ratio);


/* ==================== 关节配置函数 ==================== */

/**
 * @brief 设置关节减速比
 * @param joint_id 关节ID (1-6)
 * @param ratio 减速比值 (0.1-100.0)
 * @return true 设置成功
 * @return false 参数无效或关节ID错误
 *
 * @note 此函数仅设置内存中的值，不保存到Flash
 *       如需保存到Flash，请使用Motion_SetGearRatioAndSave
 */
bool Motion_SetGearRatio(uint8_t joint_id, float ratio);

/**
 * @brief 设置关节减速比并保存到Flash
 * @param joint_id 关节ID (1-6)
 * @param ratio 减速比值 (0.1-100.0)
 * @return true 设置成功
 * @return false 参数无效或关节ID错误
 */
bool Motion_SetGearRatioAndSave(uint8_t joint_id, float ratio);

/**
 * @brief 获取关节减速比
 * @param joint_id 关节ID (1-6)
 * @param ratio 指向存储减速比的变量的指针
 * @return true 获取成功
 * @return false 关节ID错误
 */
bool Motion_GetGearRatio(uint8_t joint_id, float *ratio);

/**
 * @brief 设置关节零位
 * @param joint_id 关节ID (1-6)
 * @param motor_pulse 零位对应的电机脉冲数
 * @return true 设置成功
 * @return false 关节ID错误
 *
 * @note 设置后，当前角度会被重置为0°
 */
bool Motion_SetZeroPosition(uint8_t joint_id, int32_t motor_pulse);

/**
 * @brief 获取关节零位脉冲数
 * @param joint_id 关节ID (1-6)
 * @param motor_pulse 指向存储零位脉冲的变量的指针
 * @return true 获取成功
 * @return false 关节ID错误
 */
bool Motion_GetZeroPosition(uint8_t joint_id, int32_t *motor_pulse);

/**
 * @brief 设置关节极限位置
 * @param joint_id 关节ID (1-6)
 * @param motor_pulse 极限位置对应的电机脉冲数
 * @return true 设置成功
 * @return false 关节ID错误
 */
bool Motion_SetLimitPosition(uint8_t joint_id, int32_t motor_pulse);

/**
 * @brief 获取关节极限位置
 * @param joint_id 关节ID (1-6)
 * @param motor_pulse 指向存储极限位置的变量的指针
 * @return true 获取成功
 * @return false 关节ID错误
 */
bool Motion_GetLimitPosition(uint8_t joint_id, int32_t *motor_pulse);

/**
 * @brief 设置零位设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 设置标志 (0=未设置, 1=已设置)
 * @return true 设置成功
 * @return false 关节ID错误
 */
bool Motion_SetZeroSetFlag(uint8_t joint_id, uint8_t flag);

/**
 * @brief 获取零位设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 指向存储标志的变量的指针
 * @return true 获取成功
 * @return false 关节ID错误
 */
bool Motion_GetZeroSetFlag(uint8_t joint_id, uint8_t *flag);

/**
 * @brief 设置极限位置设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 设置标志 (0=未设置, 1=已设置)
 * @return true 设置成功
 * @return false 关节ID错误
 */
bool Motion_SetLimitSetFlag(uint8_t joint_id, uint8_t flag);

/**
 * @brief 获取极限位置设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 指向存储标志的变量的指针
 * @return true 获取成功
 * @return false 关节ID错误
 */
bool Motion_GetLimitSetFlag(uint8_t joint_id, uint8_t *flag);

/**
 * @brief 设置位置保护开关
 * @param enable true=开启, false=关闭
 */
void Motion_SetPositionProtection(bool enable);

/**
 * @brief 获取位置保护开关状态
 * @return true=开启, false=关闭
 */
bool Motion_GetPositionProtection(void);

/**
 * @brief 检查目标位置是否在允许范围内
 * @param joint_id 关节ID (1-6)
 * @param target_pulse 目标电机脉冲数
 * @return true 在范围内
 * @return false 超出范围或未设置零位/极限位置
 */
bool Motion_CheckPositionLimit(uint8_t joint_id, int32_t target_pulse);


/* ==================== 运动控制函数 ==================== */

/**
 * @brief 关节微动
 * @param joint_id 关节ID (1-6)
 * @param direction 方向 (0=正向, 1=反向)
 * @param step_deg 微动步长（度）
 * @param speed_deg 运动速度（度/秒）
 * @return true 发送命令成功
 * @return false 参数无效或零位未设置
 *
 * @note 发送微动命令后，关节会移动指定步长
 *       此模式需要零位已设置，否则返回失败
 */
bool Motion_JogJoint(uint8_t joint_id, uint8_t direction, float step_deg, float speed_deg);

/**
 * @brief 关节微动（基于脉冲，不涉及零位）
 * @param joint_id 关节ID (1-6)
 * @param direction 方向 (0=正向, 1=反向)
 * @param step_pulses 微动脉冲数
 * @param speed_rpm 电机速度（RPM）
 * @param acc 加速度 (0-255)
 * @return true 发送命令成功
 * @return false 参数无效
 *
 * @note 使用相对当前位置的模式进行微动，不涉及零位
 *       适用于设置零位和极限位置时的微调
 */
bool Motion_JogJointByPulse(uint8_t joint_id, uint8_t direction, int32_t step_pulses,
                             uint16_t speed_rpm, uint8_t acc);

/**
 * @brief 移动关节到指定角度
 * @param joint_id 关节ID (1-6)
 * @param target_angle 目标角度（度）
 * @param speed_deg 运动速度（度/秒）
 * @return true 发送命令成功
 * @return false 参数无效
 *
 * @note 发送位置命令后，关节会移动到目标角度
 */
bool Motion_MoveToAngle(uint8_t joint_id, float target_angle, float speed_deg);

/**
 * @brief 单关节回零
 * @param joint_id 关节ID (1-6)
 * @param speed_deg 回零速度（度/秒）
 * @return true 发送命令成功
 * @return false 参数无效
 *
 * @note 关节会移动到零位位置（零位脉冲对应的位置）
 */
bool Motion_Homing(uint8_t joint_id, float speed_deg);

/**
 * @brief 所有关节同时回零
 * @param speed_deg 回零速度（度/秒）
 * @return true 发送命令成功
 * @return false 参数无效
 *
 * @note 发送同步回零命令，所有关节同时开始运动
 */
bool Motion_HomingAll(float speed_deg);

/**
 * @brief 紧急停止
 * @param address 设备地址 (0=所有, 1-6=指定关节)
 * @return true 停止成功
 *
 * @note 立即停止指定关节（或所有关节）的运动
 */
bool Motion_EmergencyStop(uint8_t address);


/* ==================== 多关节同步控制 ==================== */

/**
 * @brief 多关节同步运动
 * @param target_angles 6个关节的目标角度数组
 * @param speeds_deg 6个关节的速度数组
 * @return true 命令发送成功
 * @return false 参数无效
 *
 * @note 发送同步运动命令，最后一个关节会触发同步
 */
bool Motion_MoveJointsSync(const float *target_angles, const float *speeds_deg);

/**
 * @brief 设置多个关节的目标角度
 * @param angles 6个关节的目标角度数组
 * @return true 设置成功
 * @return false 参数无效
 *
 * @note 仅设置目标角度，不发送运动命令
 */
bool Motion_SetTargetAngles(const float *angles);

/**
 * @brief 触发同步运动
 *
 * @note 发送同步触发命令，唤醒所有待运动的关节同时开始运动
 */
void Motion_TriggerSync(void);


/* ==================== 夹爪控制函数 ==================== */

/**
 * @brief 设置夹爪PWM
 * @param mode 控制模式 (GRIPPER_MODE_*)
 * @param pwm PWM值 (0-100，仅在OPEN_CLOSE模式有效)
 * @param time_ms 运动时间(毫秒)
 * @return true 设置成功
 * @return false 参数无效
 *
 * @note 根据模式设置夹爪PWM输出：
 *       - OPEN_CLOSE: 按pwm值开合
 *       - CENTER: 回中
 *       - HOLD: 保持当前位置
 */
bool Motion_SetGripperPWM(uint8_t mode, uint8_t pwm, uint8_t time_ms);

/**
 * @brief 打开/关闭夹爪
 * @param pwm PWM值 (0-100)
 * @param time_ms 运动时间
 * @return true 设置成功
 * @return false 参数无效
 *
 * @note pwm=0完全闭合，pwm=100完全张开
 */
bool Motion_GripperOpenClose(uint8_t pwm, uint8_t time_ms);

/**
 * @brief 夹爪回中
 * @param time_ms 运动时间
 * @return true 设置成功
 */
bool Motion_GripperCenter(uint8_t time_ms);

/**
 * @brief 夹爪保持当前位置
 * @return true 设置成功
 *
 * @note 停止PWM输出，保持当前位置（需要持续PWM）
 */
bool Motion_GripperHold(void);


/* ==================== 状态更新函数 ==================== */

/**
 * @brief 更新关节状态
 * @param joint_id 关节ID (1-6)
 * @param motor_status 电机状态指针
 * @return true 更新成功
 * @return false 参数无效
 *
 * @note 根据电机返回的状态更新关节的当前角度和速度
 */
bool Motion_UpdateJointStatus(uint8_t joint_id, const EmmMotorStatus *motor_status);

/**
 * @brief 获取关节状态
 * @param joint_id 关节ID (1-6)
 * @param angle 指向存储角度的指针
 * @param speed 指向存储速度的指针
 * @param current 指向存储电流的指针
 * @return true 获取成功
 * @return false 获取失败
 *
 * @note 会先从电机读取最新状态，再更新关节数据
 */
bool Motion_GetJointStatus(uint8_t joint_id, float *angle, float *speed, uint16_t *current);


/* ==================== 参数验证函数 ==================== */

/**
 * @brief 验证关节角度是否有效
 * @param angle 角度值
 * @return true 有效
 * @return false 无效
 */
bool Motion_ValidateJointAngle(float angle);

/**
 * @brief 验证关节速度是否有效
 * @param speed 速度值
 * @return true 有效
 * @return false 无效
 */
bool Motion_ValidateJointSpeed(float speed);

/**
 * @brief 验证减速比是否有效
 * @param ratio 减速比值
 * @return true 有效
 * @return false 无效
 */
bool Motion_ValidateGearRatio(float ratio);

/**
 * @brief 验证微动步长是否有效
 * @param step 步长值
 * @return true 有效
 * @return false 无效
 */
bool Motion_ValidateJogStep(float step);

/**
 * @brief 验证微动速度是否有效
 * @param speed 速度值
 * @return true 有效
 * @return false 无效
 */
bool Motion_ValidateJogSpeed(float speed);

/**
 * @brief 验证回零速度是否有效
 * @param speed 速度值
 * @return true 有效
 * @return false 无效
 */
bool Motion_ValidateHomingSpeed(float speed);


/* ==================== 工具函数 ==================== */

/**
 * @brief 关节ID转换为地址
 * @param joint_id 关节ID
 * @return 对应的地址
 */
uint8_t Motion_JointIdToAddress(uint8_t joint_id);

/**
 * @brief 地址转换为关节ID
 * @param address 设备地址
 * @param joint_id 指向存储关节ID的指针
 * @return true 转换成功
 * @return false 地址无效
 */
bool Motion_AddressToJointId(uint8_t address, uint8_t *joint_id);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_H */
