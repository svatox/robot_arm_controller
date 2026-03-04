/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层实现 - 命令处理和系统状态管理
  *
  * @details 本文件是整个固件的应用层入口，负责：
  *         - 初始化所有子系统
  *         - 处理上位机发送的命令帧
  *         - 分发到对应的处理函数
  *         - 构建并发送响应帧
  *         - 管理系统状态（就绪、运动、停止、异常）
  *         - 定期更新关节状态
  *
  * @note 主循环流程：
  *       1. 处理上位机命令（从环形缓冲区读取）
  *       2. 定期更新关节状态（20Hz）
  *       3. 检查异常
  *       4. 夹爪超时处理
  ******************************************************************************
 */

#include "app.h"
#include "motion.h"
#include "storage.h"
#include "protocol.h"
#include "ring_buffer.h"
#include "emm_v5.h"
#include "usart.h"
#include "tim.h"
#include <string.h>
#include <math.h>

/* ==================== 静态变量 ==================== */

/** USART2接收缓冲区（DMA用）*/
static uint8_t usart2_rx_buf[64];

/** USART2环形缓冲区 - 存储从DMA接收的数据 */
static RingBuffer usart2_rb;

/** USART2帧解析器 - 解析上位机命令帧 */
static FrameParser usart2_parser;

/** 系统运行状态 */
static SystemState g_system_state = SYS_STATE_INIT;

/** 上次状态更新的时间戳（毫秒）*/
static uint32_t last_status_update = 0;


/* ==================== 私有函数声明 ==================== */

static void App_InitUSART2_DMA(void);
static void App_ProcessFrame(const ProtocolFrame *frame);


/* ==================== 初始化 ==================== */

/**
 * @brief 应用初始化
 *
 * 初始化所有子系统，为正常运行做准备
 */
void App_Init(void)
{
    // 1. 初始化环形缓冲区和帧解析器
    RingBuffer_Init(&usart2_rb);
    FrameParser_Init(&usart2_parser);

    // 2. 初始化子系统
    Storage_Init();   // Flash存储
    Motion_Init();   // 运动控制
    EmmV5_Init();    // 电机驱动

    // 3. 启动DMA接收
    App_InitUSART2_DMA();

    // 4. 设置系统为就绪状态
    g_system_state = SYS_STATE_READY;
}

/**
 * @brief 初始化USART2 DMA接收
 *
 * 配置DMA接收模式和IDLE中断
 */
static void App_InitUSART2_DMA(void)
{
    // 启动DMA接收（使用IDLE空闲中断检测接收完成）
    // HAL_UARTEx_ReceiveToIdle_DMA 会在收到数据或IDLE时触发回调
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buf, sizeof(usart2_rx_buf));

    // 启用IDLE空闲中断
    // 当USART总线空闲（超过1帧时间没有数据）时触发
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}


/* ==================== 主循环 ==================== */

/**
 * @brief 应用主循环
 *
 * 这是一个无限循环，处理所有任务
 */
void App_Run(void)
{
    while (1) {
        // 1. 处理上位机命令
        App_ProcessUartData();

        // 2. 定期更新关节状态（每50ms，即20Hz）
        if (HAL_GetTick() - last_status_update > 50) {
            App_UpdateStatus();
            last_status_update = HAL_GetTick();
        }

        // 3. 检查异常
        App_CheckExceptions();

        // 4. 夹爪超时处理（TODO）
        // 需要添加夹爪PWM超时自动停止逻辑

        // 短暂延时，避免CPU占用过高
        HAL_Delay(1);
    }
}


/* ==================== UART数据处理 ==================== */

/**
 * @brief 处理UART接收数据
 *
 * 从环形缓冲区读取数据，进行帧解析
 */
void App_ProcessUartData(void)
{
    uint8_t data;
    ProtocolFrame frame;

    // 循环读取环形缓冲区中的所有数据
    while (RingBuffer_Read(&usart2_rb, &data)) {
        // 将每个字节送入帧解析器
        int result = FrameParser_ProcessByte(&usart2_parser, data);

        if (result == 1) {
            // 返回1表示完整帧已解析完成

            // 获取解析后的帧，并进行CRC校验
            if (FrameParser_GetFrame(&usart2_parser, &frame) == 0) {
                // CRC校验通过，处理帧
                App_ProcessFrame(&frame);
            } else {
                // CRC校验失败，发送错误响应
                uint8_t resp[16];
                uint8_t resp_len;
                Frame_BuildResponse(0x08, 0x00, STATUS_CRC_ERROR, NULL, 0, resp, &resp_len);
                App_SendResponse(resp, resp_len);
            }

            // 重置解析器状态，准备解析下一帧
            FrameParser_Init(&usart2_parser);
        }
    }
}


/* ==================== 帧处理分发 ==================== */

/**
 * @brief 处理解析后的协议帧
 *
 * 根据功能码分发到对应的处理函数
 */
static void App_ProcessFrame(const ProtocolFrame *frame)
{
    // 响应缓冲区
    uint8_t response[64];
    uint8_t resp_len = 0;

    // 急停命令优先级最高，立即处理
    if (frame->function_code == FUNC_EMERGENCY_STOP) {
        App_HandleEmergencyStop(frame, response, &resp_len);
        App_SendResponse(response, resp_len);
        return;
    }

    // 根据功能码分发到对应的处理函数
    switch (frame->function_code) {
        /* 配置命令 */
        case FUNC_SET_GEAR_RATIO:
            App_HandleSetGearRatio(frame, response, &resp_len);
            break;

        case FUNC_SET_ZERO_POS:
            App_HandleSetZeroPos(frame, response, &resp_len);
            break;

        case FUNC_SAVE_CONFIG:
            App_HandleSaveConfig(frame, response, &resp_len);
            break;

        case FUNC_SET_LIMIT_POS:
            App_HandleSetLimitPos(frame, response, &resp_len);
            break;

        case FUNC_READ_LIMIT_POS:
            App_HandleReadLimitPos(frame, response, &resp_len);
            break;

        case FUNC_SET_PROTECTION:
            App_HandleSetProtection(frame, response, &resp_len);
            break;

        case FUNC_READ_PROTECTION:
            App_HandleReadProtection(frame, response, &resp_len);
            break;

        case FUNC_RESET_POSITIONS:
            App_HandleResetPositions(frame, response, &resp_len);
            break;

        /* 运动控制命令 */
        case FUNC_JOINT_JOG:
            App_HandleJointJog(frame, response, &resp_len);
            break;

        case FUNC_HOMING:
            App_HandleHoming(frame, response, &resp_len);
            break;

        case FUNC_JOINT_POSITION:
            App_HandleJointPosition(frame, response, &resp_len);
            break;

        case FUNC_GRIPPER_PWM:
            App_HandleGripperPWM(frame, response, &resp_len);
            break;

        /* 状态读取命令 */
        case FUNC_READ_SINGLE_STATUS:
            App_HandleReadSingleStatus(frame, response, &resp_len);
            break;

        case FUNC_READ_FULL_STATUS:
            App_HandleReadFullStatus(frame, response, &resp_len);
            break;

        case FUNC_READ_JOINT_ANGLE:
            App_HandleReadJointAngle(frame, response, &resp_len);
            break;

        case FUNC_READ_GRIPPER_STATUS:
            App_HandleReadGripperStatus(frame, response, &resp_len);
            break;

        /* 电机透传 */
        default:
            // 检查是否在透传范围内
            if (frame->function_code >= FUNC_MOTOR_PASSTHROUGH_BASE &&
                frame->function_code <= FUNC_MOTOR_PASSTHROUGH_MAX) {
                App_HandleMotorPassthrough(frame, response, &resp_len);
            } else {
                // 未知功能码，返回参数错误
                Frame_BuildResponse(frame->address, frame->function_code,
                                    STATUS_PARAM_ERROR, NULL, 0, response, &resp_len);
            }
            break;
    }

    // 发送响应
    if (resp_len > 0) {
        App_SendResponse(response, resp_len);
    }
}


/* ==================== 串口发送 ==================== */

/**
 * @brief 向上位机发送响应
 */
void App_SendResponse(const uint8_t *data, uint8_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100);
}


/* ==================== 配置命令处理 ==================== */

/**
 * @brief 处理设置减速比命令
 * 功能码: 0x01
 * 数据: float32 减速比
 */
void App_HandleSetGearRatio(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;

    // 检查数据长度（减速比4字节）
    if (frame->data_length < 4) {
        status = STATUS_PARAM_ERROR;
    } else {
        uint8_t joint_addr = frame->address;
        float ratio = Protocol_ReadFloat(frame->data);

        if (joint_addr == ADDR_BROADCAST) {
            // 广播地址，设置所有关节
            for (uint8_t i = 1; i <= JOINT_COUNT; i++) {
                if (!Motion_SetGearRatio(i, ratio)) {
                    status = STATUS_PARAM_ERROR;
                    break;
                }
            }
        } else if (Protocol_IsValidJointAddress(joint_addr)) {
            // 设置单个关节
            if (!Motion_SetGearRatio(joint_addr, ratio)) {
                status = STATUS_PARAM_ERROR;
            }
        } else {
            status = STATUS_PARAM_ERROR;
        }
    }

    // 构建响应帧
    Frame_BuildResponse(frame->address, FUNC_SET_GEAR_RATIO, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理设置零位命令
 * 功能码: 0x02
 * 数据: 无
 */
void App_HandleSetZeroPos(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;
    uint8_t joint_addr = frame->address;

    // 检查地址是否有效
    if (!Protocol_IsValidJointAddress(joint_addr)) {
        status = STATUS_PARAM_ERROR;
    } else {
        // 读取电机当前位置作为零位
        int32_t current_pulse;
        if (EmmV5_GetPosition(joint_addr, &current_pulse)) {
            // 设置零位
            if (!Motion_SetZeroPosition(joint_addr, current_pulse)) {
                status = STATUS_EXEC_FAILED;
            } else {
                // 设置零位标志并保存到Flash
                Motion_SetZeroSetFlag(joint_addr, 1);
            }
        } else {
            status = STATUS_EXEC_FAILED;
        }
    }

    Frame_BuildResponse(frame->address, FUNC_SET_ZERO_POS, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理保存配置命令
 * 功能码: 0x03
 */
void App_HandleSaveConfig(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;

    // 将所有配置写入Flash
    if (!Storage_SaveAllConfig()) {
        status = STATUS_EXEC_FAILED;
    }

    // 系统地址的响应
    Frame_BuildResponse(ADDR_SYSTEM, FUNC_SAVE_CONFIG, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理设置极限位置命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * 功能码: 0x04
 * 地址: 0x01-0x06
 * 参数: 无
 * 返回: 状态码
 *
 * @note 执行后会读取电机当前位置，并将其设置为极限位置
 *       同时设置"已设置"标志并保存到Flash
 */
void App_HandleSetLimitPos(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;
    uint8_t joint_addr = frame->address;

    // 检查地址是否有效
    if (!Protocol_IsValidJointAddress(joint_addr)) {
        status = STATUS_PARAM_ERROR;
    } else {
        // 读取电机当前位置作为极限位置
        int32_t current_pulse;
        if (EmmV5_GetPosition(joint_addr, &current_pulse)) {
            // 设置极限位置
            if (!Motion_SetLimitPosition(joint_addr, current_pulse)) {
                status = STATUS_EXEC_FAILED;
            }
        } else {
            status = STATUS_EXEC_FAILED;
        }
    }

    Frame_BuildResponse(frame->address, FUNC_SET_LIMIT_POS, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理读取极限位置命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * 功能码: 0x05
 * 地址: 0x01-0x06
 * 参数: 无
 * 返回: [是否已设置(1B)] + [极限位置(4B)]
 *
 * @note 返回数据中包含极限位置设置标志和实际的极限位置值
 *       如果未设置，极限位置返回0
 */
void App_HandleReadLimitPos(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;
    uint8_t data[5];

    if (!Protocol_IsValidJointAddress(frame->address)) {
        status = STATUS_PARAM_ERROR;
    } else {
        uint8_t joint_id = frame->address;
        uint8_t limit_set_flag;
        int32_t limit_position;

        // 获取极限位置设置标志
        if (!Motion_GetLimitSetFlag(joint_id, &limit_set_flag)) {
            limit_set_flag = 0;
        }

        // 获取极限位置
        if (!Motion_GetLimitPosition(joint_id, &limit_position)) {
            limit_position = 0;
        }

        // 打包数据
        data[0] = limit_set_flag;
        Protocol_WriteI32(limit_position, &data[1]);

        Frame_BuildResponse(frame->address, FUNC_READ_LIMIT_POS, status, data, 5, response, resp_len);
        return;
    }

    Frame_BuildResponse(frame->address, FUNC_READ_LIMIT_POS, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理设置位置保护开关命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * 功能码: 0x06
 * 地址: 0x08（系统）
 * 参数: 开关状态(1B) - 0=关闭, 1=开启
 * 返回: 状态码
 *
 * @note 开启后关节运动将被限制在零位和极限位置之间
 *       关闭后不限制运动范围
 */
void App_HandleSetProtection(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;

    // 检查数据长度
    if (frame->data_length < 1) {
        status = STATUS_PARAM_ERROR;
    } else {
        uint8_t enable = frame->data[0];
        Motion_SetPositionProtection(enable != 0);
    }

    Frame_BuildResponse(ADDR_SYSTEM, FUNC_SET_PROTECTION, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理读取位置保护状态命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 返回数据：1字节 - 位置保护开关状态 (0=关闭, 1=开启)
 */
void App_HandleReadProtection(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;
    uint8_t protection_state = Motion_GetPositionProtection() ? 1 : 0;

    Frame_BuildResponse(ADDR_SYSTEM, FUNC_READ_PROTECTION, status, &protection_state, 1, response, resp_len);
}

/**
 * @brief 处理重置位置命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * 功能码: 0x07
 * 地址: 0x08（系统）
 * 参数: 无
 * 返回: 状态码
 *
 * @note 执行后会：
 *       1. 清零所有关节的零位和极限位置
 *       2. 将所有设置标志设置为"未设置"
 *       3. 关闭位置保护开关
 *       常用于重新标定机械臂或恢复出厂设置
 */
void App_HandleResetPositions(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;

    // 重置存储配置（会清零所有位置和标志，并关闭保护开关）
    if (!Storage_ResetToDefault()) {
        status = STATUS_EXEC_FAILED;
    }

    // 同步更新内存中的状态
    for (uint8_t i = 1; i <= 6; i++) {
        Motion_SetZeroPosition(i, 0);
        Motion_SetZeroSetFlag(i, 0);
        Motion_SetLimitPosition(i, 0);
        Motion_SetLimitSetFlag(i, 0);
    }
    Motion_SetPositionProtection(false);

    Frame_BuildResponse(ADDR_SYSTEM, FUNC_RESET_POSITIONS, status, NULL, 0, response, resp_len);
}


/* ==================== 运动控制命令处理 ==================== */

/**
 * @brief 处理关节微动命令
 * 功能码: 0x20
 * 数据: 模式(1B) + 方向(1B) + 步长(4B) + 速度(2B)
 *
 * 模式说明：
 * - 模式0（角度模式）: 步长为角度(float)，需要零位已设置
 * - 模式1（脉冲模式）: 步长为脉冲数(int32)，不涉及零位
 *
 * 角度模式数据格式: [模式=0][方向][步长角度(float)][速度(uint16)]
 * 脉冲模式数据格式: [模式=1][方向][步长脉冲(int32)][速度(uint16)][加速度(1B)]
 */
void App_HandleJointJog(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;

    // 检查数据长度和地址
    if (frame->data_length < 6 || !Protocol_IsValidJointAddress(frame->address)) {
        status = STATUS_PARAM_ERROR;
    } else {
        uint8_t mode = frame->data[0];  // 模式: 0=角度, 1=脉冲

        if (mode == 0) {
            // 角度模式
            if (frame->data_length < 7) {
                status = STATUS_PARAM_ERROR;
            } else {
                uint8_t direction = frame->data[1];                        // 方向
                float step = Protocol_ReadFloat(&frame->data[2]);        // 步长(角度)
                uint16_t speed_raw = Protocol_ReadU16(&frame->data[6]);  // 速度原始值
                float speed = (float)speed_raw / 10.0f;                 // 转换为float

                // 执行角度模式微动
                if (!Motion_JogJoint(frame->address, direction, step, speed)) {
                    status = STATUS_EXEC_FAILED;
                } else {
                    g_system_state = SYS_STATE_MOVING;
                }
            }
        } else if (mode == 1) {
            // 脉冲模式
            if (frame->data_length < 8) {
                status = STATUS_PARAM_ERROR;
            } else {
                uint8_t direction = frame->data[1];                        // 方向
                int32_t step_pulses = Protocol_ReadI32(&frame->data[2]); // 步长(脉冲)
                uint16_t speed_rpm = Protocol_ReadU16(&frame->data[6]);   // 速度(RPM)
                uint8_t acc = frame->data[8];                            // 加速度

                // 执行脉冲模式微动
                if (!Motion_JogJointByPulse(frame->address, direction, step_pulses, speed_rpm, acc)) {
                    status = STATUS_EXEC_FAILED;
                } else {
                    g_system_state = SYS_STATE_MOVING;
                }
            }
        } else {
            status = STATUS_PARAM_ERROR;
        }
    }

    Frame_BuildResponse(frame->address, FUNC_JOINT_JOG, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理回零命令
 * 功能码: 0x21
 * 数据: 速度(float)
 */
void App_HandleHoming(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;

    if (frame->data_length < 4) {
        status = STATUS_PARAM_ERROR;
    } else {
        float speed = Protocol_ReadFloat(frame->data);

        if (frame->address == ADDR_BROADCAST) {
            // 所有关节回零
            if (!Motion_HomingAll(speed)) {
                status = STATUS_PARAM_ERROR;
            }
        } else if (Protocol_IsValidJointAddress(frame->address)) {
            // 单关节回零
            if (!Motion_Homing(frame->address, speed)) {
                status = STATUS_EXEC_FAILED;
            }
        } else {
            status = STATUS_PARAM_ERROR;
        }

        if (status == STATUS_SUCCESS) {
            g_system_state = SYS_STATE_MOVING;
        }
    }

    Frame_BuildResponse(frame->address, FUNC_HOMING, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理位置控制命令
 * 功能码: 0x22
 * 数据: 目标角度(float) + 速度(float)
 */
void App_HandleJointPosition(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;

    if (frame->data_length < 8 || !Protocol_IsValidJointAddress(frame->address)) {
        status = STATUS_PARAM_ERROR;
    } else {
        float target_angle = Protocol_ReadFloat(&frame->data[0]);  // 目标角度
        float speed = Protocol_ReadFloat(&frame->data[4]);         // 速度

        if (!Motion_MoveToAngle(frame->address, target_angle, speed)) {
            status = STATUS_EXEC_FAILED;
        } else {
            g_system_state = SYS_STATE_MOVING;
        }
    }

    Frame_BuildResponse(frame->address, FUNC_JOINT_POSITION, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理夹爪PWM命令
 * 功能码: 0x27
 * 地址: 0x07
 * 数据: 模式(1B) + PWM(1B) + 时间(1B)
 */
void App_HandleGripperPWM(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;

    // 检查数据长度和地址
    if (frame->data_length < 3 || frame->address != ADDR_GRIPPER) {
        status = STATUS_PARAM_ERROR;
    } else {
        // 解析参数
        uint8_t mode = frame->data[0];    // 模式
        uint8_t pwm = frame->data[1];     // PWM值
        uint8_t time_ms = frame->data[2]; // 运动时间

        if (!Motion_SetGripperPWM(mode, pwm, time_ms)) {
            status = STATUS_PARAM_ERROR;
        }
    }

    Frame_BuildResponse(ADDR_GRIPPER, FUNC_GRIPPER_PWM, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理急停命令
 * 功能码: 0x2F
 */
void App_HandleEmergencyStop(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    // 立即停止
    Motion_EmergencyStop(frame->address);
    g_system_state = SYS_STATE_ESTOP;

    Frame_BuildResponse(frame->address, FUNC_EMERGENCY_STOP, STATUS_SUCCESS, NULL, 0, response, resp_len);
}


/* ==================== 状态读取命令处理 ==================== */

/**
 * @brief 处理读单关节状态命令
 * 功能码: 0x41
 * 返回: 30字节数据
 */
void App_HandleReadSingleStatus(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;
    uint8_t data[30];

    if (!Protocol_IsValidJointAddress(frame->address)) {
        status = STATUS_PARAM_ERROR;
    } else {
        uint8_t joint_id = frame->address;
        EmmMotorStatus motor_status;

        // 读取电机状态
        if (EmmV5_GetStatus(joint_id, &motor_status)) {
            // 更新关节状态
            Motion_UpdateJointStatus(joint_id, &motor_status);

            JointConfig *joint = &g_motion.joints[joint_id - 1];

            /* 打包数据（30字节）：
             * [0-3]   关节实际角度 (float32)
             * [4-7]   关节目标角度 (float32)
             * [8-11]  关节实时速度 (float32)
             * [12-13] 电机相电流 (uint16)
             * [14-15] 电机转速 (int16)
             * [16-19] 位置误差 (float32)
             * [20]    使能状态 (uint8)
             * [21-22] 总线电压 (uint16)
             * [23]    堵转标志 (uint8)
             * [24-29] 预留
             */

            // 关节实际角度
            Protocol_WriteFloat(joint->current_angle, &data[0]);
            // 关节目标角度
            Protocol_WriteFloat(joint->target_angle, &data[4]);
            // 关节实时速度
            Protocol_WriteFloat(joint->current_speed, &data[8]);
            // 电机相电流
            Protocol_WriteU16(motor_status.current, &data[12]);
            // 电机转速
            Protocol_WriteI16(motor_status.speed, &data[14]);
            // 位置误差（暂未实现，设为0）
            Protocol_WriteFloat(motor_status.pos_error, &data[16]);
            // 使能状态
            data[20] = motor_status.enabled;
            // 总线电压
            Protocol_WriteU16(motor_status.voltage, &data[21]);
            // 堵转标志
            data[23] = motor_status.stall_flag;

            // 发送响应（24字节有效数据）
            Frame_BuildResponse(frame->address, FUNC_READ_SINGLE_STATUS, status, data, 24, response, resp_len);
            return;
        } else {
            status = STATUS_EXEC_FAILED;
        }
    }

    Frame_BuildResponse(frame->address, FUNC_READ_SINGLE_STATUS, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理读全量状态命令
 * 功能码: 0x42
 *
 * @note TODO: 实现完整的多帧返回（189字节 = 30×6 + 5 + 4）
 */
void App_HandleReadFullStatus(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t data[4];

    // 目前只返回简化版系统状态（4字节）
    // TODO: 实现完整的多帧返回

    // 系统电压（占位）
    Protocol_WriteU16(12000, &data[0]);  // 12V
    // UART状态
    data[2] = 0x00;  // 正常
    // Flash状态
    data[3] = Storage_IsValid() ? 0x00 : 0x01;

    Frame_BuildResponse(ADDR_SYSTEM, FUNC_READ_FULL_STATUS, STATUS_SUCCESS, data, 4, response, resp_len);
}

/**
 * @brief 处理读关节角度命令
 * 功能码: 0x43
 * 返回: float32 关节当前角度
 */
void App_HandleReadJointAngle(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t status = STATUS_SUCCESS;
    uint8_t data[4];

    if (!Protocol_IsValidJointAddress(frame->address)) {
        status = STATUS_PARAM_ERROR;
    } else {
        uint8_t joint_id = frame->address;
        EmmMotorStatus motor_status;

        // 读取电机状态
        if (EmmV5_GetStatus(joint_id, &motor_status)) {
            // 更新关节状态
            Motion_UpdateJointStatus(joint_id, &motor_status);

            // 获取关节当前角度
            JointConfig *joint = &g_motion.joints[joint_id - 1];
            Protocol_WriteFloat(joint->current_angle, data);

            Frame_BuildResponse(frame->address, FUNC_READ_JOINT_ANGLE, status, data, 4, response, resp_len);
            return;
        } else {
            status = STATUS_EXEC_FAILED;
        }
    }

    Frame_BuildResponse(frame->address, FUNC_READ_JOINT_ANGLE, status, NULL, 0, response, resp_len);
}

/**
 * @brief 处理读夹爪状态命令
 * 功能码: 0x47
 */
void App_HandleReadGripperStatus(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    uint8_t data[5];

    // 当前PWM值
    data[0] = g_motion.gripper_current_pwm;
    // 控制模式
    data[1] = g_motion.gripper_mode;
    // 开合状态（大于中值为张开）
    data[2] = (g_motion.gripper_current_pwm > g_motion.gripper_pwm_center) ? 1 : 0;
    // 运动状态
    data[3] = g_motion.gripper_moving ? 1 : 0;
    // 错误标志
    data[4] = 0x00;  // 无错误

    Frame_BuildResponse(ADDR_GRIPPER, FUNC_READ_GRIPPER_STATUS, STATUS_SUCCESS, data, 5, response, resp_len);
}


/* ==================== 电机透传 ==================== */

/**
 * @brief 处理电机透传命令
 * 功能码: 0xF0-0xFF
 */
void App_HandleMotorPassthrough(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len)
{
    // 检查地址
    if (!Protocol_IsValidJointAddress(frame->address)) {
        Frame_BuildResponse(frame->address, frame->function_code,
                            STATUS_PARAM_ERROR, NULL, 0, response, resp_len);
        return;
    }

    // 发送原始命令到电机
    EmmV5_SendCommand(frame->data, frame->data_length);

    // 读取电机响应
    uint8_t motor_resp[32];
    uint8_t resp_length = sizeof(motor_resp);

    if (EmmV5_ReadResponse(motor_resp, &resp_length, 100)) {
        // 转发电机响应
        Frame_BuildResponse(frame->address, frame->function_code,
                            STATUS_SUCCESS, motor_resp, resp_length, response, resp_len);
    } else {
        Frame_BuildResponse(frame->address, frame->function_code,
                            STATUS_EXEC_FAILED, NULL, 0, response, resp_len);
    }
}


/* ==================== 异常处理 ==================== */

/**
 * @brief 主动上报异常
 */
void App_ReportException(uint8_t address, uint8_t except_type, uint16_t except_param)
{
    uint8_t resp[16];
    uint8_t resp_len;

    Frame_BuildException(address, except_type, except_param, resp, &resp_len);
    App_SendResponse(resp, resp_len);
}

/**
 * @brief 检查系统异常
 *
 * @note TODO: 实现完整的异常检测
 *       - 电机堵转检测
 *       - 欠压检测
 *       - 通信错误检测
 */
void App_CheckExceptions(void)
{
    // 预留异常检测功能
    // 需要定期检查各电机的stall_flag和uv_flag
}


/* ==================== 状态更新 ==================== */

/**
 * @brief 更新所有关节状态
 *
 * 从电机读取最新状态，更新关节数据
 */
void App_UpdateStatus(void)
{
    // 1. 更新所有关节状态
    for (uint8_t i = 1; i <= JOINT_COUNT; i++) {
        EmmMotorStatus status;
        if (EmmV5_GetStatus(i, &status)) {
            Motion_UpdateJointStatus(i, &status);
        }
    }

    // 2. 检查是否所有关节都已到达目标
    bool any_moving = false;
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
        // 如果目标角度与当前角度差值大于0.1°，认为还在运动
        if (fabsf(g_motion.joints[i].target_angle - g_motion.joints[i].current_angle) > 0.1f) {
            any_moving = true;
            break;
        }
    }

    // 3. 如果没有关节在运动且之前是运动状态，则切换到就绪状态
    if (!any_moving && g_system_state == SYS_STATE_MOVING) {
        g_system_state = SYS_STATE_READY;
    }
}


/* ==================== 系统状态函数 ==================== */

SystemState App_GetSystemState(void)
{
    return g_system_state;
}

void App_SetSystemState(SystemState state)
{
    g_system_state = state;
}


/* ==================== UART中断回调 ==================== */

/**
 * @brief UART空闲中断回调函数
 *
 * UART接收事件回调函数（HAL_UARTEx_RxEventCallback）
 * 当DMA接收到数据或检测到IDLE空闲线路时调用
 * 将DMA缓冲区中的数据复制到环形缓冲区
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart2) {
        // 将接收到的数据复制到环形缓冲区
        // Size 表示本次接收到的数据字节数
        for (uint16_t i = 0; i < Size; i++) {
            RingBuffer_Write(&usart2_rb, usart2_rx_buf[i]);
        }

        // 重新启动DMA接收（继续等待下一帧数据）
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_buf, sizeof(usart2_rx_buf));
    }
}
