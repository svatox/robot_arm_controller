/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用层 - 命令处理和系统状态管理
  *
  * @details 本模块是整个固件的应用层入口，负责：
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

#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"


/* ==================== 系统状态定义 ==================== */

/**
 * @brief 系统运行状态
 *
 * 用于跟踪整个控制器的运行状态，
 * 便于上位机了解当前系统情况：
 * - INIT: 上电初始化中
 * - READY: 系统就绪，等待命令
 * - MOVING: 有关节正在运动
 * - ESTOP: 急停状态
 */
typedef enum {
    SYS_STATE_INIT = 0,    /**< 初始化状态 */
    SYS_STATE_READY = 1,   /**< 就绪状态 */
    SYS_STATE_MOVING = 2,  /**< 运动状态 */
    SYS_STATE_ESTOP = 3   /**< 急停状态 */
} SystemState;


/* ==================== 初始化函数 ==================== */

/**
 * @brief 应用初始化
 *
 * 初始化所有子系统：
 * - 初始化环形缓冲区和帧解析器
 * - 初始化Flash存储模块
 * - 初始化运动控制系统
 * - 初始化Emm_V5电机驱动
 * - 启动USART1 DMA接收
 *
 * @note 必须在main()中首先调用
 */
void App_Init(void);

/**
 * @brief 应用主循环
 *
 * 主循环处理所有任务：
 * - 处理上位机命令
 * - 更新关节状态
 * - 检查异常
 * - 夹爪超时处理
 *
 * @note 这是一个无限循环，不会返回
 */
void App_Run(void);


/* ==================== 命令处理函数 ==================== */

/**
 * @brief 处理设置减速比命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x01，参数为float32减速比
 */
void App_HandleSetGearRatio(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理设置零位命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x02，会读取当前电机位置并保存，同时写入Flash
 */
void App_HandleSetZeroPos(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理保存配置命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x03，将所有配置写入Flash
 */
void App_HandleSaveConfig(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理设置极限位置命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x04，将当前位置设为关节极限位置
 */
void App_HandleSetLimitPos(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理读取极限位置命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x05，返回极限位置和是否已设置标志
 */
void App_HandleReadLimitPos(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理设置位置保护开关命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x06，开启/关闭位置保护
 */
void App_HandleSetProtection(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理重置位置命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x07，清零零位和极限位置，关闭位置保护
 */
void App_HandleResetPositions(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理关节微动命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x20，参数：方向(1B) + 步长(float) + 速度(uint16/10)
 */
void App_HandleJointJog(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理回零命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x21，可单关节或全部(地址0x00)
 */
void App_HandleHoming(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理关节位置控制命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x22，参数：目标角度(float) + 速度(float)
 */
void App_HandleJointPosition(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理夹爪PWM控制命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x27，地址固定0x07(夹爪)
 *       参数：模式(1B) + PWM(1B) + 时间(1B)
 */
void App_HandleGripperPWM(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理急停命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x2F，优先级最高，可立即停止
 */
void App_HandleEmergencyStop(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理读单关节状态命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x41，返回24字节数据
 */
void App_HandleReadSingleStatus(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理读全量状态命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x42，返回系统状态(4字节)
 *       TODO: 实现完整多帧返回
 */
void App_HandleReadFullStatus(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理读夹爪状态命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0x47，返回5字节数据
 */
void App_HandleReadGripperStatus(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);

/**
 * @brief 处理电机透传命令
 * @param frame 协议帧指针
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 *
 * @note 功能码0xF0-0xFF，直接转发到电机
 */
void App_HandleMotorPassthrough(const ProtocolFrame *frame, uint8_t *response, uint8_t *resp_len);


/* ==================== 异常处理函数 ==================== */

/**
 * @brief 主动上报异常
 * @param address 发生异常的设备地址
 * @param except_type 异常类型 (EXCEPT_*)
 * @param except_param 异常参数
 *
 * @note 由异常检测模块调用，向上位机发送异常帧
 */
void App_ReportException(uint8_t address, uint8_t except_type, uint16_t except_param);

/**
 * @brief 检查系统异常
 *
 * @note 在主循环中定期调用，检查：
 *       - 电机堵转
 *       - 电机欠压
 *       - 通信错误
 *       - 控制器状态
 */
void App_CheckExceptions(void);


/* ==================== 状态更新函数 ==================== */

/**
 * @brief 更新所有关节状态
 *
 * @note 在主循环中定期调用(20Hz)：
 *       - 从每个电机读取最新状态
 *       - 更新关节的当前角度和速度
 *       - 更新系统运行状态
 */
void App_UpdateStatus(void);


/* ==================== 串口通信函数 ==================== */

/**
 * @brief 处理UART接收数据
 *
 * @note 从环形缓冲区读取数据，调用帧解析器
 *       当解析完成一帧后，调用App_ProcessFrame处理
 */
void App_ProcessUartData(void);

/**
 * @brief 向上位机发送响应数据
 * @param data 要发送的数据
 * @param len 数据长度
 *
 * @note 内部调用HAL_UART_Transmit
 */
void App_SendResponse(const uint8_t *data, uint8_t len);


/* ==================== 系统状态函数 ==================== */

/**
 * @brief 获取当前系统状态
 * @return 系统状态
 */
SystemState App_GetSystemState(void);

/**
 * @brief 设置系统状态
 * @param state 要设置的状态
 */
void App_SetSystemState(SystemState state);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
