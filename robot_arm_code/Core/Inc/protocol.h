/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   通信协议 - 帧解析、组帧和数据类型转换
  *
  * @details 本模块实现了上位机与STM32之间的自定义二进制通信协议。
  *         协议采用帧格式传输，包含帧头、地址、功能码、数据、CRC校验和帧尾。
  *
  * @note 协议格式：
  *       ┌──────┬──────┬──────┬──────┬────────┬─────┬──────┬──────┐
  *       │ 帧头 │ 地址 │ 功能码│ 数据长│ 数据   │ CRC │ 帧尾 │
  *       │ 0xAA │ 1B   │ 1B   │ 1B   │ N     │ 1B  │ 0x55AA│
  *       │ 0x55 │      │      │      │        │     │      │
  *       └──────┴──────┴──────┴──────┴────────┴─────┴──────┴──────┘
  ******************************************************************************
 */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ==================== 帧定界符 ==================== */

/**
 * @brief 帧头标识
 * @note 固定为0xAA 0x55，用于帧起始识别
 */
#define FRAME_HEADER_1      0xAA    /**< 帧头第1字节 */
#define FRAME_HEADER_2      0x55    /**< 帧头第2字节 */

/**
 * @brief 帧尾标识
 * @note 固定为0x55 0xAA，用于帧结束识别
 */
#define FRAME_TAIL_1        0x55    /**< 帧尾第1字节 */
#define FRAME_TAIL_2        0xAA    /**< 帧尾第2字节 */


/* ==================== 地址定义 ==================== */

/**
 * @brief 设备地址定义
 *
 * 地址用于标识通信的目标设备：
 * - 0x00为广播地址，发送给所有设备
 * - 0x01-0x06对应6个关节电机
 * - 0x07为夹爪控制器
 * - 0x08为系统控制器（STM32本身）
 */
#define ADDR_BROADCAST      0x00    /**< 广播地址 - 发送给所有设备 */
#define ADDR_JOINT_1        0x01    /**< 关节1电机 */
#define ADDR_JOINT_2        0x02    /**< 关节2电机 */
#define ADDR_JOINT_3        0x03    /**< 关节3电机 */
#define ADDR_JOINT_4        0x04    /**< 关节4电机 */
#define ADDR_JOINT_5        0x05    /**< 关节5电机 */
#define ADDR_JOINT_6        0x06    /**< 关节6电机 */
#define ADDR_GRIPPER        0x07    /**< 夹爪控制器 */
#define ADDR_SYSTEM         0x08    /**< 系统控制器(STM32) */


/* ==================== 功能码定义 ==================== */

/**
 * @brief 功能码 - 配置命令 (0x01-0x03)
 */
#define FUNC_SET_GEAR_RATIO     0x01    /**< 设置减速比 - 为指定关节设置减速比(float32, 0.1~100.0) */
#define FUNC_SET_ZERO_POS       0x02    /**< 设置零位 - 将当前位置设为物理零位 */
#define FUNC_SAVE_CONFIG        0x03    /**< 保存配置 - 将参数写入Flash */
#define FUNC_SET_LIMIT_POS      0x04    /**< 设置极限位置 - 将当前位置设为关节极限位置 */
#define FUNC_READ_LIMIT_POS    0x05    /**< 读取极限位置 - 读取关节极限位置及设置标志 */
#define FUNC_SET_PROTECTION    0x06    /**< 设置位置保护 - 开启/关闭位置保护 */
#define FUNC_RESET_POSITIONS   0x07    /**< 重置位置 - 清零零位和极限位置，关闭保护 */

/**
 * @brief 功能码 - 运动控制命令 (0x20-0x2F)
 */
#define FUNC_JOINT_JOG          0x20    /**< 关节微动 - 正向/反向微动指定角度 */
#define FUNC_HOMING             0x21    /**< 一键回零 - 单关节或全部关节回零 */
#define FUNC_JOINT_POSITION     0x22    /**< 关节位置控制 - 移动到目标角度 */
#define FUNC_GRIPPER_PWM        0x27    /**< 夹爪PWM控制 - PWM控制夹爪开合 */
#define FUNC_EMERGENCY_STOP     0x2F    /**< 急停 - 立即停止所有/指定关节 */

/**
 * @brief 功能码 - 状态读取命令 (0x41-0x47)
 */
#define FUNC_READ_SINGLE_STATUS 0x41    /**< 读单关节状态 - 读取指定关节详细状态 */
#define FUNC_READ_FULL_STATUS    0x42    /**< 读全量状态 - 读取所有关节+夹爪+系统状态 */
#define FUNC_READ_JOINT_ANGLE   0x43    /**< 读关节角度 - 读取指定关节当前角度 */
#define FUNC_READ_GRIPPER_STATUS 0x47   /**< 读夹爪状态 - 读取夹爪当前状态 */

/**
 * @brief 功能码 - 异常上报 (0x60)
 */
#define FUNC_EXCEPTION_REPORT   0x60    /**< 异常上报 - 主动向上位机报告异常 */

/**
 * @brief 功能码 - 电机透传 (0xF0-0xFF)
 * @note 直接转发数据到电机，用于调试
 */
#define FUNC_MOTOR_PASSTHROUGH_BASE 0xF0   /**< 电机透传起始码 */
#define FUNC_MOTOR_PASSTHROUGH_MAX  0xFF   /**< 电机透传结束码 */


/* ==================== 状态码定义 ==================== */

/**
 * @brief 命令执行状态码
 *
 * 状态码在响应帧的数据域第一个字节中返回，
 * 告诉上位机命令执行的结果
 */
#define STATUS_SUCCESS          0x00    /**< 成功 - 命令执行成功 */
#define STATUS_PARAM_ERROR      0x01    /**< 参数错误 - 参数不合法或超出范围 */
#define STATUS_EXEC_FAILED     0x02    /**< 执行失败 - 命令执行过程中出错 */
#define STATUS_CRC_ERROR       0x03    /**< CRC错误 - 数据校验失败 */
#define STATUS_NOT_READY       0x04    /**< 设备未就绪 - 设备未准备好 */


/* ==================== 异常类型定义 ==================== */

/**
 * @brief 异常类型定义
 *
 * 当发生异常时，STM32会主动向上位机发送异常帧，
 * 包含异常类型和异常参数，便于上位机进行错误处理和显示
 */
#define EXCEPT_MOTOR_STALL      0x01    /**< 电机堵转 */
#define EXCEPT_MOTOR_UNDERVOLT  0x02    /**< 电机欠压 */
#define EXCEPT_UART_ERROR       0x03    /**< UART通信错误 */
#define EXCEPT_SYS_UNDERVOLT    0x04    /**< 控制器欠压 */
#define EXCEPT_JOINT_OUT_RANGE  0x05    /**< 关节运动超限 */


/* ==================== 帧结构定义 ==================== */

/**
 * @brief 最小帧长度
 * @note 最小帧 = 帧头(2) + 地址(1) + 功能码(1) + 长度(1) + CRC(1) + 帧尾(2) = 8字节
 */
#define FRAME_MIN_LENGTH        8

/**
 * @brief 最大数据长度
 * @note 单帧最大支持54字节数据
 */
#define FRAME_MAX_DATA_LENGTH   54

/**
 * @brief 最大帧长度
 * @note 最大帧 = 帧头(2) + 地址(1) + 功能码(1) + 长度(1) + 数据(54) + CRC(1) + 帧尾(2) = 62字节
 */
#define FRAME_MAX_LENGTH        64


/**
 * @brief 协议帧结构体
 *
 * 解析后的完整帧数据，包含帧的各个组成部分
 *
 * @param address 目标设备地址 (0x00-0x08)
 * @param function_code 功能码，标识命令类型
 * @param data_length 数据域的实际长度
 * @param data 数据域内容
 * @param crc8 CRC8校验值
 */
typedef struct {
    uint8_t address;                    /**< 设备地址 */
    uint8_t function_code;               /**< 功能码 */
    uint8_t data_length;                 /**< 数据长度 */
    uint8_t data[FRAME_MAX_DATA_LENGTH]; /**< 数据域 */
    uint8_t crc8;                       /**< CRC校验码 */
} ProtocolFrame;


/**
 * @brief 帧解析状态机状态
 *
 * 帧解析是一个状态机过程，通过识别帧的不同部分来解析完整帧：
 * - WAIT_HEADER1: 等待帧头第1字节(0xAA)
 * - WAIT_HEADER2: 等待帧头第2字节(0x55)
 * - ADDR: 读取地址字节
 * - FUNC: 读取功能码字节
 * - LEN: 读取数据长度字节
 * - DATA: 读取数据域
 * - CRC: 读取CRC校验字节
 * - TAIL1: 等待帧尾第1字节(0x55)
 * - TAIL2: 等待帧尾第2字节(0xAA)，收到后帧解析完成
 */
typedef enum {
    FRAME_STATE_WAIT_HEADER1,   /**< 等待帧头1 */
    FRAME_STATE_WAIT_HEADER2,    /**< 等待帧头2 */
    FRAME_STATE_ADDR,            /**< 读取地址 */
    FRAME_STATE_FUNC,            /**< 读取功能码 */
    FRAME_STATE_LEN,             /**< 读取数据长度 */
    FRAME_STATE_DATA,            /**< 读取数据 */
    FRAME_STATE_CRC,             /**< 读取CRC */
    FRAME_STATE_TAIL1,           /**< 等待帧尾1 */
    FRAME_STATE_TAIL2            /**< 等待帧尾2 */
} FrameParseState;


/**
 * @brief 帧解析器结构体
 *
 * 包含解析过程中的所有状态和数据
 *
 * @param state 当前解析状态
 * @param buffer 原始数据缓冲
 * @param index 当前写入位置
 * @param frame 解析后的帧数据
 */
typedef struct {
    FrameParseState state;               /**< 当前解析状态 */
    uint8_t buffer[FRAME_MAX_LENGTH];   /**< 原始数据缓冲 */
    uint8_t index;                       /**< 当前缓冲写入位置 */
    ProtocolFrame frame;                 /**< 解析出的帧数据 */
} FrameParser;


/* ==================== 函数声明 ==================== */

/**
 * @brief 计算CRC8校验值
 * @param data 要计算校验的数据指针
 * @param length 数据长度
 * @return CRC8校验值
 *
 * @note 使用多项式0x31 (x^8 + x^5 + x^4 + 1)，初始值0x00
 *       校验范围：地址 + 功能码 + 数据长度 + 数据
 */
uint8_t CRC8_Calculate(const uint8_t *data, uint16_t length);

/**
 * @brief 初始化帧解析器
 * @param parser 指向帧解析器的指针
 *
 * @note 解析器初始化后处于WAIT_HEADER1状态
 */
void FrameParser_Init(FrameParser *parser);

/**
 * @brief 处理接收到的单个字节
 * @param parser 指向帧解析器的指针
 * @param byte 接收到的字节
 * @return 0 继续解析
 * @return 1 完整帧解析完成
 * @return -1 解析错误
 *
 * @note 每接收到一个字节调用此函数，函数内部根据当前状态
 *       决定如何处理该字节，并更新解析状态
 */
int FrameParser_ProcessByte(FrameParser *parser, uint8_t byte);

/**
 * @brief 获取解析完成的帧并进行CRC校验
 * @param parser 指向帧解析器的指针
 * @param frame 指向存储解析结果的帧结构体的指针
 * @return 0 成功
 * @return -1 CRC校验失败
 *
 * @note 在FrameParser_ProcessByte返回1后调用此函数获取帧数据
 */
int FrameParser_GetFrame(FrameParser *parser, ProtocolFrame *frame);

/**
 * @brief 构建响应帧
 * @param address 目标设备地址
 * @param function_code 功能码
 * @param status 执行状态码
 * @param data 响应数据（可为NULL）
 * @param data_len 数据长度
 * @param output 输出缓冲区
 * @param output_len 输出数据长度
 * @return 0 成功
 * @return -1 失败（数据过长）
 *
 * @note 自动添加帧头、帧尾和CRC校验
 */
int Frame_BuildResponse(uint8_t address, uint8_t function_code,
                        uint8_t status, const uint8_t *data, uint8_t data_len,
                        uint8_t *output, uint8_t *output_len);

/**
 * @brief 构建异常上报帧
 * @param address 发生异常的设备地址
 * @param except_type 异常类型 (EXCEPT_*)
 * @param except_param 异常参数
 * @param output 输出缓冲区
 * @param output_len 输出数据长度
 * @return 0 成功
 *
 * @note 异常帧使用功能码0x60
 */
int Frame_BuildException(uint8_t address, uint8_t except_type,
                         uint16_t except_param,
                         uint8_t *output, uint8_t *output_len);


/* ==================== 数据类型转换函数 ==================== */

/**
 * @brief 从字节数组读取uint16_t（小端序）
 * @param data 字节数组指针
 * @return 解析出的uint16_t值
 *
 * @note 低字节在前，高字节在后（Little-Endian）
 *       data[0]为低8位，data[1]为高8位
 */
uint16_t Protocol_ReadU16(const uint8_t *data);

/**
 * @brief 从字节数组读取int16_t（小端序）
 * @param data 字节数组指针
 * @return 解析出的int16_t值
 */
int16_t Protocol_ReadI16(const uint8_t *data);

/**
 * @brief 从字节数组读取uint32_t（小端序）
 * @param data 字节数组指针
 * @return 解析出的uint32_t值
 */
uint32_t Protocol_ReadU32(const uint8_t *data);

/**
 * @brief 从字节数组读取int32_t（小端序）
 * @param data 字节数组指针
 * @return 解析出的int32_t值
 */
int32_t Protocol_ReadI32(const uint8_t *data);

/**
 * @brief 从字节数组读取float（小端序）
 * @param data 字节数组指针
 * @return 解析出的float值
 *
 * @note 使用union进行float和uint32_t的类型转换
 */
float Protocol_ReadFloat(const uint8_t *data);

/**
 * @brief 将uint16_t写入字节数组（小端序）
 * @param value 要写入的uint16_t值
 * @param data 目标字节数组指针
 */
void Protocol_WriteU16(uint16_t value, uint8_t *data);

/**
 * @brief 将int16_t写入字节数组（小端序）
 * @param value 要写入的int16_t值
 * @param data 目标字节数组指针
 */
void Protocol_WriteI16(int16_t value, uint8_t *data);

/**
 * @brief 将uint32_t写入字节数组（小端序）
 * @param value 要写入的uint32_t值
 * @param data 目标字节数组指针
 */
void Protocol_WriteU32(uint32_t value, uint8_t *data);

/**
 * @brief 将int32_t写入字节数组（小端序）
 * @param value 要写入的int32_t值
 * @param data 目标字节数组指针
 */
void Protocol_WriteI32(int32_t value, uint8_t *data);

/**
 * @brief 将float写入字节数组（小端序）
 * @param value 要写入的float值
 * @param data 目标字节数组指针
 */
void Protocol_WriteFloat(float value, uint8_t *data);


/* ==================== 地址验证函数 ==================== */

/**
 * @brief 检查是否为有效的关节地址
 * @param address 要检查的地址
 * @return true 是有效的关节地址(1-6)
 * @return false 不是有效的关节地址
 */
bool Protocol_IsValidJointAddress(uint8_t address);

/**
 * @brief 检查是否为有效的设备地址
 * @param address 要检查的地址
 * @return true 是有效的设备地址
 * @return false 不是有效的设备地址
 *
 * @note 有效地址包括：0x00(广播), 0x01-0x06(关节), 0x07(夹爪), 0x08(系统)
 */
bool Protocol_IsValidAddress(uint8_t address);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
