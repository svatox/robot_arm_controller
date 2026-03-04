/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   通信协议实现 - 帧解析、CRC校验、数据类型转换
  *
  * @details 本文件实现了上位机与STM32之间的自定义二进制通信协议。
  *         包含帧解析状态机、CRC8校验、响应帧构建等功能。
  *
  * @note 协议帧格式：
  *       ┌──────┬──────┬──────┬──────┬────────┬─────┬──────┬──────┐
  *       │ 帧头 │ 地址 │ 功能码│ 数据长│ 数据   │ CRC │ 帧尾 │
  *       │ 0xAA │ 1B   │ 1B   │ 1B   │ N     │ 1B  │ 0x55AA│
  *       │ 0x55 │      │      │      │        │     │      │
  *       └──────┴──────┴──────┴──────┴────────┴─────┴──────┴──────┘
  *
  *       CRC8: 多项式0x31，初始值0x00
  *       校验范围: 地址 + 功能码 + 数据长度 + 数据
  ******************************************************************************
 */

#include "protocol.h"
#include <string.h>

/**
 * @brief CRC8查找表
 *
 * 使用查表法提高CRC计算效率
 * 多项式: 0x31 (x^8 + x^5 + x^4 + 1)
 * 初始值: 0x00
 */
static const uint8_t crc8_table[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
};

/**
 * @brief 计算CRC8校验值（查表法）
 * @param data 要计算校验的数据指针
 * @param length 数据长度
 * @return CRC8校验值
 *
 * @note 使用查表法，效率比直接计算高得多
 *       计算过程：crc = crc8_table[crc ^ data[i]]
 */
uint8_t CRC8_Calculate(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00;  // 初始值

    // 遍历每个字节
    for (uint16_t i = 0; i < length; i++) {
        // 查表得到新的CRC值
        crc = crc8_table[crc ^ data[i]];
    }

    return crc;
}

/**
 * @brief 初始化帧解析器
 * @param parser 指向帧解析器的指针
 *
 * @note 初始化后解析器处于WAIT_HEADER1状态
 *       等待接收帧头的第一个字节(0xAA)
 */
void FrameParser_Init(FrameParser *parser)
{
    // 清零解析器结构体
    memset(parser, 0, sizeof(FrameParser));

    // 设置初始状态为等待帧头
    parser->state = FRAME_STATE_WAIT_HEADER1;
}

/**
 * @brief 处理接收到的单个字节（帧解析状态机）
 * @param parser 指向帧解析器的指针
 * @param byte 接收到的字节
 * @return 0 继续解析
 * @return 1 完整帧解析完成
 * @return -1 解析错误
 *
 * @note 这是一个状态机，根据当前状态决定如何处理接收到的字节
 *       状态转换如下：
 *
 *       ┌─────────────┐  收到0xAA  ┌─────────────┐
 *       │WAIT_HEADER1│ ─────────► │WAIT_HEADER2 │
 *       └─────────────┘            └──────┬──────┘
 *          ▲                            │收到0x55
 *          │                            ▼
 *       ┌──┴──────────────┐  其他   ┌───────┐
 *       │  解析错误       │ ◄────── │  ADDR │

 *       └─────────────────┘         └───┬───┘
 *                                      │读取地址
 *                                      ▼
 *                                 ┌────────┐
 *                                 │  FUNC │ ────┐
 *                                 └────┬───┘     │
 *                                     │读取功能码   │
 *                                     ▼           │
 *                                 ┌────────┐     │
 *                                 │  LEN   │ ────┤
 *                                 └────┬───┘     │
 *                                     │读取长度   │
 *                          ┌──────────┼──────────┐│
 *                          ▼                    ▼│
 *                   ┌────────────┐     ┌────────────┐│
 *                   │ LEN == 0   │     │ LEN > 0    ││
 *                   │ (无数据)   │     │ (有数据)   ││
 *                   └─────┬──────┘     └──────┬─────┘│
 *                         │                   │      │
 *                         │          ┌────────┼────────┐│
 *                         │          ▼                 ││
 *                         │   ┌────────────┐          ││
 *                         │   │    DATA    │◄─┘      ││
 *                         │   │ (读取数据) │          ││
 *                         │   └─────┬──────┘          ││
 *                         │         │数据读完          ││
 *                         │         ▼                 ││
 *                         │   ┌────────┐               ││
 *                         │   │  CRC   │               ││
 *                         │   └────┬───┘               ││
 *                         │         │读取CRC           ││
 *                         │         ▼                  ││
 *                         │   ┌────────────┐           ││
 *                         └──►│  TAIL1/2   │           ││
 *                             └──────┬──────┘           ││
 *                                    │帧接收完成        ││
 *                                    ▼                 ││
 *                             返回 1 (帧完成) ◄────────┘│
 */
int FrameParser_ProcessByte(FrameParser *parser, uint8_t byte)
{
    switch (parser->state) {
        // 状态1: 等待帧头第1字节 (0xAA)
        case FRAME_STATE_WAIT_HEADER1:
            if (byte == FRAME_HEADER_1) {
                // 收到0xAA，记录到缓冲区，切换到下一状态
                parser->buffer[0] = byte;
                parser->state = FRAME_STATE_WAIT_HEADER2;
            }
            break;

        // 状态2: 等待帧头第2字节 (0x55)
        case FRAME_STATE_WAIT_HEADER2:
            if (byte == FRAME_HEADER_2) {
                // 收到0x55，帧头完整
                parser->buffer[1] = byte;
                parser->index = 2;  // 缓冲区索引重置
                parser->state = FRAME_STATE_ADDR;  // 读取地址
            } else {
                // 帧头不完整，回到等待状态
                parser->state = FRAME_STATE_WAIT_HEADER1;
            }
            break;

        // 状态3: 读取地址字节
        case FRAME_STATE_ADDR:
            parser->buffer[parser->index++] = byte;
            parser->frame.address = byte;
            parser->state = FRAME_STATE_FUNC;  // 读取功能码
            break;

        // 状态4: 读取功能码字节
        case FRAME_STATE_FUNC:
            parser->buffer[parser->index++] = byte;
            parser->frame.function_code = byte;
            parser->state = FRAME_STATE_LEN;  // 读取数据长度
            break;

        // 状态5: 读取数据长度字节
        case FRAME_STATE_LEN:
            parser->buffer[parser->index++] = byte;
            parser->frame.data_length = byte;

            // 检查数据长度是否合法
            if (byte > FRAME_MAX_DATA_LENGTH) {
                // 数据过长，解析错误
                parser->state = FRAME_STATE_WAIT_HEADER1;
                return -1;
            }

            // 根据数据长度决定下一状态
            if (byte == 0) {
                // 无数据，直接进入CRC校验
                parser->state = FRAME_STATE_CRC;
            } else {
                // 有数据，进入数据读取状态
                parser->state = FRAME_STATE_DATA;
            }
            break;

        // 状态6: 读取数据域
        case FRAME_STATE_DATA:
            // 记录原始数据到缓冲区
            parser->buffer[parser->index++] = byte;
            // 同时记录到frame的data数组（便于后续解析）
            // 注意：buffer中index-6的位置对应frame.data[0]
            parser->frame.data[parser->index - 6] = byte;

            /* 检查是否已读取完所有数据字节
             * 缓冲区中的位置计算：
             * header1(0) + header2(1) + addr(2) + func(3) + len(4) = 5
             * 当index = 5 + data_length 时，数据读取完成
             */
            if (parser->index >= parser->frame.data_length + 5) {
                parser->state = FRAME_STATE_CRC;
            }
            break;

        // 状态7: 读取CRC校验字节
        case FRAME_STATE_CRC:
            parser->buffer[parser->index++] = byte;
            parser->frame.crc8 = byte;
            parser->state = FRAME_STATE_TAIL1;  // 读取帧尾
            break;

        // 状态8: 等待帧尾第1字节 (0x55)
        case FRAME_STATE_TAIL1:
            if (byte == FRAME_TAIL_1) {
                parser->buffer[parser->index++] = byte;
                parser->state = FRAME_STATE_TAIL2;
            } else {
                // 帧尾错误，解析失败
                parser->state = FRAME_STATE_WAIT_HEADER1;
                return -1;
            }
            break;

        // 状态9: 等待帧尾第2字节 (0xAA)
        case FRAME_STATE_TAIL2:
            if (byte == FRAME_TAIL_2) {
                // 帧尾完整，帧解析成功
                parser->buffer[parser->index++] = byte;
                parser->state = FRAME_STATE_WAIT_HEADER1;
                return 1;  // 返回1表示完整帧已接收
            } else {
                // 帧尾错误，解析失败
                parser->state = FRAME_STATE_WAIT_HEADER1;
                return -1;
            }

        // 默认: 未知状态，回到初始状态
        default:
            parser->state = FRAME_STATE_WAIT_HEADER1;
            break;
    }

    return 0;  // 继续解析
}

/**
 * @brief 获取解析完成的帧并进行CRC校验
 * @param parser 指向帧解析器的指针
 * @param frame 指向存储解析结果的帧结构体的指针
 * @return 0 成功
 * @return -1 CRC校验失败
 *
 * @note 在FrameParser_ProcessByte返回1后调用此函数获取帧数据
 *       CRC校验范围：地址 + 功能码 + 数据长度 + 数据
 */
int FrameParser_GetFrame(FrameParser *parser, ProtocolFrame *frame)
{
    /* 计算校验字节
     * 从buffer[2]开始（跳过帧头）
     * 长度 = 地址(1) + 功能码(1) + 长度(1) + 数据(n)
     *      = 3 + data_length
     */
    uint8_t crc_data_len = 3 + parser->frame.data_length;

#if CHECKSUM_METHOD == 0
    // CRC8校验
    uint8_t calculated_checksum = CRC8_Calculate(&parser->buffer[2], crc_data_len);
#else
    // 固定校验字节
    uint8_t calculated_checksum = FIXED_CHECKSUM_BYTE;
#endif

    // 比较计算的校验字节和接收的校验字节
    if (calculated_checksum != parser->frame.crc8) {
        // 校验错误，返回失败
        return -1;
    }

    // 校验通过，复制帧数据到输出
    memcpy(frame, &parser->frame, sizeof(ProtocolFrame));
    return 0;
}

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
 *       响应帧格式：
 *       0xAA 0x55 + ADDR + FUNC + (LEN+1) + STATUS + [DATA] + CRC + 0x55 0xAA
 */
int Frame_BuildResponse(uint8_t address, uint8_t function_code,
                        uint8_t status, const uint8_t *data, uint8_t data_len,
                        uint8_t *output, uint8_t *output_len)
{
    // 检查数据长度是否超出限制
    if (data_len > FRAME_MAX_DATA_LENGTH) {
        return -1;
    }

    uint8_t idx = 0;

    // 添加帧头
    output[idx++] = FRAME_HEADER_1;
    output[idx++] = FRAME_HEADER_2;

    // 添加地址和功能码
    output[idx++] = address;
    output[idx++] = function_code;

    // 添加数据长度（实际数据 + 状态码）
    output[idx++] = data_len + 1;

    // 添加状态码
    output[idx++] = status;

    // 添加数据（如果有）
    for (uint8_t i = 0; i < data_len; i++) {
        output[idx++] = data[i];
    }

    /* 计算校验字节
     * 校验范围：地址 + 功能码 + 长度 + 状态 + 数据
     * 即从buffer[2]开始到status之后的所有字节
     */
#if CHECKSUM_METHOD == 0
    // CRC8校验
    uint8_t checksum = CRC8_Calculate(&output[2], idx - 2);
#else
    // 固定校验字节
    uint8_t checksum = FIXED_CHECKSUM_BYTE;
#endif
    output[idx++] = checksum;

    // 添加帧尾
    output[idx++] = FRAME_TAIL_1;
    output[idx++] = FRAME_TAIL_2;

    // 返回总长度
    *output_len = idx;
    return 0;
}

/**
 * @brief 构建异常上报帧
 * @param address 发生异常的设备地址
 * @param except_type 异常类型
 * @param except_param 异常参数
 * @param output 输出缓冲区
 * @param output_len 输出数据长度
 * @return 0 成功
 *
 * @note 异常帧使用功能码0x60，数据格式：
 *       异常类型(1B) + 异常参数低字节(1B) + 异常参数高字节(1B)
 */
int Frame_BuildException(uint8_t address, uint8_t except_type,
                         uint16_t except_param,
                         uint8_t *output, uint8_t *output_len)
{
    // 构造异常数据
    uint8_t data[3];
    data[0] = except_type;                                   // 异常类型
    data[1] = (uint8_t)(except_param & 0xFF);              // 参数低字节
    data[2] = (uint8_t)((except_param >> 8) & 0xFF);       // 参数高字节

    // 构建响应帧
    return Frame_BuildResponse(address, FUNC_EXCEPTION_REPORT,
                               STATUS_SUCCESS, data, 3, output, output_len);
}


/* ==================== 数据类型转换函数 ==================== */

/**
 * @brief 从字节数组读取uint16_t（小端序）
 * @param data 字节数组指针
 * @return 解析出的uint16_t值
 *
 * @note 低字节在前，高字节在后
 *       例：data[0]=0x34, data[1]=0x12 → 0x1234
 */
uint16_t Protocol_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/**
 * @brief 从字节数组读取int16_t（小端序）
 * @param data 字节数组指针
 * @return 解析出的int16_t值
 */
int16_t Protocol_ReadI16(const uint8_t *data)
{
    return (int16_t)Protocol_ReadU16(data);
}

/**
 * @brief 从字节数组读取uint32_t（小端序）
 * @param data 字节数组指针
 * @return 解析出的uint32_t值
 */
uint32_t Protocol_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

/**
 * @brief 从字节数组读取int32_t（小端序）
 * @param data 字节数组指针
 * @return 解析出的int32_t值
 */
int32_t Protocol_ReadI32(const uint8_t *data)
{
    return (int32_t)Protocol_ReadU32(data);
}

/**
 * @brief 从字节数组读取float（小端序）
 * @param data 字节数组指针
 * @return 解析出的float值
 *
 * @note 使用union进行float和uint32_t的类型转换
 *       这种方式可以避免浮点数的对齐问题
 */
float Protocol_ReadFloat(const uint8_t *data)
{
    union {
        uint32_t u32;
        float f;
    } converter;

    // 先读取为uint32_t
    converter.u32 = Protocol_ReadU32(data);

    // 再转换为float
    return converter.f;
}

/**
 * @brief 将uint16_t写入字节数组（小端序）
 * @param value 要写入的uint16_t值
 * @param data 目标字节数组指针
 */
void Protocol_WriteU16(uint16_t value, uint8_t *data)
{
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)((value >> 8) & 0xFF);
}

/**
 * @brief 将int16_t写入字节数组（小端序）
 * @param value 要写入的int16_t值
 * @param data 目标字节数组指针
 */
void Protocol_WriteI16(int16_t value, uint8_t *data)
{
    Protocol_WriteU16((uint16_t)value, data);
}

/**
 * @brief 将uint32_t写入字节数组（小端序）
 * @param value 要写入的uint32_t值
 * @param data 目标字节数组指针
 */
void Protocol_WriteU32(uint32_t value, uint8_t *data)
{
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)((value >> 8) & 0xFF);
    data[2] = (uint8_t)((value >> 16) & 0xFF);
    data[3] = (uint8_t)((value >> 24) & 0xFF);
}

/**
 * @brief 将int32_t写入字节数组（小端序）
 * @param value 要写入的int32_t值
 * @param data 目标字节数组指针
 */
void Protocol_WriteI32(int32_t value, uint8_t *data)
{
    Protocol_WriteU32((uint32_t)value, data);
}

/**
 * @brief 将float写入字节数组（小端序）
 * @param value 要写入的float值
 * @param data 目标字节数组指针
 *
 * @note 使用union进行float和uint32_t的类型转换
 */
void Protocol_WriteFloat(float value, uint8_t *data)
{
    union {
        uint32_t u32;
        float f;
    } converter;

    // 将float存入union
    converter.f = value;

    // 按uint32_t写入
    Protocol_WriteU32(converter.u32, data);
}


/* ==================== 地址验证函数 ==================== */

/**
 * @brief 检查是否为有效的关节地址
 * @param address 要检查的地址
 * @return true 是有效的关节地址(1-6)
 * @return false 不是有效的关节地址
 */
bool Protocol_IsValidJointAddress(uint8_t address)
{
    return (address >= ADDR_JOINT_1 && address <= ADDR_JOINT_6);
}

/**
 * @brief 检查是否为有效的设备地址
 * @param address 要检查的地址
 * @return true 是有效的设备地址
 * @return false 不是有效的设备地址
 */
bool Protocol_IsValidAddress(uint8_t address)
{
    return (address == ADDR_BROADCAST ||
            (address >= ADDR_JOINT_1 && address <= ADDR_JOINT_6) ||
            address == ADDR_GRIPPER ||
            address == ADDR_SYSTEM);
}
