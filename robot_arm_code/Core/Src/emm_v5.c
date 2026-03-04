/**
  ******************************************************************************
  * @file    emm_v5.c
  * @brief   Emm_V5.0闭环步进电机驱动实现
  *
  * @details 本文件实现了与Emm_V5.0闭环步进电机驱动器的通信协议。
  *         参考电机厂商示例程序实现。
  *
  * @note Emm_V5.0通信格式（来自厂商示例）：
  *       命令: [ADDR] [CMD] [PARAM...] [0x6B]
  *       响应: [ADDR] [CMD] [DATA...] [0x6B]
  *       无CRC校验，仅使用0x6B作为帧结束符
  ******************************************************************************
  */

#include "emm_v5.h"
#include "usart.h"
#include <string.h>

/* ==================== 常量定义 ==================== */

/** UART超时时间（毫秒）*/
#define EMM_UART_TIMEOUT_MS     100

/** UART缓冲区大小 */
#define EMM_UART_BUFSIZE        64


/* ==================== 静态变量 ==================== */

/** 发送缓冲区 */
static uint8_t tx_buffer[EMM_UART_BUFSIZE];

/** 接收缓冲区 */
static uint8_t rx_buffer[EMM_UART_BUFSIZE];


/* ==================== 命令构建函数 ==================== */

/**
 * @brief 构建使能命令
 * @param address 电机地址 (1-6)
 * @param enable 使能状态 (0=关闭, 1=使能)
 * @param sync_flag 多机同步标志 (0=不启用, 1=启用)
 * @param output 输出缓冲区(至少6字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0xF3] [0xAB] [ENABLE] [SYNC] [0x6B]
 */
int EmmV5_BuildEnableCmd(uint8_t address, uint8_t enable, uint8_t sync_flag, uint8_t *output)
{
    output[0] = address;               // 地址
    output[1] = 0xF3;                 // 功能码
    output[2] = 0xAB;                 // 辅助码
    output[3] = enable ? 0x01 : 0x00; // 使能状态
    output[4] = sync_flag;            // 多机同步标志
    output[5] = 0x6B;                 // 校验字节(结束符)

    return 6;
}

/**
 * @brief 构建位置模式命令
 * @param address 电机地址 (1-6)
 * @param dir 方向 (0=CW, 其他=CCW)
 * @param speed 速度(RPM), 范围0-3000(Emm)
 * @param acc 加速度, 范围0-255(0=直接启动)
 * @param pulses 脉冲数(位置)
 * @param motion_mode 运动模式 (0=相对上一目标, 1=绝对位置, 2=相对当前位置)
 * @param sync_flag 多机同步标志
 * @param output 输出缓冲区(至少13字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0xFD] [DIR] [VEL_H] [VEL_L] [ACC] [CLK4] [CLK3] [CLK2] [CLK1] [MODE] [SNF] [0x6B]
 *       motion_mode: 0=相对上一输入目标位置, 1=相对坐标零点绝对位置, 2=相对当前实时位置
 */
int EmmV5_BuildPositionCmd(uint8_t address, uint8_t dir, uint16_t speed, uint8_t acc,
                            uint32_t pulses, uint8_t motion_mode, uint8_t sync_flag, uint8_t *output)
{
    output[0] = address;                       // 地址
    output[1] = 0xFD;                         // 功能码
    output[2] = dir;                           // 方向
    output[3] = (uint8_t)(speed >> 8);         // 速度(RPM)高8位字节
    output[4] = (uint8_t)(speed >> 0);        // 速度(RPM)低8位字节
    output[5] = acc;                           // 加速度
    output[6] = (uint8_t)(pulses >> 24);      // 脉冲数(bit24 - bit31)
    output[7] = (uint8_t)(pulses >> 16);      // 脉冲数(bit16 - bit23)
    output[8] = (uint8_t)(pulses >> 8);       // 脉冲数(bit8  - bit15)
    output[9] = (uint8_t)(pulses >> 0);       // 脉冲数(bit0  - bit7 )
    output[10] = motion_mode;                  // 运动模式: 0=相对上一目标, 1=绝对位置, 2=相对当前位置
    output[11] = sync_flag;                    // 多机同步标志
    output[12] = 0x6B;                        // 校验字节(结束符)

    return 13;
}

/**
 * @brief 构建速度模式命令
 * @param address 电机地址 (1-6)
 * @param dir 方向 (0=CW, 其他=CCW)
 * @param speed 速度(RPM), 范围0-3000(Emm)
 * @param acc 加速度, 范围0-255
 * @param sync_flag 多机同步标志
 * @param output 输出缓冲区(至少8字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0xF6] [DIR] [VEL_H] [VEL_L] [ACC] [SNF] [0x6B]
 */
int EmmV5_BuildSpeedCmd(uint8_t address, uint8_t dir, uint16_t speed, uint8_t acc, uint8_t sync_flag, uint8_t *output)
{
    output[0] = address;                       // 地址
    output[1] = 0xF6;                         // 功能码
    output[2] = dir;                           // 方向
    output[3] = (uint8_t)(speed >> 8);         // 速度(RPM)高8位字节
    output[4] = (uint8_t)(speed >> 0);        // 速度(RPM)低8位字节
    output[5] = acc;                           // 加速度
    output[6] = sync_flag;                    // 多机同步标志
    output[7] = 0x6B;                        // 校验字节(结束符)

    return 8;
}

/**
 * @brief 构建立即停止命令
 * @param address 电机地址 (1-6)
 * @param sync_flag 多机同步标志
 * @param output 输出缓冲区(至少5字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0xFE] [0x98] [SNF] [0x6B]
 */
int EmmV5_BuildStopCmd(uint8_t address, uint8_t sync_flag, uint8_t *output)
{
    output[0] = address;               // 地址
    output[1] = 0xFE;                 // 功能码
    output[2] = 0x98;                 // 辅助码
    output[3] = sync_flag;            // 多机同步标志
    output[4] = 0x6B;                 // 校验字节(结束符)

    return 5;
}

/**
 * @brief 构建同步触发命令
 * @param output 输出缓冲区(至少4字节)
 * @return 命令长度
 *
 * @note 命令格式: [0xFF] [0x66] [0x6B]
 */
int EmmV5_BuildSyncTriggerCmd(uint8_t *output)
{
    output[0] = 0xFF;                 // 广播地址
    output[1] = 0x66;                 // 功能码
    output[2] = 0x6B;                 // 校验字节(结束符)

    return 3;
}

/**
 * @brief 构建将当前位置清零命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少4字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x0A] [0x6D] [0x6B]
 */
int EmmV5_BuildResetPosCmd(uint8_t address, uint8_t *output)
{
    output[0] = address;               // 地址
    output[1] = 0x0A;                 // 功能码
    output[2] = 0x6D;                 // 辅助码
    output[3] = 0x6B;                 // 校验字节(结束符)

    return 4;
}

/**
 * @brief 构建解除堵转保护命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少4字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x0E] [0x52] [0x6B]
 */
int EmmV5_BuildResetClogCmd(uint8_t address, uint8_t *output)
{
    output[0] = address;               // 地址
    output[1] = 0x0E;                 // 功能码
    output[2] = 0x52;                 // 辅助码
    output[3] = 0x6B;                 // 校验字节(结束符)

    return 4;
}

/**
 * @brief 构建读取实时位置命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少3字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x36] [0x6B]
 */
int EmmV5_BuildReadPosCmd(uint8_t address, uint8_t *output)
{
    output[0] = address;               // 地址
    output[1] = 0x36;                 // 功能码
    output[2] = 0x6B;                 // 校验字节(结束符)

    return 3;
}

/**
 * @brief 构建读取实时转速命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少3字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x35] [0x6B]
 */
int EmmV5_BuildReadSpeedCmd(uint8_t address, uint8_t *output)
{
    output[0] = address;               // 地址
    output[1] = 0x35;                 // 功能码
    output[2] = 0x6B;                 // 校验字节(结束符)

    return 3;
}

/**
 * @brief 构建读取电机状态命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少3字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x3A] [0x6B]
 *       返回: [ADDR] [CMD] [POS(4)] [SPEED(2)] [CURRENT(2)] [STATUS(1)] [VOLTAGE(2)] [0x6B]
 */
int EmmV5_BuildReadStatusCmd(uint8_t address, uint8_t *output)
{
    output[0] = address;               // 地址
    output[1] = 0x3A;                 // 功能码
    output[2] = 0x6B;                 // 校验字节(结束符)

    return 3;
}

/**
 * @brief 构建读取总线电压命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少3字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x24] [0x6B]
 */
int EmmV5_BuildReadVoltageCmd(uint8_t address, uint8_t *output)
{
    output[0] = address;               // 地址
    output[1] = 0x24;                 // 功能码
    output[2] = 0x6B;                 // 校验字节(结束符)

    return 3;
}

/**
 * @brief 构建触发回零命令
 * @param address 电机地址 (1-6)
 * @param mode 回零模式 (0=单圈就近, 1=单圈方向, 2=多圈无限位, 3=多圈有限位)
 * @param sync_flag 多机同步标志
 * @param output 输出缓冲区(至少5字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x9A] [MODE] [SNF] [0x6B]
 */
int EmmV5_BuildOriginTriggerCmd(uint8_t address, uint8_t mode, uint8_t sync_flag, uint8_t *output)
{
    output[0] = address;               // 地址
    output[1] = 0x9A;                 // 功能码
    output[2] = mode;                  // 回零模式
    output[3] = sync_flag;            // 多机同步标志
    output[4] = 0x6B;                 // 校验字节(结束符)

    return 5;
}

/**
 * @brief 构建修改PID参数命令(Emm)
 * @param address 电机地址 (1-6)
 * @param save_flag 是否存储 (0=不存储, 1=存储)
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output 输出缓冲区(至少18字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x4A] [0xC3] [SAVE] [Kp(4)] [Ki(4)] [Kd(4)] [0x6B]
 */
int EmmV5_BuildModifyPidCmd(uint8_t address, uint8_t save_flag,
                            uint32_t kp, uint32_t ki, uint32_t kd, uint8_t *output)
{
    output[0] = address;                       // 地址
    output[1] = 0x4A;                         // 功能码
    output[2] = 0xC3;                         // 辅助码
    output[3] = save_flag;                    // 是否存储
    output[4] = (uint8_t)(kp >> 24);         // Kp高8位
    output[5] = (uint8_t)(kp >> 16);         // Kp
    output[6] = (uint8_t)(kp >> 8);          // Kp
    output[7] = (uint8_t)(kp >> 0);          // Kp低8位
    output[8] = (uint8_t)(ki >> 24);         // Ki高8位
    output[9] = (uint8_t)(ki >> 16);         // Ki
    output[10] = (uint8_t)(ki >> 8);         // Ki
    output[11] = (uint8_t)(ki >> 0);         // Ki低8位
    output[12] = (uint8_t)(kd >> 24);        // Kd高8位
    output[13] = (uint8_t)(kd >> 16);        // Kd
    output[14] = (uint8_t)(kd >> 8);         // Kd
    output[15] = (uint8_t)(kd >> 0);         // Kd低8位
    output[16] = 0x6B;                        // 结束符

    return 17;
}

/**
 * @brief 构建修改速度缩放命令(Emm)
 * @param address 电机地址 (1-6)
 * @param save_flag 是否存储 (0=不存储, 1=存储)
 * @param scale_enable 速度缩放使能 (0=关闭, 1=使能缩放10倍)
 * @param output 输出缓冲区(至少6字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x4F] [0x71] [SAVE] [SCALE] [0x6B]
 *       使能后速度精度到0.1RPM
 */
int EmmV5_BuildSpeedScaleCmd(uint8_t address, uint8_t save_flag, uint8_t scale_enable, uint8_t *output)
{
    output[0] = address;               // 地址
    output[1] = 0x4F;                 // 功能码
    output[2] = 0x71;                 // 辅助码
    output[3] = save_flag;            // 是否存储
    output[4] = scale_enable;         // 速度缩放使能
    output[5] = 0x6B;                 // 结束符

    return 6;
}


/* ==================== 通信函数 ==================== */

/**
 * @brief 发送命令到电机（使用USART3）
 * @param cmd 命令数据
 * @param len 命令长度
 */
void EmmV5_SendCommand(const uint8_t *cmd, uint8_t len)
{
    // 使用HAL发送，厂商示例中使用usart_SendCmd
    for (uint8_t i = 0; i < len; i++) {
        // 直接操作DR寄存器发送字节
        HAL_UART_Transmit(&huart3, (uint8_t*)&cmd[i], 1, EMM_UART_TIMEOUT_MS);
    }
}

/**
 * @brief 发送命令并等待响应（轮询方式）
 * @param cmd 命令数据
 * @param cmd_len 命令长度
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 * @param timeout_ms 超时时间(毫秒)
 * @return true 成功
 *
 * @note 厂商示例中不验证CRC，只检查0x6B结束符
 */
bool EmmV5_SendAndWait(const uint8_t *cmd, uint8_t cmd_len,
                       uint8_t *response, uint8_t *resp_len, uint32_t timeout_ms)
{
    uint8_t rx_index = 0;
    uint32_t start_time = HAL_GetTick();
    uint8_t byte;

    // 1. 发送命令
    EmmV5_SendCommand(cmd, cmd_len);

    // 2. 循环接收响应数据
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        // 尝试读取一个字节
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, &byte, 1, 10);

        if (status == HAL_OK) {
            // 收到数据，放入缓冲区
            response[rx_index++] = byte;

            // 检查是否收到结束符0x6B
            if (byte == 0x6B && rx_index >= 4) {
                break;  // 收到结束符，退出接收循环
            }

            // 防止缓冲区溢出
            if (rx_index >= EMM_UART_BUFSIZE) {
                return false;
            }
        }
    }

    // 3. 检查是否收到数据
    if (rx_index < 4) {
        return false;
    }

    // 4. 验证结束字节
    if (response[rx_index - 1] != 0x6B) {
        return false;
    }

    // 5. 返回接收到的数据长度
    *resp_len = rx_index;

    return true;
}

/**
 * @brief 阻塞式接收响应
 * @param buffer 接收缓冲区
 * @param len 缓冲区长度
 * @param timeout_ms 超时时间(毫秒)
 * @return true 成功
 */
bool EmmV5_ReceiveResponse(uint8_t *buffer, uint8_t *len, uint32_t timeout_ms)
{
    uint8_t rx_index = 0;
    uint32_t start_time = HAL_GetTick();
    uint8_t byte;

    while ((HAL_GetTick() - start_time) < timeout_ms) {
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, &byte, 1, 10);

        if (status == HAL_OK) {
            buffer[rx_index++] = byte;

            if (byte == 0x6B && rx_index >= 4) {
                break;
            }

            if (rx_index >= EMM_UART_BUFSIZE) {
                return false;
            }
        }
    }

    if (rx_index < 4 || buffer[rx_index - 1] != 0x6B) {
        return false;
    }

    *len = rx_index;
    return true;
}

/**
 * @brief 阻塞式接收响应（旧版API兼容）
 * @param buffer 接收缓冲区
 * @param len 缓冲区长度(输入:最大长度, 输出:实际接收长度)
 * @param timeout_ms 超时时间(毫秒)
 * @return true 成功
 */
bool EmmV5_ReadResponse(uint8_t *buffer, uint8_t *len, uint32_t timeout_ms)
{
    return EmmV5_ReceiveResponse(buffer, len, timeout_ms);
}


/* ==================== 电机控制高层函数 ==================== */

/**
 * @brief 使能/禁能电机
 * @param address 电机地址 (1-6)
 * @param enable 使能状态 (0=关闭, 1=使能)
 * @return true 成功
 */
bool EmmV5_EnableMotor(uint8_t address, uint8_t enable)
{
    uint8_t len = EmmV5_BuildEnableCmd(address, enable, 0, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        // 验证响应地址和命令码
        if (response[0] == address && response[1] == 0xF3) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 移动到绝对位置
 * @param address 电机地址 (1-6)
 * @param pulses 目标脉冲数(绝对位置)
 * @param speed 速度(RPM), 范围0-3000(Emm)
 * @param acc 加速度
 * @param sync 同步标志 (0=立即执行, 1=等待同步触发)
 * @return true 成功
 */
bool EmmV5_MoveToPosition(uint8_t address, int32_t pulses, uint16_t speed, uint8_t acc, uint8_t sync)
{
    uint8_t dir = (pulses >= 0) ? 0 : 1;  // 0=CW, 1=CCW
    uint32_t abs_pulses = (pulses >= 0) ? (uint32_t)pulses : (uint32_t)(-pulses);

    // 模式1: 相对坐标零点进行绝对位置运动
    uint8_t len = EmmV5_BuildPositionCmd(address, dir, speed, acc, abs_pulses, 1, sync, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0xFD) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 相对位置移动（相对上一目标位置）
 * @param address 电机地址 (1-6)
 * @param pulses 相对脉冲数(正=正向, 负=反向)
 * @param speed 速度(RPM), 范围0-3000(Emm)
 * @param acc 加速度
 * @param sync 同步标志
 * @return true 成功
 */
bool EmmV5_MoveRelative(uint8_t address, int32_t pulses, uint16_t speed, uint8_t acc, uint8_t sync)
{
    uint8_t dir = (pulses >= 0) ? 0 : 1;  // 0=CW, 1=CCW
    uint32_t abs_pulses = (pulses >= 0) ? (uint32_t)pulses : (uint32_t)(-pulses);

    // 模式0: 相对上一输入目标位置进行相对位置运动
    uint8_t len = EmmV5_BuildPositionCmd(address, dir, speed, acc, abs_pulses, 0, sync, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0xFD) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 相对当前位置移动
 * @param address 电机地址 (1-6)
 * @param pulses 相对脉冲数(正=正向, 负=反向)
 * @param speed 速度(RPM), 范围0-3000(Emm)
 * @param acc 加速度
 * @param sync 同步标志
 * @return true 成功
 *
 * @note 此函数相对于电机当前实时位置进行运动（模式2）
 */
bool EmmV5_MoveRelativeCurrent(uint8_t address, int32_t pulses, uint16_t speed, uint8_t acc, uint8_t sync)
{
    uint8_t dir = (pulses >= 0) ? 0 : 1;  // 0=CW, 1=CCW
    uint32_t abs_pulses = (pulses >= 0) ? (uint32_t)pulses : (uint32_t)(-pulses);

    // 模式2: 相对当前实时位置进行相对位置运动
    uint8_t len = EmmV5_BuildPositionCmd(address, dir, speed, acc, abs_pulses, 2, sync, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0xFD) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 读取当前位置
 * @param address 电机地址 (1-6)
 * @param position 指向存储位置的指针
 * @return true 成功
 */
bool EmmV5_GetPosition(uint8_t address, int32_t *position)
{
    uint8_t len = EmmV5_BuildReadPosCmd(address, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0x36 && resp_len >= 8) {
            // 解析位置: 4字节
            *position = ((int32_t)response[2] << 24) |
                        ((int32_t)response[3] << 16) |
                        ((int32_t)response[4] << 8) |
                        (int32_t)response[5];
            return true;
        }
    }
    return false;
}

/**
 * @brief 读取完整状态
 * @param address 电机地址 (1-6)
 * @param status 指向存储状态的指针
 * @return true 成功
 *
 * @note 响应格式: [ADDR] [CMD] [POS(4)] [SPEED(2)] [CURRENT(2)] [STATUS(1)] [VOLTAGE(2)] [0x6B]
 */
bool EmmV5_GetStatus(uint8_t address, EmmMotorStatus *status)
{
    uint8_t len = EmmV5_BuildReadStatusCmd(address, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0x3A && resp_len >= 14) {
            // 位置: 4字节
            status->position = ((int32_t)response[2] << 24) |
                                ((int32_t)response[3] << 16) |
                                ((int32_t)response[4] << 8) |
                                (int32_t)response[5];
            // 速度: 2字节，有符号
            status->speed = (int16_t)((response[6] << 8) | response[7]);
            // 电流: 2字节，无符号
            status->current = (uint16_t)((response[8] << 8) | response[9]);
            // 状态: 1字节
            status->enabled = response[10] & 0x01;
            status->stall_flag = (response[10] >> 1) & 0x01;
            status->uv_flag = (response[10] >> 2) & 0x01;
            // 电压: 2字节
            status->voltage = (uint16_t)((response[11] << 8) | response[12]);
            return true;
        }
    }
    return false;
}

/**
 * @brief 触发同步运动
 */
void EmmV5_TriggerSync(void)
{
    uint8_t len = EmmV5_BuildSyncTriggerCmd(tx_buffer);
    EmmV5_SendCommand(tx_buffer, len);
}

/**
 * @brief 立即停止
 * @param address 电机地址 (1-6)
 * @return true 成功
 */
bool EmmV5_StopMotor(uint8_t address)
{
    uint8_t len = EmmV5_BuildStopCmd(address, 0, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0xFE) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 将当前位置清零
 * @param address 电机地址 (1-6)
 * @return true 成功
 */
bool EmmV5_ResetPosition(uint8_t address)
{
    uint8_t len = EmmV5_BuildResetPosCmd(address, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0x0A) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 解除堵转保护
 * @param address 电机地址 (1-6)
 * @return true 成功
 */
bool EmmV5_ResetClog(uint8_t address)
{
    uint8_t len = EmmV5_BuildResetClogCmd(address, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0x0E) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 读取总线电压
 * @param address 电机地址 (1-6)
 * @param voltage 指向存储电压的指针(mV)
 * @return true 成功
 */
bool EmmV5_GetVoltage(uint8_t address, uint16_t *voltage)
{
    uint8_t len = EmmV5_BuildReadVoltageCmd(address, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0x24 && resp_len >= 6) {
            // 电压: 2字节
            *voltage = (uint16_t)((response[2] << 8) | response[3]);
            return true;
        }
    }
    return false;
}

/**
 * @brief 触发回零
 * @param address 电机地址 (1-6)
 * @param mode 回零模式
 * @param sync 同步标志
 * @return true 成功
 */
bool EmmV5_OriginTrigger(uint8_t address, uint8_t mode, uint8_t sync)
{
    uint8_t len = EmmV5_BuildOriginTriggerCmd(address, mode, sync, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0x9A) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 修改PID参数(Emm)
 * @param address 电机地址 (1-6)
 * @param save_flag 是否存储 (0=不存储, 1=存储)
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @return true 成功
 */
bool EmmV5_ModifyPid(uint8_t address, uint8_t save_flag, uint32_t kp, uint32_t ki, uint32_t kd)
{
    uint8_t len = EmmV5_BuildModifyPidCmd(address, save_flag, kp, ki, kd, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0x4A) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 修改速度缩放设置(Emm)
 * @param address 电机地址 (1-6)
 * @param save_flag 是否存储 (0=不存储, 1=存储)
 * @param scale_enable 速度缩放使能 (0=关闭, 1=使能缩放10倍)
 * @return true 成功
 *
 * @note 使能后速度精度到0.1RPM
 */
bool EmmV5_SetSpeedScale(uint8_t address, uint8_t save_flag, uint8_t scale_enable)
{
    uint8_t len = EmmV5_BuildSpeedScaleCmd(address, save_flag, scale_enable, tx_buffer);
    uint8_t resp_len;
    uint8_t response[EMM_UART_BUFSIZE];

    if (EmmV5_SendAndWait(tx_buffer, len, response, &resp_len, EMM_UART_TIMEOUT_MS)) {
        if (response[0] == address && response[1] == 0x4F) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 初始化Emm_V5驱动
 */
void EmmV5_Init(void)
{
    // USART3已经在HAL层初始化
    // 此函数可留作扩展使用
}
