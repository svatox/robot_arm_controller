/**
  ******************************************************************************
  * @file    emm_v5.h
  * @brief   Emm_V5.0闭环步进电机驱动协议
  *
  * @details 本模块实现了与Emm_V5.0闭环步进电机驱动器的通信协议。
  *         参考电机厂商示例程序实现
  *
  * @note Emm_V5.0通信格式（来自厂商示例）：
  *       命令: [ADDR] [CMD] [PARAM...] [0x6B]
  *       响应: [ADDR] [CMD] [DATA...] [0x6B]
  ******************************************************************************
  */

#ifndef __EMM_V5_H
#define __EMM_V5_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


/* ==================== 命令码定义 ==================== */

/**
 * @brief Emm_V5.0命令码（来自厂商示例）
 */
#define EMM_CMD_RESET_POS         0x0A    /**< 将当前位置清零 */
#define EMM_CMD_RESET_CLOG       0x0E    /**< 解除堵转保护 */
#define EMM_CMD_READ_VER         0x1F    /**< 读取固件版本 */
#define EMM_CMD_READ_RL          0x20    /**< 读取相电阻和相电感 */
#define EMM_CMD_READ_PID         0x21    /**< 读取PID参数 */
#define EMM_CMD_READ_VBUS        0x24    /**< 读取总线电压 */
#define EMM_CMD_READ_CPHA        0x27    /**< 读取相电流 */
#define EMM_CMD_READ_ENCL        0x31    /**< 读取编码器值 */
#define EMM_CMD_READ_TPOS        0x33    /**< 读取目标位置 */
#define EMM_CMD_READ_VEL         0x35    /**< 读取实时转速 */
#define EMM_CMD_READ_CPOS        0x36    /**< 读取实时位置角度 */
#define EMM_CMD_READ_PERR        0x37    /**< 读取位置误差角度 */
#define EMM_CMD_READ_FLAG        0x3A    /**< 读取状态标志位 */
#define EMM_CMD_READ_ORG         0x3B    /**< 读取回零状态 */
#define EMM_CMD_READ_CONF        0x42    /**< 读取驱动参数 */
#define EMM_CMD_READ_STATE       0x43    /**< 读取系统状态 */
#define EMM_CMD_SET_CTRL_MODE    0x46    /**< 修改开环/闭环控制模式 */
#define EMM_CMD_ENABLE          0xF3    /**< 使能/禁能电机 */
#define EMM_CMD_REL_POS         0xF5    /**< 速度模式 */
#define EMM_CMD_POSITION        0xFD    /**< 位置模式 */
#define EMM_CMD_STOP            0xFE    /**< 立即停止 */
#define EMM_CMD_SYNC            0xFF    /**< 触发多机同步 */
#define EMM_CMD_ORIGIN_SET      0x93    /**< 设置零点位置 */
#define EMM_CMD_ORIGIN_PARAMS   0x4C    /**< 修改回零参数 */
#define EMM_CMD_ORIGIN_TRIGGER  0x9A    /**< 触发回零 */
#define EMM_CMD_ORIGIN_STOP     0x9C    /**< 强制中断回零 */
#define EMM_CMD_MODIFY_PID      0x4A    /**< 修改PID参数(Emm) */
#define EMM_CMD_SPEED_SCALE     0x4F    /**< 修改速度缩放(Emm) */

#define EMM_FRAME_END            0x6B    /**< 帧结束符 */


/* ==================== 电机参数 ==================== */

/**
 * @brief 电机参数常量
 *
 * @note 电机分辨率为3200脉冲/转（1.8°步距角 × 16细分）
 *       每脉冲对应角度 = 360° / 3200 = 0.1125°
 */
#define EMM_PULSES_PER_REV      3200        /**< 每转脉冲数 */
#define EMM_DEG_PER_PULSE       0.1125f     /**< 每脉冲度数 */


/* ==================== 数据结构 ==================== */

/**
 * @brief 电机状态结构体
 */
typedef struct {
    int32_t  position;          /**< 实时位置角度(脉冲) */
    int16_t  speed;             /**< 实时转速(RPM) */
    uint16_t current;          /**< 相电流(mA) */
    uint16_t voltage;          /**< 总线电压(mV) */
    float    pos_error;        /**< 位置误差(度) */
    uint8_t  enabled;         /**< 使能状态 */
    uint8_t  stall_flag;       /**< 堵转标志 */
    uint8_t  uv_flag;          /**< 欠压标志 */
} EmmMotorStatus;


/**
 * @brief 响应数据结构体
 */
typedef struct {
    bool    success;            /**< 是否成功 */
    uint8_t address;            /**< 电机地址 */
    uint8_t cmd_code;          /**< 命令码 */
    int32_t position;          /**< 位置 */
    int16_t speed;             /**< 速度 */
    uint16_t current;          /**< 电流 */
    uint8_t status;            /**< 状态 */
    uint16_t voltage;          /**< 电压 */
    uint8_t error_code;        /**< 错误码 */
} EmmResponse;


/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化Emm_V5驱动
 */
void EmmV5_Init(void);


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
int EmmV5_BuildEnableCmd(uint8_t address, uint8_t enable, uint8_t sync_flag, uint8_t *output);

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
                            uint32_t pulses, uint8_t motion_mode, uint8_t sync_flag, uint8_t *output);

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
int EmmV5_BuildSpeedCmd(uint8_t address, uint8_t dir, uint16_t speed, uint8_t acc, uint8_t sync_flag, uint8_t *output);

/**
 * @brief 构建立即停止命令
 * @param address 电机地址 (1-6)
 * @param sync_flag 多机同步标志
 * @param output 输出缓冲区(至少5字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0xFE] [0x98] [SNF] [0x6B]
 */
int EmmV5_BuildStopCmd(uint8_t address, uint8_t sync_flag, uint8_t *output);

/**
 * @brief 构建同步触发命令
 * @param output 输出缓冲区(至少4字节)
 * @return 命令长度
 *
 * @note 命令格式: [0xFF] [0x66] [0x6B]
 */
int EmmV5_BuildSyncTriggerCmd(uint8_t *output);

/**
 * @brief 构建将当前位置清零命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少4字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x0A] [0x6D] [0x6B]
 */
int EmmV5_BuildResetPosCmd(uint8_t address, uint8_t *output);

/**
 * @brief 构建解除堵转保护命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少4字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x0E] [0x52] [0x6B]
 */
int EmmV5_BuildResetClogCmd(uint8_t address, uint8_t *output);


/**
 * @brief 构建读取实时位置命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少3字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x36] [0x6B]
 */
int EmmV5_BuildReadPosCmd(uint8_t address, uint8_t *output);

/**
 * @brief 构建读取实时转速命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少3字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x35] [0x6B]
 */
int EmmV5_BuildReadSpeedCmd(uint8_t address, uint8_t *output);

/**
 * @brief 构建读取电机状态命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少3字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x3A] [0x6B]
 *       返回: [ADDR] [CMD] [POS(4)] [SPEED(2)] [CURRENT(2)] [STATUS(1)] [VOLTAGE(2)] [0x6B]
 */
int EmmV5_BuildReadStatusCmd(uint8_t address, uint8_t *output);

/**
 * @brief 构建读取总线电压命令
 * @param address 电机地址 (1-6)
 * @param output 输出缓冲区(至少3字节)
 * @return 命令长度
 *
 * @note 命令格式: [ADDR] [0x24] [0x6B]
 */
int EmmV5_BuildReadVoltageCmd(uint8_t address, uint8_t *output);

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
int EmmV5_BuildOriginTriggerCmd(uint8_t address, uint8_t mode, uint8_t sync_flag, uint8_t *output);

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
                            uint32_t kp, uint32_t ki, uint32_t kd, uint8_t *output);

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
int EmmV5_BuildSpeedScaleCmd(uint8_t address, uint8_t save_flag, uint8_t scale_enable, uint8_t *output);


/* ==================== 通信函数 ==================== */

/**
 * @brief 发送命令（不等待响应）
 * @param cmd 命令数据
 * @param len 命令长度
 */
void EmmV5_SendCommand(const uint8_t *cmd, uint8_t len);

/**
 * @brief 发送命令并等待响应（轮询方式）
 * @param cmd 命令数据
 * @param cmd_len 命令长度
 * @param response 响应数据缓冲区
 * @param resp_len 响应数据长度
 * @param timeout_ms 超时时间(毫秒)
 * @return true 成功
 */
bool EmmV5_SendAndWait(const uint8_t *cmd, uint8_t cmd_len,
                       uint8_t *response, uint8_t *resp_len, uint32_t timeout_ms);

/**
 * @brief 阻塞式接收响应
 * @param buffer 接收缓冲区
 * @param len 缓冲区长度
 * @param timeout_ms 超时时间(毫秒)
 * @return true 成功
 */
bool EmmV5_ReceiveResponse(uint8_t *buffer, uint8_t *len, uint32_t timeout_ms);

/**
 * @brief 阻塞式接收响应（旧版API兼容）
 * @param buffer 接收缓冲区
 * @param len 缓冲区长度(输入:最大长度, 输出:实际接收长度)
 * @param timeout_ms 超时时间(毫秒)
 * @return true 成功
 */
bool EmmV5_ReadResponse(uint8_t *buffer, uint8_t *len, uint32_t timeout_ms);


/* ==================== 电机控制高层函数 ==================== */

/**
 * @brief 使能/禁能电机
 * @param address 电机地址 (1-6)
 * @param enable 使能状态 (0=关闭, 1=使能)
 * @return true 成功
 */
bool EmmV5_EnableMotor(uint8_t address, uint8_t enable);

/**
 * @brief 移动到绝对位置
 * @param address 电机地址 (1-6)
 * @param pulses 目标脉冲数(绝对位置)
 * @param speed 速度(RPM), 范围0-3000(Emm)
 * @param acc 加速度
 * @param sync 同步标志 (0=立即执行, 1=等待同步触发)
 * @return true 成功
 */
bool EmmV5_MoveToPosition(uint8_t address, int32_t pulses, uint16_t speed, uint8_t acc, uint8_t sync);

/**
 * @brief 相对位置移动（相对上一目标位置）
 * @param address 电机地址 (1-6)
 * @param pulses 相对脉冲数(正=正向, 负=反向)
 * @param speed 速度(RPM), 范围0-3000(Emm)
 * @param acc 加速度
 * @param sync 同步标志
 * @return true 成功
 */
bool EmmV5_MoveRelative(uint8_t address, int32_t pulses, uint16_t speed, uint8_t acc, uint8_t sync);

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
bool EmmV5_MoveRelativeCurrent(uint8_t address, int32_t pulses, uint16_t speed, uint8_t acc, uint8_t sync);

/**
 * @brief 读取当前位置
 * @param address 电机地址 (1-6)
 * @param position 指向存储位置的指针
 * @return true 成功
 */
bool EmmV5_GetPosition(uint8_t address, int32_t *position);

/**
 * @brief 读取完整状态
 * @param address 电机地址 (1-6)
 * @param status 指向存储状态的指针
 * @return true 成功
 */
bool EmmV5_GetStatus(uint8_t address, EmmMotorStatus *status);

/**
 * @brief 触发同步运动
 */
void EmmV5_TriggerSync(void);

/**
 * @brief 立即停止
 * @param address 电机地址 (1-6)
 * @return true 成功
 */
bool EmmV5_StopMotor(uint8_t address);

/**
 * @brief 将当前位置清零
 * @param address 电机地址 (1-6)
 * @return true 成功
 */
bool EmmV5_ResetPosition(uint8_t address);

/**
 * @brief 解除堵转保护
 * @param address 电机地址 (1-6)
 * @return true 成功
 */
bool EmmV5_ResetClog(uint8_t address);

/**
 * @brief 读取总线电压
 * @param address 电机地址 (1-6)
 * @param voltage 指向存储电压的指针(mV)
 * @return true 成功
 */
bool EmmV5_GetVoltage(uint8_t address, uint16_t *voltage);

/**
 * @brief 触发回零
 * @param address 电机地址 (1-6)
 * @param mode 回零模式
 * @param sync 同步标志
 * @return true 成功
 */
bool EmmV5_OriginTrigger(uint8_t address, uint8_t mode, uint8_t sync);

/**
 * @brief 修改PID参数(Emm)
 * @param address 电机地址 (1-6)
 * @param save_flag 是否存储 (0=不存储, 1=存储)
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @return true 成功
 */
bool EmmV5_ModifyPid(uint8_t address, uint8_t save_flag, uint32_t kp, uint32_t ki, uint32_t kd);

/**
 * @brief 修改速度缩放设置(Emm)
 * @param address 电机地址 (1-6)
 * @param save_flag 是否存储 (0=不存储, 1=存储)
 * @param scale_enable 速度缩放使能 (0=关闭, 1=使能缩放10倍)
 * @return true 成功
 *
 * @note 使能后速度精度到0.1RPM
 */
bool EmmV5_SetSpeedScale(uint8_t address, uint8_t save_flag, uint8_t scale_enable);


#ifdef __cplusplus
}
#endif

#endif /* __EMM_V5_H */
