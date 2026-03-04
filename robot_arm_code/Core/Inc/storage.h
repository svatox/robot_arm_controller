/**
  ******************************************************************************
  * @file    storage.h
  * @brief   Flash存储管理 - 配置参数持久化
  *
  * @details 本模块负责将机械臂的配置参数保存到STM32的Flash存储器中，
  *         实现掉电不丢失。包括减速比、零位脉冲、夹爪配置等。
  *
  * @note Flash存储布局（地址0x0800C000，共1KB）：
  *       ┌─────────────┬───────────┬────────────────────┐
  *       │ 区域        │ 偏移     │ 内容               │
  *       ├─────────────┼───────────┼────────────────────┤
  *       │ 减速比      │ 0x00     │ 6个关节×4字节     │
  *       │ 零位脉冲   │ 0x18     │ 6个关节×4字节     │
  *       │ 夹爪配置   │ 0x30     │ 8字节              │
  *       │ 校验码     │ 0x38     │ Magic Number       │
  *       │ 零位设置标志│ 0x48     │ 6字节              │
  *       │ 极限位置设置│ 0x4E     │ 6字节              │
  *       │ 极限位置   │ 0x54     │ 6个关节×4字节     │
  *       │ 位置保护   │ 0x6C     │ 1字节              │
  *       └─────────────┴───────────┴────────────────────┘
  ******************************************************************************
 */

#ifndef __STORAGE_H
#define __STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


/* ==================== Flash地址定义 ==================== */

/**
 * @brief Flash存储基地址
 * @note 使用STM32F103C8T6的第3个扇区，地址0x0800C000
 *       避开前128KB程序空间，从倒数第4KB开始
 */
#define FLASH_STORAGE_BASE      0x0800C000

/**
 * @brief Flash存储区大小（字节）
 */
#define FLASH_STORAGE_SIZE      1024    /**< 1KB存储区域 */

/**
 * @brief Flash页大小（STM32F103页大小为1KB）
 * @note 使用HAL库定义：stm32f1xx_hal_flash_ex.h 中的 FLASH_PAGE_SIZE (0x400 = 1024)
 */


/* ==================== 参数偏移地址 ==================== */

/**
 * @brief 各参数区域在Flash中的偏移地址
 *
 * - 减速比区: 0x00-0x17 (24字节，6关节×4字节)
 * - 零位脉冲: 0x18-0x2F (24字节，6关节×4字节)
 * - 夹爪配置: 0x30-0x37 (8字节)
 * - 校验码:   0x38-0x3B (4字节)
 * - 零位设置标志: 0x48-0x4D (6字节)
 * - 极限位置设置标志: 0x4E-0x53 (6字节)
 * - 极限位置: 0x54-0x6B (24字节，6关节×4字节)
 * - 位置保护开关: 0x6C (1字节)
 */
#define OFFSET_GEAR_RATIO       0       /**< 减速比偏移: 24字节 */
#define OFFSET_ZERO_POSITION    24      /**< 零位脉冲偏移: 24字节 */
#define OFFSET_GRIPPER_CONFIG   48      /**< 夹爪配置偏移: 8字节 */
#define OFFSET_CHECKSUM         56      /**< 校验码偏移: 4字节 */
#define OFFSET_ZERO_SET_FLAG    72      /**< 零位设置标志偏移: 6字节 */
#define OFFSET_LIMIT_SET_FLAG   78      /**< 极限位置设置标志偏移: 6字节 */
#define OFFSET_LIMIT_POSITION   84      /**< 极限位置偏移: 24字节 */
#define OFFSET_PROTECTION_SWITCH 108   /**< 位置保护开关偏移: 1字节 */


/* ==================== 存储校验 ==================== */

/**
 * @brief 存储有效标志
 * @note 写入0x5241("RA")表示存储数据有效
 */
#define STORAGE_VALID_MAGIC     0x5241  /**< "RA" - Robot Arm */


/* ==================== 数据结构 ==================== */

/**
 * @brief 存储头部信息
 *
 * @param magic 魔术字，用于验证存储有效性
 * @param version 固件版本号
 * @param write_count 写入次数统计
 */
typedef struct {
    uint16_t magic;           /**< 魔术字 */
    uint16_t version;         /**< 版本号 */
    uint32_t write_count;     /**< 写入次数 */
} StorageHeader;


/* ==================== 初始化函数 ==================== */

/**
 * @brief 初始化存储模块
 * @return true 存储有效，已加载配置
 * @return false 存储无效，使用默认配置
 *
 * @note 初始化时检查Flash中是否有有效配置：
 *       - 如果有，加载配置到内存缓存
 *       - 如果没有，使用默认值
 */
bool Storage_Init(void);


/* ==================== 减速比操作函数 ==================== */

/**
 * @brief 保存关节减速比
 * @param joint_id 关节ID (1-6)
 * @param ratio 减速比值
 * @return true 保存成功
 * @return false 参数无效
 *
 * @note 只更新内存缓存，需要调用Storage_SaveAllConfig()写入Flash
 */
bool Storage_SaveGearRatio(uint8_t joint_id, float ratio);

/**
 * @brief 加载关节减速比
 * @param joint_id 关节ID (1-6)
 * @param ratio 指向存储减速比的变量的指针
 * @return true 加载成功
 * @return false 参数无效
 *
 * @note 优先从内存缓存读取，如无缓存则从Flash读取
 */
bool Storage_LoadGearRatio(uint8_t joint_id, float *ratio);


/* ==================== 零位脉冲操作函数 ==================== */

/**
 * @brief 保存关节零位脉冲
 * @param joint_id 关节ID (1-6)
 * @param pulse 零位脉冲值
 * @return true 保存成功
 * @return false 参数无效
 *
 * @note 只更新内存缓存，需要调用Storage_SaveAllConfig()写入Flash
 */
bool Storage_SaveZeroPosition(uint8_t joint_id, int32_t pulse);

/**
 * @brief 加载关节零位脉冲
 * @param joint_id 关节ID (1-6)
 * @param pulse 指向存储零位脉冲的变量的指针
 * @return true 加载成功
 * @return false 参数无效
 */
bool Storage_LoadZeroPosition(uint8_t joint_id, int32_t *pulse);

/**
 * @brief 保存零位设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 设置标志 (0=未设置, 1=已设置)
 * @return true 保存成功
 */
bool Storage_SaveZeroSetFlag(uint8_t joint_id, uint8_t flag);

/**
 * @brief 加载零位设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 指向存储标志的指针
 * @return true 加载成功
 */
bool Storage_LoadZeroSetFlag(uint8_t joint_id, uint8_t *flag);

/**
 * @brief 保存极限位置
 * @param joint_id 关节ID (1-6)
 * @param position 极限位置脉冲数
 * @return true 保存成功
 */
bool Storage_SaveLimitPosition(uint8_t joint_id, int32_t position);

/**
 * @brief 加载极限位置
 * @param joint_id 关节ID (1-6)
 * @param position 指向存储极限位置的指针
 * @return true 加载成功
 */
bool Storage_LoadLimitPosition(uint8_t joint_id, int32_t *position);

/**
 * @brief 保存极限位置设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 设置标志 (0=未设置, 1=已设置)
 * @return true 保存成功
 */
bool Storage_SaveLimitSetFlag(uint8_t joint_id, uint8_t flag);

/**
 * @brief 加载极限位置设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 指向存储标志的指针
 * @return true 加载成功
 */
bool Storage_LoadLimitSetFlag(uint8_t joint_id, uint8_t *flag);

/**
 * @brief 保存位置保护开关状态
 * @param enable 开关状态 (0=关闭, 1=开启)
 * @return true 保存成功
 */
bool Storage_SaveProtectionSwitch(uint8_t enable);

/**
 * @brief 加载位置保护开关状态
 * @param enable 指向存储开关状态的指针
 * @return true 加载成功
 */
bool Storage_LoadProtectionSwitch(uint8_t *enable);


/* ==================== 夹爪配置操作函数 ==================== */

/**
 * @brief 保存夹爪PWM配置
 * @param pwm_min PWM最小值
 * @param pwm_max PWM最大值
 * @param pwm_center PWM中间值
 * @return true 保存成功
 *
 * @note 只更新内存缓存，需要调用Storage_SaveAllConfig()写入Flash
 */
bool Storage_SaveGripperConfig(uint8_t pwm_min, uint8_t pwm_max, uint8_t pwm_center);

/**
 * @brief 加载夹爪PWM配置
 * @param pwm_min 指向存储PWM最小值的指针
 * @param pwm_max 指向存储PWM最大值的指针
 * @param pwm_center 指向存储PWM中间值的指针
 * @return true 加载成功
 */
bool Storage_LoadGripperConfig(uint8_t *pwm_min, uint8_t *pwm_max, uint8_t *pwm_center);


/* ==================== 批量操作函数 ==================== */

/**
 * @brief 保存所有配置到Flash
 * @return true 保存成功
 * @return false 保存失败
 *
 * @note 将内存缓存中的所有配置写入Flash：
 *       - 解锁Flash
 *       - 擦除存储页
 *       - 写入所有参数
 *       - 写入校验码
 *       - 锁定Flash
 */
bool Storage_SaveAllConfig(void);


/* ==================== 状态查询函数 ==================== */

/**
 * @brief 检查存储是否有效
 * @return true 存储有效
 * @return false 存储无效或未初始化
 *
 * @note 检查Flash中是否存在有效的校验码(0x5241)
 */
bool Storage_IsValid(void);

/**
 * @brief 重置为出厂默认配置
 * @return true 重置成功
 * @return false 重置失败
 *
 * @note 将所有参数恢复为默认值并写入Flash
 */
bool Storage_ResetToDefault(void);


/* ==================== 底层Flash操作函数 ==================== */

/**
 * @brief 解锁Flash写操作
 * @return true 解锁成功
 *
 * @note Flash在写入前必须解锁
 */
bool Flash_Unlock(void);

/**
 * @brief 锁定Flash写操作
 * @return true 锁定成功
 *
 * @note 写入完成后应锁定Flash防止误操作
 */
bool Flash_Lock(void);

/**
 * @brief 擦除Flash页
 * @param page_address 页地址
 * @return true 擦除成功
 *
 * @note Flash必须先擦除才能写入
 */
bool Flash_ErasePage(uint32_t page_address);

/**
 * @brief 写入32位数据
 * @param address 写入地址
 * @param data 要写入的数据
 * @return true 写入成功
 */
bool Flash_WriteWord(uint32_t address, uint32_t data);

/**
 * @brief 写入16位数据
 * @param address 写入地址
 * @param data 要写入的数据
 * @return true 写入成功
 */
bool Flash_WriteHalfWord(uint32_t address, uint16_t data);

/**
 * @brief 读取32位数据
 * @param address 读取地址
 * @return 读取的数据
 */
uint32_t Flash_ReadWord(uint32_t address);

/**
 * @brief 读取16位数据
 * @param address 读取地址
 * @return 读取的数据
 */
uint16_t Flash_ReadHalfWord(uint32_t address);

#ifdef __cplusplus
}
#endif

#endif /* __STORAGE_H */
