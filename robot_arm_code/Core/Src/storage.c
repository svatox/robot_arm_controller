/**
  ******************************************************************************
  * @file    storage.c
  * @brief   Flash存储实现 - 配置参数持久化
  *
  * @details 本文件实现了将机械臂的配置参数保存到STM32的Flash存储器中。
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
  *       │ 备份区     │ 0x40     │ 预留               │
  *       └─────────────┴───────────┴────────────────────┘
  ******************************************************************************
 */

#include "storage.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/* ==================== 静态变量 ==================== */

/** 减速比缓存（6个关节）*/
static float cached_gear_ratios[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

/** 零位脉冲缓存（6个关节）*/
static int32_t cached_zero_positions[6] = {0, 0, 0, 0, 0, 0};

/** 零位设置标志缓存（6个关节）*/
static uint8_t cached_zero_set_flags[6] = {0, 0, 0, 0, 0, 0};

/** 极限位置缓存（6个关节）*/
static int32_t cached_limit_positions[6] = {0, 0, 0, 0, 0, 0};

/** 极限位置设置标志缓存（6个关节）*/
static uint8_t cached_limit_set_flags[6] = {0, 0, 0, 0, 0, 0};

/** 位置保护开关缓存 */
static uint8_t cached_protection_switch = 1;  /**< 默认开启 */

/** 夹爪PWM配置缓存 */
static uint8_t cached_gripper_pwm_min = 25;
static uint8_t cached_gripper_pwm_max = 125;
static uint8_t cached_gripper_pwm_center = 75;

/** 缓存是否有效标志 */
static bool cache_valid = false;


/* ==================== 底层Flash操作 ==================== */

/**
 * @brief 解锁Flash写操作
 *
 * @note STM32的Flash在写入前必须解锁
 *       解锁序列：写入0x4567到KEYR，然后写入0x89ABCDEF到KEYR
 */
bool Flash_Unlock(void)
{
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    return (status == HAL_OK);
}

/**
 * @brief 锁定Flash写操作
 *
 * @note 写入完成后应锁定Flash防止误操作
 */
bool Flash_Lock(void)
{
    HAL_StatusTypeDef status = HAL_FLASH_Lock();
    return (status == HAL_OK);
}

/**
 * @brief 擦除Flash页
 *
 * @note Flash必须先擦除才能写入
 *       STM32的Flash擦除以页为单位
 *
 * @param page_address 页地址
 */
bool Flash_ErasePage(uint32_t page_address)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;

    // 配置擦除参数
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = page_address;
    erase_init.NbPages = 1;

    // 执行擦除
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    // 成功条件：HAL成功且无错误
    return (status == HAL_OK && page_error == 0xFFFFFFFF);
}

/**
 * @brief 写入32位数据到Flash
 */
bool Flash_WriteWord(uint32_t address, uint32_t data)
{
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);
    return (status == HAL_OK);
}

/**
 * @brief 写入16位数据到Flash
 */
bool Flash_WriteHalfWord(uint32_t address, uint16_t data)
{
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, data);
    return (status == HAL_OK);
}

/**
 * @brief 写入8位数据到Flash
 */
bool Flash_WriteByte(uint32_t address, uint8_t data)
{
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, data);
    return (status == HAL_OK);
}

/**
 * @brief 从Flash读取32位数据
 */
uint32_t Flash_ReadWord(uint32_t address)
{
    // 直接解引用读取
    return *(__IO uint32_t*)address;
}

/**
 * @brief 从Flash读取16位数据
 */
uint16_t Flash_ReadHalfWord(uint32_t address)
{
    return *(__IO uint16_t*)address;
}

/**
 * @brief 从Flash读取8位数据
 */
uint8_t Flash_ReadByte(uint32_t address)
{
    return *(__IO uint8_t*)address;
}


/* ==================== 存储初始化 ==================== */

/**
 * @brief 初始化存储模块
 *
 * 检查Flash中是否有有效配置，如有则加载
 */
bool Storage_Init(void)
{
    // 检查存储是否有效
    if (Storage_IsValid()) {
        // 存储有效，加载所有配置到缓存

        // 加载减速比
        for (uint8_t i = 1; i <= 6; i++) {
            Storage_LoadGearRatio(i, &cached_gear_ratios[i-1]);
            Storage_LoadZeroPosition(i, &cached_zero_positions[i-1]);
        }

        // 加载夹爪配置
        Storage_LoadGripperConfig(&cached_gripper_pwm_min,
                                  &cached_gripper_pwm_max,
                                  &cached_gripper_pwm_center);

        cache_valid = true;
        return true;
    }

    // 存储无效，使用默认值
    cache_valid = true;
    return false;
}

/**
 * @brief 检查存储是否有效
 *
 * @note 通过检查Magic Number是否匹配来判断
 */
bool Storage_IsValid(void)
{
    // 读取存储区域的魔术字
    uint16_t magic = Flash_ReadHalfWord(FLASH_STORAGE_BASE + OFFSET_CHECKSUM);

    // 比较魔术字
    return (magic == STORAGE_VALID_MAGIC);
}


/* ==================== 减速比操作 ==================== */

/**
 * @brief 保存关节减速比
 *
 * @note 只更新内存缓存，需要调用Storage_SaveAllConfig()写入Flash
 *
 * @param joint_id 关节ID (1-6)
 * @param ratio 减速比值
 */
bool Storage_SaveGearRatio(uint8_t joint_id, float ratio)
{
    // 检查参数
    if (joint_id < 1 || joint_id > 6) {
        return false;
    }

    // 更新缓存
    cached_gear_ratios[joint_id - 1] = ratio;

    return true;
}

/**
 * @brief 加载关节减速比
 *
 * @param joint_id 关节ID (1-6)
 * @param ratio 存储减速比的指针
 */
bool Storage_LoadGearRatio(uint8_t joint_id, float *ratio)
{
    // 检查参数
    if (joint_id < 1 || joint_id > 6 || ratio == NULL) {
        return false;
    }

    // 如果缓存有效，直接从缓存读取
    if (cache_valid) {
        *ratio = cached_gear_ratios[joint_id - 1];
        return true;
    }

    // 否则从Flash读取
    uint32_t address = FLASH_STORAGE_BASE + OFFSET_GEAR_RATIO + (joint_id - 1) * 4;
    uint32_t value = Flash_ReadWord(address);

    // 将uint32_t转换为float
    memcpy(ratio, &value, sizeof(float));
    return true;
}


/* ==================== 零位脉冲操作 ==================== */

/**
 * @brief 保存关节零位脉冲
 *
 * @note 只更新内存缓存，需要调用Storage_SaveAllConfig()写入Flash
 */
bool Storage_SaveZeroPosition(uint8_t joint_id, int32_t pulse)
{
    if (joint_id < 1 || joint_id > 6) {
        return false;
    }

    // 更新缓存
    cached_zero_positions[joint_id - 1] = pulse;

    return true;
}

/**
 * @brief 加载关节零位脉冲
 */
bool Storage_LoadZeroPosition(uint8_t joint_id, int32_t *pulse)
{
    if (joint_id < 1 || joint_id > 6 || pulse == NULL) {
        return false;
    }

    // 如果缓存有效，直接从缓存读取
    if (cache_valid) {
        *pulse = cached_zero_positions[joint_id - 1];
        return true;
    }

    // 否则从Flash读取
    uint32_t address = FLASH_STORAGE_BASE + OFFSET_ZERO_POSITION + (joint_id - 1) * 4;
    *pulse = (int32_t)Flash_ReadWord(address);
    return true;
}

/**
 * @brief 保存零位设置标志
 */
/**
 * @brief 保存零位设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 设置标志 (0=未设置, 1=已设置)
 * @return true 保存成功
 *
 * @note 仅更新内存缓存，需要调用Storage_SaveAllConfig()写入Flash
 */
bool Storage_SaveZeroSetFlag(uint8_t joint_id, uint8_t flag)
{
    if (joint_id < 1 || joint_id > 6) {
        return false;
    }
    cached_zero_set_flags[joint_id - 1] = flag ? 1 : 0;
    return true;
}

/**
 * @brief 加载零位设置标志
 */
/**
 * @brief 加载零位设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 指向存储标志的指针
 * @return true 加载成功
 *
 * @note 先检查缓存，如果缓存无效则从Flash读取
 */
bool Storage_LoadZeroSetFlag(uint8_t joint_id, uint8_t *flag)
{
    if (joint_id < 1 || joint_id > 6 || flag == NULL) {
        return false;
    }

    if (cache_valid) {
        *flag = cached_zero_set_flags[joint_id - 1];
        return true;
    }

    uint32_t address = FLASH_STORAGE_BASE + OFFSET_ZERO_SET_FLAG + (joint_id - 1);
    *flag = Flash_ReadByte(address);
    return true;
}

/**
 * @brief 保存极限位置
 */
/**
 * @brief 保存极限位置
 * @param joint_id 关节ID (1-6)
 * @param position 极限位置脉冲数
 * @return true 保存成功
 *
 * @note 仅更新内存缓存，需要调用Storage_SaveAllConfig()写入Flash
 */
bool Storage_SaveLimitPosition(uint8_t joint_id, int32_t position)
{
    if (joint_id < 1 || joint_id > 6) {
        return false;
    }
    cached_limit_positions[joint_id - 1] = position;
    return true;
}

/**
 * @brief 加载极限位置
 */
/**
 * @brief 加载极限位置
 * @param joint_id 关节ID (1-6)
 * @param position 指向存储极限位置的指针
 * @return true 加载成功
 *
 * @note 先检查缓存，如果缓存无效则从Flash读取
 */
bool Storage_LoadLimitPosition(uint8_t joint_id, int32_t *position)
{
    if (joint_id < 1 || joint_id > 6 || position == NULL) {
        return false;
    }

    if (cache_valid) {
        *position = cached_limit_positions[joint_id - 1];
        return true;
    }

    uint32_t address = FLASH_STORAGE_BASE + OFFSET_LIMIT_POSITION + (joint_id - 1) * 4;
    *position = (int32_t)Flash_ReadWord(address);
    return true;
}

/**
 * @brief 保存极限位置设置标志
 */
/**
 * @brief 保存极限位置设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 设置标志 (0=未设置, 1=已设置)
 * @return true 保存成功
 *
 * @note 仅更新内存缓存，需要调用Storage_SaveAllConfig()写入Flash
 */
bool Storage_SaveLimitSetFlag(uint8_t joint_id, uint8_t flag)
{
    if (joint_id < 1 || joint_id > 6) {
        return false;
    }
    cached_limit_set_flags[joint_id - 1] = flag ? 1 : 0;
    return true;
}

/**
 * @brief 加载极限位置设置标志
 */
/**
 * @brief 加载极限位置设置标志
 * @param joint_id 关节ID (1-6)
 * @param flag 指向存储标志的指针
 * @return true 加载成功
 *
 * @note 先检查缓存，如果缓存无效则从Flash读取
 */
bool Storage_LoadLimitSetFlag(uint8_t joint_id, uint8_t *flag)
{
    if (joint_id < 1 || joint_id > 6 || flag == NULL) {
        return false;
    }

    if (cache_valid) {
        *flag = cached_limit_set_flags[joint_id - 1];
        return true;
    }

    uint32_t address = FLASH_STORAGE_BASE + OFFSET_LIMIT_SET_FLAG + (joint_id - 1);
    *flag = Flash_ReadByte(address);
    return true;
}

/**
 * @brief 保存位置保护开关状态
 */
/**
 * @brief 保存位置保护开关状态
 * @param enable 开关状态 (0=关闭, 1=开启)
 * @return true 保存成功
 *
 * @note 仅更新内存缓存，需要调用Storage_SaveAllConfig()写入Flash
 */
bool Storage_SaveProtectionSwitch(uint8_t enable)
{
    cached_protection_switch = enable ? 1 : 0;
    return true;
}

/**
 * @brief 加载位置保护开关状态
 * @param enable 指向存储开关状态的指针
 * @return true 加载成功
 *
 * @note 先检查缓存，如果缓存无效则从Flash读取
 */
bool Storage_LoadProtectionSwitch(uint8_t *enable)
{
    if (enable == NULL) {
        return false;
    }

    if (cache_valid) {
        *enable = cached_protection_switch;
        return true;
    }

    uint32_t address = FLASH_STORAGE_BASE + OFFSET_PROTECTION_SWITCH;
    *enable = Flash_ReadByte(address);
    return true;
}


/* ==================== 夹爪配置操作 ==================== */

/**
 * @brief 保存夹爪PWM配置
 */
bool Storage_SaveGripperConfig(uint8_t pwm_min, uint8_t pwm_max, uint8_t pwm_center)
{
    // 更新缓存
    cached_gripper_pwm_min = pwm_min;
    cached_gripper_pwm_max = pwm_max;
    cached_gripper_pwm_center = pwm_center;
    return true;
}

/**
 * @brief 加载夹爪PWM配置
 */
bool Storage_LoadGripperConfig(uint8_t *pwm_min, uint8_t *pwm_max, uint8_t *pwm_center)
{
    // 如果缓存有效，直接从缓存读取
    if (cache_valid) {
        *pwm_min = cached_gripper_pwm_min;
        *pwm_max = cached_gripper_pwm_max;
        *pwm_center = cached_gripper_pwm_center;
        return true;
    }

    // 从Flash读取
    uint32_t address = FLASH_STORAGE_BASE + OFFSET_GRIPPER_CONFIG;
    uint32_t value = Flash_ReadWord(address);

    // 解析各字段
    *pwm_min = (uint8_t)(value & 0xFF);
    *pwm_max = (uint8_t)((value >> 8) & 0xFF);
    *pwm_center = (uint8_t)((value >> 16) & 0xFF);

    return true;
}


/* ==================== 批量保存 ==================== */

/**
 * @brief 保存所有配置到Flash
 *
 * @note 这是一个完整的写操作过程：
 *       1. 解锁Flash
 *       2. 擦除存储页
 *       3. 写入所有参数
 *       4. 写入魔术字
 *       5. 锁定Flash
 */
bool Storage_SaveAllConfig(void)
{
    // 1. 解锁Flash
    if (!Flash_Unlock()) {
        return false;
    }

    // 2. 擦除存储页
    if (!Flash_ErasePage(FLASH_STORAGE_BASE)) {
        Flash_Lock();
        return false;
    }

    bool success = true;
    uint32_t address = FLASH_STORAGE_BASE;

    // 3. 保存减速比（6个关节 × 4字节）
    for (uint8_t i = 0; i < 6 && success; i++) {
        uint32_t value;
        // float转换为uint32_t
        memcpy(&value, &cached_gear_ratios[i], sizeof(float));

        if (!Flash_WriteWord(address + OFFSET_GEAR_RATIO + i * 4, value)) {
            success = false;
        }
    }

    // 4. 保存零位脉冲（6个关节 × 4字节）
    for (uint8_t i = 0; i < 6 && success; i++) {
        if (!Flash_WriteWord(address + OFFSET_ZERO_POSITION + i * 4,
                             (uint32_t)cached_zero_positions[i])) {
            success = false;
        }
    }

    // 5. 保存夹爪配置（8字节 = 4字节紧凑格式）
    uint32_t gripper_value = cached_gripper_pwm_min |
                            ((uint32_t)cached_gripper_pwm_max << 8) |
                            ((uint32_t)cached_gripper_pwm_center << 16);
    if (success) {
        if (!Flash_WriteWord(address + OFFSET_GRIPPER_CONFIG, gripper_value)) {
            success = false;
        }
    }

    // 6. 保存零位设置标志（6字节）
    for (uint8_t i = 0; i < 6 && success; i++) {
        if (!Flash_WriteByte(address + OFFSET_ZERO_SET_FLAG + i, cached_zero_set_flags[i])) {
            success = false;
        }
    }

    // 7. 保存极限位置设置标志（6字节）
    for (uint8_t i = 0; i < 6 && success; i++) {
        if (!Flash_WriteByte(address + OFFSET_LIMIT_SET_FLAG + i, cached_limit_set_flags[i])) {
            success = false;
        }
    }

    // 8. 保存极限位置（6个关节 × 4字节）
    for (uint8_t i = 0; i < 6 && success; i++) {
        if (!Flash_WriteWord(address + OFFSET_LIMIT_POSITION + i * 4,
                             (uint32_t)cached_limit_positions[i])) {
            success = false;
        }
    }

    // 9. 保存位置保护开关（1字节）
    if (success) {
        if (!Flash_WriteByte(address + OFFSET_PROTECTION_SWITCH, cached_protection_switch)) {
            success = false;
        }
    }

    // 10. 写入魔术字标记存储有效
    if (success) {
        if (!Flash_WriteHalfWord(address + OFFSET_CHECKSUM, STORAGE_VALID_MAGIC)) {
            success = false;
        }
    }

    // 11. 锁定Flash
    Flash_Lock();
    return success;
}


/* ==================== 恢复默认 ==================== */

/**
 * @brief 重置为出厂默认配置
 */
bool Storage_ResetToDefault(void)
{
    // 重置缓存为默认值
    for (uint8_t i = 0; i < 6; i++) {
        cached_gear_ratios[i] = 1.0f;
        cached_zero_positions[i] = 0;
        cached_zero_set_flags[i] = 0;
        cached_limit_positions[i] = 0;
        cached_limit_set_flags[i] = 0;
    }
    cached_gripper_pwm_min = 25;
    cached_gripper_pwm_max = 125;
    cached_gripper_pwm_center = 75;
    cached_protection_switch = 0;  // 关闭位置保护

    // 保存到Flash
    return Storage_SaveAllConfig();
}
