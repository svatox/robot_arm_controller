/**
  ******************************************************************************
  * @file    ring_buffer.c
  * @brief   环形缓冲区实现 - 用于UART DMA通信的数据缓冲
  *
  * @details 本文件实现了线程安全的环形缓冲区（队列）数据结构。
  *         主要用于解决DMA接收数据与应用程序处理速度不匹配的问题。
  *
  * @note 工作原理：
  *       环形缓冲区就像一个圆形的管道，数据从一端进入，从另一端离开。
  *       当读写位置到达末尾时，会回绕到开头，形成"环"的效果。
  *
  *       ┌─────────────────────────────┐
  *       │  buffer[0] ~ buffer[255]   │
  *       │  head →                     │
  *       │              ← tail          │
  *       └─────────────────────────────┘
  *
  *       - head: 下一个写入数据的位置
  *       - tail: 下一个读取数据的位置
  *       - count: 当前存储的数据个数
  ******************************************************************************
 */

#include "ring_buffer.h"

/**
 * @brief 初始化环形缓冲区
 * @param rb 指向要初始化的环形缓冲区的指针
 *
 * @note 将head、tail、count都设为0，准备好接收数据
 *       注意：buffer数组本身的内容不会被清除
 */
void RingBuffer_Init(RingBuffer *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

/**
 * @brief 向环形缓冲区写入一个字节
 * @param rb 指向环形缓冲区的指针
 * @param data 要写入的字节数据
 * @return true 写入成功
 * @return false 缓冲区已满，写入失败
 *
 * @note 如果缓冲区已满，写入会失败
 *       写入成功后count会自动加1
 *
 * 工作流程：
 * 1. 检查缓冲区是否已满
 * 2. 将数据写入head位置
 * 3. head前移到下一个位置（到末尾则回绕到0）
 * 4. count加1
 */
bool RingBuffer_Write(RingBuffer *rb, uint8_t data)
{
    // 检查缓冲区是否已满
    if (rb->count >= RING_BUFFER_SIZE) {
        return false;
    }

    // 将数据写入head位置
    rb->buffer[rb->head] = data;

    // head前移，到末尾则回绕
    rb->head = (rb->head + 1) % RING_BUFFER_SIZE;

    // 数据计数加1
    rb->count++;
    return true;
}

/**
 * @brief 从环形缓冲区读取一个字节
 * @param rb 指向环形缓冲区的指针
 * @param data 指向存储读取数据的变量的指针
 * @return true 读取成功
 * @return false 缓冲区为空，读取失败
 *
 * @note 如果缓冲区为空，读取会失败
 *       读取成功后count会自动减1
 *
 * 工作流程：
 * 1. 检查缓冲区是否为空
 * 2. 从tail位置读取数据
 * 3. tail前移到下一个位置（到末尾则回绕）
 * 4. count减1
 */
bool RingBuffer_Read(RingBuffer *rb, uint8_t *data)
{
    // 检查缓冲区是否为空
    if (rb->count == 0) {
        return false;
    }

    // 从tail位置读取数据
    *data = rb->buffer[rb->tail];

    // tail前移，到末尾则回绕
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;

    // 数据计数减1
    rb->count--;
    return true;
}

/**
 * @brief 获取环形缓冲区中当前存储的字节数
 * @param rb 指向环形缓冲区的指针
 * @return 当前缓冲区中的数据字节数
 */
uint16_t RingBuffer_Count(RingBuffer *rb)
{
    return rb->count;
}

/**
 * @brief 判断环形缓冲区是否为空
 * @param rb 指向环形缓冲区的指针
 * @return true 缓冲区为空
 * @return false 缓冲区不为空
 */
bool RingBuffer_IsEmpty(RingBuffer *rb)
{
    return rb->count == 0;
}

/**
 * @brief 判断环形缓冲区是否已满
 * @param rb 指向环形缓冲区的指针
 * @return true 缓冲区已满
 * @return false 缓冲区未满
 *
 * @note 当缓冲区已满时，RingBuffer_Write会返回false
 */
bool RingBuffer_IsFull(RingBuffer *rb)
{
    return rb->count >= RING_BUFFER_SIZE;
}

/**
 * @brief 清空环形缓冲区
 * @param rb 指向环形缓冲区的指针
 *
 * @note 清空后head、tail、count都会重置为0
 *       buffer中的原有数据不会清除（下次写入时会被覆盖）
 */
void RingBuffer_Clear(RingBuffer *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

/**
 * @brief 批量从环形缓冲区读取多个字节
 * @param rb 指向环形缓冲区的指针
 * @param dest 指向存储读取数据的数组的指针
 * @param max_len 最多读取的字节数
 * @return 实际读取的字节数
 *
 * @note 适合需要连续读取多字节数据的场景（如帧解析）
 *       会尽可能多地读取数据，直到缓冲区为空或达到max_len
 *
 * 工作流程：
 * 1. 循环尝试读取数据
 * 2. 每成功读取一个字节，dest指针前移，计数加1
 * 3. 当缓冲区为空或达到max_len时停止
 * 4. 返回实际读取的字节数
 */
uint16_t RingBuffer_ReadBulk(RingBuffer *rb, uint8_t *dest, uint16_t max_len)
{
    uint16_t read_count = 0;

    // 循环读取，直到缓冲区为空或达到最大长度
    while (read_count < max_len && RingBuffer_Read(rb, &dest[read_count])) {
        read_count++;
    }

    return read_count;
}
