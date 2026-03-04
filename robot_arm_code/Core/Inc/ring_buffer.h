/**
  ******************************************************************************
  * @file    ring_buffer.h
  * @brief   环形缓冲区 - 用于UART DMA通信的数据缓冲
  *
  * @details 本模块实现了一个线程安全的环形缓冲区（队列），
  *         主要用于解决DMA接收数据与应用程序处理速度不匹配的问题。
  *         当DMA将数据从UART接收到缓冲区后，数据会被复制到环形缓冲区，
  *         应用程序可以从环形缓冲区按字节读取数据进行帧解析。
  ******************************************************************************
 */

#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 环形缓冲区大小
 * @note 选择256字节，可以缓冲一帧数据（最大64字节）的4倍，
 *       留有足够余量应对突发大量数据
 */
#define RING_BUFFER_SIZE 256

/**
 * @brief 环形缓冲区结构体
 *
 * 环形缓冲区是一种FIFO（先进先出）数据结构，类似于一个圆环。
 * 当读写位置到达末尾时，会回绕到开头，形成"环"的效果。
 *
 * @param buffer 数据存储数组
 * @param head 写入位置索引（下一个写入的位置）
 * @param tail 读取位置索引（下一个读取的位置）
 * @param count 当前存储的字节数
 *
 * @note 使用volatile修饰符，确保中断和主程序同时访问时的数据一致性
 */
typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];    /* 数据存储数组 */
    volatile uint16_t head;              /* 写入位置 */
    volatile uint16_t tail;              /* 读取位置 */
    volatile uint16_t count;             /* 当前数据个数 */
} RingBuffer;

/**
 * @brief 初始化环形缓冲区
 * @param rb 指向要初始化的环形缓冲区的指针
 *
 * @note 在使用环形缓冲区之前必须调用此函数进行初始化，
 *       会将head、tail、count都设为0
 */
void RingBuffer_Init(RingBuffer *rb);

/**
 * @brief 向环形缓冲区写入一个字节
 * @param rb 指向环形缓冲区的指针
 * @param data 要写入的字节数据
 * @return true 写入成功
 * @return false 缓冲区已满，写入失败
 *
 * @note 如果缓冲区已满（count >= RING_BUFFER_SIZE），写入会失败
 *       写入成功后，count会自动加1
 */
bool RingBuffer_Write(RingBuffer *rb, uint8_t data);

/**
 * @brief 从环形缓冲区读取一个字节
 * @param rb 指向环形缓冲区的指针
 * @param data 指向存储读取数据的变量的指针
 * @return true 读取成功
 * @return false 缓冲区为空，读取失败
 *
 * @note 如果缓冲区为空（count == 0），读取会失败
 *       读取成功后，count会自动减1
 */
bool RingBuffer_Read(RingBuffer *rb, uint8_t *data);

/**
 * @brief 获取环形缓冲区中当前存储的字节数
 * @param rb 指向环形缓冲区的指针
 * @return 当前缓冲区中的数据字节数
 *
 * @note 这个值在0到RING_BUFFER_SIZE之间
 */
uint16_t RingBuffer_Count(RingBuffer *rb);

/**
 * @brief 判断环形缓冲区是否为空
 * @param rb 指向环形缓冲区的指针
 * @return true 缓冲区为空
 * @return false 缓冲区不为空
 */
bool RingBuffer_IsEmpty(RingBuffer *rb);

/**
 * @brief 判断环形缓冲区是否已满
 * @param rb 指向环形缓冲区的指针
 * @return true 缓冲区已满
 * @return false 缓冲区未满
 *
 * @note 当缓冲区已满时，RingBuffer_Write会返回false
 */
bool RingBuffer_IsFull(RingBuffer *rb);

/**
 * @brief 清空环形缓冲区
 * @param rb 指向环形缓冲区的指针
 *
 * @note 清空后，head、tail、count都会重置为0
 *       但不会修改buffer中的原有数据（下次写入时会被覆盖）
 */
void RingBuffer_Clear(RingBuffer *rb);

/**
 * @brief 批量从环形缓冲区读取多个字节
 * @param rb 指向环形缓冲区的指针
 * @param dest 指向存储读取数据的数组的指针
 * @param max_len 最多读取的字节数
 * @return 实际读取的字节数
 *
 * @note 适合需要连续读取多字节数据的场景（如帧解析）
 *       会尽可能多地读取数据，直到缓冲区为空或达到max_len
 */
uint16_t RingBuffer_ReadBulk(RingBuffer *rb, uint8_t *dest, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* __RING_BUFFER_H */
