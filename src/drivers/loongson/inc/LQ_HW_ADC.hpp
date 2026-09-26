#pragma once

#include <iostream>
#include <string>
#include <mutex>  // 多线程安全锁（单线程可注释，但建议保留）

// 系统头文件（声明依赖）
#include <error.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>

#include "LQ_MAP_ADDR.hpp"  // 地址映射

using namespace std;

/****************************** ADC 硬件参数宏定义 ******************************/
#define LS_ADC_BASE_ADDR    0x1611c000  // 龙芯2K300 ADC 物理基地址（以手册为准）
// 寄存器偏移地址（与硬件手册对应）
#define ADC_SR_OFFSET       0x00    // 状态寄存器（Status Register）
#define ADC_CR1_OFFSET      0x04    // 控制寄存器1（Control Register 1）
#define ADC_CR2_OFFSET      0x08    // 控制寄存器2（Control Register 2）
#define ADC_SMPR2_OFFSET    0x10    // 采样时间寄存器2（通道0~9）
#define ADC_SQR3_OFFSET     0x34    // 规则序列寄存器3（通道选择）
#define ADC_DR_OFFSET       0x4C    // 数据寄存器（Data Register）
// 采样时间配置（所有通道共用64个ADC时钟周期，兼顾精度和速度）
#define ADC_SAMPLE_TIME     0x06    // 对应 ADC_SampleTime_64Cycles
// ADC 硬件属性
#define ADC_RESOLUTION      4096    // 12位ADC分辨率（输出范围0~4095）
#define ADC_REF_VOLTAGE     1800    // 参考电压1.8V（单位：mV）
#define ADC_CONV_TIMEOUT    10000   // 转换超时计数（防止死循环）

/****************************** 单例硬件管理类声明（管理共享资源） ******************************/
class AdcHardware {
public:
    // 单例实例获取：静态局部变量，确保全局唯一（首次调用时初始化）
    static AdcHardware& getInstance();

    // 初始化ADC硬件寄存器（映射寄存器+基础配置+校准，仅执行一次）
    bool initHardware();

    // 读取指定通道的原始ADC值（未经转换）
    int readChannelRaw(uint8_t channel);

private:
    // 单例核心：禁止外部构造、拷贝、赋值（确保唯一实例）
    AdcHardware();                                      // 私有构造
    AdcHardware(const AdcHardware&) = delete;           // 禁止拷贝构造
    AdcHardware& operator=(const AdcHardware&) = delete;// 禁止赋值

    // 析构：释放硬件资源（自动调用）
    ~AdcHardware();

    // 内部函数：解除寄存器映射，释放内存资源
    void unmapRegisters();

    // 内部函数：配置基础寄存器（所有通道共用）
    void configBaseRegisters();

    // 内部函数：ADC硬件校准（必需步骤，否则采样值不准）
    bool calibrate();

    // 内部函数：切换当前采样通道（动态配置通道参数）
    void setCurrentChannel(uint8_t channel);

private:
    bool is_hw_inited;          // 硬件是否已初始化（标记位）
    int  mem_fd;                // /dev/mem 文件描述符
    // 映射后的寄存器指针
    void* adc_base;   // 映射基地址（内核空间）
    void* adc_sr;     // 状态寄存器
    void* adc_cr1;    // 控制寄存器1
    void* adc_cr2;    // 控制寄存器2
    void* adc_smpr2;  // 采样时间寄存器2
    void* adc_sqr3;   // 规则序列寄存器3
    void* adc_dr;     // 数据寄存器
    mutex adc_mutex;  // 多线程安全锁（单线程可删除）
};

/****************************** 轻量级通道实例类声明（仅关联通道号） ******************************/
class HWAdc {
public:
    // 构造函数：创建指定通道的ADC实例，关联通道号，初始化共享硬件实例
    explicit HWAdc(uint8_t channel);

    // 读取指定通道的原始ADC值（调用共享硬件接口）
    int ReadRaw();

    // 读取ADC电压值（单位：V，自动计算）
    float ReadVoltage();

private:
    uint8_t adc_channel;  // 仅存储当前通道号（不管理硬件）
    bool is_valid;        // 实例是否有效（标记硬件初始化结果）
};
