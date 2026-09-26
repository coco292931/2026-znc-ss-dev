#include "LQ_HW_ADC.hpp"

/****************************** 单例硬件管理类（AdcHardware）实现 ******************************/

/***************************************************************************
 * @fn      AdcHardware &AdcHardware::getInstance()
 * @brief   单例实例获取：静态局部变量，确保全局唯一（首次调用时初始化）
 * @param   None
 * @return  AdcHardware& : 单例对象引用
 * @date    2025-09-29
 ***************************************************************************/
AdcHardware &AdcHardware::getInstance()
{
    static AdcHardware instance;
    return instance;
}

/***************************************************************************
 * @fn      AdcHardware::AdcHardware()
 * @brief   私有构造：初始化成员变量（仅调用一次）
 * @param   None
 * @return  None
 * @date    2025-09-29
 ***************************************************************************/
AdcHardware::AdcHardware()
    : is_hw_inited(false), mem_fd(-1), adc_base(NULL),
      adc_sr(NULL), adc_cr1(NULL), adc_cr2(NULL),
      adc_smpr2(NULL), adc_sqr3(NULL), adc_dr(NULL)
{
    // 构造时仅初始化变量，不执行硬件操作（延迟到initHardware）
}

/***************************************************************************
 * @fn      ~AdcHardware()
 * @brief   析构函数：释放硬件资源（如映射的寄存器内存）
 * @param   None
 * @return  None
 * @date    2025-09-29
 ***************************************************************************/
AdcHardware::~AdcHardware()
{
    this->unmapRegisters();
}

/***************************************************************************
 * @fn      void AdcHardware::unmapRegisters()
 * @brief   解除寄存器映射，释放内存资源
 * @param   None
 * @return  None
 * @date    2025-09-29
 ***************************************************************************/
void AdcHardware::unmapRegisters()
{
    if (this->adc_base != NULL) { munmap(this->adc_base, PAGE_SIZE); }

    this->adc_base = NULL;
    this->adc_sr = this->adc_cr1 = this->adc_cr2 = NULL;
    this->adc_smpr2 = this->adc_sqr3 = this->adc_dr = NULL;
}

/***************************************************************************
 * @fn      void AdcHardware::configBaseRegisters()
 * @brief   配置基础寄存器（所有通道共用）
 * @param   None
 * @return  None
 * @date    2025-09-29
 ***************************************************************************/
void AdcHardware::configBaseRegisters()
{
    // 1. 配置CR1：独立模式、关闭扫描、禁用中断
    LS_writel(this->adc_cr1, 0x00000000);                          // 清空CR1
    LS_writel(this->adc_cr1, LS_readl(this->adc_cr1) & ~(1 << 8)); // SCAN=0（单通道模式）
    LS_writel(this->adc_cr1, LS_readl(this->adc_cr1) & ~(1 << 9)); // EOCIE=0（禁用转换完成中断）
    LS_writel(this->adc_cr1, LS_readl(this->adc_cr1) & ~(1 << 7)); // JEOCIE=0（禁用插入通道中断）

    // 配置CR2：右对齐、单次转换、软件触发、禁用DMA
    LS_writel(this->adc_cr2, 0x00000000);
    LS_writel(this->adc_cr2, LS_readl(this->adc_cr2) & ~(1 << 11));   // ALIGN=0（数据右对齐，取低12位）
    LS_writel(this->adc_cr2, LS_readl(this->adc_cr2) & ~(1 << 1));    // CONT=0（单次转换，不循环）
    LS_writel(this->adc_cr2, LS_readl(this->adc_cr2) | (1 << 20));    // EXTTRIG=1（允许外部触发）
    LS_writel(this->adc_cr2, LS_readl(this->adc_cr2) | (0x0E << 17)); // EXTSEL=0x0E（软件触发转换）
    LS_writel(this->adc_cr2, LS_readl(this->adc_cr2) & ~(1 << 8));    // DMA=0（禁用DMA，简化操作）
}

/***************************************************************************
 * @fn      bool AdcHardware::calibrate()
 * @brief   ADC硬件校准（必需步骤，否则采样值不准）
 * @param   None
 * @return  bool: 成功返回true，失败返回false
 * @date    2025-09-29
 ***************************************************************************/
bool AdcHardware::calibrate()
{
    if (this->adc_cr2 == NULL)
    {
        std::cout << "AdcHardware: 寄存器未映射，无法校准" << std::endl;
        return false;
    }

    LS_writel(this->adc_cr2, LS_readl(this->adc_cr2) | (1 << 0)); // 使能ADC（ADON位，需两次置1激活，部分硬件要求）
    usleep(20);                                                   // 等待ADC稳定（20微秒）
    LS_writel(this->adc_cr2, LS_readl(this->adc_cr2) | (1 << 0)); // 使能ADC（ADON位，需两次置1激活，部分硬件要求）
    usleep(20);

    LS_writel(this->adc_cr2, LS_readl(this->adc_cr2) | (1 << 3)); // 重置校准（RSTCAL位）
    usleep(10);
    // 等待复位校准完成（RSTCAL位：bit3，硬件完成后自动清0）
    while ((LS_readl(this->adc_cr2) & (1 << 3)) != 0) {
        usleep(1);  // 微秒级等待，降低CPU占用（可选，避免CPU空转）
    }

    LS_writel(this->adc_cr2, LS_readl(this->adc_cr2) | (1 << 2)); // 开始校准（CAL位）
    usleep(10);
    // 等待ADC校准完成（CAL位：bit2，硬件完成后自动清0）
    while ((LS_readl(this->adc_cr2) & (1 << 2)) != 0) {
        usleep(1);  // 同上，建议保留
    }

    usleep(20); // 校准后稳定时间
    return true;
}

/***************************************************************************
 * @fn      void AdcHardware::setCurrentChannel(uint8_t channel)
 * @brief   内部函数：切换当前采样通道（动态配置通道参数）
 * @param   channel : 目标通道编号（0-7）
 * @return  None
 * @date    2025-09-29
 ***************************************************************************/
void AdcHardware::setCurrentChannel(uint8_t channel)
{
    // 配置目标通道的采样时间（64个ADC时钟周期）
    LS_writel(this->adc_smpr2, LS_readl(this->adc_smpr2) & ~(0x07 << (3 * channel)));       // 清除该通道原有采样时间
    LS_writel(this->adc_smpr2, LS_readl(this->adc_smpr2) | (ADC_SAMPLE_TIME << (3 * channel))); // 设置新采样时间

    // 配置规则序列：仅采样当前通道（SQ1=目标通道，序列长度=1
    LS_writel(this->adc_sqr3, LS_readl(this->adc_sqr3) & ~(0x1F << 0));   // 清除SQ1（第一个转换通道）
    LS_writel(this->adc_sqr3, LS_readl(this->adc_sqr3) | (channel << 0)); // SQ1=目标通道
}

/***************************************************************************
 * @fn      bool AdcHardware::initHardware()
 * @brief   初始化ADC硬件寄存器（映射寄存器+基础配置+校准，仅执行一次）
 * @param   None
 * @return  bool: 成功返回true，失败返回false
 * @date    2025-09-29
 ***************************************************************************/
bool AdcHardware::initHardware()
{
    if (this->is_hw_inited)
    {
        // 新增 Dummy 转换（任意通道均可，这里使用通道0），消耗首次全局延迟
        lock_guard<mutex> lock(this->adc_mutex);    // 加锁，防止多线程冲突
        this->setCurrentChannel(0);                 // 切换到通道0
        LS_writel(this->adc_sr,  LS_readl(this->adc_sr)  & (1 << 1));   // 清除EOC标志
        LS_writel(this->adc_cr2, LS_readl(this->adc_cr2) | (1 << 22));  // 软件触发转换

        // 等待 Dummy 转换完成（清除 EOF 标志，避免影响后续采集）
        int dummy_timeout = 0;
        while (((LS_readl(this->adc_sr) & (1 << 1)) == 0) && (dummy_timeout < ADC_CONV_TIMEOUT * 2))
        {
            usleep(1);
            dummy_timeout++;
        }

        // 必须读取 DR 寄存器（清除 EOF 标志，避免影响后续采集）
        if (dummy_timeout < ADC_CONV_TIMEOUT * 2)
        {
            LS_readl(this->adc_dr); // 丢弃 Dummy 转换结果，仅清除 EOF 标志
        }
        else
        {
            std::cout << "AdcHardware: Dummy 转换超时，硬件可能异常" << std::endl;
        }
        return true; // 已初始化，直接返回
    }

    void* adc_addr;

    this->adc_base = AddressMap(LS_ADC_BASE_ADDR, PAGE_SIZE);

    this->adc_sr    = (void*)((uintptr_t)adc_base + ADC_SR_OFFSET);
    this->adc_cr1   = (void*)((uintptr_t)adc_base + ADC_CR1_OFFSET);
    this->adc_cr2   = (void*)((uintptr_t)adc_base + ADC_CR2_OFFSET);
    this->adc_smpr2 = (void*)((uintptr_t)adc_base + ADC_SMPR2_OFFSET);
    this->adc_sqr3  = (void*)((uintptr_t)adc_base + ADC_SQR3_OFFSET);
    this->adc_dr    = (void*)((uintptr_t)adc_base + ADC_DR_OFFSET);
    
    // 配置基础寄存器
    this->configBaseRegisters();

    // 硬件校准
    if (!this->calibrate())
    {
        this->unmapRegisters();
        return false;
    }

    this->is_hw_inited = true;
    cout << "AdcHardware: 共享硬件初始化成功" << endl;
    return true;
}

/***************************************************************************
 * @fn      int AdcHardware::readChannelRaw(uint8_t channel)
 * @brief   读取指定通道的原始ADC值（未经转换）
 * @param   channel : 目标通道编号（0-7）
 * @return  int: 成功返回原始值，失败或超时返回-1/-2/-3
 * @date    2025-09-29
 ***************************************************************************/
int AdcHardware::readChannelRaw(uint8_t channel)
{
    // 检查硬件状态和通道合法性
    if (!this->is_hw_inited)
    {
        cout << "AdcHardware: 硬件未初始化，无法读取" << endl;
        return -1;
    }
    // 龙芯2K300 ADC仅支持0~7通道
    if (channel > 7)
    {
        cout << "AdcHardware: 通道" << (int)channel << "超出范围（0~7）" << endl;
        return -2;
    }

    // 加锁：防止多线程下通道切换冲突（单线程可注释）
    lock_guard<mutex> lock(adc_mutex);

    // 切换到目标通道
    setCurrentChannel(channel);

    // 触发ADC转换
    LS_writel(this->adc_sr, LS_readl(this->adc_sr) & ~(1 << 1));    // 清除EOC标志（避免上一次转换的残留）
    LS_writel(this->adc_cr2, LS_readl(this->adc_cr2) | (1 << 22));  // 置位SWSTART，触发软件转换

    // 等待转换完成（EOC标志置1），带超时保护
    int timeout = 0;
    while ((((LS_readl(this->adc_sr) & (1 << 1)) == 0) && (timeout < ADC_CONV_TIMEOUT)))
    {
        usleep(1);  // 微秒级等待，降低CPU占用
        timeout++;
    }

    // 处理超时或读取数据
    if (timeout >= ADC_CONV_TIMEOUT)
    {
        cout << "AdcHardware: 通道" << (int)channel << "转换超时" << endl;
        return -3;
    }

    // 读取数据寄存器，仅保留低12位有效数据
    return (LS_readl(this->adc_dr) & 0x0FFF);
}

/****************************** 轻量级通道实例类（HWAdc）实现 ******************************/

/***************************************************************************
 * @fn      HWAdc::HWAdc(uint8_t channel)
 * @brief   构造函数：创建指定通道的ADC实例，关联通道号，初始化共享硬件实例
 * @param   channel : 目标通道编号（0-7）
 * @return  None
 * @date    2025-09-29
 ***************************************************************************/
HWAdc::HWAdc(uint8_t channel) : adc_channel(channel), is_valid(false)
{
    // 初始化共享硬件（仅首次调用时执行，后续实例直接复用）
    if (AdcHardware::getInstance().initHardware())
    {
        is_valid = true;
        cout << "HWAdc: 通道" << (int)adc_channel << "实例创建成功" << endl;
    }
    else
    {
        cout << "HWAdc: 通道" << (int)adc_channel << "实例创建失败" << endl;
    }
}

/***************************************************************************
 * @fn      int HWAdc::ReadRaw()
 * @brief   读取指定通道的原始ADC值（调用共享硬件接口）
 * @param   None
 * @return  int: 成功返回原始值，失败或超时返回-1/-2/-3
 * @date    2025-09-29
 ***************************************************************************/
int HWAdc::ReadRaw()
{
    if (!is_valid)
    {
        cout << "HWAdc: 通道" << (int)adc_channel << "实例无效" << endl;
        return -1;
    }
    // 调用单例硬件的读取接口
    return AdcHardware::getInstance().readChannelRaw(adc_channel);
}

/***************************************************************************
 * @fn      float HWAdc::ReadVoltage()
 * @brief   读取指定通道的电压值（原始值转换为电压）
 * @param   None
 * @return  float: 成功返回电压值，失败或超时返回-1.0f
 * @date    2025-09-29
 ***************************************************************************/
float HWAdc::ReadVoltage()
{
    int raw_val = ReadRaw();
    if (raw_val < 0)
    {
        return -1.0f; // 读取失败，返回负电压标识
    }
    // 电压计算公式：(原始值 / 分辨率) * 参考电压（单位：V）
    return (float)raw_val * ADC_REF_VOLTAGE / ADC_RESOLUTION / 1000.0f;
}