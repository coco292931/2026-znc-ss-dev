/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：Linux 环境、VSCode_1.93 及以上版本、Cmake_3.16 及以上版本
@使用平台：龙芯2K0300久久派和北京龙邱智能科技龙芯久久派拓展板
@相关信息参考下列地址
    网      站：http://www.lqist.cn 
    淘 宝 店 铺：http://longqiu.taobao.com 
    程序配套视频：https://space.bilibili.com/95313236 
@软件版本：V1.0 版权所有，单位使用请先联系授权
@参考项目链接：https://github.com/AirFortressIlikara/ls2k0300_peripheral_library 

@修改日期：2026-01-4
@修改内容：
@注意事项：注意查看路径的修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "LQ_ATIM_PWM.hpp"

// 创建一个互斥锁
static pthread_mutex_t Atim_mutex = PTHREAD_MUTEX_INITIALIZER;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：AtimPwm::AtimPwm()
 * @功能说明：ATIM 控制器 PWM 模式的无参构造函数
 * @参数说明：无
 * @函数返回：无
 * @调用方法：AtimPwm MyPwm;
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
AtimPwm::AtimPwm()
{

}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：AtimPwm::AtimPwm(uint8_t gpio, uint8_t chNum, uint8_t pol, uint32_t period, uint32_t duty_cycle, int mux)
                 : Gpio(gpio), Pol(pol), Period(period), Duty_cycle(duty_cycle), ChNum(chNum - 1)
 * @功能说明：ATIM 控制器 PWM 模式的有参构造函数
 * @参数说明：gpio        :  输出 PWM 的 GPIO 引脚号
 * @参数说明：chNum       :  参数一对应的通道号（1-4）
 * @参数说明：pol         :  PWM 输出极性
 * @参数说明：period      :  PWM 周期
 * @参数说明：duty_cycle  :  PWM 占空比
 * @参数说明：mux         :  复用 GPIO 引脚
 * @函数返回：无
 * @调用方法：AtimPwm MyPwm(81, 1, LS_ATIM_INVERSED, 50, 2000);
 * @备注说明：参数六设置有默认值，可以不修改，GPIO 对应的通道号可查看：
 *           《龙芯2K0300处理器用户手册》
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
AtimPwm::AtimPwm(uint8_t gpio, uint8_t chNum, uint8_t pol, uint32_t period, uint32_t duty_cycle, int mux)
    : Gpio(gpio), Pol(pol), Period(period), Duty_cycle(duty_cycle), ChNum(chNum - 1)
{
    // 检查通道号是否有效
    if (ChNum > 3) {
        printf("Error: ATIM channel number must be between 1 and 4\n");
        return;
    }
    
    // 配置功能复用
    GpioReuse(this->Gpio, mux);
    
    // 初始化所有寄存器
    LS_writel(AddressMap(LS_ATIM_BASE_ADDR + LS_ATIM_EGR, PAGE_SIZE), 0x01);
    
    // 获取虚拟地址
    this->ATIM_ARR     = AddressMap(LS_ATIM_BASE_ADDR + LS_ATIM_ARR, PAGE_SIZE);
    this->ATIM_CCRx    = AddressMap(LS_ATIM_BASE_ADDR + LS_ATIM_CCR1 + this->ChNum * LS_ATIM_CCRx_OFS, PAGE_SIZE);
    this->ATIM_CCMR[0] = AddressMap(LS_ATIM_BASE_ADDR + LS_ATIM_CCMR1, PAGE_SIZE);
    this->ATIM_CCMR[1] = AddressMap(LS_ATIM_BASE_ADDR + LS_ATIM_CCMR2, PAGE_SIZE);
    this->ATIM_CCER    = AddressMap(LS_ATIM_BASE_ADDR + LS_ATIM_CCER, PAGE_SIZE);
    this->ATIM_CNT     = AddressMap(LS_ATIM_BASE_ADDR + LS_ATIM_CNT, PAGE_SIZE);
    this->ATIM_BDTR    = AddressMap(LS_ATIM_BASE_ADDR + LS_ATIM_BDTR, PAGE_SIZE);
    
    // 设置刹车和死区寄存器（使能主输出，不使能死区和刹车）
    LS_writel(this->ATIM_BDTR, (1 << 15));  // MOE=1，使能主输出
    
    // 设置 PWM 模式，默认为模式 2
    this->SetMode(LS_ATIM_Mode_2);
    
    // 设置极性
    this->SetPolarity(this->Pol);
    
    // 设置周期
    this->SetPeriod(this->Period);
    
    // 设置占空比
    this->SetDutyCycle(this->Duty_cycle);
    
    // 启动计数器和预装载值
    LS_writel(AddressMap(LS_ATIM_BASE_ADDR + LS_ATIM_CR1, PAGE_SIZE), 0b10000001);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void AtimPwm::SetMode(uint8_t mode)
 * @功能说明：设置 PWM 模式
 * @参数说明：mode  :  设置的模式
 * @函数返回：无
 * @调用方法：MyPwm.SetMode(LS_ATIM_Mode_2);
 * @备注说明：设置 LS_ATIM_Mode_1 为模式 1，设置 LS_ATIM_Mode_2 为模式 2
 *           默认设置为模式 2
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void AtimPwm::SetMode(uint8_t mode)
{
    this->Mode = mode;
    
    // 配置 chNum 的 PWM 模式
    uint32_t reg = LS_readl(this->ATIM_CCMR[this->ChNum / 2]) & ~(0x7 << ((ChNum % 2) * 8 + 4)) | (mode << ((ChNum % 2) * 8 + 4));
    
    // 输出模式
    reg &= ~(0x03 << (this->ChNum % 2) * 8);
    
    // 开启预装载
    reg &= ~(1 << ((this->ChNum % 2) * 8 + 3));
    reg |= (1 << ((this->ChNum % 2) * 8 + 3));
    
    LS_writel(this->ATIM_CCMR[ChNum / 2], reg);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：AtimPwm::~AtimPwm(void)
 * @功能说明：ATIM 控制器 PWM 模式的析构函数
 * @参数说明：无
 * @函数返回：无
 * @调用方法：创建的对象生命周期结束后系统自动调用
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
AtimPwm::~AtimPwm()
{
    munmap(this->ATIM_CCMR[0], PAGE_SIZE);
    munmap(this->ATIM_CCMR[1], PAGE_SIZE);
    munmap(this->ATIM_ARR, PAGE_SIZE);
    munmap(this->ATIM_CCRx, PAGE_SIZE);
    munmap(this->ATIM_CCER, PAGE_SIZE);
    munmap(this->ATIM_CNT, PAGE_SIZE);
    munmap(this->ATIM_BDTR, PAGE_SIZE);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void AtimPwm::Enable(void)
 * @功能说明：使能 PWM
 * @参数说明：无
 * @函数返回：无
 * @调用方法：MyPwm.Enable();
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void AtimPwm::Enable()
{
    LS_writel(this->ATIM_CCER, LS_readl(this->ATIM_CCER) | (0x1 << (this->ChNum * 4 + 0)));
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void AtimPwm::Disable(void)
 * @功能说明：失能 PWM
 * @参数说明：无
 * @函数返回：无
 * @调用方法：MyPwm.Disable();
 * @备注说明：无
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void AtimPwm::Disable()
{
    LS_writel(this->ATIM_CCER, LS_readl(this->ATIM_CCER) & ~(0x1 << (this->ChNum * 4 + 0)));
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void AtimPwm::SetPolarity(uint8_t pol)
 * @功能说明：设置 PWM 极性
 * @参数说明：pol  :  所设置的极性
 * @函数返回：无
 * @调用方法：MyPwm.SetPolarity(LS_ATIM_INVERSED);
 * @备注说明：.h 文件中的宏，设置 LS_ATIM_NORMAL 为正常输出，设置 LS_ATIM_INVERSED 为反相输出
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void AtimPwm::SetPolarity(uint8_t pol)
{
    // 配置 chNum 的输出极性
    uint32_t reg = LS_readl(this->ATIM_CCER) & ~(0x1 << (this->ChNum * 4 + 1)) | (pol << (this->ChNum * 4 + 1));
    LS_writel(this->ATIM_CCER, reg);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void AtimPwm::SetPeriod(uint32_t cycle)
 * @功能说明：设置 PWM 周期
 * @参数说明：cycle  :  周期值
 * @函数返回：无
 * @调用方法：MyPwm.SetPeriod(50);
 * @备注说明：假设要设置 50Hz 周期，那该参数直接设置为 50 即可，单位为 Hz
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void AtimPwm::SetPeriod(uint32_t cycle)
{
    this->Disable();
    this->Period = cycle;
    uint32_t val = LS_ATIM_CLK / this->Period - 1;
    LS_writel(this->ATIM_ARR, val);
    this->Enable();
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 * @函数名称：void AtimPwm::SetDutyCycle(uint32_t duty_cycle)
 * @功能说明：设置 PWM 占空比
 * @参数说明：duty_cycle  :  占空比
 * @函数返回：无
 * @调用方法：MyPwm.SetDutyCycle(2000);
 * @备注说明：无论周期设置为多少, 占空比范围皆是 0-10000
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void AtimPwm::SetDutyCycle(uint32_t duty_cycle)
{
    pthread_mutex_lock(&Atim_mutex);
    this->Duty_cycle = duty_cycle;
    uint32_t val = LS_ATIM_CLK / this->Period * this->Duty_cycle / LS_ATIM_DUTY_MAX;
    LS_writel(this->ATIM_CCRx, val);
    pthread_mutex_unlock(&Atim_mutex);
}