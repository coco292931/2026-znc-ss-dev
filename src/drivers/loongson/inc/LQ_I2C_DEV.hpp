#pragma once

#include <string>
#include <mutex>
#include <cstdint>
#include <cerrno>
#include <cstdio>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define LS_IIC_PATH "/dev/ls_iic"

// 龙芯I2C驱动相关使用类，基类，只能由子类继承并且只能创建一个
class LS_I2C_DEVS
{
public:
    // 禁止拷贝构造和赋值运算符
    LS_I2C_DEVS(const LS_I2C_DEVS &) = delete;
    LS_I2C_DEVS &operator=(const LS_I2C_DEVS &) = delete;

    /*!
     * @brief   获取全局唯一I2C实例
     * @param   path : I2C 驱动生成的设备文件路径
     * @return  返回全局唯一I2C实例
     * @date    2025/11/20
     */
    static LS_I2C_DEVS &GetInstance(const std::string &path = "");

    /*!
     * @brief   初始化I2C总线(打开I2C设备文件)
     * @param   path : I2C 驱动生成的设备文件路径
     * @return  成功返回 0，失败返回 -1
     * @date    2025/11/20
     */
    int8_t I2C_Init(const std::string &path);

    /*!
     * @brief   析构函数，关闭I2C设备文件描述符
     * @param   无
     * @return  无
     * @date    2025/11/20
     */
    ~LS_I2C_DEVS();

protected:
    /*!
     * @brief   私有构造函数，防止外部直接创建对象
     */
    LS_I2C_DEVS() = default;
    explicit LS_I2C_DEVS(const std::string &path)
    {
        I2C_Init(path);
    }

    // 静态成员：全局唯一(所有子类对象共享)
    static int I2C_fd;           // I2C设备文件描述符
    static std::mutex i2c_mutex; // 互斥锁，保护线程安全
};
