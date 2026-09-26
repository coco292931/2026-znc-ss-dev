#ifndef LQ_MODULE_LOADER_HPP
#define LQ_MODULE_LOADER_HPP

#include <string>
#include <vector>
#include <unordered_map>

class LQModuleLoader {
public:
    /**
     * @brief 构造函数，设置模块路径和模块列表
     * @param modules 模块列表，key为模块显示名，value为模块文件名
     */
    LQModuleLoader(const std::unordered_map<std::string, std::string>& modules);
    
    /**
     * @brief 加载所有模块
     * @return true: 所有模块加载成功, false: 加载失败
     */
    bool loadAllModules();
    
    /**
     * @brief 卸载所有模块
     * @return true: 所有模块卸载成功, false: 卸载失败
     */
    bool unloadAllModules();
    
    /**
     * @brief 加载指定模块
     * @param moduleName 模块显示名（与构造函数中的key对应）
     * @return true: 加载成功, false: 加载失败
     */
    bool loadModule(const std::string& moduleName);
    
    /**
     * @brief 卸载指定模块
     * @param moduleName 模块显示名（与构造函数中的key对应）
     * @return true: 卸载成功, false: 卸载失败
     */
    bool unloadModule(const std::string& moduleName);
    
    /**
     * @brief 检查模块是否已加载
     * @param moduleName 模块显示名
     * @return true: 已加载, false: 未加载
     */
    bool isModuleLoaded(const std::string& moduleName);
    
    /**
     * @brief 获取模块加载状态
     * @return 所有模块的加载状态
     */
    std::unordered_map<std::string, bool> getModuleStatus();
    
private:
    /**
     * @brief 执行shell命令并获取返回结果
     * @param command 要执行的命令
     * @return 命令执行结果
     */
    std::string executeCommand(const std::string& command);
    
    /**
     * @brief 检查模块是否在lsmod输出中
     * @param moduleBaseName 模块基本名（不带.ko）
     * @return true: 存在, false: 不存在
     */
    bool checkModuleInLsmod(const std::string& moduleBaseName);
    
    /**
     * @brief 从完整路径中提取模块基本名（不带路径和.ko后缀）
     * @param fullPath 完整路径
     * @return 模块基本名
     */
    std::string extractModuleBaseName(const std::string& fullPath);
    
    /**
     * @brief 从模块文件名中提取基本名（不带.ko后缀）
     * @param fileName 模块文件名
     * @return 模块基本名
     */
    std::string getModuleBaseName(const std::string& fileName);
    
private:
    // 模块列表：key为模块显示名，value为模块文件名
    std::unordered_map<std::string, std::string> modules_;
    
    // 模块加载状态
    std::unordered_map<std::string, bool> moduleStatus_;
    
    // 默认模块路径（用户主目录）
    std::string modulePath_ = "~/";
    
    // 模块加载顺序（某些模块可能有依赖关系）
    std::vector<std::string> loadOrder_;
    
    // 模块卸载顺序（与加载顺序相反）
    std::vector<std::string> unloadOrder_;
};

#endif // LQ_MODULE_LOADER_HPP