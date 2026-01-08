#pragma once

#include <string>
#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/fmt/ostr.h>

namespace fsm {

/**
 * @brief FSM日志管理器
 *
 * 提供统一的日志接口，支持控制台和文件输出
 */
class Logger {
public:
    /**
     * @brief 日志级别
     */
    enum class Level {
        TRACE = spdlog::level::trace,
        DEBUG = spdlog::level::debug,
        INFO = spdlog::level::info,
        WARN = spdlog::level::warn,
        ERROR = spdlog::level::err,
        CRITICAL = spdlog::level::critical
    };

    /**
     * @brief 初始化日志系统
     * @param name 日志器名称
     * @param log_file 日志文件路径 (可选)
     * @param level 日志级别
     * @param max_size_mb 日志文件最大大小 (MB)
     * @param max_files 最大日志文件数
     */
    static void init(
        const std::string& name = "fsm",
        const std::string& log_file = "",
        Level level = Level::INFO,
        size_t max_size_mb = 10,
        size_t max_files = 3);

    /**
     * @brief 获取日志器实例
     */
    static std::shared_ptr<spdlog::logger> get();

    /**
     * @brief 设置日志级别
     */
    static void setLevel(Level level);

    /**
     * @brief 刷新日志缓冲区
     */
    static void flush();

    /**
     * @brief 关闭日志系统
     */
    static void shutdown();

private:
    static std::shared_ptr<spdlog::logger> logger_;
    static bool initialized_;
};

// 便捷宏定义
#define FSM_LOG_TRACE(...) SPDLOG_LOGGER_TRACE(fsm::Logger::get(), __VA_ARGS__)
#define FSM_LOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(fsm::Logger::get(), __VA_ARGS__)
#define FSM_LOG_INFO(...) SPDLOG_LOGGER_INFO(fsm::Logger::get(), __VA_ARGS__)
#define FSM_LOG_WARN(...) SPDLOG_LOGGER_WARN(fsm::Logger::get(), __VA_ARGS__)
#define FSM_LOG_ERROR(...) SPDLOG_LOGGER_ERROR(fsm::Logger::get(), __VA_ARGS__)
#define FSM_LOG_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(fsm::Logger::get(), __VA_ARGS__)

}  // namespace fsm
