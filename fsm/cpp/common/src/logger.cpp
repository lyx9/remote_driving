#include "fsm/logger.hpp"
#include <iostream>

namespace fsm {

std::shared_ptr<spdlog::logger> Logger::logger_ = nullptr;
bool Logger::initialized_ = false;

void Logger::init(
    const std::string& name,
    const std::string& log_file,
    Level level,
    size_t max_size_mb,
    size_t max_files) {

    if (initialized_) {
        return;
    }

    try {
        std::vector<spdlog::sink_ptr> sinks;

        // 控制台输出 (彩色)
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(static_cast<spdlog::level::level_enum>(level));
        console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%#] %v");
        sinks.push_back(console_sink);

        // 文件输出 (可选)
        if (!log_file.empty()) {
            auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                log_file, max_size_mb * 1024 * 1024, max_files);
            file_sink->set_level(static_cast<spdlog::level::level_enum>(level));
            file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v");
            sinks.push_back(file_sink);
        }

        logger_ = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
        logger_->set_level(static_cast<spdlog::level::level_enum>(level));
        logger_->flush_on(spdlog::level::warn);

        spdlog::register_logger(logger_);
        spdlog::set_default_logger(logger_);

        initialized_ = true;

        logger_->info("FSM Logger initialized: {}", name);

    } catch (const spdlog::spdlog_ex& ex) {
        std::cerr << "Logger initialization failed: " << ex.what() << std::endl;
    }
}

std::shared_ptr<spdlog::logger> Logger::get() {
    if (!initialized_) {
        // 默认初始化
        init();
    }
    return logger_;
}

void Logger::setLevel(Level level) {
    if (logger_) {
        logger_->set_level(static_cast<spdlog::level::level_enum>(level));
    }
}

void Logger::flush() {
    if (logger_) {
        logger_->flush();
    }
}

void Logger::shutdown() {
    if (initialized_) {
        spdlog::shutdown();
        initialized_ = false;
        logger_ = nullptr;
    }
}

}  // namespace fsm
