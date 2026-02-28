#pragma once

#include <memory>
#include <string>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/fmt/ostr.h>

namespace fsm {

class Logger {
 public:
  enum class Level {
    kTrace    = spdlog::level::trace,
    kDebug    = spdlog::level::debug,
    kInfo     = spdlog::level::info,
    kWarn     = spdlog::level::warn,
    kError    = spdlog::level::err,
    kCritical = spdlog::level::critical,
  };

  // Initializes console + optional rotating file sink.
  // Must be called before any FSM_LOG_* macro is used.
  static void Init(
      const std::string& name = "fsm",
      const std::string& log_file = "",
      Level level = Level::kInfo,
      size_t max_size_mb = 10,
      size_t max_files = 3);

  static std::shared_ptr<spdlog::logger> Get();
  static void SetLevel(Level level);
  static void Flush();
  static void Shutdown();

 private:
  static std::shared_ptr<spdlog::logger> logger_;
  static bool initialized_;
};

}  // namespace fsm

#define FSM_LOG_TRACE(...)    SPDLOG_LOGGER_TRACE(fsm::Logger::Get(),    __VA_ARGS__)
#define FSM_LOG_DEBUG(...)    SPDLOG_LOGGER_DEBUG(fsm::Logger::Get(),    __VA_ARGS__)
#define FSM_LOG_INFO(...)     SPDLOG_LOGGER_INFO(fsm::Logger::Get(),     __VA_ARGS__)
#define FSM_LOG_WARN(...)     SPDLOG_LOGGER_WARN(fsm::Logger::Get(),     __VA_ARGS__)
#define FSM_LOG_ERROR(...)    SPDLOG_LOGGER_ERROR(fsm::Logger::Get(),    __VA_ARGS__)
#define FSM_LOG_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(fsm::Logger::Get(), __VA_ARGS__)
