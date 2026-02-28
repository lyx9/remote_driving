#include "fsm/safety_monitor.hpp"
#include "fsm/logger.hpp"
#include "fsm/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

namespace fsm {

// ── Constructor / Destructor ────────────────────────────────────────────────

SafetyMonitor::SafetyMonitor(uint32_t watchdog_timeout_ms) noexcept
    : watchdog_timeout_ms_(watchdog_timeout_ms) {
  last_feed_ns_.store(utils::NowNanos(), std::memory_order_relaxed);
}

SafetyMonitor::~SafetyMonitor() {
  Stop();
}

// ── Lifecycle ────────────────────────────────────────────────────────────────

void SafetyMonitor::Start() {
  if (running_.exchange(true, std::memory_order_acq_rel)) {
    FSM_LOG_WARN("SafetyMonitor: already running");
    return;
  }
  last_feed_ns_.store(utils::NowNanos(), std::memory_order_relaxed);
  watchdog_thread_ = std::thread(&SafetyMonitor::WatchdogThreadLoop, this);
  FSM_LOG_INFO("SafetyMonitor started (watchdog={}ms, ASIL-B)",
               watchdog_timeout_ms_);
}

void SafetyMonitor::Stop() {
  if (!running_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  if (watchdog_thread_.joinable()) {
    watchdog_thread_.join();
  }
  FSM_LOG_INFO("SafetyMonitor stopped");
}

// ── Watchdog feed (lock-free hot path) ───────────────────────────────────────

void SafetyMonitor::Feed() noexcept {
  last_feed_ns_.store(utils::NowNanos(), std::memory_order_relaxed);
}

uint32_t SafetyMonitor::ms_since_last_feed() const noexcept {
  const int64_t now  = utils::NowNanos();
  const int64_t last = last_feed_ns_.load(std::memory_order_relaxed);
  const int64_t diff = now - last;
  if (diff <= 0) {
    return 0U;
  }
  return static_cast<uint32_t>(diff / 1'000'000LL);
}

// ── Fault reporting ──────────────────────────────────────────────────────────

void SafetyMonitor::ReportFault(FaultCode          code,
                                AsilLevel          level,
                                const std::string& subsystem,
                                const std::string& description) {
  FaultEvent evt;
  evt.fault_code      = code;
  evt.asil_level      = level;
  evt.timestamp_ns    = utils::NowNanos();
  evt.active          = true;
  evt.acknowledged    = false;
  evt.resulting_state = MinStateForAsil(level);

  // Fixed-size copy — no heap allocation in fault path.
  std::strncpy(evt.subsystem,   subsystem.c_str(),   sizeof(evt.subsystem)   - 1U);
  std::strncpy(evt.description, description.c_str(), sizeof(evt.description) - 1U);
  evt.subsystem[sizeof(evt.subsystem)   - 1U] = '\0';
  evt.description[sizeof(evt.description) - 1U] = '\0';

  FSM_LOG_ERROR("[SafetyMonitor] Fault 0x{:04X} ({}) in '{}': {}",
                static_cast<uint32_t>(code),
                (level >= AsilLevel::kAsilB) ? "ASIL-B" : "QM/A",
                subsystem, description);

  std::lock_guard<std::mutex> lock(mu_);
  PushFault(evt);
  TransitionTo(evt.resulting_state, evt);
}

void SafetyMonitor::AcknowledgeFault(FaultCode code) noexcept {
  std::lock_guard<std::mutex> lock(mu_);
  for (auto& slot : fault_ring_) {
    if (slot.active && slot.fault_code == code) {
      slot.acknowledged = true;
      return;
    }
  }
}

bool SafetyMonitor::ClearFault(FaultCode code) noexcept {
  std::lock_guard<std::mutex> lock(mu_);
  for (auto& slot : fault_ring_) {
    if (slot.active && slot.fault_code == code) {
      if (slot.asil_level >= AsilLevel::kAsilB) {
        FSM_LOG_WARN("SafetyMonitor: ASIL-B fault 0x{:04X} cannot be cleared without Reset()",
                     static_cast<uint32_t>(code));
        return false;
      }
      slot.active = false;
      if (fault_count_ > 0U) {
        --fault_count_;
      }
      FSM_LOG_INFO("SafetyMonitor: Fault 0x{:04X} cleared", static_cast<uint32_t>(code));
      return true;
    }
  }
  return false;  // Not found
}

// ── State queries ────────────────────────────────────────────────────────────

SafetyState SafetyMonitor::state() const noexcept {
  return state_.load(std::memory_order_acquire);
}

bool SafetyMonitor::is_remote_control_permitted() const noexcept {
  const SafetyState s = state_.load(std::memory_order_acquire);
  return (s == SafetyState::kNominal) || (s == SafetyState::kDegraded);
}

std::vector<FaultEvent> SafetyMonitor::GetActiveFaults() const {
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<FaultEvent> result;
  result.reserve(fault_count_);
  // Iterate ring newest-first.
  for (uint32_t i = 0U; i < kMaxFaults; ++i) {
    const uint32_t idx = (fault_head_ + kMaxFaults - 1U - i) % kMaxFaults;
    if (fault_ring_[idx].active) {
      result.push_back(fault_ring_[idx]);
    }
  }
  return result;
}

// ── Actions & callbacks ───────────────────────────────────────────────────────

void SafetyMonitor::RegisterFailSafeAction(std::function<void()> action) {
  std::lock_guard<std::mutex> lock(mu_);
  fail_safe_actions_.push_back(std::move(action));
}

void SafetyMonitor::set_state_change_callback(SafetyStateCallback cb) {
  std::lock_guard<std::mutex> lock(mu_);
  state_change_cb_ = std::move(cb);
}

// ── Recovery ─────────────────────────────────────────────────────────────────

bool SafetyMonitor::Reset() {
  std::lock_guard<std::mutex> lock(mu_);
  // Check for any remaining active ASIL-B+ faults.
  for (const auto& slot : fault_ring_) {
    if (slot.active && slot.asil_level >= AsilLevel::kAsilB) {
      FSM_LOG_WARN("SafetyMonitor::Reset() blocked by ASIL-B fault 0x{:04X}",
                   static_cast<uint32_t>(slot.fault_code));
      return false;
    }
  }
  // Clear all QM/ASIL-A faults and reset state.
  for (auto& slot : fault_ring_) {
    slot.active = false;
  }
  fault_count_ = 0U;
  state_.store(SafetyState::kNominal, std::memory_order_release);
  last_feed_ns_.store(utils::NowNanos(), std::memory_order_relaxed);
  FSM_LOG_INFO("SafetyMonitor: Reset to kNominal");
  return true;
}

// ── Private helpers ──────────────────────────────────────────────────────────

SafetyState SafetyMonitor::MinStateForAsil(AsilLevel level) noexcept {
  switch (level) {
    case AsilLevel::kQm:
      return SafetyState::kDegraded;
    case AsilLevel::kAsilA:
      return SafetyState::kDegraded;
    case AsilLevel::kAsilB:
      return SafetyState::kFault;
    case AsilLevel::kAsilC:
      return SafetyState::kFailSafe;
    case AsilLevel::kAsilD:
      return SafetyState::kFailSafe;
    default:
      return SafetyState::kFault;  // Conservative default
  }
}

void SafetyMonitor::PushFault(const FaultEvent& evt) noexcept {
  // Overwrite the oldest slot (ring buffer).
  fault_ring_[fault_head_] = evt;
  fault_head_ = (fault_head_ + 1U) % kMaxFaults;
  if (fault_count_ < kMaxFaults) {
    ++fault_count_;
  }
}

void SafetyMonitor::TransitionTo(SafetyState next, const FaultEvent& trigger) {
  // State can only increase (kNominal < kDegraded < kFault < kFailSafe).
  const SafetyState current = state_.load(std::memory_order_acquire);
  if (static_cast<uint8_t>(next) <= static_cast<uint8_t>(current)) {
    return;  // Already at or beyond this level — no transition needed.
  }

  state_.store(next, std::memory_order_release);

  const char* state_str = "UNKNOWN";
  switch (next) {
    case SafetyState::kNominal:  state_str = "NOMINAL";   break;
    case SafetyState::kDegraded: state_str = "DEGRADED";  break;
    case SafetyState::kFault:    state_str = "FAULT";     break;
    case SafetyState::kFailSafe: state_str = "FAIL-SAFE"; break;
    default:                     state_str = "UNKNOWN";   break;
  }
  FSM_LOG_CRITICAL("[SafetyMonitor] State → {} (fault 0x{:04X})",
                   state_str, static_cast<uint32_t>(trigger.fault_code));

  if (state_change_cb_) {
    state_change_cb_(next, trigger);
  }

  if (next == SafetyState::kFailSafe) {
    ActivateFailSafe(trigger);
  }
}

void SafetyMonitor::ActivateFailSafe(const FaultEvent& trigger) {
  FSM_LOG_CRITICAL("[SafetyMonitor] FAIL-SAFE ACTIVATED — executing {} actions",
                   fail_safe_actions_.size());
  for (const auto& action : fail_safe_actions_) {
    if (action) {
      action();
    }
  }
  (void)trigger;  // Available for future crash-dump use
}

// ── Watchdog thread ───────────────────────────────────────────────────────────

void SafetyMonitor::WatchdogThreadLoop() {
  // Check every 50 ms (10× faster than typical 500 ms timeout for responsiveness).
  static constexpr int64_t kCheckIntervalMs = 50;

  while (running_.load(std::memory_order_acquire)) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(kCheckIntervalMs));

    if (!running_.load(std::memory_order_acquire)) {
      break;
    }

    const uint32_t elapsed_ms = ms_since_last_feed();

    if (elapsed_ms >= watchdog_timeout_ms_) {
      FSM_LOG_CRITICAL("[SafetyMonitor] Watchdog expired! {}ms without feed "
                       "(limit={}ms)",
                       elapsed_ms, watchdog_timeout_ms_);

      ReportFault(FaultCode::kSysWatchdogExpired,
                  AsilLevel::kAsilB,
                  "safety_monitor",
                  "Watchdog timeout — remote control heartbeat lost");
      // Reset feed time to avoid flooding subsequent iterations.
      last_feed_ns_.store(utils::NowNanos(), std::memory_order_relaxed);

    } else if (elapsed_ms >= (watchdog_timeout_ms_ / 2U)) {
      // Pre-timeout warning at 50% of deadline.
      FSM_LOG_WARN("[SafetyMonitor] Watchdog approaching deadline: {}ms/{}ms",
                   elapsed_ms, watchdog_timeout_ms_);
    }
  }
}

}  // namespace fsm
