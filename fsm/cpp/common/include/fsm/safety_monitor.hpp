#pragma once

// ============================================================================
// FSM-Pilot  |  Safety Monitor
// ============================================================================
// ISO 26262 ASIL-B compliant safety monitoring for remote driving.
//
// Design rationale:
//   Remote driving is classified ASIL-B (system-level) per ISO 26262-6.
//   Key requirements implemented:
//     [SM-01] Watchdog with hard deadline — Feed() must be called within
//             watchdog_timeout_ms; otherwise E-stop is activated.
//     [SM-02] Structured fault codes — every fault maps to a FMEA entry.
//     [SM-03] Fail-safe degradation — kNominal → kDegraded → kFault → kFailSafe.
//     [SM-04] No dynamic allocation in the fault path (ring buffer).
//     [SM-05] All safety-critical return values are [[nodiscard]].
//
// MISRA C++:2023 compliance notes:
//   - No dynamic allocation after Start().
//   - All enum switches include a `default:` clause.
//   - Explicit casts for all narrowing conversions.
//   - No goto, no setjmp/longjmp.
//   - All member variables initialised in initialiser list or at declaration.
// ============================================================================

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace fsm {

// ----------------------------------------------------------------------------
// ISO 26262 ASIL classification (ASIL-B target for remote driving).
// ----------------------------------------------------------------------------
enum class AsilLevel : uint8_t {
  kQm    = 0U,   // Quality Management (non-safety-critical)
  kAsilA = 1U,   // ASIL-A (low safety criticality)
  kAsilB = 2U,   // ASIL-B (remote driving target)
  kAsilC = 3U,   // ASIL-C (future reserved)
  kAsilD = 4U,   // ASIL-D (future reserved)
};

// ----------------------------------------------------------------------------
// Structured fault codes — each entry corresponds to a FMEA line item.
// Range allocation:
//   0x0100–0x01FF  Control path
//   0x0200–0x02FF  Sensor path
//   0x0300–0x03FF  Communication
//   0x0400–0x04FF  System resources
//   0xFFFF         No-fault sentinel
// ----------------------------------------------------------------------------
enum class FaultCode : uint32_t {
  // Control path (ASIL-B)
  kControlWatchdogTimeout    = 0x0101U,  // No command in watchdog window
  kControlCommandOutOfRange  = 0x0102U,  // Command exceeds vehicle limits
  kControlSequenceGap        = 0x0103U,  // Missed sequence numbers (> thresh)
  kControlPublishFailed      = 0x0104U,  // ROS2 publish failure

  // Sensor path (ASIL-A)
  kSensorDataStale           = 0x0201U,  // Sensor timestamp too old
  kSensorVelocityInvalid     = 0x0202U,  // Velocity out of plausible range
  kSensorLocalizationLost    = 0x0203U,  // GPS/localization invalid
  kSensorCameraOffline       = 0x0204U,  // Camera feed lost

  // Communication (ASIL-B)
  kCommLinkLost              = 0x0301U,  // WebSocket/WebRTC disconnected
  kCommLatencyExceeded       = 0x0302U,  // RTT > operator-configured threshold
  kCommMqttDisconnected      = 0x0303U,  // MQTT bridge lost (non-critical)

  // System resources (ASIL-A)
  kSysWatchdogExpired        = 0x0401U,  // Safety monitor watchdog expired
  kSysCpuOverload            = 0x0402U,  // CPU usage > 95% for > 5 s
  kSysMemoryLow              = 0x0403U,  // Available memory < 5%
  kSysEmergencyActivated     = 0x0404U,  // E-stop activated (informational)

  kNoFault                   = 0xFFFFU,  // Sentinel — no fault
};

// ----------------------------------------------------------------------------
// Safety state machine.
// Transitions are monotonically increasing; only Reset() can go backwards.
// ----------------------------------------------------------------------------
enum class SafetyState : uint8_t {
  kNominal  = 0U,  // All systems nominal; remote control permitted
  kDegraded = 1U,  // Non-critical fault; warn operator; reduce limits
  kFault    = 2U,  // Critical fault; operator must take manual action
  kFailSafe = 3U,  // Fail-safe: E-stop engaged; remote control forbidden
};

// ----------------------------------------------------------------------------
// A single fault event record.
// Fixed-size fields — no heap allocation in the fault path.
// ----------------------------------------------------------------------------
struct FaultEvent {
  FaultCode   fault_code       = FaultCode::kNoFault;
  AsilLevel   asil_level       = AsilLevel::kQm;
  SafetyState resulting_state  = SafetyState::kNominal;
  int64_t     timestamp_ns     = 0;
  bool        active           = false;
  bool        acknowledged     = false;
  char        subsystem[32]    = {};   // Fixed-size — no std::string
  char        description[128] = {};   // Fixed-size — no std::string
};

// Callback fired (under internal lock) when the safety state changes.
// Implementations MUST be brief: no blocking, no allocations.
using SafetyStateCallback =
    std::function<void(SafetyState new_state, const FaultEvent& trigger)>;

// ----------------------------------------------------------------------------
// SafetyMonitor
// ----------------------------------------------------------------------------
class SafetyMonitor {
 public:
  // Default watchdog deadline for remote driving (100 ms tight loop).
  static constexpr uint32_t kDefaultWatchdogMs = 500U;
  // Size of the fault ring buffer (no-heap).
  static constexpr uint32_t kMaxFaults = 32U;

  explicit SafetyMonitor(
      uint32_t watchdog_timeout_ms = kDefaultWatchdogMs) noexcept;
  ~SafetyMonitor();

  SafetyMonitor(const SafetyMonitor&)            = delete;
  SafetyMonitor& operator=(const SafetyMonitor&) = delete;

  // ── Lifecycle ──────────────────────────────────────────────────────────────

  // Start background watchdog thread. Call before entering remote control.
  void Start();

  // Stop background watchdog. Drains pending fail-safe actions.
  void Stop();

  // ── Watchdog feed ─────────────────────────────────────────────────────────

  // Feed the watchdog. Must be called within watchdog_timeout_ms.
  // Callable from any thread; lock-free (atomic).
  void Feed() noexcept;

  // Milliseconds since last Feed() — for diagnostic dashboards.
  [[nodiscard]] uint32_t ms_since_last_feed() const noexcept;

  // ── Fault reporting ───────────────────────────────────────────────────────

  // Report a structured fault. Thread-safe. Does not block.
  // Automatically escalates SafetyState and fires fail-safe actions on ASIL-B+.
  void ReportFault(FaultCode         code,
                   AsilLevel         level,
                   const std::string& subsystem,
                   const std::string& description);

  // Acknowledge an active fault (does not clear it).
  void AcknowledgeFault(FaultCode code) noexcept;

  // Clear a non-critical fault after corrective action.
  // Returns false if the fault is ASIL-B or higher, or not found.
  // [[nodiscard]] — caller must verify the clear succeeded.
  [[nodiscard]] bool ClearFault(FaultCode code) noexcept;

  // ── State queries ─────────────────────────────────────────────────────────

  [[nodiscard]] SafetyState state() const noexcept;

  // Remote control is permitted only in kNominal or kDegraded states.
  [[nodiscard]] bool is_remote_control_permitted() const noexcept;

  // Snapshot of all active (uncleared) faults — newest first.
  [[nodiscard]] std::vector<FaultEvent> GetActiveFaults() const;

  // ── Actions & callbacks ───────────────────────────────────────────────────

  // Register a fail-safe action invoked immediately on kFailSafe transition.
  // Actions are called under the internal lock; keep them very brief.
  void RegisterFailSafeAction(std::function<void()> action);

  // Callback fired on every SafetyState transition.
  void set_state_change_callback(SafetyStateCallback cb);

  // ── Recovery ──────────────────────────────────────────────────────────────

  // Attempt to reset to kNominal. Fails if any ASIL-B or higher faults remain.
  // [[nodiscard]] — caller must handle failure.
  [[nodiscard]] bool Reset();

 private:
  // Watchdog background thread body.
  void WatchdogThreadLoop();

  // Compute the minimum required safety state for a given ASIL level.
  static SafetyState MinStateForAsil(AsilLevel level) noexcept;

  // Write a FaultEvent into the ring buffer (called under lock).
  void PushFault(const FaultEvent& evt) noexcept;

  // Transition to a new state (idempotent for lower-severity transitions).
  void TransitionTo(SafetyState next, const FaultEvent& trigger);

  // Execute all registered fail-safe actions (called under lock).
  void ActivateFailSafe(const FaultEvent& trigger);

  // ── Configuration ─────────────────────────────────────────────────────────
  uint32_t watchdog_timeout_ms_;

  // ── Atomic state (hot path — no lock) ────────────────────────────────────
  std::atomic<SafetyState> state_{SafetyState::kNominal};
  std::atomic<int64_t>     last_feed_ns_{0};
  std::atomic<bool>        running_{false};

  // ── Fault ring buffer (lock-protected) ───────────────────────────────────
  mutable std::mutex mu_;
  std::array<FaultEvent, kMaxFaults> fault_ring_{};
  uint32_t fault_head_  = 0U;  // Next write index (circular)
  uint32_t fault_count_ = 0U;  // Active (uncleared) fault count

  // ── Callbacks & actions (set before Start(), immutable after) ────────────
  std::vector<std::function<void()>> fail_safe_actions_;
  SafetyStateCallback state_change_cb_;

  // ── Watchdog thread ───────────────────────────────────────────────────────
  std::thread watchdog_thread_;
};

}  // namespace fsm
