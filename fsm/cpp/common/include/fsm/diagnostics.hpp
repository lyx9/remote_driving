#pragma once

// ============================================================================
// FSM-Pilot  |  Diagnostics Manager
// ============================================================================
// Structured, per-vehicle diagnostic reporting for vehicle debugging.
//
// Provides a lightweight key-value store of named diagnostic items, each with
// a status level (OK / WARN / ERROR / STALE) and a value.  Designed for:
//   - Real-time dashboard display via REST API
//   - Fault correlation with SafetyMonitor
//   - Per-vehicle debug channel accessible without ROS2 tooling
//
// Thread-safe.  No dynamic allocation in the update hot path.
// ============================================================================

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace fsm {

// ----------------------------------------------------------------------------
// Diagnostic item status (mirrors ROS2 diagnostic_msgs/DiagnosticStatus).
// ----------------------------------------------------------------------------
enum class DiagnosticStatus : uint8_t {
  kOk    = 0U,  // Operating normally
  kWarn  = 1U,  // Warning — monitor closely
  kError = 2U,  // Error — action required
  kStale = 3U,  // No data received in expected window
};

// ----------------------------------------------------------------------------
// A single diagnostic item.
// ----------------------------------------------------------------------------
struct DiagnosticItem {
  std::string     key;           // Unique identifier, e.g. "webrtc.rtt_ms"
  std::string     display_name;  // Human-readable label
  std::string     value_str;     // String representation of the value
  double          value_double  = 0.0;  // Numeric value (NaN if non-numeric)
  std::string     unit;          // Physical unit, e.g. "ms", "%", "m/s"
  DiagnosticStatus status       = DiagnosticStatus::kOk;
  int64_t          timestamp_ns = 0;
  std::string      subsystem;   // Owning subsystem, e.g. "webrtc", "sensor"
};

// ----------------------------------------------------------------------------
// DiagnosticsManager
//
// Aggregates diagnostic items from multiple subsystems.  Items are identified
// by a string key; repeated updates to the same key overwrite the previous
// value (last-write-wins).
//
// Design:
//   - Items stored in a flat vector for cache-friendly iteration.
//   - Find-or-insert is O(N) on the item count (typically < 64 items).
//   - No heap allocation in the Update() fast path (items pre-sized).
// ----------------------------------------------------------------------------
class DiagnosticsManager {
 public:
  // Pre-allocate capacity for expected number of diagnostic items.
  explicit DiagnosticsManager(size_t reserve_capacity = 64U);
  ~DiagnosticsManager() = default;

  DiagnosticsManager(const DiagnosticsManager&)            = delete;
  DiagnosticsManager& operator=(const DiagnosticsManager&) = delete;

  // ── Update API ────────────────────────────────────────────────────────────

  // Update a numeric diagnostic item.
  void Update(const std::string&  key,
              const std::string&  display_name,
              double              value,
              const std::string&  unit,
              DiagnosticStatus    status,
              const std::string&  subsystem = "");

  // Update a string-value diagnostic item.
  void UpdateStr(const std::string&  key,
                 const std::string&  display_name,
                 const std::string&  value,
                 DiagnosticStatus    status,
                 const std::string&  subsystem = "");

  // Mark all items in a subsystem as STALE (call on subsystem timeout).
  void MarkSubsystemStale(const std::string& subsystem);

  // Remove all items for a subsystem.
  void ClearSubsystem(const std::string& subsystem);

  // ── Query API ─────────────────────────────────────────────────────────────

  // All diagnostic items (snapshot — thread-safe copy).
  [[nodiscard]] std::vector<DiagnosticItem> GetAll() const;

  // Items whose status is kWarn, kError, or kStale.
  [[nodiscard]] std::vector<DiagnosticItem> GetFailed() const;

  // Count of items by status.
  [[nodiscard]] size_t count_ok()    const;
  [[nodiscard]] size_t count_warn()  const;
  [[nodiscard]] size_t count_error() const;
  [[nodiscard]] size_t count_stale() const;

  // Overall health: kOk if no ERROR/STALE items, kWarn if any WARN.
  [[nodiscard]] DiagnosticStatus overall_status() const;

  // Serialise all items to JSON for REST API response.
  [[nodiscard]] nlohmann::json ToJson() const;

 private:
  // Find existing item by key (not locked — caller must hold mu_).
  DiagnosticItem* FindItem(const std::string& key);
  const DiagnosticItem* FindItem(const std::string& key) const;

  mutable std::mutex mu_;
  std::vector<DiagnosticItem> items_;
};

// Helper: convert DiagnosticStatus to a string for JSON/logging.
inline const char* DiagnosticStatusToString(DiagnosticStatus s) noexcept {
  switch (s) {
    case DiagnosticStatus::kOk:    return "ok";
    case DiagnosticStatus::kWarn:  return "warn";
    case DiagnosticStatus::kError: return "error";
    case DiagnosticStatus::kStale: return "stale";
    default:                       return "unknown";
  }
}

}  // namespace fsm
