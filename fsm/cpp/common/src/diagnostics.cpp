#include "fsm/diagnostics.hpp"
#include "fsm/utils.hpp"

#include <algorithm>
#include <cmath>

namespace fsm {

// ── Constructor ──────────────────────────────────────────────────────────────

DiagnosticsManager::DiagnosticsManager(size_t reserve_capacity) {
  items_.reserve(reserve_capacity);
}

// ── Update API ────────────────────────────────────────────────────────────────

void DiagnosticsManager::Update(const std::string& key,
                                const std::string& display_name,
                                double             value,
                                const std::string& unit,
                                DiagnosticStatus   status,
                                const std::string& subsystem) {
  std::lock_guard<std::mutex> lock(mu_);
  DiagnosticItem* item = FindItem(key);
  if (item == nullptr) {
    items_.emplace_back();
    item = &items_.back();
    item->key = key;
  }
  item->display_name  = display_name;
  item->value_double  = value;
  item->value_str     = std::to_string(value);
  item->unit          = unit;
  item->status        = status;
  item->timestamp_ns  = utils::NowNanos();
  item->subsystem     = subsystem;
}

void DiagnosticsManager::UpdateStr(const std::string& key,
                                   const std::string& display_name,
                                   const std::string& value,
                                   DiagnosticStatus   status,
                                   const std::string& subsystem) {
  std::lock_guard<std::mutex> lock(mu_);
  DiagnosticItem* item = FindItem(key);
  if (item == nullptr) {
    items_.emplace_back();
    item = &items_.back();
    item->key = key;
  }
  item->display_name  = display_name;
  item->value_str     = value;
  item->value_double  = std::numeric_limits<double>::quiet_NaN();
  item->status        = status;
  item->timestamp_ns  = utils::NowNanos();
  item->subsystem     = subsystem;
}

void DiagnosticsManager::MarkSubsystemStale(const std::string& subsystem) {
  std::lock_guard<std::mutex> lock(mu_);
  for (auto& item : items_) {
    if (item.subsystem == subsystem) {
      item.status = DiagnosticStatus::kStale;
    }
  }
}

void DiagnosticsManager::ClearSubsystem(const std::string& subsystem) {
  std::lock_guard<std::mutex> lock(mu_);
  items_.erase(
      std::remove_if(items_.begin(), items_.end(),
                     [&subsystem](const DiagnosticItem& it) {
                       return it.subsystem == subsystem;
                     }),
      items_.end());
}

// ── Query API ─────────────────────────────────────────────────────────────────

std::vector<DiagnosticItem> DiagnosticsManager::GetAll() const {
  std::lock_guard<std::mutex> lock(mu_);
  return items_;  // Thread-safe copy
}

std::vector<DiagnosticItem> DiagnosticsManager::GetFailed() const {
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<DiagnosticItem> result;
  for (const auto& item : items_) {
    if (item.status != DiagnosticStatus::kOk) {
      result.push_back(item);
    }
  }
  return result;
}

size_t DiagnosticsManager::count_ok() const {
  std::lock_guard<std::mutex> lock(mu_);
  return static_cast<size_t>(std::count_if(items_.begin(), items_.end(),
      [](const DiagnosticItem& it) {
        return it.status == DiagnosticStatus::kOk;
      }));
}

size_t DiagnosticsManager::count_warn() const {
  std::lock_guard<std::mutex> lock(mu_);
  return static_cast<size_t>(std::count_if(items_.begin(), items_.end(),
      [](const DiagnosticItem& it) {
        return it.status == DiagnosticStatus::kWarn;
      }));
}

size_t DiagnosticsManager::count_error() const {
  std::lock_guard<std::mutex> lock(mu_);
  return static_cast<size_t>(std::count_if(items_.begin(), items_.end(),
      [](const DiagnosticItem& it) {
        return it.status == DiagnosticStatus::kError;
      }));
}

size_t DiagnosticsManager::count_stale() const {
  std::lock_guard<std::mutex> lock(mu_);
  return static_cast<size_t>(std::count_if(items_.begin(), items_.end(),
      [](const DiagnosticItem& it) {
        return it.status == DiagnosticStatus::kStale;
      }));
}

DiagnosticStatus DiagnosticsManager::overall_status() const {
  std::lock_guard<std::mutex> lock(mu_);
  DiagnosticStatus worst = DiagnosticStatus::kOk;
  for (const auto& item : items_) {
    if (static_cast<uint8_t>(item.status) >
        static_cast<uint8_t>(worst)) {
      worst = item.status;
    }
  }
  return worst;
}

nlohmann::json DiagnosticsManager::ToJson() const {
  std::lock_guard<std::mutex> lock(mu_);
  nlohmann::json arr = nlohmann::json::array();
  for (const auto& item : items_) {
    nlohmann::json obj;
    obj["key"]          = item.key;
    obj["display_name"] = item.display_name;
    obj["value"]        = item.value_str;
    obj["unit"]         = item.unit;
    obj["status"]       = DiagnosticStatusToString(item.status);
    obj["subsystem"]    = item.subsystem;
    obj["timestamp_ns"] = item.timestamp_ns;
    if (!std::isnan(item.value_double)) {
      obj["value_double"] = item.value_double;
    }
    arr.push_back(std::move(obj));
  }

  nlohmann::json result;
  result["items"]          = std::move(arr);
  result["overall_status"] = DiagnosticStatusToString(overall_status());
  result["counts"]["ok"]   = count_ok();
  result["counts"]["warn"] = count_warn();
  result["counts"]["error"]= count_error();
  result["counts"]["stale"]= count_stale();
  // Note: counts computed without lock above — approximate under race,
  // acceptable for diagnostic display.
  return result;
}

// ── Private helpers ───────────────────────────────────────────────────────────

DiagnosticItem* DiagnosticsManager::FindItem(const std::string& key) {
  for (auto& item : items_) {
    if (item.key == key) {
      return &item;
    }
  }
  return nullptr;
}

const DiagnosticItem* DiagnosticsManager::FindItem(
    const std::string& key) const {
  for (const auto& item : items_) {
    if (item.key == key) {
      return &item;
    }
  }
  return nullptr;
}

}  // namespace fsm
