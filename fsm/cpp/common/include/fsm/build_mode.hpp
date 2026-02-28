#pragma once

// ============================================================================
// FSM-Pilot  |  Build Mode Definitions
// ============================================================================
// Compile-time feature flags for production vs. demo separation.
//
// Set via CMake options:
//   -DFSM_DEMO_MODE=ON       Build with simulated vehicle (no real hardware)
//   -DFSM_DEBUG_VERBOSE=ON   Enable per-vehicle verbose debug logging
//
// Usage:
//   #include "fsm/build_mode.hpp"
//   #ifdef FSM_BUILD_DEMO
//     // demo-only code
//   #endif
//   #ifndef FSM_BUILD_DEMO
//     // production-only code
//   #endif
//
// Design principle (Waymo internal standard):
//   Demo code and production code share the same interfaces but are compiled
//   separately.  The `PlatformAdapter` abstraction provides the seam:
//     - Production: AutowarePlatformAdapter (real ROS2 hardware topics)
//     - Demo:       SimulatedVehicle        (synthetic sine-wave data)
//   No runtime `if (demo_mode)` branching in safety-critical paths.
// ============================================================================

namespace fsm {

// Populated from CMake; see CMakeLists.txt option(FSM_DEMO_MODE ...).
#ifdef FSM_DEMO_MODE
  inline constexpr bool kDemoMode = true;
  inline constexpr const char* kBuildVariant = "DEMO";
#else
  inline constexpr bool kDemoMode = false;
  inline constexpr const char* kBuildVariant = "PRODUCTION";
#endif

#ifdef FSM_DEBUG_VERBOSE
  inline constexpr bool kDebugVerbose = true;
#else
  inline constexpr bool kDebugVerbose = false;
#endif

// FSM version string (updated by CI).
inline constexpr const char* kFsmVersion = "1.2.0";

}  // namespace fsm

// Convenience macros for conditional compilation guards.
#ifdef FSM_DEMO_MODE
  #define FSM_DEMO_ONLY(...)  __VA_ARGS__
  #define FSM_PROD_ONLY(...)
#else
  #define FSM_DEMO_ONLY(...)
  #define FSM_PROD_ONLY(...)  __VA_ARGS__
#endif
