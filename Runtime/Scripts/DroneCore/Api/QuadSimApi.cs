// Assets/Scripts/DroneCore/Api/QuadSimApi.cs
// Phase 5: Per-Drone API Facade
//
// PURPOSE:
//   Single entry point for controlling a specific drone.
//   Internal scripts, External RPC, and (eventually) UI all interact
//   with the drone through this facade rather than touching subsystems directly.
//
// OWNERSHIP:
//   - Delegates to: QuadPawn, FlightCommandProxy, ModeCoordinator, SensorManager
//   - Does NOT implement logic itself — pure facade pattern
//   - Lives on the drone prefab (added by DroneRootBootstrap)
//   - Initialized by QuadPawn.InitializeSubsystems()
//
// ACCESS PATTERN:
//   SimCoreApi handles sim-wide concerns (time, authority) — no drone references.
//   QuadSimApi handles per-drone concerns — and provides static accessors for convenience:
//     QuadSimApi.Selected()  → currently selected drone
//     QuadSimApi.Get(0)      → drone by index
//   This keeps dependency direction clean: DroneCore → SimCore, never the reverse.
//
// TELEMETRY DESIGN:
//   - GetSensorData() → measured state (IMU + GPS). Use for control algorithms.
//   - GetTelemetry()  → operational state + controller outputs. Use for logging/monitoring.
//   - No Rigidbody ground truth in either path.

using System;
using UnityEngine;
using SimCore;
using SimCore.Common;
using DroneCore.Controllers;
using DroneCore.Core;
using DroneCore.Interfaces;
using RobotCore;

namespace DroneCore.Api
{
    /// <summary>
    /// Per-drone API facade. Primary interface for Internal scripts and External RPC.
    /// Attach to the same GameObject as QuadPawn (added automatically by DroneRootBootstrap).
    /// 
    /// Access patterns:
    /// <code>
    ///   // Get the currently selected drone
    ///   var drone = QuadSimApi.Selected();
    ///   
    ///   // Get drone by index (multi-drone)
    ///   var drone0 = QuadSimApi.Get(0);
    ///   var drone1 = QuadSimApi.Get(1);
    ///   
    ///   // Use the API
    ///   drone.SetMode(GoalMode.Rate);
    ///   drone.SendCommand(InputSource.Internal, new Axis4(0, 0, 5, 0.41f));
    ///   var sensors = drone.GetSensorData();
    /// </code>
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class QuadSimApi : MonoBehaviour
    {
        // ============================================================================
        // Static Convenience Accessors
        // ============================================================================

        /// <summary>
        /// Get the QuadSimApi for the currently selected drone.
        /// Returns null if no drone is selected or not yet initialized.
        /// </summary>
        public static QuadSimApi Selected()
        {
            var dm = DroneManager.Get();
            return dm?.SelectedDrone?.Api;
        }

        /// <summary>
        /// Get the QuadSimApi for a drone by index.
        /// Returns null if index is out of range or drone hasn't initialized.
        /// </summary>
        public static QuadSimApi Get(int index)
        {
            var dm = DroneManager.Get();
            if (dm == null) return null;

            var drones = dm.RegisteredDrones;
            if (index < 0 || index >= drones.Count) return null;

            return drones[index]?.Api;
        }

        /// <summary>
        /// Number of registered drones. Convenience wrapper around DroneManager.
        /// </summary>
        public static int DroneCount
        {
            get
            {
                var dm = DroneManager.Get();
                return dm?.DroneCount ?? 0;
            }
        }

        // ============================================================================
        // References (set during Initialize)
        // ============================================================================

        private QuadPawn _pawn;
        private FlightCommandProxy _proxy;
        private ModeCoordinator _modeCoordinator;
        private SensorManager _sensorManager;
        private CascadedController _controller;

        private bool _initialized;

        // ============================================================================
        // Public Accessors
        // ============================================================================

        /// <summary>Has this API been initialized with valid references?</summary>
        public bool IsInitialized => _initialized;

        /// <summary>The drone's unique identifier.</summary>
        public string DroneID => _pawn != null ? _pawn.DroneID : name;

        /// <summary>Current GoalMode from ModeCoordinator.</summary>
        public GoalMode CurrentMode =>
            _modeCoordinator != null ? _modeCoordinator.ActiveMode : GoalMode.None;

        /// <summary>Current ControllerKind from ModeCoordinator.</summary>
        public ControllerKind CurrentController =>
            _modeCoordinator != null ? _modeCoordinator.ActiveController : ControllerKind.Cascade;

        /// <summary>Direct access to the underlying QuadPawn (advanced use only).</summary>
        public QuadPawn Pawn => _pawn;

        /// <summary>Direct access to the command proxy (advanced use only).</summary>
        public FlightCommandProxy CommandProxy => _proxy;

        /// <summary>Direct access to the mode coordinator (advanced use only).</summary>
        public ModeCoordinator ModeCoord => _modeCoordinator;

        // ============================================================================
        // Events (forwarded from subsystems)
        // ============================================================================

        /// <summary>Fired when GoalMode changes on this drone.</summary>
        public event Action<GoalMode, GoalMode> OnModeChanged;

        /// <summary>Fired when ControllerKind changes on this drone.</summary>
        public event Action<ControllerKind, ControllerKind> OnControllerChanged;

        // ============================================================================
        // Initialization
        // ============================================================================

        /// <summary>
        /// Initialize with references to the drone's subsystems.
        /// Called by QuadPawn during InitializeSubsystems().
        /// </summary>
        public void Initialize(QuadPawn pawn, FlightCommandProxy proxy,
                               ModeCoordinator modeCoordinator, SensorManager sensorManager)
        {
            if (_initialized)
            {
                Debug.LogWarning($"[QuadSimApi] {name}: Already initialized. Ignoring duplicate init.");
                return;
            }

            _pawn = pawn;
            _proxy = proxy;
            _modeCoordinator = modeCoordinator;
            _sensorManager = sensorManager;

            _controller = _pawn != null ? _pawn.Controller : null;

            if (_modeCoordinator != null)
            {
                _modeCoordinator.OnModeChanged += HandleModeChanged;
                _modeCoordinator.OnControllerChanged += HandleControllerChanged;
            }

            _initialized = true;

            Debug.Log($"[QuadSimApi] {DroneID}: Initialized " +
                      $"(pawn={_pawn != null}, proxy={_proxy != null}, " +
                      $"modeCoord={_modeCoordinator != null}, sensors={_sensorManager != null})");
        }

        private void OnDestroy()
        {
            if (_modeCoordinator != null)
            {
                _modeCoordinator.OnModeChanged -= HandleModeChanged;
                _modeCoordinator.OnControllerChanged -= HandleControllerChanged;
            }
        }

        // ============================================================================
        // Mode & Controller Control
        // ============================================================================

        /// <summary>
        /// Set the active GoalMode for this drone.
        /// Triggers ModeCoordinator reset policies (PID integrator reset, goal clear).
        /// </summary>
        public bool SetMode(GoalMode mode)
        {
            if (!AssertInitialized("SetMode")) return false;

            if (_modeCoordinator == null)
            {
                Debug.LogError($"[QuadSimApi] {DroneID}: No ModeCoordinator — cannot set mode.");
                return false;
            }

            if (_modeCoordinator.ActiveMode == mode)
                return false;

            _modeCoordinator.SetMode(mode);
            return true;
        }

        /// <summary>
        /// Set the active controller type for this drone.
        /// Triggers a full reset (integrators + dynamics) per ModeCoordinator reset policy.
        /// </summary>
        public bool SetController(ControllerKind kind)
        {
            if (!AssertInitialized("SetController")) return false;

            if (_modeCoordinator == null)
            {
                Debug.LogError($"[QuadSimApi] {DroneID}: No ModeCoordinator — cannot set controller.");
                return false;
            }

            if (_modeCoordinator.ActiveController == kind)
                return false;

            _modeCoordinator.SetController(kind);
            return true;
        }

        // ============================================================================
        // Command Sending
        // ============================================================================

        /// <summary>
        /// Send a command to this drone. Uses the current ActiveMode from ModeCoordinator.
        /// The proxy validates the caller against the current authority.
        /// </summary>
        public bool SendCommand(InputSource source, Axis4 value)
        {
            if (!AssertInitialized("SendCommand")) return false;

            if (_proxy == null)
            {
                Debug.LogError($"[QuadSimApi] {DroneID}: No FlightCommandProxy — cannot send command.");
                return false;
            }

            GoalMode mode = _modeCoordinator != null ? _modeCoordinator.ActiveMode : GoalMode.Rate;
            _proxy.SetCommand(value, mode, source);
            return true;
        }

        /// <summary>
        /// Send a command with an explicit GoalMode override.
        /// Sets the mode via ModeCoordinator first, then sends the command.
        /// </summary>
        public bool SendCommand(InputSource source, Axis4 value, GoalMode mode)
        {
            if (!AssertInitialized("SendCommand")) return false;

            if (_modeCoordinator != null && _modeCoordinator.ActiveMode != mode)
            {
                _modeCoordinator.SetMode(mode);
            }

            if (_proxy == null)
            {
                Debug.LogError($"[QuadSimApi] {DroneID}: No FlightCommandProxy — cannot send command.");
                return false;
            }

            _proxy.SetCommand(value, mode, source);
            return true;
        }

        // ============================================================================
        // Reset Operations
        // ============================================================================

        /// <summary>Full drone reset: physics, controller state, command proxy.</summary>
        public void ResetAll()
        {
            if (!AssertInitialized("ResetAll")) return;

            _pawn?.ResetDrone();

            if (_proxy != null && _modeCoordinator != null)
            {
                _proxy.ResetToSafe(_modeCoordinator.ActiveMode);
            }

            Debug.Log($"[QuadSimApi] {DroneID}: Full reset");
        }

        /// <summary>Reset drone to a specific position and rotation.</summary>
        public void ResetPose(Vector3 position, Quaternion rotation)
        {
            if (!AssertInitialized("ResetPose")) return;

            _pawn?.ResetPosition(position, rotation);

            if (_proxy != null && _modeCoordinator != null)
            {
                _proxy.ResetToSafe(_modeCoordinator.ActiveMode);
            }

            Debug.Log($"[QuadSimApi] {DroneID}: Pose reset to {position}");
        }

        /// <summary>Level the drone (zero roll/pitch, keep yaw) and zero velocities.</summary>
        public void ResetRotation()
        {
            if (!AssertInitialized("ResetRotation")) return;
            _pawn?.ResetRotation();
        }

        /// <summary>Zero all velocities without moving the drone.</summary>
        public void ResetPhysics()
        {
            if (!AssertInitialized("ResetPhysics")) return;
            _pawn?.ResetPhysics();
        }

        /// <summary>Reset only controller state (PID integrators, allocator).</summary>
        public void ResetController()
        {
            if (!AssertInitialized("ResetController")) return;
            _pawn?.ResetControllerState();
        }

        // ============================================================================
        // Sensor Data (Measured State)
        // ============================================================================

        /// <summary>
        /// Get the latest sensor readings (IMU + GPS).
        /// This is the measured state — what a real flight controller would see.
        /// Use this for control algorithms and sim-to-real workflows.
        /// </summary>
        public SensorData GetSensorData()
        {
            if (_sensorManager == null) return default;
            return _sensorManager.Latest;
        }

        // ============================================================================
        // Telemetry (Operational State + Controller Outputs)
        // ============================================================================

        /// <summary>
        /// Get the cascaded controller's internal telemetry (desired values from cascade).
        /// </summary>
        public CascadedController.CascadeTelemetrySnapshot GetControllerTelemetry()
        {
            if (_controller == null) return default;
            return _controller.TelemetrySnapshot;
        }

        /// <summary>
        /// Get a combined telemetry snapshot: operational state + controller outputs.
        /// Does NOT include measured state — use GetSensorData() for that.
        /// </summary>
        public DroneTelemetry GetTelemetry()
        {
            var telem = new DroneTelemetry
            {
                DroneID = DroneID,
                Mode = CurrentMode,
                Controller = CurrentController,
                ControllerTelemetry = GetControllerTelemetry()
            };

            if (_controller != null && _controller.LastMotor01 != null)
            {
                telem.MotorOutputs = new Motor4
                {
                    FL = _controller.LastMotor01[0],
                    FR = _controller.LastMotor01[1],
                    BL = _controller.LastMotor01[2],
                    BR = _controller.LastMotor01[3]
                };
            }
            if (_controller != null && _controller.Thrusts != null)
            {
                telem.MotorThrusts = new Motor4
                {
                    FL = _controller.Thrusts[0],
                    FR = _controller.Thrusts[1],
                    BL = _controller.Thrusts[2],
                    BR = _controller.Thrusts[3]
                };
            }

            return telem;
        }

        // ============================================================================
        // Configuration
        // ============================================================================

        /// <summary>Whether this drone has had its YAML config applied.</summary>
        public bool HasConfig => _pawn != null && _pawn.HasConfigApplied;

        /// <summary>Force a reload of the drone's YAML configuration.</summary>
        public void ReloadConfig()
        {
            if (!AssertInitialized("ReloadConfig")) return;
            _pawn?.ReloadYaml();
        }

        /// <summary>Re-apply the current config without reloading from disk.</summary>
        public void ReapplyConfig()
        {
            if (!AssertInitialized("ReapplyConfig")) return;
            _pawn?.ReapplyConfig();
        }

        // ============================================================================
        // Internal Helpers
        // ============================================================================

        private bool AssertInitialized(string caller)
        {
            if (_initialized) return true;

            Debug.LogWarning($"[QuadSimApi] {name}: {caller}() called before initialization. " +
                             "Ensure QuadPawn.InitializeSubsystems() has run.");
            return false;
        }

        private void HandleModeChanged(GoalMode oldMode, GoalMode newMode) =>
            OnModeChanged?.Invoke(oldMode, newMode);

        private void HandleControllerChanged(ControllerKind oldKind, ControllerKind newKind) =>
            OnControllerChanged?.Invoke(oldKind, newKind);
    }

    // ============================================================================
    // Telemetry Data Struct
    // ============================================================================

    /// <summary>
    /// Combined telemetry snapshot for a single drone.
    /// Captures operational state and controller outputs — NOT measured state.
    /// For measured state (IMU, GPS), use QuadSimApi.GetSensorData().
    /// </summary>
    public struct DroneTelemetry
    {
        public string DroneID;
        public GoalMode Mode;
        public ControllerKind Controller;
        public Motor4 MotorOutputs;   
        public Motor4 MotorThrusts;  
        public CascadedController.CascadeTelemetrySnapshot ControllerTelemetry;
    }
}