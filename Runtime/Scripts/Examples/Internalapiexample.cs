// Assets/Scripts/Examples/InternalApiExample.cs
//
// EXAMPLE: Comprehensive Internal API usage via SimCoreApi + QuadSimApi
//
// ACCESS PATTERN:
//   SimCoreApi.Get()       → sim-wide: time control, authority
//   QuadSimApi.Selected()  → per-drone: mode, commands, sensors, telemetry, resets
//   QuadSimApi.Get(i)      → per-drone by index (multi-drone scenarios)
//
// HOW TO USE:
//   1. Drop this script onto any GameObject in your scene (NOT on the drone prefab)
//   2. Set ControlAuthorityManager default source to Internal (or leave as UI — script requests authority)
//   3. Hit Play — the script exercises the full API surface
//
// WHAT THIS DEMONSTRATES:
//   - SimCoreApi: authority lifecycle, time control, sim status
//   - QuadSimApi: SetMode, SendCommand, GetSensorData, GetTelemetry, resets
//   - Static accessors: QuadSimApi.Selected(), QuadSimApi.DroneCount
//   - Event subscriptions for mode/authority changes
//   - Coroutine init pattern for Unity timing
//   - Proper cleanup on destroy

using System.Collections;
using UnityEngine;
using SimCore;
using SimCore.Common;
using DroneCore.Api;

public class InternalApiExample : MonoBehaviour
{
    // ============================================================================
    // Configuration (tweak in Inspector)
    // ============================================================================

    [Header("Rate Mode Commands")]
    [SerializeField] private float rollRate = 0f;
    [SerializeField] private float pitchRate = 0f;
    [SerializeField] private float yawRate = 30f;
    [SerializeField] private float throttle = 0.41f;

    [Header("Velocity Mode Commands")]
    [SerializeField] private float forwardVel = 1.0f;
    [SerializeField] private float lateralVel = 0f;
    [SerializeField] private float velYawRate = 0f;
    [SerializeField] private float verticalVel = 0f;

    [Header("Demo Timing")]
    [SerializeField] private float rateModeDuration = 5f;
    [SerializeField] private float velocityModeDuration = 5f;

    [Header("Behavior")]
    [SerializeField] private bool requestAuthorityOnStart = true;

    [Tooltip("Run the full demo sequence (mode switches, resets, time control). " +
             "If false, just sends Rate commands continuously.")]
    [SerializeField] private bool runFullDemo = true;

    [Header("Logging")]
    [SerializeField] private int logSensorEveryNFrames = 60;
    [SerializeField] private int logTelemetryEveryNFrames = 120;

    // ============================================================================
    // Runtime State
    // ============================================================================

    private SimCoreApi _sim;
    private QuadSimApi _drone;
    private bool _isActive;
    private int _frameCount;

    private enum DemoPhase { RateMode, VelocityMode, ResetDemo, TimeControlDemo, Done }
    private DemoPhase _phase = DemoPhase.RateMode;
    private float _phaseTimer;

    // ============================================================================
    // Lifecycle
    // ============================================================================

    private void Start()
    {
        StartCoroutine(InitializeDelayed());
    }

    private IEnumerator InitializeDelayed()
    {
        // Wait one frame for drone spawning and initialization
        yield return null;

        // ---- Step 1: Get sim-wide API ----
        _sim = SimCoreApi.Get();
        if (_sim == null)
        {
            Debug.LogError("[ApiExample] No SimCoreApi found! Add SimCoreApi to SimRoot.");
            enabled = false;
            yield break;
        }

        // ---- Step 2: Register as Internal source and request authority ----
        _sim.NotifyInternalConnected();
        Debug.Log("[ApiExample] Registered as Internal source (Connected).");

        if (requestAuthorityOnStart)
        {
            _isActive = _sim.RequestAuthority(InputSource.Internal);
            if (_isActive)
                Debug.Log("[ApiExample] Authority granted for Internal.");
            else
                Debug.LogWarning("[ApiExample] Authority request denied. Commands will be rejected.");
        }

        // ---- Step 3: Get per-drone API via static accessor ----
        _drone = QuadSimApi.Selected();
        if (_drone == null || !_drone.IsInitialized)
        {
            Debug.LogError("[ApiExample] No initialized drone found!");
            enabled = false;
            yield break;
        }
        Debug.Log($"[ApiExample] Got drone API: {_drone.DroneID} " +
                  $"(total drones: {QuadSimApi.DroneCount})");

        // ---- Step 4: Subscribe to events ----
        _drone.OnModeChanged += (oldMode, newMode) =>
            Debug.Log($"[ApiExample] Mode changed: {oldMode} -> {newMode}");

        _drone.OnControllerChanged += (oldCtrl, newCtrl) =>
            Debug.Log($"[ApiExample] Controller changed: {oldCtrl} -> {newCtrl}");

        _sim.OnAuthorityChanged += (oldAuth, newAuth) =>
            Debug.Log($"[ApiExample] Authority changed: {oldAuth} -> {newAuth}");

        // ---- Step 5: Set initial mode ----
        _drone.SetMode(GoalMode.Rate);
        Debug.Log($"[ApiExample] Set mode to Rate. Current mode: {_drone.CurrentMode}");

        // ---- Step 6: Log initial sim status ----
        var status = _sim.GetStatus();
        Debug.Log($"[ApiExample] Sim status — paused={status.IsPaused}, timeScale={status.TimeScale}, " +
                  $"authority={status.CurrentAuthority}, dt={status.FixedDtSec:F4}s");

        _phase = DemoPhase.RateMode;
        _phaseTimer = 0f;

        Debug.Log("[ApiExample] Initialization complete. Starting command loop.");
    }

    private void Update()
    {
        if (!_isActive || _drone == null) return;

        _frameCount++;
        _phaseTimer += Time.deltaTime;

        if (runFullDemo)
            RunDemoSequence();
        else
            SendRateCommands();

        if (logSensorEveryNFrames > 0 && _frameCount % logSensorEveryNFrames == 0)
            LogSensorData();

        if (logTelemetryEveryNFrames > 0 && _frameCount % logTelemetryEveryNFrames == 0)
            LogTelemetry();
    }

    private void OnDestroy()
    {
        if (_sim != null)
        {
            _sim.NotifyInternalDisconnected();
        }
        Debug.Log("[ApiExample] Destroyed. Internal source disconnected.");
    }

    // ============================================================================
    // Demo Sequence
    // ============================================================================

    private void RunDemoSequence()
    {
        switch (_phase)
        {
            case DemoPhase.RateMode:
                SendRateCommands();
                if (_phaseTimer >= rateModeDuration)
                    TransitionTo(DemoPhase.VelocityMode);
                break;

            case DemoPhase.VelocityMode:
                SendVelocityCommands();
                if (_phaseTimer >= velocityModeDuration)
                    TransitionTo(DemoPhase.ResetDemo);
                break;

            case DemoPhase.ResetDemo:
                RunResetDemo();
                TransitionTo(DemoPhase.TimeControlDemo);
                break;

            case DemoPhase.TimeControlDemo:
                RunTimeControlDemo();
                TransitionTo(DemoPhase.Done);
                break;

            case DemoPhase.Done:
                if (_drone.CurrentMode != GoalMode.Rate)
                    _drone.SetMode(GoalMode.Rate);
                SendRateCommands();
                break;
        }
    }

    private void TransitionTo(DemoPhase next)
    {
        Debug.Log($"[ApiExample] === Phase transition: {_phase} -> {next} ===");
        _phase = next;
        _phaseTimer = 0f;
    }

    // ============================================================================
    // Command Sending
    // ============================================================================

    private void SendRateCommands()
    {
        if (_drone.CurrentMode != GoalMode.Rate)
            _drone.SetMode(GoalMode.Rate);

        Axis4 cmd = new Axis4(rollRate, pitchRate, yawRate, throttle);
        _drone.SendCommand(InputSource.Internal, cmd);
    }

    private void SendVelocityCommands()
    {
        // Overload that sets mode + sends command in one call
        Axis4 cmd = new Axis4(forwardVel, lateralVel, velYawRate, verticalVel);
        _drone.SendCommand(InputSource.Internal, cmd, GoalMode.Velocity);
    }

    // ============================================================================
    // Reset Demo
    // ============================================================================

    private void RunResetDemo()
    {
        Debug.Log("[ApiExample] --- Reset Demo ---");

        // Read pre-reset sensor state
        var sensors = _drone.GetSensorData();
        Debug.Log($"[ApiExample] Pre-reset GPS position: {sensors.GpsPosition}");

        // Reset controller only (PIDs)
        _drone.ResetController();
        Debug.Log("[ApiExample] ResetController() — PIDs cleared, position unchanged");

        // Reset to a specific pose
        Vector3 resetPos = new Vector3(0f, 3f, 0f);
        _drone.ResetPose(resetPos, Quaternion.identity);
        Debug.Log($"[ApiExample] ResetPose({resetPos}) — drone repositioned");

        // Full reset
        _drone.ResetAll();
        Debug.Log("[ApiExample] ResetAll() — full reset complete");

        // Back to Rate mode
        _drone.SetMode(GoalMode.Rate);
    }

    // ============================================================================
    // Time Control Demo
    // ============================================================================

    private void RunTimeControlDemo()
    {
        Debug.Log("[ApiExample] --- Time Control Demo ---");

        Debug.Log($"[ApiExample] Sim time: {_sim.SimTimeSec:F3}s, dt: {_sim.FixedDtSec:F4}s");

        _sim.Pause();
        Debug.Log($"[ApiExample] Paused: {_sim.IsPaused}");

        _sim.StepOnce();
        Debug.Log($"[ApiExample] Stepped once. Sim time: {_sim.SimTimeSec:F3}s");

        _sim.StepMany(10);
        Debug.Log("[ApiExample] Queued 10 steps");

        _sim.SetTimeScale(2.0);
        Debug.Log($"[ApiExample] TimeScale set to {_sim.TimeScale}");

        _sim.Resume();
        Debug.Log($"[ApiExample] Resumed: paused={_sim.IsPaused}");

        _sim.SetTimeScale(1.0);
        Debug.Log($"[ApiExample] TimeScale reset to {_sim.TimeScale}");
    }

    // ============================================================================
    // Sensor & Telemetry Logging
    // ============================================================================

    private void LogSensorData()
    {
        var s = _drone.GetSensorData();
        if (!s.ImuValid) return;

        Vector3 attDeg = s.ImuAttitude;
        Vector3 angVelDeg = s.ImuAngVel * Mathf.Rad2Deg;

        Debug.Log($"[ApiExample] SENSORS — " +
                  $"att=({attDeg.x:F1}, {attDeg.y:F1}, {attDeg.z:F1})° " +
                  $"angVel=({angVelDeg.x:F1}, {angVelDeg.y:F1}, {angVelDeg.z:F1})°/s " +
                  $"gps=({s.GpsPosition.x:F2}, {s.GpsPosition.y:F2}, {s.GpsPosition.z:F2})m " +
                  $"vel=({s.ImuVel.x:F2}, {s.ImuVel.y:F2}, {s.ImuVel.z:F2})m/s");
    }

    private void LogTelemetry()
    {
        var t = _drone.GetTelemetry();

        Debug.Log($"[ApiExample] TELEMETRY — " +
                  $"mode={t.Mode} ctrl={t.Controller} " +
                  $"motors=({t.MotorOutputs.FL:F3}, {t.MotorOutputs.FR:F3}, " +
                  $"{t.MotorOutputs.BL:F3}, {t.MotorOutputs.BR:F3})");
    }
}