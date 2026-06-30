// Assets/Scripts/SimCore/SimCoreApi.cs
// Phase 6: Global Simulation API Facade
//
// PURPOSE:
//   Single entry point for sim-wide operations: time control, authority management,
//   and global events. Lives on SimRoot alongside SimulationManager and ControlAuthorityManager.
//
// LAYER RULE:
//   SimCore is the parent layer. It does NOT reference DroneCore or DroneCore.Api.
//   For per-drone access, users go through QuadSimApi.Selected() or QuadSimApi.Get(i).
//   This keeps dependency direction clean: DroneCore → SimCore, never the reverse.
//
// OWNERSHIP:
//   - Delegates to: SimulationManager, ControlAuthorityManager
//   - Does NOT implement logic itself — pure facade pattern
//   - Singleton MonoBehaviour (one per scene, on SimRoot)
//
// DOES NOT:
//   - Control individual drones (use QuadSimApi)
//   - Access DroneManager or QuadPawn (that's DroneCore's job)
//   - Run physics (SimulationManager)
//   - Manage source state machines (ControlAuthorityManager)

using System;
using RobotCore.Core_Utils;
using UnityEngine;
using SimCore.Common;

namespace SimCore
{
    /// <summary>
    /// Global simulation API facade. Primary interface for sim-wide operations.
    /// Attach to SimRoot GameObject alongside SimulationManager and ControlAuthorityManager.
    /// 
    /// Usage:
    /// <code>
    ///   var sim = SimCoreApi.Get();
    ///   sim.Pause();
    ///   sim.SetTimeScale(2.0);
    ///   sim.RequestAuthority(InputSource.Internal);
    /// </code>
    /// 
    /// For per-drone access, use QuadSimApi directly:
    /// <code>
    ///   var drone = QuadSimApi.Selected();   // current drone
    ///   var drone0 = QuadSimApi.Get(0);      // by index
    /// </code>
    /// </summary>
    [DefaultExecutionOrder(-950)]
    [DisallowMultipleComponent]
    public sealed class SimCoreApi : MonoBehaviour
    {
        // ============================================================================
        // References (resolved in Awake)
        // ============================================================================

        [Header("Core References (auto-resolved if not set)")]
        [SerializeField] private SimulationManager simManager;
        [SerializeField] private ControlAuthorityManager authorityManager;
        public GeoReferencingSystem Geo { get; private set; }

        
        private bool _initialized;

        // ============================================================================
        // Singleton
        // ============================================================================

        private static SimCoreApi _instance;

        /// <summary>
        /// Get cached SimCoreApi instance. Falls back to scene find with warning.
        /// Prefer direct inspector references for performance-critical code.
        /// </summary>
        public static SimCoreApi Get()
        {
            if (_instance != null) return _instance;
            _instance = FindFirstObjectByType<SimCoreApi>();
            if (_instance != null)
                Debug.LogWarning("[SimCoreApi] Instance resolved via Find. Prefer direct reference.");
            return _instance;
        }

        // ============================================================================
        // Public Accessors
        // ============================================================================

        /// <summary>Has this API resolved all required references?</summary>
        public bool IsInitialized => _initialized;

        /// <summary>Direct access to SimulationManager (advanced use).</summary>
        public SimulationManager SimManager => simManager;

        /// <summary>Direct access to ControlAuthorityManager (advanced use).</summary>
        public ControlAuthorityManager AuthorityManager => authorityManager;

        // ============================================================================
        // Events (forwarded from ControlAuthorityManager)
        // ============================================================================

        /// <summary>Fired when authority changes. Args: (oldSource, newSource).</summary>
        public event Action<InputSource, InputSource> OnAuthorityChanged;

        /// <summary>Fired when a source's status changes. Args: (source, oldStatus, newStatus).</summary>
        public event Action<InputSource, SourceStatus, SourceStatus> OnSourceStatusChanged;

        /// <summary>Fired when an External client requests connection.</summary>
        public event Action OnExternalConnectionRequest;

        /// <summary>Fired when waiting for a source to connect.</summary>
        public event Action<InputSource> OnWaitingForSource;

        /// <summary>Fired when a source connection times out.</summary>
        public event Action OnConnectionTimeout;

        // ============================================================================
        // Unity Lifecycle
        // ============================================================================

        private void Awake()
        {
            _instance = this;
            Geo = new GeoReferencingSystem(
                47.397742,
                8.545594,
                488.0);
            ResolveReferences();
        }

        private void OnDestroy()
        {
            UnsubscribeEvents();
            if (_instance == this) _instance = null;
        }

        private void ResolveReferences()
        {
            if (simManager == null)
            {
                simManager = GetComponent<SimulationManager>();
                if (simManager == null)
                    simManager = FindFirstObjectByType<SimulationManager>();
            }

            if (authorityManager == null)
            {
                authorityManager = GetComponent<ControlAuthorityManager>();
                if (authorityManager == null)
                    authorityManager = ControlAuthorityManager.Get();
            }

            if (simManager == null)
                Debug.LogError("[SimCoreApi] No SimulationManager found! Time control will not work.");
            if (authorityManager == null)
                Debug.LogError("[SimCoreApi] No ControlAuthorityManager found! Authority control will not work.");

            SubscribeEvents();

            _initialized = (simManager != null && authorityManager != null);

            Debug.Log($"[SimCoreApi] Initialized (sim={simManager != null}, authority={authorityManager != null})");
        }

        private void SubscribeEvents()
        {
            if (authorityManager != null)
            {
                authorityManager.OnAuthorityChanged += HandleAuthorityChanged;
                authorityManager.OnSourceStatusChanged += HandleSourceStatusChanged;
                authorityManager.OnExternalConnectionRequest += HandleExternalConnectionRequest;
                authorityManager.OnWaitingForSource += HandleWaitingForSource;
                authorityManager.OnConnectionTimeout += HandleConnectionTimeout;
            }
        }

        private void UnsubscribeEvents()
        {
            if (authorityManager != null)
            {
                authorityManager.OnAuthorityChanged -= HandleAuthorityChanged;
                authorityManager.OnSourceStatusChanged -= HandleSourceStatusChanged;
                authorityManager.OnExternalConnectionRequest -= HandleExternalConnectionRequest;
                authorityManager.OnWaitingForSource -= HandleWaitingForSource;
                authorityManager.OnConnectionTimeout -= HandleConnectionTimeout;
            }
        }

        // ============================================================================
        // Time Control
        // ============================================================================

        /// <summary>Pause the simulation. Physics stops advancing.</summary>
        public void Pause()
        {
            if (simManager == null) { WarnMissing("SimulationManager", "Pause"); return; }
            simManager.SetPaused(true);
        }

        /// <summary>Resume the simulation from a paused state.</summary>
        public void Resume()
        {
            if (simManager == null) { WarnMissing("SimulationManager", "Resume"); return; }
            simManager.SetPaused(false);
        }

        /// <summary>Toggle pause state.</summary>
        public void TogglePause()
        {
            if (simManager == null) { WarnMissing("SimulationManager", "TogglePause"); return; }
            simManager.TogglePaused();
        }

        /// <summary>Whether the simulation is currently paused.</summary>
        public bool IsPaused => simManager != null && simManager.IsPaused;

        /// <summary>
        /// Advance exactly one physics step. Works even when paused.
        /// Useful for debugging, lockstep integration, and RL stepping.
        /// </summary>
        public void StepOnce()
        {
            if (simManager == null) { WarnMissing("SimulationManager", "StepOnce"); return; }
            simManager.StepOnce();
        }

        /// <summary>
        /// Queue N physics steps. Useful for RL fast-forward or lockstep bursts.
        /// </summary>
        public void StepMany(int steps)
        {
            if (simManager == null) { WarnMissing("SimulationManager", "StepMany"); return; }
            simManager.StepMany(steps);
        }

        /// <summary>
        /// Set the simulation time scale. 1.0 = real-time, 2.0 = 2x speed, 0.5 = half speed.
        /// Does not affect physics dt — only how many steps are taken per rendered frame.
        /// </summary>
        public void SetTimeScale(double scale)
        {
            if (simManager == null) { WarnMissing("SimulationManager", "SetTimeScale"); return; }
            simManager.SetTimeScale(scale);
        }

        /// <summary>Current simulation time scale.</summary>
        public double TimeScale => simManager != null ? ClockFactory.Scaled.TimeScale : 1.0;

        /// <summary>
        /// Switch between FreeRun (real-time) and Manual (step-by-step) modes.
        /// </summary>
        public void SetRunMode(SimulationManager.RunMode mode)
        {
            if (simManager == null) { WarnMissing("SimulationManager", "SetRunMode"); return; }
            simManager.SetRunMode(mode);
        }

        /// <summary>Current run mode.</summary>
        public SimulationManager.RunMode RunMode =>
            simManager != null ? simManager.Mode : SimulationManager.RunMode.FreeRun;

        /// <summary>The fixed physics timestep in seconds (e.g., 0.004 at 250Hz).</summary>
        public double FixedDtSec => simManager != null ? simManager.BaseDtSec : 0.004;

        /// <summary>Current simulation time in nanoseconds.</summary>
        public long SimTimeNanos => ClockFactory.IsInitialized ? ClockFactory.Clock.NowNanos : 0;

        /// <summary>Current simulation time in seconds.</summary>
        public double SimTimeSec => SimTimeNanos / 1_000_000_000.0;

        /// <summary>
        /// Reset the entire simulation: time resets to 0, all systems notified.
        /// </summary>
        public void ResetSimulation()
        {
            if (simManager == null) { WarnMissing("SimulationManager", "ResetSimulation"); return; }
            simManager.ResetSimulation();
        }

        // ============================================================================
        // Authority Control
        // ============================================================================

        /// <summary>Who currently has authority to send commands.</summary>
        public InputSource CurrentAuthority =>
            authorityManager != null ? authorityManager.EffectiveAuthority : InputSource.UI;

        /// <summary>
        /// Request authority for a specific source. Returns true if granted.
        /// The source must be in Connected status to receive authority.
        /// </summary>
        public bool RequestAuthority(InputSource source)
        {
            if (authorityManager == null) { WarnMissing("ControlAuthorityManager", "RequestAuthority"); return false; }
            return authorityManager.RequestAuthority(source);
        }

        /// <summary>
        /// Set the default authority source (what the system waits for on startup).
        /// Does not immediately switch authority — call RequestAuthority for that.
        /// </summary>
        public void SetDefaultAuthority(InputSource source)
        {
            if (authorityManager == null) { WarnMissing("ControlAuthorityManager", "SetDefaultAuthority"); return; }
            authorityManager.SetDefaultSource(source);
        }

        /// <summary>Immediately switch authority to UI for this session.</summary>
        public void SwitchToUI()
        {
            if (authorityManager == null) { WarnMissing("ControlAuthorityManager", "SwitchToUI"); return; }
            authorityManager.SwitchToUIForSession();
        }

        /// <summary>Get the current status of a specific input source.</summary>
        public SourceStatus GetSourceStatus(InputSource source)
        {
            if (authorityManager == null) return SourceStatus.Unavailable;
            return authorityManager.GetSourceStatus(source);
        }

        /// <summary>Check if a specific source currently has authority.</summary>
        public bool IsAuthority(InputSource source)
        {
            if (authorityManager == null) return source == InputSource.UI;
            return authorityManager.IsAuthority(source);
        }

        /// <summary>Whether External connections have been denied for this session.</summary>
        public bool IsExternalDenied =>
            authorityManager != null && authorityManager.IsExternalDenied;

        // --- Internal Source Lifecycle ---

        /// <summary>Notify that an Internal source is available (loaded but not commanding).</summary>
        public void NotifyInternalAvailable()
        {
            if (authorityManager == null) { WarnMissing("ControlAuthorityManager", "NotifyInternalAvailable"); return; }
            authorityManager.NotifyInternalAvailable();
        }

        /// <summary>Notify that an Internal source is connected and ready to command.</summary>
        public void NotifyInternalConnected()
        {
            if (authorityManager == null) { WarnMissing("ControlAuthorityManager", "NotifyInternalConnected"); return; }
            authorityManager.NotifyInternalConnected();
        }

        /// <summary>Notify that an Internal source has disconnected.</summary>
        public void NotifyInternalDisconnected()
        {
            if (authorityManager == null) { WarnMissing("ControlAuthorityManager", "NotifyInternalDisconnected"); return; }
            authorityManager.NotifyInternalDisconnected();
        }

        // --- External Source Lifecycle ---

        /// <summary>Notify that an External client is attempting to connect.</summary>
        public void NotifyExternalConnectionAttempt()
        {
            if (authorityManager == null) { WarnMissing("ControlAuthorityManager", "NotifyExternalConnectionAttempt"); return; }
            authorityManager.NotifyExternalConnectionAttempt();
        }

        /// <summary>Accept a pending External connection.</summary>
        public void AcceptExternalConnection()
        {
            if (authorityManager == null) { WarnMissing("ControlAuthorityManager", "AcceptExternalConnection"); return; }
            authorityManager.AcceptExternalConnection();
        }

        /// <summary>Deny a pending External connection (sticky for session).</summary>
        public void DenyExternalConnection()
        {
            if (authorityManager == null) { WarnMissing("ControlAuthorityManager", "DenyExternalConnection"); return; }
            authorityManager.DenyExternalConnection();
        }
        
        /// <summary>Notify that the External client has disconnected.</summary>
        public void NotifyExternalDisconnected()
        {
            if (authorityManager == null) { WarnMissing("ControlAuthorityManager", "NotifyExternalDisconnected"); return; }
            authorityManager.NotifyExternalDisconnected();
        }
        // ============================================================================
        // Status Snapshot
        // ============================================================================

        /// <summary>
        /// Get a combined status snapshot of the simulation.
        /// Useful for external monitoring, logging, or RPC status endpoints.
        /// </summary>
        public SimStatus GetStatus()
        {
            return new SimStatus
            {
                IsPaused = IsPaused,
                TimeScale = TimeScale,
                RunMode = RunMode,
                SimTimeSec = SimTimeSec,
                FixedDtSec = FixedDtSec,
                CurrentAuthority = CurrentAuthority,
                UIStatus = GetSourceStatus(InputSource.UI),
                InternalStatus = GetSourceStatus(InputSource.Internal),
                ExternalStatus = GetSourceStatus(InputSource.External)
            };
        }

        // ============================================================================
        // Internal Helpers
        // ============================================================================

        private void WarnMissing(string component, string caller)
        {
            Debug.LogWarning($"[SimCoreApi] {caller}() — No {component} available.");
        }

        private void HandleAuthorityChanged(InputSource old, InputSource @new) =>
            OnAuthorityChanged?.Invoke(old, @new);

        private void HandleSourceStatusChanged(InputSource src, SourceStatus old, SourceStatus @new) =>
            OnSourceStatusChanged?.Invoke(src, old, @new);

        private void HandleExternalConnectionRequest() =>
            OnExternalConnectionRequest?.Invoke();

        private void HandleWaitingForSource(InputSource src) =>
            OnWaitingForSource?.Invoke(src);

        private void HandleConnectionTimeout() =>
            OnConnectionTimeout?.Invoke();
    }

    // ============================================================================
    // Sim Status Snapshot
    // ============================================================================

    /// <summary>
    /// Combined simulation status for monitoring, logging, or RPC.
    /// Does not include drone count — that's DroneCore's concern.
    /// </summary>
    public struct SimStatus
    {
        public bool IsPaused;
        public double TimeScale;
        public SimulationManager.RunMode RunMode;
        public double SimTimeSec;
        public double FixedDtSec;
        public InputSource CurrentAuthority;
        public SourceStatus UIStatus;
        public SourceStatus InternalStatus;
        public SourceStatus ExternalStatus;
    }
}