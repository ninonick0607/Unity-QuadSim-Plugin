// Assets/Scripts/SimCore/SimulationManager.cs

using System;
using System.Collections.Generic;
using UnityEngine;

namespace SimCore
{
    /// <summary>
    /// Unity equivalent to your UE SimulationManager/TimeController backbone:
    /// - Owns authoritative stepping (manual Physics.Simulate).
    /// - Owns pause/play/single-step/time-scale.
    /// - Drives a deterministic fixed-step pipeline: PrePhysics -> Physics -> PostPhysics.
    /// </summary>
    [DefaultExecutionOrder(-1000)] // run as early as possible
    public sealed class SimulationManager : MonoBehaviour
    {
        public enum RunMode
        {
            FreeRun,   // advance based on real time * timeScale
            Manual     // only advance when StepOnce() is requested (API-driven / lockstep-ready)
        }

        [Header("Timing")]
        [Tooltip("Base simulation rate (Hz). 250 Hz = dt 0.004s.")]
        [SerializeField] private double baseHz = 250.0;

        [Tooltip("How sim time advances each rendered frame (FreeRun) or only via explicit steps (Manual).")]
        [SerializeField] private RunMode runMode = RunMode.FreeRun;

        [Tooltip("When in FreeRun: simAdvance = realDelta * timeScale.")]
        [SerializeField] private double timeScale = 1.0;

        [Tooltip("Safety cap: max fixed steps per rendered frame (prevents spiral-of-death).")]
        [SerializeField] private int maxStepsPerFrame = 1000;

        [Header("Control")]
        [SerializeField] private bool startPaused = false;

        [Header("Debug")]
        [SerializeField] private bool logEffectiveHz = true;
        [SerializeField] private float logIntervalSeconds = 1.0f;

        private List<ISimulatable> RegisteredManagers = new List<ISimulatable>(128);
        private bool _startFired = false;

        // FreeRun accumulator in seconds (sim-time to execute, converted into fixed dt steps).
        private double _accumulatorSec;

        // Manual stepping queue count (for API-driven stepping / future lockstep).
        private int _pendingSteps;

        // Debug
        private float _logTimer;
        private int _stepsThisInterval;

        public double BaseDtSec => ClockFactory.BaseDtSec;
        public RunMode Mode => runMode;

        public bool IsPaused => ClockFactory.Scaled.IsPaused;

        public void SetTimeScale(double ts)
        {
            timeScale = ts;
            ClockFactory.SetTimeScale(timeScale);
        }
        /// <summary>
        /// Run N fixed steps immediately on the main thread.
        /// Use this for lockstep RPC calls that must return post-step sensors.
        /// </summary>
        public void StepImmediate(int steps)
        {
            if (steps <= 0) return;
            RunFixedSteps(steps);
        }
        private void Awake()
        {
            Application.runInBackground = true;        // CRITICAL: Unity throttles to ~single-digit
            QualitySettings.vSyncCount = 0;            // fps when unfocused — and you're always
            Application.targetFrameRate = -1;          // focused on the terminal running Optuna
            
            Physics.simulationMode = SimulationMode.Script;
            ClockFactory.Initialize(baseHz: baseHz, startNanos: 0, paused: startPaused, timeScale: timeScale);
        }
        
        private void Start()
        {
            _startFired = true;
            foreach (var s in RegisteredManagers)
                s.OnSimulationStart(this);
        }
        
        private void OnDestroy()
        {
            // Restore default so exiting play mode doesn't leave editor in weird state.
            Physics.simulationMode = SimulationMode.FixedUpdate;
        }

        private void Update()
        {
            // Keep timeScale in sync if edited during play mode.
            ClockFactory.SetTimeScale(timeScale);

            // Decide how much sim-time to advance this frame.
            if (ClockFactory.Scaled.IsPaused)
            {
                // No stepping unless manual steps are queued.
            }
            else if (runMode == RunMode.FreeRun)
            {
                // Advance based on wall-frame delta * timeScale; then consume into fixed steps.
                double advance = ClockFactory.Scaled.ComputeAdvanceSeconds(Time.unscaledDeltaTime);
                _accumulatorSec += advance;
            }

            // Manual mode ignores accumulator; only StepOnce/StepMany enqueues steps.
            int stepsToRun = 0;

            if (runMode == RunMode.FreeRun)
            {
                double dt = BaseDtSec;
                if (dt <= 0) return;

                // Convert accumulated sim time into fixed steps.
                stepsToRun = (int)Math.Floor(_accumulatorSec / dt);
                if (stepsToRun > 0)
                {
                    _accumulatorSec -= stepsToRun * dt;
                }
            }

            // Add pending manual steps (works in both modes; useful for single-step while paused).
            if (_pendingSteps > 0)
            {
                stepsToRun += _pendingSteps;
                _pendingSteps = 0;
            }

            if (stepsToRun <= 0)
            {
                TickDebug(0);
                return;
            }

            if (stepsToRun > maxStepsPerFrame)
            {
                // Drop extra steps; keep sim responsive. This is a deliberate non-realtime behavior guard.
                stepsToRun = maxStepsPerFrame;
                _accumulatorSec = 0; // avoid runaway accumulation
            }

            // Run fixed-step pipeline.
            RunFixedSteps(stepsToRun);

            TickDebug(stepsToRun);
        }

        private void RunFixedSteps(int steps)
        {
            double dt = BaseDtSec;
            float dtF = (float)dt;

            for (int i = 0; i < steps; i++)
            {
                long now = ClockFactory.StepOnce();

                // Pre-physics: controllers read commands, compute actuator targets, etc.
                for (int k = 0; k < RegisteredManagers.Count; k++)
                {
                    RegisteredManagers[k].PrePhysicsStep(dt, now);
                }

                // Physics step (deterministic fixed dt)
                Physics.Simulate(dtF);

                // Post-physics: sensors sample, logging, IO publish, etc.
                for (int k = 0; k < RegisteredManagers.Count; k++)
                {
                    RegisteredManagers[k].PostPhysicsStep(dt, now);
                }

                _stepsThisInterval++;
            }
        }

        private void TickDebug(int stepsRanThisFrame)
        {
            if (!logEffectiveHz) return;

            _logTimer += Time.unscaledDeltaTime;
            if (_logTimer < logIntervalSeconds) return;

            float interval = _logTimer;
            _logTimer = 0;

            double effectiveHz = _stepsThisInterval / interval;
            _stepsThisInterval = 0;

            Debug.Log($"[SimulationManager] mode={runMode} paused={ClockFactory.Scaled.IsPaused} dt={BaseDtSec:F6}s " +
                      $"timeScale={timeScale:F2} effHz={effectiveHz:F1} steps/frame(last)={stepsRanThisFrame}");
        }

        /// <summary>
        /// Rebuilds the registry of simulatable systems (drones, sensors, IO).
        /// Call this if you dynamically spawn/despawn drones.
        /// </summary>
        public void RegisterManagers(ISimulatable manager)
        {
            if (!RegisteredManagers.Contains(manager))
            {
                RegisteredManagers.Add(manager);
                // If Start already ran, catch up the late registrant
                if (_startFired)
                    manager.OnSimulationStart(this);
            }
        }
        public void UnRegisterManager(ISimulatable manager)
        {
            RegisteredManagers.Remove(manager);
        }
        
        // ---- External control surface (API / UI / future PX4 lockstep) ----

        public void SetPaused(bool paused)
        {
            ClockFactory.SetPaused(paused);
        }

        public void TogglePaused()
        {
            ClockFactory.SetPaused(!ClockFactory.Scaled.IsPaused);
        }

        /// <summary>
        /// Queue exactly one fixed step. Works even if paused. Intended for UI "single step" and lockstep.
        /// </summary>
        public void StepOnce()
        {
            _pendingSteps += 1;
        }

        /// <summary>
        /// Queue N fixed steps. Useful for RL fast-forward or lockstep bursts.
        /// </summary>
        public void StepMany(int steps)
        {
            if (steps <= 0) return;
            _pendingSteps += steps;
        }

        public void SetRunMode(RunMode mode)
        {
            runMode = mode;
            // When entering Manual mode, clear accumulator so you don't "catch up" unexpectedly later.
            if (runMode == RunMode.Manual)
            {
                _accumulatorSec = 0;
            }
        }

        /// <summary>
        /// Reset sim-time to 0 and notify systems to reset their internal state.
        /// You will later also reset rigidbodies here (or via QuadPawn.Reset()).
        /// </summary>
        public void ResetSimulation()
        {
            ClockFactory.Reset(newStartNanos: 0);
            _accumulatorSec = 0;
            _pendingSteps = 0;

            foreach (var s in RegisteredManagers)
            {
                s.OnSimulationReset(this);
            }
        }
    }

    /// <summary>
    /// Unity equivalent of your ISimulatable/loop callbacks used by SimulationManager.
    /// Pure contract: no Unity time usage inside implementations.
    /// </summary>
    public interface ISimulatable
    {
        /// <summary>Lower runs earlier. Use this to impose deterministic ordering.</summary>
        int ExecutionOrder { get; }

        string DebugName { get; }

        void OnSimulationStart(SimulationManager sim);
        void OnSimulationReset(SimulationManager sim);

        /// <param name="dtSec">Fixed dt seconds (0.004 at 250Hz)</param>
        /// <param name="nowNanos">ClockFactory.Clock.NowNanos after stepping</param>
        void PrePhysicsStep(double dtSec, long nowNanos);

        void PostPhysicsStep(double dtSec, long nowNanos);
    }
}
