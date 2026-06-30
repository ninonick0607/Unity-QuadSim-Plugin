// Assets/Scripts/DroneCore/Interfaces/FlightCommandProxy.cs
// Phase 10 Polish — Stale Command Log Throttle
//
// PURPOSE:
//   Stores the winning command for the current tick.
//   Validates that only the authority source can write.
//   Falls back to safe commands when the authority source goes silent.
//
// PHASE 10 CHANGE (Step 2a):
//   Added _staleFallbackLogged flag to suppress repeated "Stale command detected"
//   warnings. The warning fires once on first detection, then goes quiet until a
//   fresh command arrives via SetCommand(). This prevents log spam after reset()
//   where ~0.5s of stale ticks fire before the next Python command arrives.

using SimCore;
using SimCore.Common;
using UnityEngine;

namespace DroneCore.Interfaces
{
    /// <summary>
    /// Stores the winning command for the current tick.
    /// Validates that only the authority source can write.
    /// Falls back to safe commands when the authority source goes silent.
    /// 
    /// CascadedController reads from this via ICommandSource — that interface is unchanged.
    /// </summary>
    public sealed class FlightCommandProxy : MonoBehaviour, ICommandSource
    {
        // ============================================================================
        // Serialized State (visible in Inspector for debugging)
        // ============================================================================
        
        [Header("Current Command")]
        [SerializeField] private Axis4 _command;
        [SerializeField] private GoalMode _mode = GoalMode.None;
        [SerializeField] private InputSource _lastWriter = InputSource.UI;
        
        [Header("Authority")]
        [SerializeField] private InputSource _activeSource = InputSource.UI;
        
        [Header("Diagnostics")]
        [SerializeField] private double _lastCommandTimeSec;
        [SerializeField] private int _rejectedCount;
        [SerializeField] private bool _usingFallback;

        // ============================================================================
        // Authority Reference
        // ============================================================================
        
        private ControlAuthorityManager _authorityManager;
        
        /// <summary>
        /// Inject reference to the authority manager. Called during initialization.
        /// If not set, falls back to ControlAuthorityManager.Get() on first use.
        /// </summary>
        public void SetAuthorityManager(ControlAuthorityManager manager)
        {
            _authorityManager = manager;
        }

        // ============================================================================
        // Stale Command Detection
        // ============================================================================
        
        [Tooltip("Seconds without a command before falling back to safe. 0 = disabled.")]
        [SerializeField] private float staleCommandThresholdSec = 0.5f;

        /// <summary>
        /// Phase 10 Polish (Step 2a): Tracks whether we've already logged the stale
        /// fallback warning. Prevents log spam when the proxy stays stale for many
        /// ticks (e.g. after reset, before the next Python command arrives).
        /// Reset to false in SetCommand() when a fresh command arrives.
        /// </summary>
        private bool _staleFallbackLogged;

        // ============================================================================
        // ICommandSource Implementation (read interface — unchanged for controllers)
        // ============================================================================
        
        public Axis4 GetCommandValue()
        {
            CheckForStaleCommand();
            return _command;
        }
        
        public GoalMode GetGoalMode() => _mode;
        public InputSource GetActiveSource() => _activeSource;
        
        // ============================================================================
        // Extended Read Interface
        // ============================================================================
        
        /// <summary>Who last successfully wrote a command.</summary>
        public InputSource GetLastWriter() => _lastWriter;
        
        /// <summary>Timestamp of the last valid command (sim seconds).</summary>
        public double GetLastCommandTime() => _lastCommandTimeSec;
        
        /// <summary>How many commands have been rejected since last successful write.</summary>
        public int GetRejectedCount() => _rejectedCount;
        
        /// <summary>True if the proxy is currently returning safe fallback commands.</summary>
        public bool IsUsingFallback() => _usingFallback;

        // ============================================================================
        // Write Interface (Source Validated)
        // ============================================================================
        
        /// <summary>
        /// Set the current command. Validates that the caller matches the effective authority.
        /// Returns false if rejected (caller is not the authority source).
        /// </summary>
        public bool SetCommand(Axis4 inCmd, GoalMode inMode, InputSource caller)
        {
            // Resolve authority — prefer injected reference, fallback to singleton
            var authority = GetEffectiveAuthority();
            
            if (caller != authority)
            {
                _rejectedCount++;
                
                // Throttle warnings to avoid log spam
                if (_rejectedCount <= 3 || _rejectedCount % 100 == 0)
                {
                    Debug.LogWarning($"[Proxy] REJECT #{_rejectedCount} Caller={caller} Authority={authority} " +
                                     $"Mode={inMode} Cmd={inCmd}");
                }
                return false;
            }
            
            _command = inCmd;
            _mode = inMode;
            _lastWriter = caller;
            _lastCommandTimeSec = GetSimTime();
            _rejectedCount = 0;
            _usingFallback = false;
            _staleFallbackLogged = false;  // Phase 10: Reset log throttle on fresh command
            
            return true;
        }
        
        /// <summary>
        /// Bypass authority check. Used internally for:
        ///   - Reset flows (QuadPawn.ResetControllerState)
        ///   - Safe fallback injection
        ///   - Initialization
        /// Should NOT be called by command sources.
        /// </summary>
        public void ForceSetCommand(Axis4 inCmd, GoalMode inMode, InputSource caller)
        {
            _command = inCmd;
            _mode = inMode;
            _lastWriter = caller;
            _lastCommandTimeSec = GetSimTime();
            _usingFallback = false;
            _staleFallbackLogged = false;  // Phase 10: Reset log throttle
        }
        
        // ============================================================================
        // Authority Sync
        // ============================================================================
        
        /// <summary>
        /// Explicitly set the active source. Called by ControlAuthorityManager when
        /// authority changes. Clears rejected count.
        /// </summary>
        public void SetActiveSource(InputSource inActive)
        {
            if (_activeSource == inActive) return;

            Debug.Log($"[Proxy] ActiveSource: {_activeSource} -> {inActive}");
            _activeSource = inActive;
            _rejectedCount = 0;
        }
        
        // ============================================================================
        // Safe Fallback
        // ============================================================================
        
        /// <summary>
        /// Reset the proxy to a safe state. Zero commands, given mode, clear diagnostics.
        /// Used during drone reset and initialization.
        /// </summary>
        public void ResetToSafe(GoalMode safeMode = GoalMode.None)
        {
            _command = Axis4.ZeroValue;
            _mode = safeMode;
            _lastCommandTimeSec = GetSimTime();
            _rejectedCount = 0;
            _usingFallback = false;
            _staleFallbackLogged = false;  // Phase 10: Reset log throttle on reset
        }
        
        /// <summary>
        /// Check if the current command is stale and inject safe fallback if needed.
        /// Called automatically on read (GetCommandValue).
        ///
        /// Phase 10 Polish (Step 2a): Now only logs the warning ONCE per stale episode.
        /// Subsequent ticks while still stale are silent. The _staleFallbackLogged flag
        /// resets when SetCommand() receives a fresh command.
        /// </summary>
        private void CheckForStaleCommand()
        {
            if (staleCommandThresholdSec <= 0f) return;
            if (_mode == GoalMode.None) return;
            
            double now = GetSimTime();
            double age = now - _lastCommandTimeSec;
            
            if (age > staleCommandThresholdSec && !_usingFallback)
            {
                _usingFallback = true;
                _command = Axis4.ZeroValue; // Zero rates / zero throttle

                // Phase 10: Only log once per stale episode
                if (!_staleFallbackLogged)
                {
                    _staleFallbackLogged = true;
                    Debug.LogWarning($"[Proxy] Stale command detected (age={age:F2}s, threshold={staleCommandThresholdSec}s). " +
                                     $"Falling back to safe command. Authority={_activeSource}, LastWriter={_lastWriter}");
                }
            }
        }
        
        // ============================================================================
        // Helpers
        // ============================================================================
        
        private InputSource GetEffectiveAuthority()
        {
            if (_authorityManager != null)
                return _authorityManager.EffectiveAuthority;
            
            // Lazy resolve
            _authorityManager = ControlAuthorityManager.Get();
            if (_authorityManager != null)
                return _authorityManager.EffectiveAuthority;
            
            // No authority manager in scene — fall back to local _activeSource
            return _activeSource;
        }
        
        private double GetSimTime()
        {
            // Use ClockFactory if available, otherwise fallback to Unity time
            try
            {
                return ClockFactory.Clock.NowNanos / 1_000_000_000.0;            
            }
            catch
            {
                return Time.timeAsDouble;
            }
        }
    }
}