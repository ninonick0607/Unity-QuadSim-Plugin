// Assets/Scripts/DroneCore/Core/ModeCoordinator.cs
// Phase 4: Per-Drone Mode & Controller Switching
//
// PURPOSE:
//   Single source of truth for which GoalMode and ControllerKind are active on this drone.
//   Handles reset policies when mode, controller, or authority changes.
//   Emits events so UI and other systems can react.
//
// OWNERSHIP:
//   - ActiveMode (GoalMode): what stage of the cascade to enter at
//   - ActiveController (ControllerKind): which controller implementation runs
//   - Reset policy execution on transitions
//
// DOES NOT:
//   - Run controllers (that's QuadPawn.RunController)
//   - Store or validate commands (that's FlightCommandProxy)
//   - Manage authority (that's ControlAuthorityManager)
//
// RESET POLICIES (from API plan):
//   Mode switch    → Reset PID integrators + clear goals. Do NOT reset dynamics.
//   Authority switch → Reset PID integrators + clear goals. Optionally reset dynamics (default: yes for External).
//   Controller switch → Reset everything (integrators + dynamics).

using System;
using UnityEngine;
using SimCore;
using SimCore.Common;
using DroneCore.Interfaces;

namespace DroneCore.Core
{
    /// <summary>
    /// Per-drone component that owns mode/controller state and executes reset policies.
    /// Attach to the same GameObject as QuadPawn.
    /// 
    /// ModeCoordinator writes the GoalMode into the FlightCommandProxy so the controller
    /// reads it each tick. This replaces the previous pattern where ControlDeckController
    /// or external callers set the mode directly on the proxy alongside commands.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ModeCoordinator : MonoBehaviour
    {
        // ============================================================================
        // Configuration
        // ============================================================================
        
        [Header("Defaults")]
        [Tooltip("Starting GoalMode for this drone.")]
        [SerializeField] private GoalMode startingMode = GoalMode.Rate;
        
        [Tooltip("Starting controller for this drone.")]
        [SerializeField] private ControllerKind startingController = ControllerKind.Cascade;

        [Header("Reset Policy")]
        [Tooltip("Reset drone dynamics (position/velocity) on authority switch to External.")]
        [SerializeField] private bool resetDynamicsOnExternalAuthority = true;
        
        [Tooltip("Reset drone dynamics on any authority switch (not just External).")]
        [SerializeField] private bool resetDynamicsOnAnyAuthoritySwitch = false;

        // ============================================================================
        // Events
        // ============================================================================
        
        /// <summary>Fired when GoalMode changes. Includes old and new mode.</summary>
        public event Action<GoalMode, GoalMode> OnModeChanged;
        
        /// <summary>Fired when ControllerKind changes. Includes old and new kind.</summary>
        public event Action<ControllerKind, ControllerKind> OnControllerChanged;

        // ============================================================================
        // Runtime State
        // ============================================================================
        
        private GoalMode _activeMode;
        private ControllerKind _activeController;
        private bool _initialized;
        
        // References (set during Initialize)
        private QuadPawn _pawn;
        private FlightCommandProxy _proxy;
        private ControlAuthorityManager _authorityManager;

        // ============================================================================
        // Public Accessors
        // ============================================================================
        
        /// <summary>Current GoalMode for this drone.</summary>
        public GoalMode ActiveMode => _activeMode;
        
        /// <summary>Current ControllerKind for this drone.</summary>
        public ControllerKind ActiveController => _activeController;
        
        /// <summary>Has this coordinator been initialized?</summary>
        public bool IsInitialized => _initialized;

        // ============================================================================
        // Initialization
        // ============================================================================
        
        /// <summary>
        /// Initialize with references to the drone's subsystems.
        /// Called by QuadPawn during InitializeSubsystems().
        /// </summary>
        public void Initialize(QuadPawn pawn, FlightCommandProxy proxy)
        {
            _pawn = pawn;
            _proxy = proxy;
            
            // Resolve authority manager
            _authorityManager = ControlAuthorityManager.Get();
            
            // Subscribe to authority changes
            if (_authorityManager != null)
            {
                _authorityManager.OnAuthorityChanged += HandleAuthorityChanged;
            }
            else
            {
                Debug.LogWarning($"[ModeCoordinator] {name}: No ControlAuthorityManager found. " +
                                 "Authority-triggered resets will not work.");
            }
            
            // Set initial state
            _activeMode = startingMode;
            _activeController = startingController;
            
            // Push initial mode to proxy so controller sees it on first tick
            if (_proxy != null)
            {
                _proxy.ResetToSafe(_activeMode);
            }
            
            _initialized = true;
            Debug.Log($"[ModeCoordinator] {name}: Initialized (mode={_activeMode}, controller={_activeController})");
        }

        // ============================================================================
        // Mode Switching
        // ============================================================================
        
        /// <summary>
        /// Switch GoalMode. No-op if already in the requested mode.
        /// Resets PID integrators and clears goals. Does NOT reset dynamics.
        /// </summary>
        public bool SetMode(GoalMode newMode)
        {
            if (!_initialized)
            {
                Debug.LogWarning($"[ModeCoordinator] {name}: SetMode called before initialization.");
                return false;
            }
            
            if (newMode == _activeMode)
            {
                return true; // No-op, already in this mode
            }
            
            // Validate: Geometric controller ignores GoalModes
            if (_activeController == ControllerKind.Geometric && 
                newMode != GoalMode.None)
            {
                Debug.LogWarning($"[ModeCoordinator] {name}: GoalMode switching not applicable " +
                                 $"for Geometric controller. Ignoring SetMode({newMode}).");
                return false;
            }
            
            var oldMode = _activeMode;
            _activeMode = newMode;
            
            // Execute reset policy: integrators + goals, NOT dynamics
            ResetControllersOnly();
            
            // Push new mode to proxy
            if (_proxy != null)
            {
                // Use ForceSet because this is a mode change, not a command from a source.
                // The proxy's mode field must reflect ModeCoordinator's decision.
                var currentAuthority = _authorityManager != null 
                    ? _authorityManager.EffectiveAuthority 
                    : InputSource.UI;
                _proxy.ForceSetCommand(Axis4.ZeroValue, _activeMode, currentAuthority);
            }
            
            Debug.Log($"[ModeCoordinator] {name}: Mode {oldMode} -> {newMode}");
            OnModeChanged?.Invoke(oldMode, newMode);
            
            return true;
        }

        // ============================================================================
        // Controller Switching
        // ============================================================================
        
        /// <summary>
        /// Switch ControllerKind. No-op if already using the requested controller.
        /// Resets everything: integrators, goals, AND dynamics.
        /// </summary>
        public bool SetController(ControllerKind newController)
        {
            if (!_initialized)
            {
                Debug.LogWarning($"[ModeCoordinator] {name}: SetController called before initialization.");
                return false;
            }
            
            if (newController == _activeController)
            {
                return true; // No-op
            }
            
            var oldController = _activeController;
            _activeController = newController;
            
            // Controller switch: full reset (integrators + dynamics)
            ResetAll();
            
            // If switching to Geometric, mode becomes irrelevant
            // If switching to Cascade, keep current mode
            if (newController == ControllerKind.Geometric)
            {
                _activeMode = GoalMode.None;
            }
            
            Debug.Log($"[ModeCoordinator] {name}: Controller {oldController} -> {newController}");
            OnControllerChanged?.Invoke(oldController, newController);
            
            return true;
        }

        // ============================================================================
        // Authority Change Handler
        // ============================================================================
        
        /// <summary>
        /// Called when global authority changes. Executes reset policy.
        /// </summary>
        private void HandleAuthorityChanged(InputSource oldSource, InputSource newSource)
        {
            if (!_initialized) return;
            
            Debug.Log($"[ModeCoordinator] {name}: Authority changed {oldSource} -> {newSource}. Executing reset policy.");
            
            // Always reset integrators and goals on authority switch
            ResetControllersOnly();
            
            // Optionally reset dynamics
            bool shouldResetDynamics = resetDynamicsOnAnyAuthoritySwitch ||
                                       (resetDynamicsOnExternalAuthority && newSource == InputSource.External);
            
            if (shouldResetDynamics)
            {
                if (_pawn != null)
                {
                    _pawn.ResetPhysics();
                    Debug.Log($"[ModeCoordinator] {name}: Dynamics reset (authority -> {newSource})");
                }
            }
            
            // Update proxy's active source to match new authority
            if (_proxy != null)
            {
                _proxy.SetActiveSource(newSource);
            }
        }

        // ============================================================================
        // Reset Helpers
        // ============================================================================
        
        /// <summary>
        /// Reset PID integrators and clear goals. Does NOT reset drone dynamics.
        /// Used for mode switches and authority switches.
        /// </summary>
        private void ResetControllersOnly()
        {
            if (_pawn != null)
            {
                _pawn.ResetControllerState();
            }
        }
        
        /// <summary>
        /// Full reset: integrators, goals, AND dynamics.
        /// Used for controller switches.
        /// </summary>
        private void ResetAll()
        {
            if (_pawn != null)
            {
                _pawn.ResetDrone();
            }
        }

        // ============================================================================
        // Query Helpers
        // ============================================================================
        
        /// <summary>Is the cascade controller active?</summary>
        public bool IsCascade => _activeController == ControllerKind.Cascade;
        
        /// <summary>Is the geometric controller active?</summary>
        public bool IsGeometric => _activeController == ControllerKind.Geometric;
        
        /// <summary>Is the drone in a mode where it should be actively flying?</summary>
        public bool IsFlightActive => _activeMode != GoalMode.None;
        
        /// <summary>Is the drone in a direct control mode (Rate or Passthrough)?</summary>
        public bool IsDirectControl => _activeMode == GoalMode.Rate || _activeMode == GoalMode.Passthrough;

        // ============================================================================
        // Cleanup
        // ============================================================================
        
        private void OnDestroy()
        {
            if (_authorityManager != null)
            {
                _authorityManager.OnAuthorityChanged -= HandleAuthorityChanged;
            }
        }
    }
}