
using System;
using UnityEngine;
using SimCore.Common;

namespace SimCore
{
    /// <summary>
    /// Global authority state machine. Tracks which InputSource has control,
    /// manages source lifecycle, and handles External connection accept/deny flow.
    /// 
    /// Authority is global (not per-drone) — all drones share one authority source.
    /// Can be extended to per-drone later if needed.
    /// </summary>
    [DefaultExecutionOrder(-900)] // After SimulationManager (-1000), before DroneManager (-500)
    public sealed class ControlAuthorityManager : MonoBehaviour
    {
        // ============================================================================
        // Configuration
        // ============================================================================
        
        [Header("Authority Settings")]
        [Tooltip("Which source should have authority on startup. Can be overridden by YAML config.")]
        [SerializeField] private InputSource defaultSource = InputSource.UI;
        
        [Tooltip("Seconds to wait for default source before showing switch-to-UI banner.")]
        [SerializeField] private float connectionTimeoutSec = 60f;
        
        [Tooltip("Auto-accept External connections when External is the default source.")]
        [SerializeField] private bool autoAcceptWhenDefault = true;

        // ============================================================================
        // Events
        // ============================================================================
        
        /// <summary>Fired when authority switches from one source to another.</summary>
        public event Action<InputSource, InputSource> OnAuthorityChanged;
        
        /// <summary>Fired when any source's lifecycle status changes.</summary>
        public event Action<InputSource, SourceStatus, SourceStatus> OnSourceStatusChanged;
        
        /// <summary>Fired when an External client wants to connect and needs user approval.</summary>
        public event Action OnExternalConnectionRequest;
        
        /// <summary>Fired when waiting for the default source to connect (with timeout info).</summary>
        public event Action<InputSource> OnWaitingForSource;
        
        /// <summary>Fired when waiting timeout expires. UI can show "Switch to UI" option.</summary>
        public event Action OnConnectionTimeout;

        // ============================================================================
        // Runtime State
        // ============================================================================
        
        private InputSource _effectiveAuthority;
        private InputSource _configuredDefault;
        private bool _sessionOverrideActive;
        
        private readonly SourceStatus[] _sourceStatuses = new SourceStatus[3]; // indexed by InputSource
        
        // Connection waiting state
        private bool _waitingForConnection;
        private float _waitTimer;
        private bool _externalPendingApproval;

        // ============================================================================
        // Public Accessors
        // ============================================================================
        
        /// <summary>Which source currently has authority to send commands.</summary>
        public InputSource EffectiveAuthority => _effectiveAuthority;
        
        /// <summary>The configured default source (from YAML or inspector).</summary>
        public InputSource ConfiguredDefault => _configuredDefault;
        
        /// <summary>True if the user switched away from the default source for this session.</summary>
        public bool IsSessionOverride => _sessionOverrideActive;
        
        /// <summary>True if waiting for the default source to connect.</summary>
        public bool IsWaitingForConnection => _waitingForConnection;
        
        /// <summary>True if an External connection is pending user approval.</summary>
        public bool IsExternalPendingApproval => _externalPendingApproval;
        
        /// <summary>Get the status of a specific source.</summary>
        public SourceStatus GetSourceStatus(InputSource source) => _sourceStatuses[(int)source];

        // ============================================================================
        // Singleton Access (optional, prefer direct references)
        // ============================================================================
        
        private static ControlAuthorityManager _instance;
        
        /// <summary>
        /// Get cached instance. Falls back to scene find with warning.
        /// Prefer direct inspector references over this.
        /// </summary>
        public static ControlAuthorityManager Get()
        {
            if (_instance != null) return _instance;
            _instance = FindFirstObjectByType<ControlAuthorityManager>();
            if (_instance != null)
                Debug.LogWarning("[AuthorityManager] Instance resolved via Find. Prefer direct reference.");
            return _instance;
        }

        // ============================================================================
        // Unity Lifecycle
        // ============================================================================
        
        private void Awake()
        {
            _instance = this;
            _configuredDefault = defaultSource;
            
            // Initialize source statuses
            // UI is always Available (never Unavailable)
            SetSourceStatusInternal(InputSource.UI, SourceStatus.Available);
            SetSourceStatusInternal(InputSource.Internal, SourceStatus.Unavailable);
            SetSourceStatusInternal(InputSource.External, SourceStatus.Unavailable);
            
            // Set initial authority to the default source
            _effectiveAuthority = defaultSource;
            _sessionOverrideActive = false;
            
            // If default is UI, mark it as Connected immediately
            if (defaultSource == InputSource.UI)
            {
                SetSourceStatusInternal(InputSource.UI, SourceStatus.Connected);
            }
            else
            {
                // Start waiting for the configured default source
                _waitingForConnection = true;
                _waitTimer = 0f;
            }
            
            Debug.Log($"[AuthorityManager] Initialized: default={defaultSource}, authority={_effectiveAuthority}");
        }

        private void Update()
        {
            if (!_waitingForConnection) return;
            
            _waitTimer += Time.unscaledDeltaTime;
            
            if (_waitTimer >= connectionTimeoutSec)
            {
                _waitTimer = 0f; // Reset for next timeout cycle
                Debug.Log($"[AuthorityManager] Connection timeout for {_configuredDefault}. " +
                          $"User can switch to UI or keep waiting.");
                OnConnectionTimeout?.Invoke();
            }
        }

        private void OnDestroy()
        {
            if (_instance == this) _instance = null;
        }

        // ============================================================================
        // Authority Control (Public API)
        // ============================================================================
        
        /// <summary>
        /// Switch authority to a different source. Only succeeds if the target source
        /// is in Connected status. Fires OnAuthorityChanged.
        /// </summary>
        public bool RequestAuthority(InputSource newSource)
        {
            if (newSource == _effectiveAuthority)
            {
                Debug.Log($"[AuthorityManager] RequestAuthority({newSource}): already active, no-op.");
                return true;
            }
            
            var status = GetSourceStatus(newSource);
            if (status != SourceStatus.Connected && status != SourceStatus.Available)
            {
                Debug.LogWarning($"[AuthorityManager] RequestAuthority({newSource}) DENIED: " +
                                 $"status={status}. Source must be Connected or Available.");
                return false;
            }
            
            var old = _effectiveAuthority;
            _effectiveAuthority = newSource;
            
            // Track if this is a session override
            if (newSource != _configuredDefault)
                _sessionOverrideActive = true;
            
            Debug.Log($"[AuthorityManager] Authority: {old} -> {newSource}" +
                      (_sessionOverrideActive ? " (session override)" : ""));
            
            OnAuthorityChanged?.Invoke(old, newSource);
            return true;
        }
        
        /// <summary>
        /// Session override: switch to UI regardless of configured default.
        /// Does not change the YAML default — next session starts with configured default again.
        /// </summary>
        public void SwitchToUIForSession()
        {
            _waitingForConnection = false;
            _sessionOverrideActive = true;
            
            // Ensure UI is Connected
            SetSourceStatus(InputSource.UI, SourceStatus.Connected);
            
            var old = _effectiveAuthority;
            _effectiveAuthority = InputSource.UI;
            
            Debug.Log($"[AuthorityManager] Session override: {old} -> UI");
            OnAuthorityChanged?.Invoke(old, InputSource.UI);
        }

        /// <summary>
        /// Update the configured default source. Takes effect for the current session
        /// (resets session override) and persists if you save YAML.
        /// </summary>
        public void SetDefaultSource(InputSource newDefault)
        {
            _configuredDefault = newDefault;
            defaultSource = newDefault;
            _sessionOverrideActive = false;
            Debug.Log($"[AuthorityManager] Default source changed to: {newDefault}");
        }

        // ============================================================================
        // Source Lifecycle (called by source adapters)
        // ============================================================================
        
        /// <summary>
        /// Called when a source becomes available (e.g., Internal script registers,
        /// RPC server starts listening). Does not grant authority.
        /// </summary>
        public void SetSourceStatus(InputSource source, SourceStatus newStatus)
        {
            var old = _sourceStatuses[(int)source];
            if (old == newStatus) return;
            
            // Denied is sticky for External — can't transition out of Denied in same session
            if (old == SourceStatus.Denied && source == InputSource.External)
            {
                Debug.LogWarning($"[AuthorityManager] External is Denied for this session. Ignoring status change to {newStatus}.");
                return;
            }
            
            SetSourceStatusInternal(source, newStatus);
            
            Debug.Log($"[AuthorityManager] Source {source}: {old} -> {newStatus}");
            OnSourceStatusChanged?.Invoke(source, old, newStatus);
            
            // If the default source just connected and we were waiting, stop waiting
            if (source == _configuredDefault && newStatus == SourceStatus.Connected && _waitingForConnection)
            {
                _waitingForConnection = false;
                _waitTimer = 0f;
                
                // Auto-grant authority if this is the configured default
                if (_effectiveAuthority != source)
                {
                    RequestAuthority(source);
                }
                
                Debug.Log($"[AuthorityManager] Default source {source} connected. Waiting resolved.");
            }
        }
        
        /// <summary>
        /// Called when an External client attempts to connect.
        /// If External is default and autoAcceptWhenDefault is true, auto-accepts.
        /// Otherwise, fires OnExternalConnectionRequest for UI to show popup.
        /// </summary>
        public void NotifyExternalConnectionAttempt()
        {
            var currentStatus = GetSourceStatus(InputSource.External);
            
            if (currentStatus == SourceStatus.Denied)
            {
                Debug.Log("[AuthorityManager] External connection attempt blocked — Denied for this session.");
                return;
            }
            
            if (_configuredDefault == InputSource.External && autoAcceptWhenDefault)
            {
                // Auto-accept when External is the configured default
                AcceptExternalConnection();
                return;
            }
            
            // Need user approval
            _externalPendingApproval = true;
            Debug.Log("[AuthorityManager] External connection request — awaiting user approval.");
            OnExternalConnectionRequest?.Invoke();
        }
        
        /// <summary>
        /// User approved the External connection. Resets drone, switches authority.
        /// </summary>
        public void AcceptExternalConnection()
        {
            _externalPendingApproval = false;
            
            SetSourceStatus(InputSource.External, SourceStatus.Connected);
            
            var old = _effectiveAuthority;
            _effectiveAuthority = InputSource.External;
            
            if (old != _effectiveAuthority)
            {
                Debug.Log($"[AuthorityManager] External connection ACCEPTED. Authority: {old} -> External");
                OnAuthorityChanged?.Invoke(old, InputSource.External);
            }
        }
        
        /// <summary>
        /// User denied the External connection. Blocks all future attempts for this session.
        /// </summary>
        public void DenyExternalConnection()
        {
            _externalPendingApproval = false;
            
            SetSourceStatus(InputSource.External, SourceStatus.Denied);
            Debug.Log("[AuthorityManager] External connection DENIED for this session.");
        }
        
        /// <summary>
        /// Called when an External client disconnects. Does NOT auto-switch authority.
        /// The proxy will use safe fallback commands until authority is explicitly changed.
        /// </summary>
        public void NotifyExternalDisconnected()
        {
            if (GetSourceStatus(InputSource.External) == SourceStatus.Denied) return;
            
            SetSourceStatus(InputSource.External, SourceStatus.Available);
            Debug.Log("[AuthorityManager] External disconnected. Authority unchanged — proxy uses safe fallback.");
        }
        
        /// <summary>
        /// Called when an Internal script registers itself in the scene.
        /// </summary>
        public void NotifyInternalAvailable()
        {
            SetSourceStatus(InputSource.Internal, SourceStatus.Available);
        }
        
        /// <summary>
        /// Called when an Internal script starts actively publishing commands.
        /// </summary>
        public void NotifyInternalConnected()
        {
            SetSourceStatus(InputSource.Internal, SourceStatus.Connected);
        }
        
        /// <summary>
        /// Called when an Internal script is removed or stops publishing.
        /// </summary>
        public void NotifyInternalDisconnected()
        {
            SetSourceStatus(InputSource.Internal, SourceStatus.Unavailable);
        }
        
        // ============================================================================
        // Query Helpers
        // ============================================================================
        
        /// <summary>Is the given source the current authority?</summary>
        public bool IsAuthority(InputSource source) => source == _effectiveAuthority;
        
        /// <summary>Can this source potentially take authority? (Connected or Available)</summary>
        public bool CanRequestAuthority(InputSource source)
        {
            var status = GetSourceStatus(source);
            return status == SourceStatus.Connected || status == SourceStatus.Available;
        }
        
        /// <summary>Is the External source blocked for this session?</summary>
        public bool IsExternalDenied => GetSourceStatus(InputSource.External) == SourceStatus.Denied;

        // ============================================================================
        // Internal
        // ============================================================================
        
        private void SetSourceStatusInternal(InputSource source, SourceStatus status)
        {
            _sourceStatuses[(int)source] = status;
        }
    }
}