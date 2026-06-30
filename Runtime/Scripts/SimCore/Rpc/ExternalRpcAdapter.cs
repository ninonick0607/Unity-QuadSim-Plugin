// Assets/Scripts/SimCore/Rpc/ExternalRpcAdapter.cs
// Phase 9a + 9b + 9c: External RPC Adapter
//
// PURPOSE:
//   MonoBehaviour that lives on SimRoot. Bridges the background ZMQ server thread
//   to the Unity main thread where SimCoreApi and QuadSimApi calls must happen.
//
// RESPONSIBILITIES:
//   - Start/stop the RpcServerThread
//   - Drain incoming requests in Update() and dispatch to method handlers
//   - Manage External source lifecycle through SimCoreApi
//   - Track connection state and heartbeat timeout
//   - Publish telemetry on PUB socket at configurable rate (Phase 9c)
//
// PHASE 9a: connect / disconnect / heartbeat / get_status
// PHASE 9b: set_mode, send_command, get_sensor_data, get_telemetry,
//           set_controller, reset_all, reset_pose, reset_rotation,
//           reset_physics, reset_controller, pause, resume, step,
//           set_time_scale, reset_simulation
// PHASE 9c: subscribe / unsubscribe, PUB telemetry streaming

using System;
using System.Collections.Generic;
using UnityEngine;
using SimCore.Common;
using DroneCore.Api;
using MathUtil;
using RobotCore;

namespace SimCore.Rpc
{
    /// <summary>
    /// Bridges External Python clients to the simulation via ZeroMQ.
    /// Attach to SimRoot alongside SimCoreApi.
    ///
    /// The adapter starts a background thread with REP + PUB sockets.
    /// Requests arrive on the background thread, get queued to main thread,
    /// processed against SimCoreApi/QuadSimApi, and responses are sent back.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ExternalRpcAdapter : MonoBehaviour
    {
        // ============================================================================
        // Configuration
        // ============================================================================

        [Header("Server Settings")]
        [Tooltip("Port for the REQ/REP command socket.")]
        [SerializeField] private int commandPort = 5555;

        [Tooltip("Port for the PUB telemetry socket.")]
        [SerializeField] private int telemetryPort = 5556;

        [Tooltip("Start the RPC server automatically on Awake.")]
        [SerializeField] private bool autoStart = true;

        [Header("Connection Settings")]
        [Tooltip("Seconds without a heartbeat before considering the client disconnected.")]
        [SerializeField] private float heartbeatTimeoutSec = 5.0f;

        [Tooltip("Maximum requests to process per frame (prevents stalling).")]
        [SerializeField] private int maxRequestsPerFrame = 10;

        [Header("Telemetry Streaming (Phase 9c)")]
        [Tooltip("Default publish rate in Hz. Client can override via subscribe. 0 = disabled.")]
        [SerializeField] private float defaultPublishHz = 50f;

        // ============================================================================
        // Runtime State
        // ============================================================================

        private RpcServerThread _server;
        private SimCoreApi _sim;

        // Connection tracking
        private bool _clientConnected;
        private string _clientName;
        private float _lastHeartbeatTime;

        // Pending accept/deny (when External is not auto-accepted)
        private bool _pendingConnectionApproval;
        private QueuedRequest _pendingConnectRequest;

        // Method dispatch table
        private Dictionary<string, Func<RpcRequest, RpcResponse>> _handlers;

        // Telemetry streaming state (Phase 9c)
        private bool _streamingActive;
        private float _publishHz;
        private float _publishTimer;
        private HashSet<string> _subscribedTopics = new() { "sensors", "telemetry" };

        // ============================================================================
        // String ↔ Enum Maps (Phase 9b)
        // ============================================================================

        private static readonly Dictionary<string, GoalMode> GoalModeMap = new()
        {
            ["none"]        = GoalMode.None,
            ["position"]    = GoalMode.Position,
            ["velocity"]    = GoalMode.Velocity,
            ["acceleration"] = GoalMode.Acceleration,
            ["accel"]       = GoalMode.Acceleration,
            ["angle"]       = GoalMode.Angle,
            ["rate"]        = GoalMode.Rate,
            ["passthrough"] = GoalMode.Passthrough,
            ["wrench"]      = GoalMode.Wrench,
            ["wrench_bypassed"] = GoalMode.WrenchBypassed,

        };

        private static readonly Dictionary<GoalMode, string> GoalModeReverseMap = new()
        {
            [GoalMode.None]        = "none",
            [GoalMode.Position]    = "position",
            [GoalMode.Velocity]    = "velocity",
            [GoalMode.Acceleration] = "acceleration",
            [GoalMode.Angle]       = "angle",
            [GoalMode.Rate]        = "rate",
            [GoalMode.Passthrough] = "passthrough",
            [GoalMode.Wrench]      = "wrench",
            [GoalMode.WrenchBypassed] = "wrench_bypassed",
        };

        private static readonly Dictionary<string, ControllerKind> ControllerKindMap = new()
        {
            ["cascade"]   = ControllerKind.Cascade,
            ["geometric"] = ControllerKind.Geometric,
        };

        private static readonly Dictionary<ControllerKind, string> ControllerKindReverseMap = new()
        {
            [ControllerKind.Cascade]   = "cascade",
            [ControllerKind.Geometric] = "geometric",
        };

        // ============================================================================
        // Public Accessors
        // ============================================================================

        /// <summary>True if the ZMQ server thread is running.</summary>
        public bool IsServerRunning => _server?.IsRunning ?? false;

        /// <summary>True if an External client is currently connected.</summary>
        public bool IsClientConnected => _clientConnected;

        /// <summary>Name of the connected client (from connect message), or null.</summary>
        public string ClientName => _clientName;
        
        private SimFrame _clientFrame = SimFrame.FLU;

        /// <summary>Command port number.</summary>
        public int CommandPort => commandPort;

        /// <summary>Telemetry port number.</summary>
        public int TelemetryPort => telemetryPort;
        
        /// <summary>Set ports before StartServer(). CLI overrides still win (applied in Start).</summary>
        public void ConfigurePorts(int cmd, int tel)
        {
            commandPort = cmd;
            telemetryPort = tel;
        }
        public event Action<Dictionary<string, object>> OnVizPayload;

        // ============================================================================
        // Unity Lifecycle
        // ============================================================================

        private void Awake()
        {
            BuildHandlerTable();
        }

        private void Start()
        {
            ApplyCommandLineOverrides();
            _sim = SimCoreApi.Get();
            if (_sim == null)
            {
                Debug.LogError("[RpcAdapter] No SimCoreApi found. ExternalRpcAdapter requires SimCoreApi on SimRoot.");
                enabled = false;
                return;
            }

            // Subscribe to authority events for accept/deny flow
            _sim.OnAuthorityChanged += HandleAuthorityChanged;

            if (autoStart)
            {
                StartServer();
            }
        }

        private void Update()
        {
            if (_server == null) return;

            // Drain log messages from background thread
            DrainLogs();

            // Process incoming requests
            ProcessRequests();

            // Check heartbeat timeout
            CheckHeartbeatTimeout();

            // Publish telemetry stream (Phase 9c)
            PublishTick();
        }

        private void OnDestroy()
        {
            StopServer();

            if (_sim != null)
            {
                _sim.OnAuthorityChanged -= HandleAuthorityChanged;
            }
        }

        private void OnApplicationQuit()
        {
            StopServer();
        }

        // ============================================================================
        // Server Lifecycle
        // ============================================================================

        /// <summary>Start the ZMQ server thread.</summary>
        ///
        private void ApplyCommandLineOverrides()
        {
            var args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
            {
                if (args[i] == "-rpcPort" && int.TryParse(args[i + 1], out int rp))
                    commandPort = rp;
                if (args[i] == "-telemetryPort" && int.TryParse(args[i + 1], out int tp))
                    telemetryPort = tp;
            }
            Debug.Log($"[RpcAdapter] Ports: REP={commandPort} PUB={telemetryPort}");
        }
        public void StartServer()
        {
            if (_server != null && _server.IsRunning)
            {
                Debug.LogWarning("[RpcAdapter] Server already running.");
                return;
            }

            _server = new RpcServerThread(commandPort, telemetryPort);
            _server.Start();

            Debug.Log($"[RpcAdapter] Starting RPC server — REP:{commandPort} PUB:{telemetryPort}");
        }

        /// <summary>Stop the ZMQ server thread and clean up.</summary>
        public void StopServer()
        {
            if (_server == null) return;

            // If a client was connected, notify disconnect
            if (_clientConnected)
            {
                HandleClientDisconnect("server_shutdown");
            }

            _server.Stop();
            _server.Dispose();
            _server = null;

            Debug.Log("[RpcAdapter] RPC server stopped.");
        }

        // ============================================================================
        // Request Processing (Main Thread)
        // ============================================================================

        private void ProcessRequests()
        {
            int processed = 0;

            while (processed < maxRequestsPerFrame && _server.TryDequeue(out var queued))
            {
                processed++;

                RpcResponse response;

                try
                {
                    if (_handlers.TryGetValue(queued.Request.Method, out var handler))
                    {
                        // Any valid request from a connected client proves liveness —
                        // refresh heartbeat so active clients don't time out.
                        if (_clientConnected)
                        {
                            _lastHeartbeatTime = Time.unscaledTime;
                        }

                        response = handler(queued.Request);
                    }
                    else
                    {
                        response = RpcResponse.Error(
                            $"unknown_method: {queued.Request.Method}",
                            queued.Request.RequestId);
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogError($"[RpcAdapter] Exception handling '{queued.Request.Method}': {ex}");
                    response = RpcResponse.Error("internal_error", queued.Request.RequestId);
                }

                // Serialize and signal the background thread
                byte[] responseBytes = RpcSerializer.SerializeResponse(response);
                queued.SetResponse(responseBytes);
            }
        }

        private void DrainLogs()
        {
            while (_server.TryDequeueLog(out var level, out var message))
            {
                switch (level)
                {
                    case RpcServerThread.LogLevel.Info:
                        Debug.Log(message);
                        break;
                    case RpcServerThread.LogLevel.Warning:
                        Debug.LogWarning(message);
                        break;
                    case RpcServerThread.LogLevel.Error:
                        Debug.LogError(message);
                        break;
                }
            }
        }

        // ============================================================================
        // Heartbeat Monitoring
        // ============================================================================

        private void CheckHeartbeatTimeout()
        {
            if (!_clientConnected) return;
            if (heartbeatTimeoutSec <= 0f) return;

            float elapsed = Time.unscaledTime - _lastHeartbeatTime;

            if (elapsed > heartbeatTimeoutSec)
            {
                Debug.LogWarning($"[RpcAdapter] Heartbeat timeout ({elapsed:F1}s > {heartbeatTimeoutSec}s). " +
                                 $"Disconnecting client '{_clientName}'.");
                HandleClientDisconnect("heartbeat_timeout");
            }
        }

        // ============================================================================
        // Handler Table
        // ============================================================================

        private void BuildHandlerTable()
        {
            _handlers = new Dictionary<string, Func<RpcRequest, RpcResponse>>
            {
                // --- Phase 9a: Connection lifecycle ---
                ["connect"]          = HandleConnect,
                ["disconnect"]       = HandleDisconnect,
                ["heartbeat"]        = HandleHeartbeat,
                ["get_status"]       = HandleGetStatus,
                ["set_frame"] = HandleSetFrame,

                // --- Phase 9b: Drone control ---
                ["set_mode"]         = HandleSetMode,
                ["send_command"]     = HandleSendCommand,
                ["get_sensor_data"]  = HandleGetSensorData,
                ["get_telemetry"]    = HandleGetTelemetry,
                ["set_controller"]   = HandleSetController,

                // --- Phase 9b: Reset operations ---
                ["reset_all"]        = HandleResetAll,
                ["reset_pose"]       = HandleResetPose,
                ["reset_rotation"]   = HandleResetRotation,
                ["reset_physics"]    = HandleResetPhysics,
                ["reset_controller"] = HandleResetController,

                // --- Phase 9b: Simulation control ---
                ["pause"]            = HandlePause,
                ["resume"]           = HandleResume,
                ["step"]             = HandleStep,
                ["step_with_command"] = HandleStepWithCommand,
                ["set_time_scale"]   = HandleSetTimeScale,
                ["reset_simulation"] = HandleResetSimulation,
                ["set_wind"]         = HandleSetWind,
                ["set_geo_origin"] = HandleSetGeoOrigin,
                ["update_viz"] = HandleUpdateViz,
                // --- Phase 9c: Telemetry streaming ---
                ["subscribe"]        = HandleSubscribe,
                ["unsubscribe"]      = HandleUnsubscribe,
            };
        }

        /// <summary>
        /// Register additional method handlers (for Phase 9c, 9d extensions).
        /// Call this before or after server start — handlers are on the main thread.
        /// </summary>
        public void RegisterHandler(string method, Func<RpcRequest, RpcResponse> handler)
        {
            _handlers[method] = handler;
        }

        // ============================================================================
        // Phase 9a Handlers: Connection Lifecycle
        // ============================================================================

        // --- connect ---

        private RpcResponse HandleConnect(RpcRequest req)
        {
            if (_clientConnected)
            {
                return RpcResponse.Error("already_connected", req.RequestId);
            }

            string clientName = req.GetString("client_name") ?? "unknown";

            Debug.Log($"[RpcAdapter] Connection request from '{clientName}'");

            // Check if External is denied for this session
            if (_sim.IsExternalDenied)
            {
                Debug.Log($"[RpcAdapter] Rejecting '{clientName}' — External is denied for this session.");
                return RpcResponse.Error("denied", req.RequestId);
            }

            // Notify the authority manager
            _sim.NotifyExternalConnectionAttempt();

            // Check if it was auto-accepted (External is default + autoAccept)
            var externalStatus = _sim.GetSourceStatus(InputSource.External);
            if (externalStatus == SourceStatus.Connected)
            {
                // Auto-accepted
                CompleteConnection(clientName);
                return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
                {
                    ["message"] = "connected",
                    ["server"] = "QuadSim",
                    ["command_port"] = commandPort,
                    ["telemetry_port"] = telemetryPort,
                });
            }

            if (externalStatus == SourceStatus.Denied)
            {
                return RpcResponse.Error("denied", req.RequestId);
            }

 
            _sim.AcceptExternalConnection();
            CompleteConnection(clientName);

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["message"] = "connected",
                ["server"] = "QuadSim",
                ["command_port"] = commandPort,
                ["telemetry_port"] = telemetryPort,
            });
        }

        private void CompleteConnection(string clientName)
        {
            _clientConnected = true;
            _clientName = clientName;
            _lastHeartbeatTime = Time.unscaledTime;

            Debug.Log($"[RpcAdapter] Client '{clientName}' connected. External source active.");
        }

        // --- disconnect ---

        private RpcResponse HandleDisconnect(RpcRequest req)
        {
            if (!_clientConnected)
            {
                return RpcResponse.Ok(req.RequestId); // Idempotent
            }

            Debug.Log($"[RpcAdapter] Client '{_clientName}' requested disconnect.");
            HandleClientDisconnect("client_request");

            return RpcResponse.Ok(req.RequestId);
        }

        // --- heartbeat ---

        private RpcResponse HandleHeartbeat(RpcRequest req)
        {
            if (!_clientConnected)
            {
                return RpcResponse.Error("not_connected", req.RequestId);
            }

            _lastHeartbeatTime = Time.unscaledTime;

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["sim_time"] = _sim.SimTimeSec,
                ["is_paused"] = _sim.IsPaused,
            });
        }

        
        private RpcResponse HandleSetFrame(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
    
            string frameStr = req.GetString("frame");
            if (frameStr == null)
                return RpcResponse.Error("missing_param: frame", req.RequestId);
    
            switch (frameStr.ToLowerInvariant())
            {
                case "flu":
                    _clientFrame = SimFrame.FLU;
                    break;
                case "frd":
                    _clientFrame = SimFrame.FRD;
                    break;
                case "unity":
                    _clientFrame = SimFrame.UnityBody;
                    break;
                default:
                    return RpcResponse.Error($"invalid_frame: {frameStr}", req.RequestId);
            }
    
            // Also update the sensor output frame so reads match writes
            if (TryGetSelectedDrone(req, out var drone, out var droneErr))
            {
                // You may need to expose a method to set the sensor frame on QuadSimApi
                // For now, this ensures symmetry
            }
    
            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["frame"] = frameStr.ToLowerInvariant()
            });
        }
        
        // --- get_status ---

        private RpcResponse HandleGetStatus(RpcRequest req)
        {
            var status = _sim.GetStatus();

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["is_paused"] = status.IsPaused,
                ["time_scale"] = status.TimeScale,
                ["sim_time"] = status.SimTimeSec,
                ["fixed_dt"] = status.FixedDtSec,
                ["authority"] = status.CurrentAuthority.ToString(),
                ["ui_status"] = status.UIStatus.ToString(),
                ["internal_status"] = status.InternalStatus.ToString(),
                ["external_status"] = status.ExternalStatus.ToString(),
                ["client_connected"] = _clientConnected,
                ["client_name"] = _clientName ?? "",
            });
        }

        // ============================================================================
        // Phase 9b Handlers: Drone Control
        // ============================================================================

        // --- set_mode ---

        private RpcResponse HandleSetMode(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            string modeStr = req.GetString("mode");
            if (modeStr == null)
            {
                return RpcResponse.Error("missing_param: mode", req.RequestId);
            }

            if (!GoalModeMap.TryGetValue(modeStr.ToLowerInvariant(), out var goalMode))
            {
                return RpcResponse.Error($"invalid_mode: {modeStr}", req.RequestId);
            }

            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.SetMode(goalMode);

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["mode"] = GoalModeReverseMap[drone.CurrentMode],
            });
        }

        // --- send_command ---

        private RpcResponse HandleSendCommand(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            if (_sim.CurrentAuthority != InputSource.External)
            {
                return RpcResponse.Error("authority_rejected", req.RequestId);
            }

            // Extract raw values from client (in client's frame convention)
            float x = req.GetFloat("x");
            float y = req.GetFloat("y");
            float z = req.GetFloat("z");
            float w = req.GetFloat("w");

            // Optional mode override
            string modeStr = req.GetString("mode");
            GoalMode? goalMode = null;
    
            if (modeStr != null)
            {
                if (!GoalModeMap.TryGetValue(modeStr.ToLowerInvariant(), out var gm))
                    return RpcResponse.Error($"invalid_mode: {modeStr}", req.RequestId);
                goalMode = gm;
            }
    
            // Determine the active mode (either from override or current)
            GoalMode activeMode = goalMode ?? drone.CurrentMode;
    
            // Remap client frame → internal Axis4 layout
            Axis4 axis4 = RemapClientCommand(x, y, z, w, activeMode, _clientFrame);

            bool success;
            if (goalMode.HasValue)
                success = drone.SendCommand(InputSource.External, axis4, goalMode.Value);
            else
                success = drone.SendCommand(InputSource.External, axis4);

            if (!success)
                return RpcResponse.Error("command_failed", req.RequestId);

            return RpcResponse.Ok(req.RequestId);
            }
        private static Axis4 RemapClientCommand(float x, float y, float z, float w, 
                                                 GoalMode mode, SimFrame clientFrame)
        {
            if (clientFrame == SimFrame.UnityBody)
            {
                // No remapping — client is sending raw Axis4 layout
                return new Axis4(x, y, z, w);
            }
            
            // For FLU/FRD clients, the meaning of (x,y,z,w) depends on mode
            switch (mode)
            {
                case GoalMode.Position:
                    // Client sends: (X, Y, Z, Yaw_deg) in client world frame
                    // Internal Axis4 layout: (X_world, Y_world, Yaw_deg, Alt_world)
                    //
                    // Convert position from client frame → Unity world:
                    Vector3 posClient = new Vector3(x, y, z);
                    Vector3 posUnity = Frames.InverseTransformWorldPosition(posClient, clientFrame);
                    // Map into Axis4 layout: (X, Z, Yaw, Y_alt)
                    return new Axis4(posUnity.x, posUnity.z, w, posUnity.y);
         
                case GoalMode.Velocity:
                    // Client sends: (Vx, Vy, Vz, YawRate_dps) in client body frame
                    // Internal Axis4 layout: (Vx, Vy, YawRate, Vz)
                    //
                    // Convert velocity from client frame → Unity body:
                    Vector3 velClient = new Vector3(x, y, z);
                    Vector3 velUnity = Frames.InverseTransformLinear(velClient, clientFrame);
                    // Map into Axis4 layout: (Vx, Vz, YawRate, Vy_alt)
                    return new Axis4(velUnity.x, velUnity.z, w, velUnity.y);

                case GoalMode.Acceleration:
                    // Client sends: (Ax, Ay, Az, YawRate_dps) in client body/output frame.
                    // Internal Axis4 layout: (Ax, Ay, YawRate, Az).
                    // Acceleration is a true vector, so it uses the same linear transform
                    // as body-frame velocity.
                    Vector3 accelClient = new Vector3(x, y, z);
                    Vector3 accelUnity = Frames.InverseTransformLinear(accelClient, clientFrame);
                    return new Axis4(accelUnity.x, accelUnity.z, w, accelUnity.y);
         
                case GoalMode.Angle:
                case GoalMode.Rate:
                    // Client sends: (Roll, Pitch, YawRate, Throttle)
                    // These are already in the correct layout — no reorder needed.
                    // Frame signs for attitude commands would come from Frames.TransformAttitude
                    // but for now we assume the client sends in the frame the controller expects.
                    return new Axis4(x, y, z, w);
                    
                case GoalMode.WrenchBypassed:
                    // Client sends: (tx, ty, tz, thrust) — torques in client body frame
                    // Thrust is a scalar along body vertical — no frame conversion needed.
                    //
                    // Torques are pseudovectors (same transform as angular velocity).
                    // Use Frames.InverseTransformAngularVelocity to convert client→Unity,
                    // which handles both the axis swap AND chirality correction.
                    Vector3 torqueClient = new Vector3(x, y, z);
                    Vector3 torqueUnity = Frames.InverseTransformAngularVelocity(torqueClient, clientFrame);
                    return new Axis4(torqueUnity.x, torqueUnity.y, torqueUnity.z, w);
                case GoalMode.Wrench:
                    // External wrench routed through the effectiveness-matrix
                    // allocator (built in the FLU internal frame). The FLU->Unity
                    // mapping happens implicitly inside the allocate->actuate round
                    // trip, so — unlike GoalMode.Wrench — do NOT convert here.
                    // Assumes clientFrame == allocator build frame (FLU). If you add
                    // FRD clients, convert torque client->FLU at this point.
                    return new Axis4(x, y, z, w);   // (Mx, My, Mz, f), FLU passthrough
                case GoalMode.Passthrough:
                case GoalMode.None:
                default:
                    // No transformation — motor values and None don't need remapping
                    return new Axis4(x, y, z, w);
            }
        }
        // --- get_sensor_data ---

        private RpcResponse HandleGetSensorData(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;
 
            var s = drone.GetSensorData();
            return RpcResponse.Ok(req.RequestId, SensorDataToDict(s));
        }

        // --- get_telemetry ---

        private RpcResponse HandleGetTelemetry(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            var t = drone.GetTelemetry();

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["drone_id"]           = t.DroneID ?? "",
                ["mode"]               = GoalModeReverseMap.GetValueOrDefault(t.Mode, "unknown"),
                ["controller"]         = ControllerKindReverseMap.GetValueOrDefault(t.Controller, "unknown"),
                ["motors"]             = new object[] { t.MotorOutputs.FL, t.MotorOutputs.FR,
                                                        t.MotorOutputs.BL, t.MotorOutputs.BR },
                ["desired_rates_deg"]  = Vec3ToArray(t.ControllerTelemetry.DesiredRatesDeg),
                ["desired_angles_deg"] = Vec3ToArray(t.ControllerTelemetry.DesiredAnglesDeg),
                ["desired_vel"]        = Vec3ToArray(t.ControllerTelemetry.DesiredVel),
                ["desired_accel"]      = Vec3ToArray(t.ControllerTelemetry.DesiredAccel),
                ["external_cmd"]       = new object[] { t.ControllerTelemetry.ExternalCmd.X,
                                                        t.ControllerTelemetry.ExternalCmd.Y,
                                                        t.ControllerTelemetry.ExternalCmd.Z,
                                                        t.ControllerTelemetry.ExternalCmd.W },
                ["motor_thrusts"]      = new object[] { t.MotorThrusts.FL, t.MotorThrusts.FR,
                    t.MotorThrusts.BL, t.MotorThrusts.BR },
            });
        }

        // --- set_controller ---

        private RpcResponse HandleSetController(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            string kindStr = req.GetString("controller");
            if (kindStr == null)
            {
                return RpcResponse.Error("missing_param: controller", req.RequestId);
            }

            if (!ControllerKindMap.TryGetValue(kindStr.ToLowerInvariant(), out var kind))
            {
                return RpcResponse.Error($"invalid_controller: {kindStr}", req.RequestId);
            }

            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.SetController(kind);

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["controller"] = ControllerKindReverseMap[drone.CurrentController],
            });
        }

        // ============================================================================
        // Environment Handlers: Plant / Disturbance
        // ============================================================================

        // --- set_wind ---
        // Toggles (and optionally re-speeds) every WindModule in the scene. Wind is
        // plant configuration, not vehicle control, so it requires a connected client
        // but NOT External drone authority — same tier as pause / set_time_scale.
        private RpcResponse HandleSetWind(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            bool enabled = req.GetBool("enabled", true);

            // wind_speed is optional — only overridden if the client sent it.
            bool hasSpeed = req.Params.ContainsKey("wind_speed");
            float windSpeed = req.GetFloat("wind_speed", 0f);

            int n = 0;
            foreach (var w in Experimental.WindModule.Active)
            {
                if (w == null) continue;
                w.SetEnabled(enabled);
                if (hasSpeed) w.SetWindSpeed(windSpeed);
                n++;
            }

            if (n == 0)
                Debug.LogWarning("[RpcAdapter] set_wind: no WindModule in scene — nothing toggled.");
            else
                Debug.Log($"[RpcAdapter] set_wind: enabled={enabled}" +
                          (hasSpeed ? $", wind_speed={windSpeed}" : "") + $" -> {n} module(s).");

            var data = new Dictionary<string, object> { ["enabled"] = enabled, ["modules"] = n };
            if (hasSpeed) data["wind_speed"] = windSpeed;
            return RpcResponse.Ok(req.RequestId, data);
        }


        private RpcResponse HandleSetGeoOrigin(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            float lat = req.GetFloat("lat", 30.4636f);
            float lon = req.GetFloat("lon", -86.5533f);
            float alt = req.GetFloat("alt", 0.0f);
            
            _sim.Geo.SetOrigin(lat, lon, alt);
            
            return RpcResponse.Ok(req.RequestId);
        }
        
        // ============================================================================
        // Phase 9b Handlers: Reset Operations
        // ============================================================================

        // --- reset_all ---

        private RpcResponse HandleResetAll(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.ResetAll();

            return RpcResponse.Ok(req.RequestId);
        }

        // --- reset_pose ---

        private RpcResponse HandleResetPose(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;
 
            // Read client-frame position
            float px = req.GetFloat("x", 0f);
            float py = req.GetFloat("y", 0f);
            float pz = req.GetFloat("z", 0f);
 
            // Read client-frame quaternion (defaults to identity)
            float qx = req.GetFloat("qx", 0f);
            float qy = req.GetFloat("qy", 0f);
            float qz = req.GetFloat("qz", 0f);
            float qw = req.GetFloat("qw", 1f);
 
            // Convert from client frame → Unity
            Vector3 position = Frames.InverseTransformWorldPosition(
                new Vector3(px, py, pz), _clientFrame);
            Quaternion rotation = Frames.InverseTransformQuaternion(
                new Quaternion(qx, qy, qz, qw), _clientFrame);
 
            drone.ResetPose(position, rotation);
 
            return RpcResponse.Ok(req.RequestId);
        }

        // --- reset_rotation ---

        private RpcResponse HandleResetRotation(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.ResetRotation();

            return RpcResponse.Ok(req.RequestId);
        }

        // --- reset_physics ---

        private RpcResponse HandleResetPhysics(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.ResetPhysics();

            return RpcResponse.Ok(req.RequestId);
        }

        // --- reset_controller ---

        private RpcResponse HandleResetController(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;
            if (!TryGetSelectedDrone(req, out var drone, out var droneErr)) return droneErr;

            drone.ResetController();

            return RpcResponse.Ok(req.RequestId);
        }

        // ============================================================================
        // Phase 9b Handlers: Simulation Control
        // ============================================================================

        // --- pause ---

        private RpcResponse HandlePause(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            _sim.Pause();

            return RpcResponse.Ok(req.RequestId);
        }

        // --- resume ---

        private RpcResponse HandleResume(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            _sim.Resume();

            return RpcResponse.Ok(req.RequestId);
        }

        // --- step ---

        private RpcResponse HandleStep(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            int count = req.GetInt("count", 1);
            if (count <= 0)
                return RpcResponse.Error("invalid_param: count must be > 0", req.RequestId);

            if (_sim == null || _sim.SimManager == null)
                return RpcResponse.Error("simulation_manager_missing", req.RequestId);

            _sim.SimManager.StepImmediate(count);

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["sim_time"] = _sim.SimTimeSec,
                ["steps_ran"] = count,
            });
        }

        private int _px4ProfileCount;
        private double _px4CmdMs;
        private double _px4StepMs;
        private double _px4VizMs;
        private double _px4SensorMs;
        private double _px4TotalMs;

        private static double TicksToMs(long ticks)
        {
            return ticks * 1000.0 / System.Diagnostics.Stopwatch.Frequency;
        }

        private RpcResponse HandleStepWithCommand(RpcRequest req)
        {
            var cmdResp = HandleSendCommand(req);
            if (!cmdResp.Success) return cmdResp;

            var stepResp = HandleStep(req);
            if (!stepResp.Success) return stepResp;

            return HandleGetSensorData(req);
        }
        
        private RpcResponse HandleUpdateViz(RpcRequest req)
        {
            OnVizPayload?.Invoke(req.Params);
            return RpcResponse.Ok(req.RequestId);
        }
        
        // --- set_time_scale ---

        private RpcResponse HandleSetTimeScale(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            float scale = req.GetFloat("scale", 1.0f);

            if (scale <= 0f)
            {
                return RpcResponse.Error("invalid_param: scale must be > 0", req.RequestId);
            }

            _sim.SetTimeScale(scale);

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["time_scale"] = _sim.TimeScale,
            });
        }

        // --- reset_simulation ---

        private RpcResponse HandleResetSimulation(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            _sim.ResetSimulation();

            return RpcResponse.Ok(req.RequestId);
        }

        // ============================================================================
        // Phase 9c Handlers: Telemetry Streaming
        // ============================================================================

        // --- subscribe ---

        private static Dictionary<string, object> SensorDataToDict(SensorData s) => new Dictionary<string, object>
        {
            ["imu_ang_vel"]     = Vec3ToArray(s.ImuAngVel),
            ["imu_attitude"]    = Vec3ToArray(s.ImuAttitude),
            ["imu_accel"]       = Vec3ToArray(s.ImuAccel),
            ["imu_vel"]         = Vec3ToArray(s.ImuVel),
            ["imu_orientation"] = QuatToArray(s.ImuOrientation),
            ["imu_timestamp"]   = s.ImuTimestampSec,
            ["imu_valid"]       = s.ImuValid,
            ["gps_position"]    = Vec3ToArray(s.GpsPosition),
            ["gps_timestamp"]   = s.GpsTimestampSec,
            ["gps_valid"]       = s.GpsValid,
            ["mag_field"]            = Vec3ToArray(s.MagField),
            ["mag_heading_deg"]      = s.MagHeadingDeg,
            ["mag_declination_deg"]  = s.MagDeclinationDeg,
            ["mag_timestamp"]        = s.MagTimestampSec,
            ["mag_valid"]            = s.MagValid,
            ["baro_pressure_pa"]       = s.BaroPressurePa,
            ["baro_pressure_hpa"]      = s.BaroPressureHPa,
            ["baro_temperature_c"]     = s.BaroTemperatureC,
            ["baro_altitude_msl"]      = s.BaroAltitudeMSL,
            ["baro_pressure_altitude"] = s.BaroPressureAltitude,
            ["baro_timestamp"]         = s.BaroTimestampSec,
            ["baro_valid"]             = s.BaroValid,
            ["gps_lat"]     = s.GpsLatDeg,
            ["gps_lon"]     = s.GpsLonDeg,
            ["gps_alt"]     = s.GpsAltM,
            ["gps_vel_ned"] = Vec3ToArray(s.GpsVelNED),
        };
        
        
        private RpcResponse HandleSubscribe(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            float hz = req.GetFloat("hz", defaultPublishHz);
            if (hz <= 0f) hz = defaultPublishHz;
            if (hz > 200f) hz = 200f; // Cap at 200 Hz to prevent flooding

            _publishHz = hz;

            // Parse optional topic list — default to both
            _subscribedTopics.Clear();
            string topicsStr = req.GetString("topics");
            if (topicsStr != null)
            {
                // Accept comma-separated or single topic
                foreach (var t in topicsStr.Split(','))
                {
                    string trimmed = t.Trim().ToLowerInvariant();
                    if (trimmed == "sensors" || trimmed == "telemetry")
                    {
                        _subscribedTopics.Add(trimmed);
                    }
                }
            }

            // If no valid topics parsed, default to both
            if (_subscribedTopics.Count == 0)
            {
                _subscribedTopics.Add("sensors");
                _subscribedTopics.Add("telemetry");
            }

            _streamingActive = true;
            _publishTimer = 0f;

            Debug.Log($"[RpcAdapter] Streaming started: {_publishHz} Hz, " +
                      $"topics=[{string.Join(", ", _subscribedTopics)}]");

            return RpcResponse.Ok(req.RequestId, new Dictionary<string, object>
            {
                ["telemetry_port"] = telemetryPort,
                ["hz"] = _publishHz,
                ["topics"] = new List<object>(_subscribedTopics),
            });
        }

        // --- unsubscribe ---

        private RpcResponse HandleUnsubscribe(RpcRequest req)
        {
            if (!RequireConnected(req, out var err)) return err;

            StopStreaming();

            return RpcResponse.Ok(req.RequestId);
        }

        // ============================================================================
        // Telemetry Publishing (Phase 9c)
        // ============================================================================

        private void PublishTick()
        {
            if (!_streamingActive || !_clientConnected) return;
            if (_publishHz <= 0f) return;

            _publishTimer += Time.deltaTime;
            float interval = 1f / _publishHz;

            if (_publishTimer < interval) return;
            _publishTimer -= interval;

            // Clamp to prevent spiral if frame rate drops below publish rate
            if (_publishTimer > interval) _publishTimer = 0f;

            PublishTelemetry();
        }

        private void PublishTelemetry()
        {
            var drone = QuadSimApi.Selected();
            if (drone == null) return;

            if (_subscribedTopics.Contains("sensors"))
            {
                var s = drone.GetSensorData();
                byte[] frame = RpcSerializer.SerializePublish("sensors", SensorDataToDict(s));
                _server.EnqueuePublish(frame);
            }

            if (_subscribedTopics.Contains("telemetry"))
            {
                var t = drone.GetTelemetry();
                var telemData = new Dictionary<string, object>
                {
                    ["drone_id"]           = t.DroneID ?? "",
                    ["mode"]               = GoalModeReverseMap.GetValueOrDefault(t.Mode, "unknown"),
                    ["controller"]         = ControllerKindReverseMap.GetValueOrDefault(t.Controller, "unknown"),
                    ["motors"]             = new object[] { t.MotorOutputs.FL, t.MotorOutputs.FR,
                                                            t.MotorOutputs.BL, t.MotorOutputs.BR },
                    ["desired_rates_deg"]  = Vec3ToArray(t.ControllerTelemetry.DesiredRatesDeg),
                    ["desired_angles_deg"] = Vec3ToArray(t.ControllerTelemetry.DesiredAnglesDeg),
                    ["desired_vel"]        = Vec3ToArray(t.ControllerTelemetry.DesiredVel),
                    ["desired_accel"]      = Vec3ToArray(t.ControllerTelemetry.DesiredAccel),
                    ["external_cmd"]       = new object[] { t.ControllerTelemetry.ExternalCmd.X,
                                                            t.ControllerTelemetry.ExternalCmd.Y,
                                                            t.ControllerTelemetry.ExternalCmd.Z,
                                                            t.ControllerTelemetry.ExternalCmd.W },
                };

                byte[] frame = RpcSerializer.SerializePublish("telemetry", telemData);
                _server.EnqueuePublish(frame);
            }
        }

        private void StopStreaming()
        {
            if (_streamingActive)
            {
                Debug.Log("[RpcAdapter] Streaming stopped.");
            }

            _streamingActive = false;
            _publishTimer = 0f;
        }

        // ============================================================================
        // Disconnect Handling
        // ============================================================================

        private void HandleClientDisconnect(string reason)
        {
            string name = _clientName ?? "unknown";

            // Stop streaming on disconnect
            StopStreaming();

            _clientConnected = false;
            _clientName = null;
            _lastHeartbeatTime = 0f;

            // Notify authority manager
            if (_sim != null)
            {
                _sim.NotifyExternalDisconnected();
            }

            Debug.Log($"[RpcAdapter] Client '{name}' disconnected. Reason: {reason}");
        }

        // ============================================================================
        // Event Handlers
        // ============================================================================

        private void HandleAuthorityChanged(InputSource oldAuth, InputSource newAuth)
        {
            // If authority moved away from External while a client is connected,
            // the client's commands will be rejected by the proxy. We don't
            // disconnect them — they can still read telemetry and get_status.
            if (_clientConnected && oldAuth == InputSource.External && newAuth != InputSource.External)
            {
                Debug.Log($"[RpcAdapter] Authority changed from External to {newAuth}. " +
                          "Client remains connected but commands will be rejected.");
            }
        }

        // ============================================================================
        // Shared Helpers (Phase 9b)
        // ============================================================================

        /// <summary>
        /// Guard: require the client to be connected. Returns false and sets the
        /// error response if not connected.
        /// </summary>
        private bool RequireConnected(RpcRequest req, out RpcResponse errorResponse)
        {
            if (_clientConnected)
            {
                errorResponse = default;
                return true;
            }

            errorResponse = RpcResponse.Error("not_connected", req.RequestId);
            return false;
        }

        /// <summary>
        /// Guard: resolve the selected drone via QuadSimApi.Selected().
        /// Returns false and sets the error response if no drone is available.
        /// Phase 9b is single-drone — multi-drone index parameter is a future addition.
        /// </summary>
        private bool TryGetSelectedDrone(RpcRequest req, out QuadSimApi drone, out RpcResponse errorResponse)
        {
            drone = QuadSimApi.Selected();

            if (drone != null)
            {
                errorResponse = default;
                return true;
            }

            errorResponse = RpcResponse.Error("no_drone", req.RequestId);
            return false;
        }

        /// <summary>Convert Vector3 to float[] for MessagePack serialization.</summary>
        private static object[] Vec3ToArray(Vector3 v)
        {
            return new object[] { v.x, v.y, v.z };
        }

        /// <summary>Convert Quaternion to float[] for MessagePack serialization.</summary>
        private static object[] QuatToArray(Quaternion q)
        {
            return new object[] { q.x, q.y, q.z, q.w };
        }
    }
}