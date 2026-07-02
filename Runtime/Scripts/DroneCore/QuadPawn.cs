using System;
using DroneCore.Controllers;
using DroneCore.Core;
using DroneCore.Interfaces;
using RobotCore;
using SimCore;
using SimCore.Common;
using UnityEngine;
using Yaml.Drone;
using DroneCore.Api;

namespace DroneCore
{
    /// <summary>
    /// QuadPawn is the physical drone entity — the "vehicle" that exists in the world.
    /// It owns references to all drone subsystems and orchestrates the physics step:
    ///   Sensors → Controller → Apply Wrench to Rigidbody
    ///
    /// DroneManager iterates QuadPawns in PrePhysicsStep and calls PhysicsStep().
    /// QuadPawn is NOT an ISimulatable — DroneManager drives it.
    ///
    /// Mirrors UE's AQuadPawn: owns the CustomPhysicsTick sequence,
    /// buffered force application, and controller routing.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    [DisallowMultipleComponent]
    public sealed class QuadPawn : MonoBehaviour
    {
        // ============================================================================
        // Serialized Fields — Set in Inspector / Prefab
        // ============================================================================

        [Header("Core Physics")]
        [SerializeField] private Rigidbody rb;

        [Header("Motor Transforms")]
        [SerializeField] private Transform motorFL;
        [SerializeField] private Transform motorFR;
        [SerializeField] private Transform motorBL;
        [SerializeField] private Transform motorBR;

        [Header("Subsystem References")]
        [SerializeField] private CascadedController controller;
        [SerializeField] private SensorManager sensorManager;
        [SerializeField] private FlightCommandProxy commandProxy;
        [SerializeField] private ModeCoordinator modeCoordinator;
        [SerializeField] private QuadSimApi quadSimApi;
        
        [Header("Camera")]
        [SerializeField] private Transform fpvCameraPoint;

        [Header("Visuals")]
        [SerializeField] private Transform[] propellerMeshes = new Transform[4];

        // ============================================================================
        // Runtime State
        // ============================================================================

        // Canonical motor order: [0]=FL, [1]=FR, [2]=BL, [3]=BR
        private readonly Transform[] _motors = new Transform[4];
        private readonly float[] _propellerRPMs = new float[4];

        // Config
        private DroneConfig _config;
        private bool _configApplied;

        // Rotor physics derived from config (matches UE's RotorPhysics member)
        private RotorPhysicsDerived _rotorPhysics;

        // Unique identifier
        public string DroneID { get; private set; }

        // Initialization tracking
        private bool _sensorsReady;
        private bool _controllerReady;
        private bool _initialized;
        public bool HasConfigApplied => _configApplied;

        // Active controller type (matches UE's currentControllerType)
        [Header("Controller Selection")]
        [SerializeField] private ControllerKind currentControllerKind = ControllerKind.Cascade;

        // Buffered wrench (matches UE's PhysicsState pattern)
        // In Unity we don't need atomics since PrePhysicsStep and Physics.Simulate
        // run on the same thread, but we keep the generation pattern for clarity.
        private Vector3 _bufferedForce;
        private Vector3 _bufferedTorque;
        private uint _wrenchGeneration;
        private uint _lastAppliedGeneration;

        // Manager reference (set explicitly by DroneManager, not searched)
        private DroneManager _droneManager;

        // ============================================================================
        // Public Accessors
        // ============================================================================

        public void SetDroneManager(DroneManager manager) => _droneManager = manager;

        public Rigidbody Rigidbody => rb;
        public Transform[] Motors => _motors;
        public Transform MotorFL => _motors[0];
        public Transform MotorFR => _motors[1];
        public Transform MotorBL => _motors[2];
        public Transform MotorBR => _motors[3];

        public CascadedController Controller => controller;
        public SensorManager Sensors => sensorManager;
        public FlightCommandProxy CommandProxy => commandProxy;
        public ModeCoordinator ModeCoordinator => modeCoordinator;
        public QuadSimApi Api => quadSimApi;
        public DroneConfig Config => _config;
        public RotorPhysicsDerived RotorPhysics => _rotorPhysics;
        public bool IsInitialized => _initialized;
        public ControllerKind CurrentControllerKind => currentControllerKind;

        // ============================================================================
        // Unity Lifecycle
        // ============================================================================

        private void Reset()
        {
            // Editor helper: auto-grab rigidbody when component is added
            rb = GetComponent<Rigidbody>();
        }

        private void Awake()
        {
            DroneID = gameObject.name;

            // Auto-bind subsystem references if not set in prefab
            if (rb == null) rb = GetComponent<Rigidbody>();
            if (controller == null) controller = GetComponent<CascadedController>();
            if (sensorManager == null) sensorManager = GetComponent<SensorManager>();
            if (commandProxy == null) commandProxy = GetComponent<FlightCommandProxy>();
            if (modeCoordinator == null) modeCoordinator = GetComponent<ModeCoordinator>();
            if (quadSimApi == null) quadSimApi = GetComponent<QuadSimApi>();
            
            // Build motor array from serialized references
            _motors[0] = motorFL;
            _motors[1] = motorFR;
            _motors[2] = motorBL;
            _motors[3] = motorBR;

            // Safety fallback: if motor refs weren’t assigned, try find by name
            if (_motors[0] == null) _motors[0] = transform.Find("MotorFL");
            if (_motors[1] == null) _motors[1] = transform.Find("MotorFR");
            if (_motors[2] == null) _motors[2] = transform.Find("MotorBL");
            if (_motors[3] == null) _motors[3] = transform.Find("MotorBR");

            if (controller == null) Debug.LogError($"[QuadPawn] {DroneID}: Missing CascadedController!");
            if (sensorManager == null) Debug.LogError($"[QuadPawn] {DroneID}: Missing SensorManager!");
            if (commandProxy == null) Debug.LogError($"[QuadPawn] {DroneID}: Missing FlightCommandProxy!");
        }


        private void Start()
        {
            // Register with DroneManager (injected by DroneManager.SpawnDroneWithConfig)
            if (_droneManager != null)
            {
                _droneManager.RegisterDrone(this);
                Debug.Log($"[QuadPawn] Registered with DroneManager: {DroneID}");
            }
            else
            {
                // Fallback for scene-placed drones — prefer explicit injection
                _droneManager = DroneManager.Get();
                if (_droneManager != null)
                {
                    _droneManager.RegisterDrone(this);
                    Debug.LogWarning($"[QuadPawn] {DroneID}: Registered via fallback Get(). Prefer SetDroneManager().");
                }
                else
                {
                    Debug.LogWarning($"[QuadPawn] {DroneID}: No DroneManager — drone will not be managed.");
                }
            }

            // If config was applied before Start(), initialize now
            if (_configApplied)
            {
                InitializeSubsystems();
            }
        }

        private void OnDestroy()
        {
            if (_droneManager != null)
            {
                _droneManager.UnregisterDrone(this);
            }
        }

        // ============================================================================
        // Configuration (called by DroneManager after spawning)
        // ============================================================================

        /// <summary>
        /// Apply a drone configuration. Mirrors UE's AQuadPawn::ApplyConfig.
        /// </summary>
        public void ApplyConfig(DroneConfig config)
        {
            _config = config;
            _configApplied = true;

            Debug.Log($"[QuadPawn] {DroneID}: Applying config...");

            ApplyPhysicalConfig();

            if (_config.DroneParams.Arm_Length > 0f)
            {
                _rotorPhysics = new RotorPhysicsDerived();
                Debug.Log($"[QuadPawn] {DroneID}: RotorPhysics computed (arm={_config.DroneParams.Arm_Length}m)");
            }
            InitializeSubsystems();


            ApplyControllerConfig();

            Debug.Log("==== Rotor Geometry Check ====");

            for (int i = 0; i < _motors.Length; i++)
            {
                if (_motors[i] == null) continue;

                Vector3 local = _motors[i].localPosition;
                Vector3 world = _motors[i].position;

                float armLocalXZ = new Vector2(local.x, local.z).magnitude;
                float armWorldXZ = new Vector2(
                    world.x - transform.position.x,
                    world.z - transform.position.z
                ).magnitude;

                Debug.Log(
                    $"Motor {i}: " +
                    $"local={local} " +
                    $"world={world} " +
                    $"armLocal={armLocalXZ:F3}m " +
                    $"armWorld={armWorldXZ:F3}m");
            }
        }

        private void ApplyPhysicalConfig()
        {
            if (rb == null) return;

            // Mass
            if (_config.DroneParams.Mass > 0.01f)
                rb.mass = _config.DroneParams.Mass;
            else
                rb.mass = 1.28f; // Default

            if (_config.DroneParams.CenterOfMass != Vector3.zero)
                rb.centerOfMass = _config.DroneParams.CenterOfMass;
            if (_config.DroneParams.Inertia != Vector3.zero)
                 rb.inertiaTensor = _config.DroneParams.Inertia;
            if (_config.DroneParams.LinearDrag_Coeff >= 0)
                rb.linearDamping = _config.DroneParams.LinearDrag_Coeff;
            if (_config.DroneParams.AngularDrag_Coeff >= 0)
                rb.angularDamping = _config.DroneParams.AngularDrag_Coeff;

            // Rigidbody setup for simulation
            rb.useGravity = true;
            rb.isKinematic = false;
           // rb.interpolation = RigidbodyInterpolation.None;
           // rb.collisionDetectionMode = CollisionDetectionMode.Discrete;
            rb.maxAngularVelocity = 100f;
            Debug.Log($"[QuadPawn] {DroneID}: Inertia tensor = {rb.inertiaTensor}");
            Debug.Log($"[QuadPawn] {DroneID}: Physical config applied (mass={rb.mass}kg)");
        }

        private void ApplyControllerConfig()
        {
            // Cascaded controller
            if (controller != null)
            {
                
                _rotorPhysics.ComputeRotor(_config.RotorParams.Model, _config.DroneParams.Arm_Length);
                controller.SetRotorPhysics(_rotorPhysics, _config);
                Debug.Log($"[QuadPawn] {DroneID}: CascadedController config applied");
            }

            // Future: geometric controller
            // if (geoController != null)
            //     geoController.ApplyConfig(_config, _rotorPhysics);
        }

        // ============================================================================
        // Initialization
        // ============================================================================

        private void InitializeSubsystems()
        {
            if (_initialized) return;

            Debug.Log($"[QuadPawn] {DroneID}: Initializing subsystems...");

            // 1. Sensors
            if (sensorManager != null)
            {
                sensorManager.OnStart();
                _sensorsReady = true;
                Debug.Log($"[QuadPawn] {DroneID}: SensorManager initialized");
            }

            // 2. Controller — only ready if sensors are
            if (controller != null && _sensorsReady)
            {
                // Controller handles its own internal init (allocator, gains, etc.)
                // QuadPawn just needs to know it exists and sensors are available.
                controller.Initialize(this, _config, sensorManager);
                _controllerReady = true;
                Debug.Log($"[QuadPawn] {DroneID}: Controller initialized");
            }
            
            if (modeCoordinator != null && commandProxy != null)
            {
                modeCoordinator.Initialize(this, commandProxy);
                Debug.Log($"[QuadPawn] {DroneID}: ModeCoordinator initialized");
            }
            
            if (quadSimApi != null)
            {
                quadSimApi.Initialize(this, commandProxy, modeCoordinator, sensorManager);
                Debug.Log($"[QuadPawn] {DroneID}: QuadSimApi initialized");
            }
            _initialized = true;
            Debug.Log($"[QuadPawn] {DroneID}: Initialization complete");
        }
        
        public void PhysicsStep(double dtSec, long nowNanos)
        {
            if (!_initialized) return;

            float dt = (float)dtSec;

            // --- Step 1: Update Sensors ---
            if (sensorManager != null && _sensorsReady)
            {
                sensorManager.UpdateSensors(dtSec, nowNanos);
            }

            // --- Step 2: Run Active Controller ---
            if (_controllerReady)
            {
                RunController(dt);
            }

            // --- Step 3: Apply Buffered Wrench to Rigidbody ---
            ApplyBufferedForces();
        }

        private void RunController(float dt)
        {
            // Use ModeCoordinator as source of truth if available, else fall back to local field
            ControllerKind kind = modeCoordinator != null 
                ? modeCoordinator.ActiveController 
                : currentControllerKind;

            switch (kind)
            {
                case ControllerKind.Cascade:
                    if (controller != null)
                        controller.StepController(dt);
                    break;

                case ControllerKind.Geometric:
                    // Future: geoController.Update(sensorManager.Latest, dt);
                    break;
            }
        }

        // ============================================================================
        // Buffered Force Application (mirrors UE's SetBodyWrench / ApplyBufferedForces)
        // ============================================================================

        /// <summary>
        /// Set the net wrench to apply at next physics step.
        /// Called by controllers after computing control output.
        /// Force is body-frame Z thrust, Torque is body-frame (X,Y,Z).
        /// </summary>
        public void SetBodyWrench(Vector3 netForce, Vector3 netTorque)
        {
            _bufferedForce = netForce;
            _bufferedTorque = netTorque;
            _wrenchGeneration++;

            // Wake body if significant force
            if (rb != null && (netForce.sqrMagnitude > 1f || netTorque.sqrMagnitude > 0.1f))
            {
                rb.WakeUp();
            }
        }

        /// <summary>
        /// Apply buffered wrench to rigidbody. Body-local to world transform.
        /// Called at end of PhysicsStep, before Physics.Simulate().
        /// </summary>
        private void ApplyBufferedForces()
        {
            if (rb == null) return;
            if (_wrenchGeneration == _lastAppliedGeneration) return;

            Vector3 bodyForce = new Vector3(0, _bufferedForce.y, 0);            
            
            // Transform body-local force/torque to world space
            Vector3 worldForce = transform.TransformDirection(_bufferedForce);
            Vector3 worldTorque = transform.TransformDirection(_bufferedTorque);

            rb.AddForce(worldForce, ForceMode.Force);
            rb.AddTorque(worldTorque, ForceMode.Force);

            _lastAppliedGeneration = _wrenchGeneration;
        }

        // ============================================================================
        // Reset Functions
        // ============================================================================

        /// <summary>
        /// Full reset — position, physics, controller state.
        /// </summary>
        public void ResetDrone()
        {
            ResetPhysics();
            ResetControllerState();
            Debug.Log($"[QuadPawn] {DroneID}: Full reset complete");
        }

        /// <summary>
        /// Reset to specific position and rotation.
        /// </summary>
        public void ResetPosition(Vector3? targetPosition = null, Quaternion? targetRotation = null)
        {
            Vector3 pos = targetPosition ?? Vector3.up;
            Quaternion rot = targetRotation ?? Quaternion.identity;

            if (rb == null)
                rb = GetComponent<Rigidbody>();

            if (rb != null)
            {
                // Rigidbody is authoritative for physics + sensors.
                rb.position = pos;
                rb.rotation = rot;
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;

                // Clear sleeping/contact weirdness after teleport.
                rb.Sleep();
                rb.WakeUp();
            }

            // Keep Transform synchronized too.
            transform.SetPositionAndRotation(pos, rot);
            Physics.SyncTransforms();

            // Clear buffered wrench/controller state.
            ResetPhysics();
            ResetControllerState();

            if (sensorManager != null)
                sensorManager.OnReset();

            Debug.Log($"[QuadPawn] {DroneID}: Position reset to {pos}, rb.position={(rb != null ? rb.position.ToString() : "NULL")}");
        }
        /// <summary>
        /// Level the drone (zero roll/pitch, keep yaw), zero velocities.
        /// </summary>
        public void ResetRotation()
        {
            if (rb == null) return;

            Vector3 euler = transform.eulerAngles;
            transform.rotation = Quaternion.Euler(0f, euler.y, 0f);

            rb.angularVelocity = Vector3.zero;
            rb.linearVelocity = Vector3.zero;
        }

        /// <summary>
        /// Zero all velocities without moving.
        /// </summary>
        public void ResetPhysics()
        {
            if (rb == null) return;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            // Clear buffered wrench
            _bufferedForce = Vector3.zero;
            _bufferedTorque = Vector3.zero;
            _wrenchGeneration = 0;
            _lastAppliedGeneration = 0;

            rb.WakeUp();
        }

        /// <summary>
        /// Reset all controller and command state.
        /// </summary>
        public void ResetControllerState()
        {
            // Opaque reset — controller handles its own internals (PIDs, allocator, etc.)
            if (controller != null)
            {
                controller.ResetControllerState();
            }

            // if (commandProxy != null)
            // {
            //     commandProxy.ResetToSafe();
            // }

            // Clear any pending wrench
            _bufferedForce = Vector3.zero;
            _bufferedTorque = Vector3.zero;
        }

        // ============================================================================
        // Config Reload (mirrors UE)
        // ============================================================================

        public void ReapplyConfig()
        {
            if (_configApplied)
                ApplyConfig(_config);
        }

        public void ReloadYaml()
        {
            if (_droneManager != null)
                _droneManager.ReloadCurrentDrone(false);
        }

        // ============================================================================
        // Visual Updates (called by DroneManager from Update)
        // ============================================================================

        public void UpdateVisuals(float deltaTime)
        {
            UpdatePropellerVisuals(deltaTime);
            // Future: debug draw, effects
        }

        private void UpdatePropellerVisuals(float deltaTime)
        {
            if (propellerMeshes == null || propellerMeshes.Length != 4) return;
            if (controller == null) return;

            var motorOutputs = controller.LastMotor01;
            if (motorOutputs == null || motorOutputs.Count != 4) return;

            // Thrust model params from config or defaults
            float thrustCoef = 1e-5f;
            float propDiameter = 0.127f;
            float airDensity = 1.225f;
            float maxThrustPerMotor = 4.12f;

            if (_configApplied)
            {
                if (_config.RotorParams.Model.ThrustCoeff > 0)
                    thrustCoef = _config.RotorParams.Model.ThrustCoeff;
                if (_config.RotorParams.Model.Prop_Diameter > 0)
                    propDiameter = _config.RotorParams.Model.Prop_Diameter;
                if (_config.FlightParams.MaxThrust > 0)
                    maxThrustPerMotor = _config.FlightParams.MaxThrust / 4f;
            }

            float denom = thrustCoef * airDensity * Mathf.Pow(propDiameter, 4);
            if (denom < 1e-10f) return;

            // Spin direction from config or default X-config
            int[] spinDir = { 1, -1, -1, 1 };
            if (_configApplied && _config.RotorParams.Rotors != null && _config.RotorParams.Rotors.Length == 4)
            {
                for (int i = 0; i < 4; i++)
                    spinDir[i] = _config.RotorParams.Rotors[i].Spin_Dir;
            }

            for (int i = 0; i < 4; i++)
            {
                if (propellerMeshes[i] == null) continue;

                float thrust01 = Mathf.Max(0f, motorOutputs[i]);
                float thrustN = thrust01 * maxThrustPerMotor;
                float rpm = 60f * Mathf.Sqrt(thrustN / denom);
                _propellerRPMs[i] = rpm;

                float direction = spinDir[i];
                float frameRotation = (rpm / 60f) * 360f * deltaTime * direction;
                propellerMeshes[i].Rotate(0f, 0f, frameRotation, Space.Self);            }
        }

        // ============================================================================
        // Camera (stubs — future CameraRig component)
        // ============================================================================

        public void ForceFPVCameraActive()
        {
            // TODO: CameraRig.SetMode(FPV)
            Debug.Log($"[QuadPawn] {DroneID}: ForceFPVCameraActive (stub)");
        }

        public void SwitchCamera()
        {
            // TODO: CameraRig.CycleMode()
            Debug.Log($"[QuadPawn] {DroneID}: SwitchCamera (stub)");
        }

        public Transform GetFPVCameraPoint()
        {
            return fpvCameraPoint != null ? fpvCameraPoint : transform;
        }

        // ============================================================================
        // Physics Helpers
        // ============================================================================

        /// <summary>
        /// Body inertia tensor in SI (kg·m²). Unity already uses SI.
        /// </summary>
        public Vector3 GetBodyInertiaSI()
        {
            return rb != null ? rb.inertiaTensor : Vector3.zero;
        }
    }
}