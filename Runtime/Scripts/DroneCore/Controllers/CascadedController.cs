using System;
using System.Collections.Generic;
using MathUtil;
using DroneCore.Common;
using DroneCore.Controllers.AxisControllers;
using DroneCore.Controls;
using DroneCore.Interfaces;
using RobotCore;
using SimCore.Common;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Controllers
{
    /// <summary>
    /// Cascaded flight controller implementing full Position -> Velocity -> Angle -> Rate cascade.
    /// Mirrors UE's UQuadDroneController.
    /// 
    /// NOT an ISimulatable — QuadPawn drives this via Initialize(), Update(), ResetControllerState().
    /// </summary>
    [RequireComponent(typeof(QuadPawn))]
    [DisallowMultipleComponent]
    public sealed class CascadedController : MonoBehaviour
    {
        // ============================================================================
        // Enums & Structs
        // ============================================================================

        public enum EPIDStage
        { 
            Position,
            Velocity,
            Angle,
            Rate
        }

        [Serializable]
        public struct PIDKey : IEquatable<PIDKey>
        {
            public EPIDStage Stage;
            public byte Axis;

            public bool Equals(PIDKey other) => Stage == other.Stage && Axis == other.Axis;
            public override bool Equals(object obj) => obj is PIDKey other && Equals(other);
            public override int GetHashCode() => HashCode.Combine(Stage, Axis);
        }

        [Serializable]
        public class PIDInfo
        {
            public string Label;
            public QuadPIDController PID; // non-owning reference
        }

        [Serializable]
        public class CascadeAxis
        {
            public PositionController Position;
            public VelocityController Velocity;
            public AngleController Angle;
            public AcroController Rate;
        }

        [Serializable]
        public struct CascadeTelemetrySnapshot
        {
            public GoalMode Mode;
            public Axis4 ExternalCmd;
            public Vector3 DesiredVel;
            public Vector3 DesiredAccel;
            public Vector3 DesiredAnglesDeg;
            public Vector3 DesiredRatesDeg;
            public bool HasVelSetpoint;
            public bool HasAccelSetpoint;
            public bool HasAngleSetpoint;
            public bool HasRateSetpoint;
        }

        // ============================================================================
        // Runtime References (set via Initialize, not inspector)
        // ============================================================================

        private QuadPawn _body;
        private SensorManager _sensorManager;
        private ICommandSource _goalSource;

        // ============================================================================
        // Configuration & Physics
        // ============================================================================

        private RotorPhysicsDerived _rotorPhysics;
        private DroneConfig _cachedConfig;
        private bool _configApplied;

        // ============================================================================
        // Runtime State
        // ============================================================================

        private long _lastControllerUpdateTime;
        private SensorData _lastSensorData;
        private bool _hasValidSensorData;

        // Derived Controls
        private Vector2 _desiredAttitude = Vector2.zero;
        private readonly CascadeAxis[] _axes = new CascadeAxis[4];
        private ControlAllocator _controlAllocator = new ControlAllocator();

        // Limits & Constants (Cached for speed)
        private float _maxAngleRateDeg;
        private float _maxYawRateDeg;
        private float _hoverThrottle01;

        // Telemetry Data
        private CascadeTelemetrySnapshot _telemetry;
        private Dictionary<PIDKey, PIDInfo> _pidMap = new Dictionary<PIDKey, PIDInfo>();

        // Mode Transition
        private GoalMode _lastMode = GoalMode.None;
        private bool _lastModeValid;

        // Motor outputs
        private readonly float[] _motor = new float[4];
        private readonly float[] _thrusts = new float[4];
        private int _waDebugTick;
        private bool _initialized;

        // ── Plant / actuation (external-wrench allocated path) ──
        [Header("Plant / Actuation")]
        [SerializeField] private bool  enableMotorLag      = false;   // inspector toggle
        [SerializeField] private float motorLagTauSeconds  = 0.020f;
        [SerializeField] private float allocatedMinIdle01  = 0.0f;    // 0 lets motors reach 0 at loop apex
        private readonly MotorLagFilter _motorLag = new MotorLagFilter();
        
        // ============================================================================
        // Public Accessors
        // ============================================================================

        public IReadOnlyList<float> LastMotor01 => _motor;
        public IReadOnlyList<float> Thrusts => _thrusts;
        public CascadeTelemetrySnapshot TelemetrySnapshot => _telemetry;
        public IReadOnlyDictionary<PIDKey, PIDInfo> PIDMap => _pidMap;

        // ============================================================================
        // Initialization (called by QuadPawn)
        // ============================================================================

        public void Initialize(QuadPawn body, DroneConfig config, SensorManager sensors)
        {
            _body = body;
            _sensorManager = sensors;
            _goalSource = body.CommandProxy;
            //_thrusters = body.Thrusters;

            if (_body == null)
            {
                Debug.LogError($"[CascadedController] {name}: QuadPawn reference is null!");
                return;
            }

            // Prevent Unity angular velocity clamping
            if (_body.Rigidbody != null)
            {
                _body.Rigidbody.maxAngularVelocity = 100f;
            }

            // Initialize axis array
            for (int i = 0; i < 4; i++)
            {
                _axes[i] = new CascadeAxis();
            }

            ResetControllerState();

            _initialized = true;
            Debug.Log($"[CascadedController] {name}: Initialized");
        }

        /// <summary>
        /// Set rotor physics and rebuild allocator. Mirrors UE's SetRotorPhysics.
        /// Called by QuadPawn.ApplyControllerConfig after RotorPhysicsDerived is computed.
        /// </summary>
        
        private bool _axesBuilt;

        public void SetRotorPhysics(RotorPhysicsDerived rotorPhysics, DroneConfig config)
        {
            _rotorPhysics = rotorPhysics;
            _cachedConfig = config;
            _configApplied = true;

            _maxAngleRateDeg = config.FlightParams.MaxRateRollPitch;
            _maxYawRateDeg = config.FlightParams.MaxRateYaw;
            // Compute hover throttle from mass
            if (_body != null && _body.Rigidbody != null)
            {
                float currentMass = _body.Rigidbody.mass;
                _hoverThrottle01 = _rotorPhysics.ComputeHoverThrottle(currentMass, 4);

                // --- FIX 1: Math/Physics Alignment ---
                Vector3 centroid = Vector3.zero;
                var rotors = _cachedConfig.RotorParams.Rotors;
                
                for (int i = 0; i < rotors.Length; i++)
                    centroid += rotors[i].Location_Body;
                
                if (rotors.Length > 0) centroid /= rotors.Length;

                // Shift Unity's physics pivot
                _body.Rigidbody.centerOfMass = centroid;

                // Shift mathematical pivot. This satisfies the Allocator's 
                // decoupled pseudo-inverse requirement (Sum of positions = 0)
                for (int i = 0; i < rotors.Length; i++)
                {
                    rotors[i].Location_Body -= centroid;
                }
                
                float totalMaxThrust = _rotorPhysics.MaxThrust * 4.0f;
                float hoverForce = currentMass * 9.81f;
                float twr = (hoverForce > 0.0f) ? totalMaxThrust / hoverForce : 0.0f;

                Debug.Log($"[CascadedController] {name}:");
                Debug.Log($"  - Mass: {currentMass:F3} kg");
                Debug.Log($"  - Max Thrust/Motor: {_rotorPhysics.MaxThrust:F2} N");
                Debug.Log($"  - Hover Throttle: {_hoverThrottle01:F2}");
                Debug.Log($"  - TWR: {twr:F2}");
            }
            
            if (_axesBuilt) return;
            if (!_configApplied) return;
            if (_goalSource == null || _sensorManager == null) return;
            // Setup axes and rebuild allocator
            AxisSetup();
            BuildPIDTuningMap();
            _controlAllocator.RebuildEffectiveness(_cachedConfig.RotorParams.Rotors, _rotorPhysics, GetCurrentFrame());
            _axesBuilt = true;

            Debug.Log($"[CascadedController] {name}: RotorConfig applied");
        }
        

        // ============================================================================
        // Axis Setup
        // ============================================================================

        private void AxisSetup()
        {
            for (int axis = 0; axis < 4; axis++)
            {
                _axes[axis].Velocity = new VelocityController(_cachedConfig);
                _axes[axis].Rate = new AcroController(_cachedConfig);

                if (axis != 2) // No position controller for yaw
                    _axes[axis].Position = new PositionController(_cachedConfig);
                if (axis < 3) // Roll, Pitch, Yaw have angle controllers
                    _axes[axis].Angle = new AngleController(_cachedConfig);
            }

            // Initialize all controllers
            for (int axis = 0; axis < 4; axis++)
            {
                if (_axes[axis].Position != null)
                    _axes[axis].Position.Initialize(axis, _goalSource, _sensorManager);

                _axes[axis].Velocity.Initialize(axis, _goalSource, _sensorManager);
                _axes[axis].Rate.Initialize(axis, _goalSource, _sensorManager);

                if (_axes[axis].Angle != null)
                    _axes[axis].Angle.Initialize(axis, _goalSource, _sensorManager);
            }
        }

        // ============================================================================
        // Reset (called by QuadPawn)
        // ============================================================================

        public void ResetControllerState()
        {
            // Reset all cascade controllers
            for (int axis = 0; axis < 4; axis++)
            {
                if (_axes[axis].Position != null) _axes[axis].Position.Reset();
                if (_axes[axis].Velocity != null) _axes[axis].Velocity.Reset();
                if (_axes[axis].Angle != null) _axes[axis].Angle.Reset();
                if (_axes[axis].Rate != null) _axes[axis].Rate.Reset();
                if (_axes[axis].Angle != null) _axes[axis].Angle.ClearExternalGoal();
            }

            _desiredAttitude = Vector2.zero;

            for (int axis = 0; axis < 4; axis++)
            {
                if (_axes[axis].Velocity != null) _axes[axis].Velocity.ClearExternalGoal();
                if (_axes[axis].Rate != null) _axes[axis].Rate.ClearExternalGoal();
            }

            // Zero thrusts and motors
            for (int i = 0; i < 4; i++)
            {
                _thrusts[i] = 0f;
                _motor[i] = 0f;
                _motorLag.Reset(4);
            }

            // Preserve mode/source, just zero the command
            if (_body != null && _body.CommandProxy != null)
            {
                var keepMode = _body.CommandProxy.GetGoalMode();
                var owner = _body.CommandProxy.GetActiveSource();
                _body.CommandProxy.ForceSetCommand(new Axis4(), keepMode, owner);
            }

            Debug.Log($"[CascadedController] {name}: State reset");
        }

        // ============================================================================
        // Update (called by QuadPawn.RunController)
        // ============================================================================
        
        public void StepController(float deltaTime)
        {
            if (!_configApplied) return;

            float actualDeltaTime = Mathf.Clamp(deltaTime, 0.001f, 0.1f);

            if (_goalSource != null)
            {
                GoalMode newMode = _goalSource.GetGoalMode();
                if (!_lastModeValid)
                {
                    _lastMode = newMode;
                    _lastModeValid = true;
                }
                else if (newMode != _lastMode)
                {
                    OnGoalModeChanged(newMode, _lastMode);
                    _lastMode = newMode;
                }
            }

            _hasValidSensorData = true;
            FlightController(actualDeltaTime);
        }
        
        
        // ============================================================================
        // Flight Controller Core
        // ============================================================================

        private void FlightController(float deltaTime)
        {
            if (_body == null || _goalSource == null) return;

            // Single source of truth for Mode
            GoalMode mode = _goalSource.GetGoalMode();
            Axis4 cmd = _goalSource.GetCommandValue();

            InputSource source = (_body != null && _body.CommandProxy != null) ? _body.CommandProxy.GetActiveSource() : InputSource.UI;
            bool bGamepadAngle = (mode == GoalMode.Angle && source == InputSource.UI);

            TelemetryInit(mode, cmd);

            // --- WRENCH MODE ---
            // This is to bypass directly to the rigid body
            if (mode == GoalMode.WrenchBypassed)
            {
                Vector3 torque = new Vector3(cmd.X, cmd.Y, cmd.Z);
                Vector3 force = new Vector3(0, cmd.W, 0); 
                _body.SetBodyWrench(force, torque);
                return;
            }

            if (mode == GoalMode.Wrench)
            {
                float[] motor01 = new float[4];
                Vector3 mFlu = new Vector3(cmd.X, cmd.Y, cmd.Z);

                float sMax = _controlAllocator.AllocateWrench(
                    cmd.W,
                    mFlu.x,
                    mFlu.y,
                    mFlu.z,
                    _rotorPhysics.MaxThrust,
                    allocatedMinIdle01,
                    motor01
                );

                // Plant: optional first-order motor lag (live toggle), then actuate.
                _motorLag.Enabled    = enableMotorLag;
                _motorLag.TauSeconds = motorLagTauSeconds;
                _motorLag.Apply(motor01, deltaTime);

                QuadActuationModel.ComputeBodyWrench(
                    motor01,
                    _cachedConfig.RotorParams.Rotors,
                    _rotorPhysics,
                    out Vector3 netForce,
                    out Vector3 netTorque
                );
                
                _body.SetBodyWrench(netForce, netTorque);

                for (int i = 0; i < 4; i++)
                {
                    _motor[i]   = motor01[i];
                    _thrusts[i] = motor01[i] * _rotorPhysics.MaxThrust;
                }

                return;
            }
            
            // --- NONE MODE ---
            if (mode == GoalMode.None)
            {
                _body.SetBodyWrench(Vector3.zero, Vector3.zero);
                for (int i = 0; i < 4; i++) _thrusts[i] = 0.0f;
                return;
            }

            Axis4 output = new Axis4();

            // 1. Clear external goals
            for (int axis = 0; axis < 4; axis++)
            {
                if (_axes[axis].Velocity != null) _axes[axis].Velocity.ClearExternalGoal();
                if (_axes[axis].Angle != null) _axes[axis].Angle.ClearExternalGoal();
                if (_axes[axis].Rate != null) _axes[axis].Rate.ClearExternalGoal();
            }

            // 2. POSITION STAGE
            if (mode == GoalMode.Position)
            {
                // Position PID outputs are velocity setpoints in the output frame.
                // Output frame convention:
                //   .x = forward, .y = lateral, .z = altitude
                // This holds for FLU (y=left, z=up) and FRD (y=right, z=down).
                Vector3 vSpOutputFrame = Vector3.zero;
 
                if (_axes[0].Position != null)
                {
                    _axes[0].Position.Update(deltaTime);
                    vSpOutputFrame.x = _axes[0].Position.GetOutput();  // forward
                }
                if (_axes[1].Position != null)
                {
                    _axes[1].Position.Update(deltaTime);
                    vSpOutputFrame.y = _axes[1].Position.GetOutput();  // lateral
                }
                if (_axes[3].Position != null)
                {
                    _axes[3].Position.Update(deltaTime);
                    vSpOutputFrame.z = _axes[3].Position.GetOutput();  // altitude
                }
 
                // Rotate horizontal components into heading frame
                Quaternion bodyRot = _body.transform.rotation;
                Vector3 vSpHeading = OutputFrameVelToHeadingVel(vSpOutputFrame, bodyRot);
 
                // Constrain velocity: XY = horizontal, Z = vertical (in output frame)
                // TODO: DEFINE THESE IN CONFIG
                Vector3 vSpConstrained = ConstrainVelSetpoint_HeadingFrame(vSpHeading, 4.0f, 10f, 8f);
 
                // The velocity setpoint is already in the output frame —
                // no Frames.TransformLinear needed.
                if (_axes[0].Velocity != null) _axes[0].Velocity.SetExternalGoal(vSpConstrained.x);
                if (_axes[1].Velocity != null) _axes[1].Velocity.SetExternalGoal(vSpConstrained.y);
                if (_axes[3].Velocity != null) _axes[3].Velocity.SetExternalGoal(vSpConstrained.z);
 
                _telemetry.DesiredVel = vSpConstrained;
                _telemetry.HasVelSetpoint = true;
            }
 

            // 3. ACCELERATION STAGE
            // Acceleration mode bypasses Position and Velocity PID. The external
            // command is a desired acceleration setpoint in the output/body frame:
            //   X = ax [m/s^2], Y = ay [m/s^2], Z = yaw-rate [deg/s], W = az [m/s^2].
            Vector3 accelCmd = Vector3.zero;
            if (mode == GoalMode.Acceleration)
            {
                accelCmd = new Vector3(cmd.X, cmd.Y, cmd.W);
                _telemetry.DesiredAccel = accelCmd;
                _telemetry.HasAccelSetpoint = true;
            }

            // 4. VELOCITY STAGE
            if (mode == GoalMode.Position || mode == GoalMode.Velocity || mode == GoalMode.Angle)
            {
                if (mode == GoalMode.Velocity)
                {
                    _telemetry.DesiredVel = new Vector3(cmd[FlightAxes.X], cmd[FlightAxes.Y], cmd[FlightAxes.Z]);
                    _telemetry.HasVelSetpoint = true;
                }

                for (int axis = 0; axis < 4; axis++)
                {
                    // Angle mode: only Z velocity matters
                    if (mode == GoalMode.Angle && axis != 3)
                        continue;

                    // Gamepad Angle mode handles throttle manually
                    if (bGamepadAngle && axis == 3)
                        continue;

                    if (_axes[axis].Velocity != null)
                        _axes[axis].Velocity.Update(deltaTime);
                }
            }

            // 5. ANGLE STAGE
            if (mode != GoalMode.Rate)
            {
                if (mode == GoalMode.Angle)
                {
                    _telemetry.DesiredAnglesDeg = new Vector3(cmd.X, cmd.Y, cmd.Z);
                    _telemetry.HasAngleSetpoint = true;

                    if (bGamepadAngle)
                    {
                        _telemetry.DesiredRatesDeg.z = cmd.Z; // Yaw is rate-controlled on Gamepad
                        _telemetry.HasRateSetpoint = true;
                    }
                }

                // Roll/pitch source
                if (mode == GoalMode.Position || mode == GoalMode.Velocity)
                {
                    float velocityOutputX = _axes[0].Velocity.GetOutput();
                    float velocityOutputY = _axes[1].Velocity.GetOutput();
                    Vector2 attCmd = AccelerationToAttitude(velocityOutputX, velocityOutputY);

                    _axes[1].Angle.SetExternalGoal(attCmd.x);
                    _axes[0].Angle.SetExternalGoal(attCmd.y);

                    _telemetry.DesiredAnglesDeg.x = attCmd.x;
                    _telemetry.DesiredAnglesDeg.y = attCmd.y;
                    _telemetry.HasAngleSetpoint = true;
                }
                else if (mode == GoalMode.Acceleration)
                {
                    Vector2 attCmd = AccelerationToAttitude(accelCmd.x, accelCmd.y);

                    _axes[1].Angle.SetExternalGoal(attCmd.x);
                    _axes[0].Angle.SetExternalGoal(attCmd.y);

                    _telemetry.DesiredAnglesDeg.x = attCmd.x;
                    _telemetry.DesiredAnglesDeg.y = attCmd.y;
                    _telemetry.HasAngleSetpoint = true;
                }

                for (int axis = 0; axis < 3; axis++)
                {
                    // Acceleration mode commands yaw-rate directly, so do not let
                    // the yaw-angle PID integrate against a meaningless yaw angle.
                    if (mode == GoalMode.Acceleration && axis == 2)
                        continue;

                    if (_axes[axis].Angle != null)
                        _axes[axis].Angle.Update(deltaTime);
                }

                // Wire Angle -> Rate
                for (int axis = 0; axis < 3; axis++)
                {
                    float angleOutput = _axes[axis].Angle != null ? _axes[axis].Angle.GetOutput() : 0f;

                    // In gamepad Angle mode: yaw should be a RATE setpoint from Cmd.Z (deg/s)
                    if ((bGamepadAngle || mode == GoalMode.Acceleration) && axis == 2)
                    {
                        angleOutput = cmd.Z;
                    }

                    if (_axes[axis].Rate != null)
                        _axes[axis].Rate.SetExternalGoal(angleOutput);

                    if (axis == 0)
                    {
                        _telemetry.DesiredRatesDeg.x = angleOutput;
                        _telemetry.HasRateSetpoint = true;
                    }
                    if (axis == 1)
                    {
                        _telemetry.DesiredRatesDeg.y = angleOutput;
                        _telemetry.HasRateSetpoint = true;
                    }
                    if (axis == 2)
                    {
                        _telemetry.DesiredRatesDeg.z = angleOutput;
                        _telemetry.HasRateSetpoint = true;
                    }
                }
            }

            // 6. RATE STAGE
            if (mode == GoalMode.Rate)
            {
                _telemetry.DesiredRatesDeg = new Vector3(cmd.X, cmd.Y, cmd.Z);
                _telemetry.HasRateSetpoint = true;
            }

            for (int axis = 0; axis < 3; axis++)
            {
                if (_axes[axis].Rate != null)
                {
                    _axes[axis].Rate.Update(deltaTime);
                    output[axis] = _axes[axis].Rate.GetOutput();
                }
            }

            // 7. THROTTLE
            const float MinIdleThrottle = 0.04f;
            if (mode == GoalMode.Rate || bGamepadAngle)
            {
                output[3] = Mathf.Max(cmd.W, MinIdleThrottle);
            }
            else
            {
                float zAccel = (mode == GoalMode.Acceleration)
                    ? accelCmd.z
                    : _axes[3].Velocity.GetOutput();
                output[3] = _hoverThrottle01 * (1.0f + zAccel / 9.81f);
            }

            if (mode == GoalMode.Passthrough)
            {
                // Direct motor passthrough — bypass cascade and allocator entirely.
                // cmd packing: X=FL, Y=FR, Z=BL, W=BR (matches Motor4 order)
                float[] motor01 = new float[4];
                motor01[0] = Mathf.Clamp01(cmd.X);  // FL
                motor01[1] = Mathf.Clamp01(cmd.Y);  // FR
                motor01[2] = Mathf.Clamp01(cmd.Z);  // BL
                motor01[3] = Mathf.Clamp01(cmd.W);  // BR
                
                _motorLag.Enabled    = enableMotorLag;
                _motorLag.TauSeconds = motorLagTauSeconds;
                _motorLag.Apply(motor01, deltaTime);
                
                // Compute body wrench from motor commands through the actuation model
                Vector3 netForce, netTorque;
                QuadActuationModel.ComputeBodyWrench(
                    motor01,
                    _cachedConfig.RotorParams.Rotors,
                    _rotorPhysics,
                    out netForce,
                    out netTorque
                );
                _body.SetBodyWrench(netForce, netTorque);

                // Store for telemetry / UI
                for (int i = 0; i < 4; i++)
                {
                    _motor[i] = motor01[i];
                    _thrusts[i] = motor01[i] * _rotorPhysics.MaxThrust;
                }
                return;
            }
    
            
            ThrustMixer(output, deltaTime);
        }

        // ============================================================================
        // Thrust Mixer
        // ============================================================================

        private void ThrustMixer(Axis4 output, float deltaTime)
        {
            float rollOut = output[0];
            float pitchOut = output[1];
            float yawOut = output[2];
            float thrustOut = output[3];
            float yawAuthorityScale = 1.0f; // Tune this

            // Normalize rate PID outputs to torque demands (-1 to 1)
            float rollTorque = Mathf.Clamp(rollOut / _maxAngleRateDeg, -1f, 1f);
            float pitchTorque = Mathf.Clamp(pitchOut / _maxAngleRateDeg, -1f, 1f);
            float yawTorque = Mathf.Clamp(yawOut / _maxYawRateDeg, -1f, 1f) * yawAuthorityScale;
            // Calculate throttle
            float throttle01 = Mathf.Clamp(thrustOut, 0f, 1f);
            
            // Tilt compensation
            float tiltRad = Mathf.Deg2Rad * Mathf.Sqrt(
                _desiredAttitude.x * _desiredAttitude.x +
                _desiredAttitude.y * _desiredAttitude.y
            );
            float tiltComp = 1f / Mathf.Max(0.5f, Mathf.Cos(tiltRad));
            throttle01 *= tiltComp;

            const float MinIdleThrottle = 0.04f;
            throttle01 = Mathf.Clamp(throttle01, MinIdleThrottle, 1f);

            // Use effectiveness-based allocation
            float[] motor01 = new float[4];
            //float yawScale = 1.0f;

            _controlAllocator.AllocateYawLimited(rollTorque,pitchTorque,yawTorque,throttle01,MinIdleThrottle,motor01);

            // Fallback if allocator fails
            if (motor01 == null || motor01.Length != 4)
            {
                motor01 = new float[4];
                for (int i = 0; i < 4; i++) motor01[i] = throttle01;
            }
            
            _motorLag.Enabled    = enableMotorLag;
            _motorLag.TauSeconds = motorLagTauSeconds;
            _motorLag.Apply(motor01, deltaTime);
            
            // Compute body wrench
            Vector3 netForce, netTorque;
            QuadActuationModel.ComputeBodyWrench(motor01,_cachedConfig.RotorParams.Rotors,_rotorPhysics,out netForce,out netTorque);
            _body.SetBodyWrench(netForce, netTorque);

            // Store for UI/debugging
            for (int i = 0; i < 4; i++)
            {
                _motor[i] = motor01[i];
                _thrusts[i] = motor01[i] * _rotorPhysics.MaxThrust;
            }
        }

        // ============================================================================
        // Utilities
        // ============================================================================

        private Vector2 AccelerationToAttitude(float accelX, float accelY)
        {
            const float g = 9.81f;

            Vector3 bodyZ = new Vector3(accelX, accelY, g);
            bodyZ.Normalize();

            float pitchRad = Mathf.Atan2(bodyZ.x, bodyZ.z);
            float rollRad  = Mathf.Atan2(bodyZ.y, bodyZ.z);
            
            return new Vector2(Mathf.Rad2Deg * pitchRad, Mathf.Rad2Deg * -rollRad);
        }
        private SimFrame GetCurrentFrame()
        {
            if (_body != null && _sensorManager != null)
            {
                return _sensorManager.TargetFrame;
            }
            return SimFrame.FLU; // Default fallback
        }

        private Vector3 OutputFrameVelToHeadingVel(Vector3 vOutputFrame, Quaternion bodyRot)
        {
            // Extract yaw angle from Unity's rotation (always around Unity Y axis)
            float yawDeg = bodyRot.eulerAngles.y;
            float yawRad = yawDeg * Mathf.Deg2Rad;
 
            // In the output frame (FLU/FRD), horizontal plane is XY.
            // Rotate the horizontal components by -yaw to get heading-aligned frame.
            float cosY = Mathf.Cos(yawRad);
            float sinY = Mathf.Sin(yawRad);
 
            // 2D rotation of (x, y) by -yaw:
            //   x_heading =  cos(yaw) * x + sin(yaw) * y
            //   y_heading = -sin(yaw) * x + cos(yaw) * y
            float hx =  cosY * vOutputFrame.x + sinY * vOutputFrame.y;
            float hy = -sinY * vOutputFrame.x + cosY * vOutputFrame.y;
 
            // Z (altitude) passes through unchanged
            return new Vector3(hx, hy, vOutputFrame.z);
        }

        private Vector3 ConstrainVelSetpoint_HeadingFrame(
            Vector3 vSp,
            float vXyMax,
            float vUpMax,
            float vDownMax)
        {
            Vector3 output = vSp;

            // XY magnitude clamp
            Vector2 vxy = new Vector2(output.x, output.y);
            float n = vxy.magnitude;
            if (n > vXyMax && n > 1e-6f)
            {
                Vector2 clamped = vxy * (vXyMax / n);
                output.x = clamped.x;
                output.y = clamped.y;
            }

            // Z clamp
            output.z = Mathf.Clamp(output.z, -vUpMax, vDownMax);

            return output;
        }

        // ============================================================================
        // PID Tuning Interface
        // ============================================================================

        public bool GetPIDGains(EPIDStage stage, byte axis, out float p, out float i, out float d)
        {
            p = i = d = 0f;

            PIDKey key = new PIDKey { Stage = stage, Axis = axis };
            if (!_pidMap.TryGetValue(key, out PIDInfo info) || info.PID == null)
                return false;

            p = info.PID.Kp;
            i = info.PID.Ki;
            d = info.PID.Kd;
            return true;
        }

        public bool SetPIDGainTerm(EPIDStage stage, byte axis, int term, float value, bool resetIntegral)
        {
            PIDKey key = new PIDKey { Stage = stage, Axis = axis };
            if (!_pidMap.TryGetValue(key, out PIDInfo info) || info.PID == null)
                return false;

            float p = info.PID.Kp;
            float i = info.PID.Ki;
            float d = info.PID.Kd;

            if (term == 0) p = value;
            else if (term == 1) i = value;
            else if (term == 2) d = value;
            else return false;

            info.PID.SetGains(p, i, d);

            if (resetIntegral)
                info.PID.Reset();

            return true;
        }

        public void GetPIDTuningList(out List<PIDKey> outKeys, out List<string> outLabels)
        {
            outKeys = new List<PIDKey>();
            outLabels = new List<string>();

            foreach (var pair in _pidMap)
            {
                outKeys.Add(pair.Key);
                outLabels.Add(pair.Value.Label);
            }
        }

        private void BuildPIDTuningMap()
        {
            _pidMap.Clear();

            void Add(EPIDStage stage, byte axis, string label, QuadPIDController pid)
            {
                if (pid == null) return;
                _pidMap[new PIDKey { Stage = stage, Axis = axis }] = new PIDInfo { Label = label, PID = pid };
            }

            // POSITION
            if (_axes[0].Position != null) Add(EPIDStage.Position, 0, "Position X", _axes[0].Position.GetPID());
            if (_axes[1].Position != null) Add(EPIDStage.Position, 1, "Position Y", _axes[1].Position.GetPID());
            if (_axes[3].Position != null) Add(EPIDStage.Position, 3, "Position Z", _axes[3].Position.GetPID());

            // VELOCITY
            if (_axes[0].Velocity != null) Add(EPIDStage.Velocity, 0, "Velocity X", _axes[0].Velocity.GetPID());
            if (_axes[1].Velocity != null) Add(EPIDStage.Velocity, 1, "Velocity Y", _axes[1].Velocity.GetPID());
            if (_axes[3].Velocity != null) Add(EPIDStage.Velocity, 3, "Velocity Z", _axes[3].Velocity.GetPID());

            // ANGLE
            if (_axes[0].Angle != null) Add(EPIDStage.Angle, 0, "Angle Roll", _axes[0].Angle.GetPID());
            if (_axes[1].Angle != null) Add(EPIDStage.Angle, 1, "Angle Pitch", _axes[1].Angle.GetPID());
            if (_axes[2].Angle != null) Add(EPIDStage.Angle, 2, "Angle Yaw", _axes[2].Angle.GetPID());

            // RATE
            if (_axes[0].Rate != null) Add(EPIDStage.Rate, 0, "Rate Roll", _axes[0].Rate.GetPID());
            if (_axes[1].Rate != null) Add(EPIDStage.Rate, 1, "Rate Pitch", _axes[1].Rate.GetPID());
            if (_axes[2].Rate != null) Add(EPIDStage.Rate, 2, "Rate Yaw", _axes[2].Rate.GetPID());
        }

        public bool IsPIDStageActive(EPIDStage stage, byte axis)
        {
            GoalMode mode = _goalSource != null ? _goalSource.GetGoalMode() : GoalMode.None;

            switch (stage)
            {
                case EPIDStage.Position:
                    return mode == GoalMode.Position;

                case EPIDStage.Velocity:
                    if (mode == GoalMode.Angle)
                        return axis == 3; // Z velocity still runs for throttle
                    return mode == GoalMode.Position || mode == GoalMode.Velocity;

                case EPIDStage.Angle:
                    return mode != GoalMode.Rate;

                case EPIDStage.Rate:
                    return true;

                default:
                    return false;
            }
        }

        // ============================================================================
        // Telemetry
        // ============================================================================

        private void TelemetryInit(GoalMode mode, Axis4 cmd)
        {
            _telemetry.Mode = mode;
            _telemetry.ExternalCmd = cmd;
            _telemetry.DesiredVel = Vector3.zero;
            _telemetry.DesiredAccel = Vector3.zero;
            _telemetry.DesiredAnglesDeg = Vector3.zero;
            _telemetry.DesiredRatesDeg = Vector3.zero;
            _telemetry.HasVelSetpoint = false;
            _telemetry.HasAccelSetpoint = false;
            _telemetry.HasAngleSetpoint = false;
            _telemetry.HasRateSetpoint = false;
        }

        // ============================================================================
        // Mode Transition
        // ============================================================================

        private void OnGoalModeChanged(GoalMode newMode, GoalMode oldMode)
        {
            for (int axis = 0; axis < 4; axis++)
            {
                if (_axes[axis].Position != null) _axes[axis].Position.Reset();
                if (_axes[axis].Velocity != null) _axes[axis].Velocity.Reset();
                if (_axes[axis].Angle != null) _axes[axis].Angle.Reset();
                if (_axes[axis].Rate != null) _axes[axis].Rate.Reset();
                if (_axes[axis].Angle != null) _axes[axis].Angle.ClearExternalGoal();
            }

            for (int axis = 0; axis < 4; axis++)
            {
                if (_axes[axis].Velocity != null) _axes[axis].Velocity.ClearExternalGoal();
                if (_axes[axis].Rate != null) _axes[axis].Rate.ClearExternalGoal();
            }

            Debug.Log($"[CascadedController] GoalMode changed {oldMode} -> {newMode}");
        }

        // ============================================================================
        // Public Accessors
        // ============================================================================

        public float GetCurrentThrustOutput(int thrusterIndex)
        {
            if (thrusterIndex >= 0 && thrusterIndex < _thrusts.Length)
                return _thrusts[thrusterIndex];
            return 0.0f;
        }

        public bool IsConfigApplied() => _configApplied;
    }
}