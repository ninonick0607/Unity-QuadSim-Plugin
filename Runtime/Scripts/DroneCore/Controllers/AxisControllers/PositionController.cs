using DroneCore.Common;
using DroneCore.Interfaces;
using RobotCore;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Controllers.AxisControllers
{
    public class PositionController
    {
        [SerializeField] private QuadPIDController _pidSet = new QuadPIDController();
        public QuadPIDController Pid => _pidSet;

        private int _axis;
        private ICommandSource _cmdGoal;
        private SensorManager _sensorManager;
        private DroneConfig _config;
        private float _output;
        private float _externalGoal;
        private bool _bInitialized;

        public PositionController(DroneConfig inConfig)
        {
            _config = inConfig;
        }
        
        public void Initialize(int axis, ICommandSource cmd, SensorManager sensors)
        {
            _axis = axis;
            _cmdGoal = cmd;
            _sensorManager = sensors;
            PositionPID PID = _config.Position;
    
            if (axis == 3)
                _pidSet.SetLimits(-_config.FlightParams.MaxVelZ, _config.FlightParams.MaxVelZ);
            else
                _pidSet.SetLimits(-_config.FlightParams.MaxVelXY, _config.FlightParams.MaxVelXY);
    
            _pidSet.SetGains(PID.GetPGains()[_axis], PID.GetIGains()[_axis], PID.GetDGains()[_axis]);
            _bInitialized = true;
        }

        public void Update(float deltaTime)
        {
            if (!_bInitialized || _cmdGoal == null || _sensorManager == null || _pidSet == null)
            {
                _output = 0f;
                return;
            }

            SensorData StateData = _sensorManager.Latest;
            float Goal = _cmdGoal.GetCommandValue()[_axis];

            // Axis 2 (yaw) passes through — no position PID for yaw
            if (_axis == 2)
            {
                _output = Goal;
                return;
            }
            
            // GPS position is in the output frame (e.g. FLU).
            // Cascade axis mapping:
            //   Axis 0 (forward)  → output frame .x
            //   Axis 1 (lateral)  → output frame .y
            //   Axis 3 (altitude) → output frame .z
            //
            // This works for FLU (x=fwd, y=left, z=up) and FRD (x=fwd, y=right, z=down).
            // The PID sign is correct in both cases because the command frame matches
            // the measurement frame — both are in the output frame.
            float State;
            switch (_axis)
            {
                case 0: State = StateData.GpsPosition.x; break;  // forward
                case 1: State = StateData.GpsPosition.y; break;  // lateral
                case 3: State = StateData.GpsPosition.z; break;  // altitude
                default: State = 0f; break;
            }

            _output = _pidSet.Calculate(Goal, State, deltaTime);
        }
   
        public void Reset()
        {
            if (_pidSet != null) _pidSet.Reset();
            _output = 0.0f;
        }

        public void SetExternalGoal(float Value)
        {
            _externalGoal = Value;
        }

        public void ClearExternalGoal()
        {
            _externalGoal = 0.0f; 
        }

        public float GetOutput() { return _output; }
        public float GetExternalGoal() { return _externalGoal; }

        public QuadPIDController GetPID()
        {
            return _pidSet;
        }
    }
}