using DroneCore.Common;
using DroneCore.Interfaces;
using RobotCore; // SensorManager, SensorData
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Controllers.AxisControllers
{
    [System.Serializable]
    public sealed class AngleController // per-axis, UE-style
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
        private bool _bUseExternalGoal;

        public AngleController(DroneConfig inConfig)
        {
            _config = inConfig;
        }

        public void Initialize(int axis, ICommandSource cmd, SensorManager sensors)
        {
            _axis = axis;
            _cmdGoal = cmd;
            _sensorManager = sensors;
            AnglePID PID = _config.Angle;

            // Angle PID output is a rate setpoint (deg/s)
            if (axis == 2)
                _pidSet.SetLimits(-_config.FlightParams.MaxRateYaw, _config.FlightParams.MaxRateYaw);
            else
                _pidSet.SetLimits(-_config.FlightParams.MaxRateRollPitch, _config.FlightParams.MaxRateRollPitch);

            _pidSet.SetGains(PID.GetPGains()[_axis], PID.GetIGains()[_axis], PID.GetDGains()[_axis]);

            _bInitialized = true;
        }
        
        public void Update(float deltaTime)
        {
            if (!_bInitialized || _sensorManager == null || _pidSet == null || (_cmdGoal == null && !_bUseExternalGoal))
            {
                _output = 0.0f;
                return;
            }

            SensorData StateData = _sensorManager.Latest;
            float StateAngle = StateData.ImuAttitude[_axis];
            float Goal = _bUseExternalGoal ? _externalGoal :  _cmdGoal.GetCommandValue()[_axis];
            
            if (_axis == 2 || _axis == 3)
            {
                _output = Goal;
                return;
            }

            _output = _pidSet.Calculate(Goal, StateAngle, deltaTime);
        }
        public void Reset()
        {
            if(_pidSet!=null) _pidSet.Reset();
            _output = 0.0f;
            _bUseExternalGoal = false;
        }
        public void SetExternalGoal(float Value)
        {
            _externalGoal = Value;
            _bUseExternalGoal = true;
        }
        public void ClearExternalGoal()
        {
            // TODO: Double check using this is actually needed or if its updated during runtime
            _externalGoal = 0.0f;
            _bUseExternalGoal = false;
        }
        public float GetOutput() { return _output; }
        public float GetExternalGoal() { return _externalGoal; }
        public QuadPIDController GetPID()
        {
            return _pidSet;
        }
        
    }
}
