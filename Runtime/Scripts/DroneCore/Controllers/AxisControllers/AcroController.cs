using DroneCore.Common;
using DroneCore.Interfaces;
using RobotCore;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Controllers.AxisControllers
{
    [System.Serializable]
    public sealed class AcroController
    {
        [SerializeField] private QuadPIDController _pidSet = new QuadPIDController();
        public QuadPIDController Pid => _pidSet;

        // Axis index: 0=roll, 1=pitch, 2=yaw, 3=throttle (pass-through)
        public int _axis { get; private set; }
        private ICommandSource _cmdGoal;
        private float _externalGoal;
        private SensorManager _sensorManager;
        private DroneConfig _config;
        
        private bool _bInitialized;
        private bool _bUseExternalGoal;
        
        // TODO: Setting but private? We dont need outside variables setting it at all
        private float _output;
        
        public AcroController(DroneConfig inConfig)
        {
            _config = inConfig;
        }
        
        public void Initialize(int axis, ICommandSource cmd, SensorManager sensors)
        {
            _axis = axis;
            _cmdGoal = cmd;
            _sensorManager = sensors;
            AcroPID PID = _config.Acro;
            if (axis != 2 && axis != 3)
                _pidSet.SetLimits(-_config.FlightParams.MaxRateRollPitch, _config.FlightParams.MaxRateRollPitch);
            else if (axis == 2)
                _pidSet.SetLimits(-_config.FlightParams.MaxRateYaw, _config.FlightParams.MaxRateYaw);
            
            _pidSet.SetGains(PID.GetPGains()[_axis], PID.GetIGains()[_axis], PID.GetDGains()[_axis]);

            _bInitialized = true;
            Debug.Log("[AcroController] Initialized");

            // if (iLimit > 0f) pid.SetIntegralLimits(iLimit);
        }

        public void Update(float deltaTime)
        {
            if (!_bInitialized || _sensorManager == null || _pidSet == null || (_cmdGoal == null && !_bUseExternalGoal))
            {
                _output = 0.0f;
                return;
            }

            float StateRate = _sensorManager.Latest.ImuAngVel[_axis];
            float Goal = _bUseExternalGoal ? _externalGoal : _cmdGoal.GetCommandValue()[_axis];
            float StateRateDegS = Mathf.Rad2Deg * StateRate;
            
            if (_axis == 3)
            {
                _output = Goal;
                return;
            }

            _output = _pidSet.Calculate(Goal, StateRateDegS, deltaTime);
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
