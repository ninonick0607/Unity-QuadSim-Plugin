using System;
using DroneCore.Common;
using DroneCore.Interfaces;
using RobotCore; // SensorManager, SensorData
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Controllers.AxisControllers
{
    public class VelocityController
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
        private float maxAcceleration;
        public VelocityController(DroneConfig inConfig)
        {
            _config = inConfig;
        }
        
        public void Initialize(int axis, ICommandSource cmd, SensorManager sensors)
        {
            _axis = axis;
            _cmdGoal = cmd;
            _sensorManager = sensors;
            VelocityPID PID = _config.Velocity;

            float maxAccelLimit = Mathf.Tan(_config.FlightParams.MaxAngle * Mathf.Deg2Rad) * 9.81f;
            if (axis == 3)
                maxAccelLimit = 9.81f;
    
            _pidSet.SetLimits(-maxAccelLimit, maxAccelLimit);
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
            float Goal = _bUseExternalGoal ? _externalGoal :  _cmdGoal.GetCommandValue()[_axis];

            if (_axis == 2)
            {
                _output = Goal;
                return;
            }

            if (_axis == 3)
            {
                float StateZ = StateData.ImuVel.z;
                _output = _pidSet.Calculate(Goal, StateZ, deltaTime);
                return;
            }
            float StateVel = StateData.ImuVel[_axis];
            _output = _pidSet.Calculate(Goal, StateVel, deltaTime);
            
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