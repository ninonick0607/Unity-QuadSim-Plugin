using SimCore;
using MathUtil;
using UnityEngine;
using RobotCore.Core_Utils;

namespace RobotCore.Sensors
{
    public sealed class BaroSensor
    {
        private float UpdateRate = 50.0f; // documented intent; cadence is owned by SensorManager's limiter

        private float BaroDriftPa = 0.0f;
        private float PressureNoiseStdDev = 0.5f;
        private float DriftPaPerSec = 0.005f;

        private readonly GaussianRng _rng;

        private float PressureOffset = 0.0f;
        private float TemperatureOffset = 0.0f;

        private const float ISA_LAPSE_RATE = 0.0065f;
        private const float ISA_TEMPERATURE_MSL = 288.15f;
        private const float ISA_PRESSURE_MSL = 101325.0f;
        private const float ISA_EXPONENT = 5.256f; // g*M/(R*L)

        private Rigidbody _rb;
        private Transform _bodyTransform;
        private GeoReferencingSystem _geo;

        private double _lastSampleSec;
        private bool _hasPrevSample;

        public float LastPressure { get; private set; }
        public float LastPressureHPa => LastPressure / 100.0f; // HIL_SENSOR.abs_pressure units
        public float LastTemperature { get; private set; } // degC
        public float EstimatedAltitude { get; private set; } // true geometric MSL altitude (input)

        // Altitude derived back from the noisy pressure — this is PX4 HIL_SENSOR.pressure_alt,
        // and the "altitude the baro believes" once noise/drift/offset are applied.
        public float PressureAltitude
        {
            get
            {
                float ratio = Mathf.Pow(LastPressure / ISA_PRESSURE_MSL, 1.0f / ISA_EXPONENT);
                return (ISA_TEMPERATURE_MSL / ISA_LAPSE_RATE) * (1.0f - ratio);
            }
        }

        public double LastTimestampSec { get; private set; }
        public bool IsValid { get; private set; }

        public BaroSensor(Rigidbody rb, Transform bodyTransform, int noiseSeed = 3)
        {
            _rb = rb;
            _bodyTransform = bodyTransform;
            _rng = new GaussianRng(noiseSeed);
        }

        public void Reset()
        {
            BaroDriftPa = 0.0f;
            _hasPrevSample = false;
            _lastSampleSec = 0.0;
            LastPressure = ISA_PRESSURE_MSL;
            LastTemperature = ISA_TEMPERATURE_MSL - 273.15f;
            EstimatedAltitude = 0.0f;
            LastTimestampSec = 0.0;
            IsValid = false;
        }

        public void Initialize(GeoReferencingSystem geo)
        {
            _geo = geo;
            IsValid = (_geo != null && _rb != null);
        }

        // Called by SensorManager at baro rate. dt is computed internally so drift stays
        // correct regardless of how the manager schedules us.
        public void Sample(double nowSec, bool addNoise)
        {
            if (_geo == null || _rb == null)
            {
                IsValid = false;
                return;
            }

            float altMsl = GetAltitudeMSL();
            EstimatedAltitude = altMsl;

            CalculatePressureFromAltitude(altMsl); // sets clean LastPressure + LastTemperature

            if (addNoise)
            {
                double dt = _hasPrevSample ? (nowSec - _lastSampleSec) : 0.0;
                BaroDriftPa += DriftPaPerSec * (float)dt;
                LastPressure += BaroDriftPa;
                LastPressure += PressureNoiseStdDev * _rng.Next();
            }

            LastPressure += PressureOffset;
            LastTemperature += TemperatureOffset;

            _lastSampleSec = nowSec;
            _hasPrevSample = true;
            LastTimestampSec = nowSec;
            IsValid = true;
        }

        private float GetAltitudeMSL()
        {
            // Single frame boundary: world -> FLU, Z is up. Origin height + up = MSL altitude.
            float up = Frames.TransformWorldPosition(_rb.position, SimFrame.FLU).z;
            return (float)(_geo.OriginHeight + up);
        }

        private void CalculatePressureFromAltitude(float altitudeMsl)
        {
            float tLocal = ISA_TEMPERATURE_MSL - ISA_LAPSE_RATE * altitudeMsl;
            float ratio = Mathf.Pow(ISA_TEMPERATURE_MSL / tLocal, ISA_EXPONENT);
            LastPressure = ISA_PRESSURE_MSL / ratio;
            LastTemperature = tLocal - 273.15f;
        }
    }
}