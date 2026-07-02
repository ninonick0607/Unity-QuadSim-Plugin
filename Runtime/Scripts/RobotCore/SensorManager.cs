using DroneCore;
using MathUtil;
using RobotCore.Sensors;
using SimCore;
using UnityEngine;
using RobotCore.Core_Utils;

namespace RobotCore
{
    [RequireComponent(typeof(QuadPawn))]
    [DisallowMultipleComponent]
    public sealed class SensorManager : MonoBehaviour
    {
        [Header("Refs")]
        [SerializeField] private Rigidbody rb;
        [SerializeField] private Transform bodyTransform;

        [Header("Output Frame")]
        public SimFrame outputFrame = SimFrame.FLU;

        [Header("Rates (Hz)")]
        public double imuHz = 250.0;
        public double gpsHz = 10.0;
        public double baroHz = 50.0;  
        public double magHz = 50.0;
        
        [Header("Noise")]
        public bool enableNoise = false;   // set TRUE for PX4 SITL/HIL. NOTE: baro-only today.

        [Header("Georeferencing (PX4 origin)")]
        public double originLatDeg  = 47.397742;  // PX4 SITL default home (Zurich)
        public double originLonDeg  = 8.545594;
        public double originHeightM = 488.0;
        
        [Header("Debug Logging")]
        public bool logSensors = true;
        public float logHz = 2f;

        private double _logAccum;
        
        public SimFrame TargetFrame = SimFrame.FLU; 
        private FrequencyLimiter _imuLimiter;
        private FrequencyLimiter _gpsLimiter;
        private FrequencyLimiter _baroLimiter;
        private FrequencyLimiter _magLimiter;
        private FrequencyLimiter _logLimiter;

        public GeoReferencingSystem Geo { get; private set; }
        public IMUSensor IMU { get; private set; }
        public GPSSensor GPS { get; private set; }
        public BaroSensor Baro { get; private set; }
        public MagSensor Mag { get; private set; }
        

        public SensorData Latest { get; private set; }
        public int noiseSeed = 12345; 
        public int ExecutionOrder => 50;
        public string DebugName => name;
        
        [ContextMenu("Toggle Output Frame  FLU <-> FRD")]
        public void ToggleOutputFrame()
        {
            outputFrame = (outputFrame == SimFrame.FRD) ? SimFrame.FLU : SimFrame.FRD;
            Debug.Log($"[SensorManager] outputFrame → {outputFrame}");
        }
        
        private void ResolveRefsOrThrow()
        {
            if (rb == null) rb = GetComponentInParent<Rigidbody>();
            if (rb == null)
                throw new System.Exception($"SensorManager on '{name}' cannot find Rigidbody in parents. Put it under DroneRoot.");

            if (bodyTransform == null) bodyTransform = rb.transform;
        }

        private void Awake()
        {
            ResolveRefsOrThrow();
        }
        
        /// <summary>Apply scene-level env BEFORE OnStart(). Single writer for seed/noise/origin.
        /// The +1/+2/+3/+4 per-sensor offsets are still applied from noiseSeed inside OnStart().</summary>
        public void Configure(int seed, bool noise, double latDeg, double lonDeg, double heightM)
        {
            noiseSeed     = seed;
            enableNoise   = noise;
            originLatDeg  = latDeg;
            originLonDeg  = lonDeg;
            originHeightM = heightM;
        }
        public void OnStart()
        {
            ResolveRefsOrThrow();

            IMU  = new IMUSensor(rb, bodyTransform, noiseSeed + 1);
            GPS  = new GPSSensor(rb, noiseSeed + 2);

            Geo = SimCoreApi.Get().Geo;
            Geo.SetOrigin(originLatDeg, originLonDeg,originHeightM);
            
            Baro = new BaroSensor(rb, bodyTransform, noiseSeed + 3);
            
            Mag = new MagSensor(rb, bodyTransform, noiseSeed + 4);

            
            IMU.OutputFrame = outputFrame;
            GPS.OutputFrame = outputFrame;
            Mag.OutputFrame = outputFrame;

            IMU.Reset();
            IMU.Initialize(rb);

            GPS.Reset();
            GPS.Initialize(Geo);

            Baro.Reset();           
            Baro.Initialize(Geo);  

            Mag.Reset();
            Mag.Initialize(Geo);
            
            long now = 0;
            _imuLimiter  = new FrequencyLimiter(imuHz, now);
            _gpsLimiter  = new FrequencyLimiter(gpsHz, now);
            _baroLimiter = new FrequencyLimiter(baroHz, now);   
            _magLimiter = new FrequencyLimiter(magHz, now);

            _logLimiter  = new FrequencyLimiter(logHz <= 0 ? 1e-6 : logHz, now);

            Latest = default;
        }

        public void OnReset()
        {
            IMU.Reset();
            IMU.Initialize(rb);
            
            GPS.Reset();
            GPS.Initialize(Geo);

            Baro.Reset();
            Baro.Initialize(Geo);
            
            Mag.Reset();
            Mag.Initialize(Geo);
            
            long now = ClockFactory.Clock.NowNanos;

            _imuLimiter?.Reset(now);
            _gpsLimiter?.Reset(now);
            _baroLimiter?.Reset(now);
            _magLimiter?.Reset(now);
            _logLimiter?.Reset(now);

            Latest = default;
        }

        public void UpdateSensors(double dtSec, long nowNanos)
        {
            double nowSec = nowNanos * 1e-9;

            // Keep output frame synced for both sensors
            IMU.OutputFrame = outputFrame;
            GPS.OutputFrame = outputFrame;
            Mag.OutputFrame = outputFrame;

            var latest = Latest;

            // IMU
            if (_imuLimiter.ShouldRunAndConsume(nowNanos))
            {
                IMU.Sample(nowSec, (float)dtSec, enableNoise);   // was: IMU.Sample(nowSec, (float)dtSec);

                latest.ImuAngVel = IMU.LastAngVel;
                latest.ImuAttitude = IMU.LastAtt;
                latest.ImuOrientation = IMU.LastOrientation; 
                latest.ImuAccel = IMU.LastLinAcc;
                latest.ImuVel = IMU.LastVel;
                latest.ImuTimestampSec = IMU.LastTimestampSec;
                latest.ImuValid = IMU.IsValid;
            }

            // GPS — position is now already in OutputFrame from GPSSensor
            if (_gpsLimiter.ShouldRunAndConsume(nowNanos))
            {
                GPS.Sample(nowSec, enableNoise);

                latest.GpsPosition     = GPS.LastPosition;
                latest.GpsTimestampSec = GPS.LastTimestampSec;
                latest.GpsValid        = GPS.HasFix;

                latest.GpsLatDeg = GPS.LastLatDeg;
                latest.GpsLonDeg = GPS.LastLonDeg;
                latest.GpsAltM   = GPS.LastAltM;
                latest.GpsVelNED = GPS.LastVelNED;
            }

            // Baro
            if (_baroLimiter.ShouldRunAndConsume(nowNanos))
            {
                Baro.Sample(nowSec, enableNoise);

                latest.BaroPressurePa       = Baro.LastPressure;
                latest.BaroPressureHPa      = Baro.LastPressureHPa;
                latest.BaroTemperatureC     = Baro.LastTemperature;
                latest.BaroAltitudeMSL      = Baro.EstimatedAltitude;
                latest.BaroPressureAltitude = Baro.PressureAltitude;
                latest.BaroTimestampSec     = Baro.LastTimestampSec;
                latest.BaroValid            = Baro.IsValid;
            }
            // Mag
            if (_magLimiter.ShouldRunAndConsume(nowNanos))
            {
                Mag.Sample(nowSec, (float)dtSec, enableNoise);

                latest.MagField          = Mag.LastMagField;
                latest.MagHeadingDeg     = Mag.MagneticHeading;
                latest.MagDeclinationDeg = Mag.MagneticDeclination;
                latest.MagTimestampSec   = Mag.LastTimestampSec;
                latest.MagValid          = Mag.IsValid;
            }

            Latest = latest;
        }
    }
}