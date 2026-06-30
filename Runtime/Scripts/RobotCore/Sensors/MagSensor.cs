using SimCore;
using MathUtil;
using UnityEngine;
using RobotCore.Core_Utils;

namespace RobotCore.Sensors
{
    public sealed class MagSensor
    {
        public float MagNoiseStdDev = 0.005f;                 // Gauss
        public float MagOffsetX = 0f, MagOffsetY = 0f, MagOffsetZ = 0f;  // Gauss bias

        public const float CALIBRATION_DURATION = 1.0f;       // vestigial; mirrors Unreal, gates nothing

        private readonly GaussianRng _rng;
        private Rigidbody _rb;
        private Transform _bodyTransform;
        private GeoReferencingSystem _geo;

        public SimFrame OutputFrame = SimFrame.FLU;

        public Vector3 LastMagField     { get; private set; } // body, OutputFrame, Gauss  (PX4 wants FRD)
        public Vector3 EarthMagFieldNED { get; private set; } // geographic NED, Gauss (debug)
        public float   MagneticHeading      { get; private set; } // deg, debug (≈ declination — see note)
        public float   MagneticDeclination  { get; private set; } // deg, from WMM
        public double  LastTimestampSec { get; private set; }
        public bool    IsValid          { get; private set; }

        public bool  IsCalibrating { get; private set; }
        private float _calibrationTime;
        private bool  _earthFieldValid;
        private bool  _declinationLogged;

        public MagSensor(Rigidbody rb, Transform bodyTransform, int noiseSeed = 4)
        {
            _rb = rb;
            _bodyTransform = bodyTransform;
            _rng = new GaussianRng(noiseSeed);
        }

        public void Reset()
        {
            LastMagField = Vector3.zero;
            EarthMagFieldNED = Vector3.zero;
            MagneticHeading = 0f;
            MagneticDeclination = 0f;
            LastTimestampSec = 0.0;
            IsValid = false;
            IsCalibrating = true;
            _calibrationTime = 0f;
            _earthFieldValid = false;
            _declinationLogged = false;
        }

        public void Initialize(GeoReferencingSystem geo)
        {
            _geo = geo;
            IsValid = (_geo != null && _rb != null && _bodyTransform != null);
        }

        public void Sample(double nowSec, float dtSec, bool addNoise)
        {
            if (_geo == null || _rb == null || _bodyTransform == null)
            {
                IsValid = false;
                return;
            }

            if (IsCalibrating)
            {
                _calibrationTime += dtSec;
                if (_calibrationTime >= CALIBRATION_DURATION) IsCalibrating = false;
            }

            UpdateEarthMagField();
            if (!_earthFieldValid) { IsValid = false; return; }

            // Earth field NED (Gauss) -> Unity world (X=East, Y=Up, Z=North):
            //   N -> +Z, E -> +X, D -> -Y
            float n = EarthMagFieldNED.x, e = EarthMagFieldNED.y, d = EarthMagFieldNED.z;
            Vector3 fieldWorldUnity = new Vector3(e, -d, n);

            // World -> body (Unity), then -> OutputFrame. Same machinery as the IMU.
            Vector3 fieldBodyUnity = _bodyTransform.InverseTransformDirection(fieldWorldUnity);
            Vector3 field = Frames.TransformLinear(fieldBodyUnity, OutputFrame);

            if (addNoise)
                field += new Vector3(_rng.Next(), _rng.Next(), _rng.Next()) * MagNoiseStdDev;

            field.x += MagOffsetX;
            field.y += MagOffsetY;
            field.z += MagOffsetZ;

            LastMagField = field;
            CalculateMagneticHeading();

            LastTimestampSec = nowSec;
            IsValid = true;
        }

        private void UpdateEarthMagField()
        {
            // Convention: FLU world position == ENU (East, North, Up). See alignment note.
            Vector3 flu = Frames.TransformWorldPosition(_rb.position, SimFrame.FLU);
            _geo.EnuToGeodetic(flu.x, flu.y, flu.z, out double lat, out double lon, out _);

            EarthMagFieldNED    = GeoMagneticTables.GetMagFieldVectorNED((float)lat, (float)lon);
            MagneticDeclination = GeoMagneticTables.GetMagDeclinationDegrees((float)lat, (float)lon);

            if (!_declinationLogged)
            {
                Debug.Log($"[Mag] WMM declination={MagneticDeclination:F2}deg  lat={lat:F5} lon={lon:F5}");
                _declinationLogged = true;
            }
            _earthFieldValid = true;
        }

        private void CalculateMagneticHeading()
        {
            // Rotate measured body field back to Unity world and read the horizontal bearing.
            // This points at *magnetic* North regardless of vehicle yaw, so the value tracks
            // declination — it's a transform self-consistency check, not vehicle heading.
            Vector3 bodyUnity  = Frames.InverseTransformLinear(LastMagField, OutputFrame);
            Vector3 worldUnity = _bodyTransform.TransformDirection(bodyUnity);

            float headingRad = Mathf.Atan2(worldUnity.x /*East*/, worldUnity.z /*North*/);
            float headingDeg = Mathf.Rad2Deg * headingRad;
            if (headingDeg < 0f) headingDeg += 360f;
            MagneticHeading = headingDeg;
        }
    }
}