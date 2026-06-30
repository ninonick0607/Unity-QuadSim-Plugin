using UnityEngine;

namespace RobotCore
{
    public struct SensorData
    {
        // IMU (frame: selected output frame)
        public Vector3 ImuAngVel;  
        public Vector3 ImuAttitude;
        public Vector3 ImuAccel;     
        public Vector3 ImuVel;
        public Quaternion ImuOrientation;
        public double ImuTimestampSec;
        public bool ImuValid;

        // GPS
        public Vector3 GpsPosition;    
        public double GpsTimestampSec;
        public bool GpsValid;
        
        // Baro (pressure in Pa, altitudes in m, temp in degC)
        public float  BaroPressurePa;
        public float  BaroPressureHPa;       // HIL_SENSOR.abs_pressure
        public float  BaroTemperatureC;      // HIL_SENSOR.temperature
        public float  BaroAltitudeMSL;       // true input altitude (ground truth)
        public float  BaroPressureAltitude;  // HIL_SENSOR.pressure_alt (derived from pressure)
        public double BaroTimestampSec;
        public bool   BaroValid;
        
        // Mag (body frame, OutputFrame, Gauss)
        public Vector3 MagField;
        public float   MagHeadingDeg;
        public float   MagDeclinationDeg;
        public double  MagTimestampSec;
        public bool    MagValid;
        
        // GPS geodetic + NED velocity
        public double  GpsLatDeg;
        public double  GpsLonDeg;
        public double  GpsAltM;        // ellipsoidal
        public Vector3 GpsVelNED;
    }
}