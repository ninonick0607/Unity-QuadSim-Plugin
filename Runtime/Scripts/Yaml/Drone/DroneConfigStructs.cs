//using UnityEngine.Addres; // Required

using System;
using UnityEngine;
using Mathf = UnityEngine.Mathf;

namespace Yaml.Drone
{
    public struct ModelType
    {
        public string Name;
        //public AssetReference BodyMesh;
        
    }

    public struct DroneParameters
    {
        public float Mass;
        public Vector3 CenterOfMass;
        public Vector3 Inertia;
        public float Arm_Length;
        public float LinearDrag_Coeff;
        public float AngularDrag_Coeff;
    }

    public struct RotorModel
    {
        public float Prop_Diameter;
        public float MaxRPM;
        public float ThrustCoeff;
        public float TorqueCoeff;

    }

    public struct RotorInstance
    {
        // FL, FR, BL, BR
        public int ID;
        public Vector3 Location_Body;
        // +1 = CW, -1 = CCW
        public int Spin_Dir;
    }
    
    public struct RotorModelInstance
    {
        public RotorModel Model;
        public RotorInstance[] Rotors;
        
        public RotorModelInstance(int numRotors, RotorModel model)
        {
            Model = model;
            Rotors = new RotorInstance[numRotors];
        }
    }

    public struct CamParameters
    {
        public float CamArm_Length;
        public float FOV;
        public float CamPitch;
        public Vector3 CamSocket_Offset;
    }

    public struct PID
    {
        public float P;
        public float I;
        public float D;
    }
    public struct AcroPID
    {
        public PID AcroRoll {get;set;}
        public PID AcroPitch {get;set;}
        public PID AcroYaw {get;set;}

        public Vector4 GetPGains()
        {
            Vector4 PGains = new Vector4(AcroRoll.P, AcroPitch.P, AcroYaw.P, 0.0f);
            return PGains;
        }
        public Vector4 GetIGains()
        {
            Vector4 IGains = new Vector4(AcroRoll.I, AcroPitch.I, AcroYaw.I, 0.0f);
            return IGains;
        }
        public Vector4 GetDGains()
        {
            Vector4 DGains = new Vector4(AcroRoll.D, AcroPitch.D, AcroYaw.D, 0.0f);
            return DGains;
        }
    }
    public struct AnglePID
    {
        public PID AngleRoll {get;set;}
        public PID AnglePitch {get;set;}
        public PID AngleYaw {get;set;}
        
        public Vector4 GetPGains()
        {
            Vector4 PGains = new Vector4(AngleRoll.P, AnglePitch.P, AngleYaw.P, 0.0f);
            return PGains;
        }
        public Vector4 GetIGains()
        {
            Vector4 IGains = new Vector4(AngleRoll.I, AnglePitch.I, AngleYaw.I, 0.0f);
            return IGains;
        }
        public Vector4 GetDGains()
        {
            Vector4 DGains = new Vector4(AngleRoll.D, AnglePitch.D, AngleYaw.D, 0.0f);
            return DGains;
        }
    }
    public struct VelocityPID
    {
        public PID VelX {get;set;}
        public PID VelY {get;set;}
        public PID VelZ {get;set;}
        
        public Vector4 GetPGains()
        {
            Vector4 PGains = new Vector4(VelX.P, VelY.P,0.0f, VelZ.P);
            return PGains;
        }
        public Vector4 GetIGains()
        {
            Vector4 IGains = new Vector4(VelX.I, VelY.I,0.0f, VelZ.I);
            return IGains;
        }
        public Vector4 GetDGains()
        {
            Vector4 DGains = new Vector4(VelX.D, VelY.D, 0.0f, VelZ.D);
            return DGains;
        }
    }    
    public struct PositionPID
    {
        public PID PosX {get;set;}
        public PID PosY {get;set;}
        public PID PosZ {get;set;}
        
        public Vector4 GetPGains()
        {
            Vector4 PGains = new Vector4(PosX.P, PosY.P,0.0f, PosZ.P);
            return PGains;
        }
        public Vector4 GetIGains()
        {
            Vector4 IGains = new Vector4(PosX.I, PosY.I,0.0f, PosZ.I);
            return IGains;
        }
        public Vector4 GetDGains()
        {
            Vector4 DGains = new Vector4(PosX.D, PosY.D, 0.0f, PosZ.D);
            return DGains;
        }
    }

    public struct FlightParameters
    {
        public float MaxVelXY;
        public float MaxVelZ;
        public float MaxAngle;
        public float MaxRateRollPitch;
        public float MaxRateYaw;
        public float MaxThrust;
        public float MaxPID;
        
        // Nav
        public float AcceptableDistance;
    }

    // TODO: Not in use at the moment, make sure to implement down the line
    public struct ObstacleParameters
    {
        public Vector3 OuterBoundary;
        public Vector3 InnerBoundary;
        public Vector3 SpawnLocation;
    }

    public struct DroneConfig
    {
        public ModelType ModelParams;
        public DroneParameters DroneParams;
        public RotorModelInstance RotorParams;
        public CamParameters CamParams;
        public FlightParameters FlightParams;
        
        public AcroPID Acro;
        public AnglePID Angle;
        public VelocityPID Velocity;
        public PositionPID Position;
        
        // Source File TODO: Lets try and implement this
        public string SourceFile;
        
        // Should add isValid or unnecessary? 
    }
    
    // Math Stuff for Rotors
    public struct RotorPhysicsDerived
    {
        private float RevPerSec;
        private float MaxOmega;
        private float MaxOmegaSqr;
        public float MaxThrust;
        public float MaxTorque;
        private float MaxRollTorqueBody;
        private float MaxPitchTorqueBody;
        private float MaxYawTorqueBody;

        private const float AirDensity = 1.225f;

        private  RotorModel Model;
        private float ArmLengthMeters;
        
        // TODO: Out InModel? 
        public void ComputeRotor(RotorModel InModel,float InArmLengthMeters)
        {
            Model = InModel;
            ArmLengthMeters = InArmLengthMeters;
            
            RevPerSec = Model.MaxRPM / 60.0f;
            MaxOmega = RevPerSec * 2.0f * (float)Math.PI;
            MaxOmegaSqr = MaxOmega * MaxOmega;
            
            float n2 = RevPerSec * RevPerSec;
            float D = Model.Prop_Diameter;
            
            // Thrust  = Ct * rho * n^2 * D^4
            MaxThrust = Model.ThrustCoeff * AirDensity * n2 * (float)Math.Pow(D, 4);
            // Torque = Cp * rho * n^2 * D^5/2PI
            MaxTorque = Model.TorqueCoeff * AirDensity * n2 * (((float)Math.Pow(D, 5))/ ((float)(2.5 * Math.PI)));
            
            // TODO: Original code says MaxThrust instead of MaxTorque, veriy this
            MaxRollTorqueBody = MaxTorque * ArmLengthMeters;
            MaxPitchTorqueBody = MaxTorque * ArmLengthMeters;
            
            // TODO: Hardcoded max yaw torque for 4 motors
            MaxYawTorqueBody = 4.0f * MaxTorque;
        }

        public float ComputeHoverThrottle(float VehicleMassKg, int MotorCount = 4)
        {
            if (MotorCount <= 0 || MaxThrust <= 0.0f) return 0.5f;
            float perMotorHoverN = (VehicleMassKg * 9.81f) / (float)MotorCount;
            return Mathf.Clamp(perMotorHoverN / MaxThrust, 0.0f, 1.0f);
        }
        
        public float ControlToThrust(float Control01, float AirDensityRatio = 1.0f)
        {
            return Mathf.Clamp(Control01, 0.0f, 1.0f) * MaxThrust * AirDensityRatio;
        }

        public float ControlToYawTorque(float Control01, int TurningDir, float AirDensityRatio = 1.0f)
        {
            return Mathf.Clamp(Control01, 0.0f, 1.0f) * MaxTorque * (float)TurningDir * AirDensityRatio;
        }
        
    }
    
}
