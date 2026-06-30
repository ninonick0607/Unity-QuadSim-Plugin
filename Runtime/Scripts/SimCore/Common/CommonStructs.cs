// Assets/Scripts/SimCore/Common/CommonStructs.cs
// Phase 1: CommonStructs Verification & Enhancement
//
// CHANGES FROM EXISTING:
//   - Added FlightCommand.SafeHover() factory for authority fallback
//   - Added FlightCommand.IsValid property  
//   - Added Motor4 indexer and Zero factory
//   - Added Wrench6 Zero factory
//   - All enums verified complete per API plan
//   - All payload structs verified complete per API plan

using System;
using UnityEngine;

namespace SimCore.Common
{
    // ==================== Enums ==================== //
    
    /// <summary>
    /// Who is producing commands: the in-sim UI, a user C# script, or an external RPC client.
    /// </summary>
    public enum InputSource : byte { UI, Internal, External }
    
    /// <summary>
    /// Lifecycle state of a command source.
    /// UI is never Unavailable. Internal/External can be any state.
    /// Denied is External-only (user clicked "Deny" for the session).
    /// </summary>
    public enum SourceStatus : byte { Unavailable, Available, Connected, Denied }
    
    /// <summary>
    /// Which controller implementation is active. Independent of GoalMode.
    /// </summary>
    public enum ControllerKind : byte { Cascade, Geometric }
    
    /// <summary>
    /// Which stage of the cascade to enter at. None = motors off.
    /// Geometric controller ignores this entirely.
    /// </summary>
    public enum GoalMode : byte
    {
        None = 0,
        Position = 1,
        Velocity = 2,
        Angle = 3,
        Rate = 4,
        Passthrough = 5,
        Wrench = 6,
        WrenchBypassed = 7,
        Acceleration = 8
    }
    
    // ==================== Command Payloads ==================== //
    
    /// <summary>
    /// 6-DOF wrench for direct force/torque application. Only used in GoalMode.Wrench.
    /// Bypasses the entire cascade and actuation model — applies directly to Rigidbody.
    /// </summary>
    public struct Wrench6
    {
        public Vector3 Force;
        public Vector3 Torque;
        
        public static Wrench6 Zero => new Wrench6 { Force = Vector3.zero, Torque = Vector3.zero };
    }
    
    /// <summary>
    /// Raw motor commands [0..1] for Passthrough mode. Skips cascade, goes through actuation model.
    /// </summary>
    public struct Motor4
    {
        public float FL, FR, BL, BR;
        
        public float this[int index]
        {
            get => index switch { 0 => FL, 1 => FR, 2 => BL, 3 => BR, _ => throw new IndexOutOfRangeException() };
            set { switch (index) { case 0: FL = value; break; case 1: FR = value; break; case 2: BL = value; break; case 3: BR = value; break; default: throw new IndexOutOfRangeException(); } }
        }
        
        public static Motor4 Zero => new Motor4 { FL = 0, FR = 0, BL = 0, BR = 0 };
    }
    
    /// <summary>
    /// 4-axis command payload. Interpretation depends on GoalMode:
    ///   Position:    X=X_m,      Y=Y_m,         Z=Yaw_deg,       W=Alt_m
    ///   Velocity:    X=Vx_mps,   Y=Vy_mps,      Z=YawRate_dps,   W=Vz_mps
    ///   Acceleration:X=Ax_mps2,Y=Ay_mps2,     Z=YawRate_dps,   W=Az_mps2
    ///   Angle:       X=Roll_deg, Y=Pitch_deg,    Z=YawRate_dps,   W=Throttle_01
    ///   Rate:        X=RollRate, Y=PitchRate,    Z=YawRate_dps,   W=Throttle_01
    ///   Passthrough: X=M0,       Y=M1,           Z=M2,            W=M3
    /// </summary>
    [Serializable]
    public struct Axis4
    {
        public float X;
        public float Y;
        public float Z;
        public float W;

        public Axis4(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        // Vector views
        public Vector3 XYZ
        {
            get => new Vector3(X, Y, Z);
            set { X = value.x; Y = value.y; Z = value.z; }
        }

        // Semantic aliases (for Angle/Rate modes)
        public float Roll    { get => X; set => X = value; }
        public float Pitch   { get => Y; set => Y = value; }
        public float Yaw     { get => Z; set => Z = value; }
        
        // Semantic alias for W depending on mode
        public float Throttle { get => W; set => W = value; }   // Rate mode
        public float AltOrVz  { get => W; set => W = value; }   // Position/Velocity/Angle modes

        public float this[int index]
        {
            get => index switch
            {
                0 => X, 1 => Y, 2 => Z, 3 => W,
                _ => throw new IndexOutOfRangeException($"Axis4 index out of bounds: {index}")
            };
            set
            {
                switch (index)
                {
                    case 0: X = value; break;
                    case 1: Y = value; break;
                    case 2: Z = value; break;
                    case 3: W = value; break;
                    default: throw new IndexOutOfRangeException($"Axis4 index out of bounds: {index}");
                }
            }
        }

        public void Zero() => X = Y = Z = W = 0f;
        public static Axis4 ZeroValue => new Axis4(0, 0, 0, 0);
        
        public override string ToString() => $"({X:F3}, {Y:F3}, {Z:F3}, {W:F3})";
    }
    
    
    // ==================== Command Envelope ==================== //
    
    /// <summary>
    /// Complete command envelope. Carries the mode, source, timestamp, and payload.
    /// Only one payload field is active depending on Mode:
    ///   - Wrench mode uses Wrench field
    ///   - Passthrough mode uses Motors field (or Cmd with M0-M3 in XYZW)
    ///   - All other modes use Cmd (Axis4)
    /// </summary>
    public struct FlightCommand
    {
        public GoalMode Mode;
        public InputSource Source;
        public double TimestampSec;
        public Axis4 Cmd;
        public Wrench6 Wrench;
        public Motor4 Motors;

        /// <summary>Empty/invalid command.</summary>
        public static FlightCommand None => new FlightCommand { Mode = GoalMode.None };
        
        /// <summary>Has this command been populated with real data?</summary>
        public bool IsValid => Mode != GoalMode.None && TimestampSec > 0;
        
        /// <summary>
        /// Safe fallback command — zero rates, zero throttle.
        /// Used when the authority source hasn't sent a command yet or disconnects.
        /// The mode passed in determines the shape of the safe command.
        /// </summary>
        public static FlightCommand SafeFallback(GoalMode currentMode, InputSource source, double timestamp)
        {
            var cmd = new FlightCommand
            {
                Mode = currentMode,
                Source = source,
                TimestampSec = timestamp,
                Cmd = Axis4.ZeroValue,
                Wrench = Wrench6.Zero,
                Motors = Motor4.Zero
            };
            return cmd;
        }
    }
}