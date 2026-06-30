// Assets/Scripts/SimCore/Config/QuadSimSceneConfig.cs
//
// Researcher-facing scene configuration for the QuadSimEnvironment prefab.
//
// Drop the QuadSimEnvironment prefab into an empty scene, point its
// QuadSimEnvironmentBootstrap at one of these assets, hit Play. No C# editing.
//
// Create assets via:  Assets > Create > QuadSim > Scene Config
//
using UnityEngine;

namespace SimCore
{
    [CreateAssetMenu(
        fileName = "QuadSimSceneConfig",
        menuName = "QuadSim/Scene Config",
        order = 0)]
    public sealed class QuadSimSceneConfig : ScriptableObject
    {
        // =====================================================================
        // Drone
        // =====================================================================
        [Header("Drone")]
        [Tooltip("YAML file name (without extension) in StreamingAssets/Configs/ " +
                 "that DroneConfigLoader reads. Selects mass/inertia/motor params.")]
        public string droneConfigName = "flightmare_quad";

        // =====================================================================
        // RPC (Python SDK boundary)
        // =====================================================================
        [Header("RPC Ports")]
        [Tooltip("ZMQ REQ/REP command port. SDK default 5555.")]
        public int commandPort = 5555;

        [Tooltip("ZMQ PUB/SUB telemetry port. SDK default 5556.")]
        public int telemetryPort = 5556;

        // =====================================================================
        // Geodetic origin (WGS84) — where QGC shows the vehicle; EKF2 latches
        // this from the first HIL_GPS fix.
        // =====================================================================
        [Header("GPS / Geodetic Origin")]
        [Tooltip("Latitude in degrees. Default: Eglin AFB.")]
        public double originLatitudeDeg = 30.4636;

        [Tooltip("Longitude in degrees. Default: Eglin AFB.")]
        public double originLongitudeDeg = -86.5533;

        [Tooltip("Ellipsoidal altitude in meters.")]
        public double originAltitudeM = 0.0;

        // =====================================================================
        // Sensor noise — keep OFF for clean determinism, ON for PX4 SITL.
        // Per-sensor streams derive from masterSeed + fixed offsets
        // (IMU +1, GPS +2, baro +3, mag +4) inside the drone's SensorManager.
        // =====================================================================
        [Header("Sensor Noise")]
        [Tooltip("Master Gaussian RNG seed. Per-sensor offsets are applied downstream " +
                 "so parallel runs stay reproducible.")]
        public int masterSeed = 12345;

        [Tooltip("Inject sensor noise. Default false (deterministic). Set true for PX4 SITL / HIL.")]
        public bool enableSensorNoise = false;

        // =====================================================================
        // Wind — applied as a world-frame force directly to the Rigidbody.
        // Human-friendly heading+speed; bootstrap resolves to a world vector.
        // =====================================================================
        // WindModule is a trigger volume: it blows along its local +Z (forward) and
        // applies drag to any Rigidbody inside it. Direction = the zone's orientation;
        // the bootstrap yaws the zone transform from windHeadingDeg so you can steer
        // wind from this asset without touching the hierarchy.
        [Header("Wind")]
        [Tooltip("Enable the wind disturbance (drives WindModule.SetEnabled).")]
        public bool windEnabled = false;

        [Tooltip("Fan jet air speed at the source face, m/s (drives WindModule.SetWindSpeed).")]
        public float windSpeed = 0f;

        [Tooltip("Wind heading (yaw degrees) applied to the WindZone transform. " +
                 "0 = the zone's authored forward. Leave at 0 to steer by rotating the zone by hand.")]
        public float windHeadingDeg = 0f;
    }
}