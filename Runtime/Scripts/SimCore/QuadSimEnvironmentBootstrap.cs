// Assets/Scripts/SimCore/Config/QuadSimEnvironmentBootstrap.cs
//
// Reads a QuadSimSceneConfig asset and pushes each field into the existing
// subsystems at startup. Lives on the QuadSimEnvironment prefab root.
//
// TIMING (why this works without a refactor):
//   SimulationManager  [-1000]  Awake: ClockFactory.Initialize (timing)
//   DroneManager       [-500]   Awake: reads PlayerPrefs config name
//                               OnSimulationStart (fired from SimManager.Start):
//                               loads YAML + SPAWNS the drone
//   ExternalRpcAdapter [0]      Start: ApplyCommandLineOverrides + StartServer (binds)
//   THIS bootstrap     [0]      Awake: writes ports + drone-manager fields
//                               Start: geo origin + wind
//
//   Every Awake completes before any Start, and the spawn happens in Start,
//   so anything written here in Awake is visible to the spawn and to the
//   adapter's StartServer. Default order (0) runs after DroneManager.Awake
//   (-500), so the asset's config name overrides the PlayerPrefs "last used".

using UnityEngine;
using SimCore.Rpc;
using DroneCore.Core;

namespace SimCore
{
    [DefaultExecutionOrder(0)]
    public sealed class QuadSimEnvironmentBootstrap : MonoBehaviour
    {
        [Header("Config")]
        [Tooltip("The scene configuration asset. Set wind/GPS/seeds/ports/drone here — no C#.")]
        [SerializeField] private QuadSimSceneConfig config;

        [Header("Scene References (auto-resolved if left empty)")]
        [SerializeField] private ExternalRpcAdapter rpcAdapter;
        [SerializeField] private DroneManager droneManager;

        [Tooltip("Empty child transform marking where the drone spawns.")]
        [SerializeField] private Transform droneSpawnPoint;

        [Tooltip("The WindModule trigger volume on WindZone_Prefab.")]
        [SerializeField] private Experimental.WindModule windZone;

        // ---------------------------------------------------------------------
        // Awake: everything the spawn / socket-bind needs to see.
        // ---------------------------------------------------------------------
        private void Awake()
        {
            if (config == null)
            {
                Debug.LogError("[EnvBootstrap] No QuadSimSceneConfig assigned. " +
                               "Subsystems will use their own inspector defaults.");
                return;
            }

            ResolveReferences();

            // --- RPC ports -> adapter (read later in adapter.Start/StartServer) ---
            if (rpcAdapter != null)
                rpcAdapter.ConfigurePorts(config.commandPort, config.telemetryPort);

            // --- Drone selection + spawn point -> DroneManager (read in OnSimulationStart) ---
            if (droneManager != null)
            {
                droneManager.SetSelectedConfig(config.droneConfigName);

                if (droneSpawnPoint != null)
                    droneManager.SpawnOrigin = droneSpawnPoint.position;

                // Scene env (seed / noise / PX4 origin) is kept separate from the
                // drone-model DroneConfig. DroneManager forwards it into the drone's
                // SensorManager at spawn, BEFORE SensorManager.OnStart() runs — so the
                // GPS origin and RNG seed are correct from the very first sample.
                // (This is why geo origin is NOT set on Geo directly here: SensorManager
                //  is the single writer of the origin, and we feed it the right value.)
                droneManager.SetSceneEnv(
                    config.masterSeed,
                    config.enableSensorNoise,
                    config.originLatitudeDeg,
                    config.originLongitudeDeg,
                    config.originAltitudeM);
            }

            Debug.Log($"[EnvBootstrap] Awake applied: ports=({config.commandPort},{config.telemetryPort}) " +
                      $"drone='{config.droneConfigName}' seed={config.masterSeed} noise={config.enableSensorNoise}");
        }

        // ---------------------------------------------------------------------
        // Start: scene-global things whose owner is fully initialized by now.
        // (Geo origin is NOT here — it rides the spawn passthrough into
        //  SensorManager so there's no race with GPS.Initialize.)
        // ---------------------------------------------------------------------
        private void Start()
        {
            if (config == null) return;

            // --- Wind -> WindModule trigger volume --------------------------
            // WindModule needs no drone reference (OnTriggerStay catches any
            // Rigidbody). Direction = zone orientation; we yaw it from the asset.
            if (windZone != null)
            {
                if (config.windHeadingDeg != 0f)
                    windZone.transform.rotation = Quaternion.Euler(0f, config.windHeadingDeg, 0f);
                windZone.SetWindSpeed(config.windSpeed);
                windZone.SetEnabled(config.windEnabled);
                Debug.Log($"[EnvBootstrap] Wind: enabled={config.windEnabled} " +
                          $"speed={config.windSpeed} headingDeg={config.windHeadingDeg}");
            }
        }

        // ---------------------------------------------------------------------
        private void ResolveReferences()
        {
            if (rpcAdapter == null)
                rpcAdapter = FindFirstObjectByType<ExternalRpcAdapter>();
            if (droneManager == null)
                droneManager = FindFirstObjectByType<DroneManager>();
            if (windZone == null)
                windZone = FindFirstObjectByType<Experimental.WindModule>();

            if (rpcAdapter == null)
                Debug.LogWarning("[EnvBootstrap] No ExternalRpcAdapter found; ports not applied.");
            if (droneManager == null)
                Debug.LogWarning("[EnvBootstrap] No DroneManager found; drone/seed not applied.");
            if (droneSpawnPoint == null)
                Debug.LogWarning("[EnvBootstrap] No DroneSpawnPoint set; using DroneManager's default origin.");
        }
    }
}