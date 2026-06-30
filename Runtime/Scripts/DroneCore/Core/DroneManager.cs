    using System.Collections.Generic;
using SimCore;
using UnityEngine;
using Yaml.Drone;

namespace DroneCore.Core
{
    /// <summary>
    /// DroneManager is the central registry and orchestrator for all drones in the simulation.
    /// It handles spawning, selection, configuration, and drives the physics step pipeline.
    ///
    /// Implements ISimulatable so SimulationManager drives the loop:
    ///   PrePhysicsStep  → iterate QuadPawns → pawn.PhysicsStep(dt, now)
    ///   PostPhysicsStep → (reserved for future use)
    ///   Update()        → visual updates (prop spin, effects)
    /// </summary>
    ///
    [DefaultExecutionOrder(-500)]
    public sealed class DroneManager : MonoBehaviour, ISimulatable
    {
        // ============================================================================
        // Serialized Fields
        // ============================================================================

        [Header("Spawning")]
        [Tooltip("Prefab to instantiate when spawning drones.")]
        [SerializeField] private QuadPawn quadPawnPrefab;

        [Tooltip("Default spawn location. Set in inspector or override via SpawnOrigin property.")]
        [SerializeField] private Vector3 defaultSpawnOrigin = new Vector3(0f, 1f, 0f);

        [Tooltip("Offset between multiple spawned drones.")]
        [SerializeField] private float spawnOffset = 1.5f;

        [Header("Configuration")]
        [Tooltip("Default config name to load.")]
        [SerializeField] private string selectedConfigName = "StandardDrone_Config";

        [Header("Core References")]
        [Tooltip("SimulationManager reference. Set in inspector, fallback to scene find.")]
        [SerializeField] private SimulationManager simManager;

        [Header("Swarm")]
        [Tooltip("Enable swarm mode (all drones receive same commands).")]
        [SerializeField] private bool swarmMode = false;
        
        [Header("Scene Env")]
        [SerializeField] private int     masterSeed       = 12345;
        [SerializeField] private bool    enableSensorNoise = false;
        [SerializeField] private double  originLat        = 30.4636;
        [SerializeField] private double  originLon        = -86.5533;
        [SerializeField] private double  originAlt        = 0.0;
        // ============================================================================
        // Runtime State
        // ============================================================================

        private readonly List<QuadPawn> _registeredDrones = new List<QuadPawn>();
        private int _selectedDroneIndex = 0;
        private DroneConfig _currentConfig;
        private bool _hasCurrentConfig;

        // ============================================================================
        // ISimulatable Implementation
        // ============================================================================

        public int ExecutionOrder => 10;
        public string DebugName => "DroneManager";

        // ============================================================================
        // Public Accessors
        // ============================================================================

        public IReadOnlyList<QuadPawn> RegisteredDrones => _registeredDrones;
        public int DroneCount => _registeredDrones.Count;
        public int SelectedDroneIndex => _selectedDroneIndex;
        public string SelectedConfigName => selectedConfigName;
        public bool SwarmMode { get => swarmMode; set => swarmMode = value; }
        public Vector3 SpawnOrigin { get => defaultSpawnOrigin; set => defaultSpawnOrigin = value; }

        /// <summary>
        /// Currently selected drone, or null.
        /// </summary>
        public QuadPawn SelectedDrone
        {
            get
            {
                if (_selectedDroneIndex >= 0 && _selectedDroneIndex < _registeredDrones.Count)
                    return _registeredDrones[_selectedDroneIndex];
                return null;
            }
        }

        // ============================================================================
        // Static Accessor
        // ============================================================================

        private static DroneManager _instance;

        /// <summary>
        /// Get cached DroneManager instance. Falls back to scene find with warning.
        /// </summary>
        public static DroneManager Get()
        {
            if (_instance != null) return _instance;

            _instance = FindFirstObjectByType<DroneManager>();
            if (_instance != null)
                Debug.LogWarning("[DroneManager] Instance resolved via FindFirstObjectByType. Prefer direct reference.");

            return _instance;
        }

        // ============================================================================
        // Unity Lifecycle
        // ============================================================================

        private void Awake()
        {
            _instance = this;
    
            string savedConfig = PlayerPrefs.GetString("QuadSim.DefaultConfig", "");
            if (!string.IsNullOrEmpty(savedConfig))
                selectedConfigName = savedConfig;

            Debug.Log($"[DroneManager] Awake: selectedConfig={selectedConfigName}");

            // Register with SimManager during Awake so we're in the list 
            // before SimulationManager.Start() fires OnSimulationStart
            if (simManager == null)
            {
                simManager = FindFirstObjectByType<SimulationManager>();
                if (simManager != null)
                    Debug.LogWarning("[DroneManager] SimulationManager resolved via find. Prefer inspector assignment.");
            }
            if (simManager != null)
            {
                simManager.RegisterManagers(this);
                Debug.Log("[DroneManager] Registered with SimulationManager");
            }
        }

        private void Start()
        {
        }

        private void OnDestroy()
        {
            if (simManager != null)
            {
                simManager.UnRegisterManager(this);
            }

            if (_instance == this)
            {
                _instance = null;
            }
        }

        // ============================================================================
        // ISimulatable — Simulation Lifecycle
        // ============================================================================

        public void OnSimulationStart(SimulationManager sim)
        {
            simManager = sim;

            _hasCurrentConfig = DroneConfigLoader.LoadConfig(selectedConfigName, out _currentConfig);
            if (_hasCurrentConfig)
                Debug.Log($"[DroneManager] Loaded config: {selectedConfigName}");
            else
                Debug.LogWarning($"[DroneManager] Failed to load config: {selectedConfigName}");

            // If no drones exist yet, spawn one now.
            if (_registeredDrones.Count == 0)
            {
                Debug.Log("[DroneManager] No drones registered; spawning default drone.");
                SpawnDroneWithConfig(selectedConfigName, defaultSpawnOrigin, Quaternion.identity);
            }
            else
            {
                if (_hasCurrentConfig)
                {
                    foreach (var d in _registeredDrones)
                    {
                        if (d == null) continue;
                        if (!d.HasConfigApplied)
                            d.ApplyConfig(_currentConfig);
                    }
                }
            }
        }

        public void OnSimulationReset(SimulationManager sim)
        {
            Debug.Log("[DroneManager] OnSimulationReset");
            ResetAllDrones();
        }

        /// <summary>
        /// Drive the physics step for all drones.
        /// SimulationManager calls this BEFORE Physics.Simulate().
        /// Each QuadPawn runs: Sensors → Controller → Apply Wrench.
        /// </summary>
        public void PrePhysicsStep(double dtSec, long nowNanos)
        {
            CleanupInvalidDrones();

            for (int i = 0; i < _registeredDrones.Count; i++)
            {
                QuadPawn drone = _registeredDrones[i];
                if (drone != null)
                {
                    drone.PhysicsStep(dtSec, nowNanos);
                }
            }
        }

        public void PostPhysicsStep(double dtSec, long nowNanos)
        {
            // Reserved for future use (logging, telemetry publish, etc.)
        }

        // ============================================================================
        // Visual Updates — Called by Unity Update (rendering frame)
        // ============================================================================

        private void Update()
        {
            float dt = Time.deltaTime;
            for (int i = 0; i < _registeredDrones.Count; i++)
            {
                QuadPawn drone = _registeredDrones[i];
                if (drone != null)
                {
                    drone.UpdateVisuals(dt);
                }
            }
        }

        // ============================================================================
        // Spawning
        // ============================================================================

        /// <summary>
        /// Spawn a drone using the currently selected config.
        /// </summary>
        public QuadPawn SpawnDrone(Vector3 position, Quaternion rotation)
        {
            return SpawnDroneWithConfig(selectedConfigName, position, rotation);
        }

        /// <summary>
        /// Spawn a drone with a specific config name.
        /// </summary>
        public QuadPawn SpawnDroneWithConfig(string configName, Vector3 position, Quaternion rotation)
        {
            if (quadPawnPrefab == null)
            {
                Debug.LogError("[DroneManager] Cannot spawn: QuadPawnPrefab not set!");
                return null;
            }

            // Load config
            if (!DroneConfigLoader.LoadConfig(configName, out DroneConfig config))
            {
                Debug.LogError($"[DroneManager] Failed to load config: {configName}");
                return null;
            }

            // Determine spawn position
            Vector3 spawnPos = DetermineSpawnPosition(position);
            Quaternion spawnRot = rotation == Quaternion.identity ? DetermineSpawnRotation() : rotation;

            // Instantiate
            QuadPawn newDrone = Instantiate(quadPawnPrefab, spawnPos, spawnRot);
            if (newDrone == null)
            {
                Debug.LogError("[DroneManager] Failed to instantiate QuadPawn prefab");
                return null;
            }
            
            newDrone.name = $"Drone_{_registeredDrones.Count:D2}";
            
            if (newDrone.Sensors != null)
                newDrone.Sensors.Configure(masterSeed, enableSensorNoise, originLat, originLon, originAlt);
            
            // Inject manager reference BEFORE Start() runs
            newDrone.SetDroneManager(this);

            // Apply config (triggers QuadPawn initialization)
            newDrone.ApplyConfig(config);

            // Select the new drone
            int newIndex = GetDroneIndex(newDrone);
            if (newIndex >= 0)
            {
                SelectDroneByIndex(newIndex, true);
            }

            Debug.Log($"[DroneManager] Spawned drone: {newDrone.name} at {spawnPos}");
            return newDrone;
        }

        private Vector3 DetermineSpawnPosition(Vector3 requestedPosition)
        {
            if (requestedPosition != Vector3.zero)
                return requestedPosition;

            // Stack next to last drone
            if (_registeredDrones.Count > 0)
            {
                QuadPawn last = _registeredDrones[_registeredDrones.Count - 1];
                if (last != null)
                    return last.transform.position + last.transform.right * spawnOffset;
            }

            return defaultSpawnOrigin;
        }

        private Quaternion DetermineSpawnRotation()
        {
            if (_registeredDrones.Count > 0)
            {
                QuadPawn last = _registeredDrones[_registeredDrones.Count - 1];
                if (last != null)
                    return last.transform.rotation;
            }
            return Quaternion.identity;
        }

        // ============================================================================
        // Registration
        // ============================================================================

        public void RegisterDrone(QuadPawn drone)
        {
            if (drone == null) return;

            if (_registeredDrones.Contains(drone))
            {
                Debug.LogWarning($"[DroneManager] Drone already registered: {drone.name}");
                return;
            }

            bool wasEmpty = _registeredDrones.Count == 0;
            _registeredDrones.Add(drone);

            Debug.Log($"[DroneManager] Registered drone: {drone.name} (total: {_registeredDrones.Count})");

            if (wasEmpty)
            {
                SelectDroneByIndex(0, true);
            }
        }

        public void UnregisterDrone(QuadPawn drone)
        {
            if (drone == null) return;

            int index = _registeredDrones.IndexOf(drone);
            if (index < 0) return;

            _registeredDrones.RemoveAt(index);

            if (_selectedDroneIndex >= _registeredDrones.Count)
            {
                _selectedDroneIndex = Mathf.Max(0, _registeredDrones.Count - 1);
            }

            Debug.Log($"[DroneManager] Unregistered drone: {drone.name} (remaining: {_registeredDrones.Count})");
        }

        // ============================================================================
        // Selection
        // ============================================================================

        public int GetDroneIndex(QuadPawn drone) => _registeredDrones.IndexOf(drone);

        public void SelectDroneByIndex(int index, bool alsoPossess = false)
        {
            if (index < 0 || index >= _registeredDrones.Count) return;

            QuadPawn target = _registeredDrones[index];
            if (target == null) return;

            _selectedDroneIndex = index;

            if (alsoPossess)
            {
                target.ForceFPVCameraActive();
            }

            Debug.Log($"[DroneManager] Selected drone: {target.name} (index {index})");
        }

        public void SelectNextDrone()
        {
            if (_registeredDrones.Count == 0) return;
            int next = (_selectedDroneIndex + 1) % _registeredDrones.Count;
            SelectDroneByIndex(next, true);
        }

        public void SelectPreviousDrone()
        {
            if (_registeredDrones.Count == 0) return;
            int prev = (_selectedDroneIndex - 1 + _registeredDrones.Count) % _registeredDrones.Count;
            SelectDroneByIndex(prev, true);
        }

        // ============================================================================
        // Configuration
        // ============================================================================

        public void SetSelectedConfig(string configName)
        {
            if (string.IsNullOrEmpty(configName)) return;
            selectedConfigName = configName;
            Debug.Log($"[DroneManager] Selected config: {configName}");
        }

        public void SetSceneEnv(int seed, bool noise, double lat, double lon, double alt)
        {
            masterSeed = seed; enableSensorNoise = noise;
            originLat = lat; originLon = lon; originAlt = alt;
        }
        
        public List<string> GetAvailableConfigs()
        {
            return DroneConfigLoader.GetAvailableConfigs();
        }

        public void SaveCurrentConfigAsDefault()
        {
            if (string.IsNullOrEmpty(selectedConfigName)) return;
            PlayerPrefs.SetString("QuadSim.DefaultConfig", selectedConfigName);
            PlayerPrefs.Save();
            Debug.Log($"[DroneManager] Saved default config: {selectedConfigName}");
        }

        public void ReloadCurrentDrone(bool resetPosition = true)
        {
            QuadPawn oldDrone = SelectedDrone;
            if (oldDrone == null)
            {
                Debug.LogWarning("[DroneManager] No drone selected to reload");
                return;
            }

            Vector3 pos = resetPosition ? defaultSpawnOrigin : oldDrone.transform.position;
            Quaternion rot = resetPosition ? Quaternion.identity : oldDrone.transform.rotation;

            Destroy(oldDrone.gameObject);
            CleanupInvalidDrones();

            SpawnDroneWithConfig(selectedConfigName, pos, rot);
        }

        // ============================================================================
        // Reset
        // ============================================================================

        public void ResetAllDrones()
        {
            for (int i = 0; i < _registeredDrones.Count; i++)
            {
                QuadPawn drone = _registeredDrones[i];
                if (drone == null) continue;

                Vector3 pos = defaultSpawnOrigin + Vector3.right * (i * spawnOffset);
                drone.ResetPosition(pos, Quaternion.identity);
            }

            Debug.Log($"[DroneManager] Reset {_registeredDrones.Count} drones");
        }

        public void ResetSelectedDrone()
        {
            QuadPawn drone = SelectedDrone;
            if (drone == null) return;
            drone.ResetPosition(defaultSpawnOrigin, Quaternion.identity);
        }

        // ============================================================================
        // Utility
        // ============================================================================

        private void CleanupInvalidDrones()
        {
            for (int i = _registeredDrones.Count - 1; i >= 0; i--)
            {
                if (_registeredDrones[i] == null)
                    _registeredDrones.RemoveAt(i);
            }
        }

        public string GetStateJson()
        {
            return $"{{\"drone_count\": {_registeredDrones.Count}, \"selected_index\": {_selectedDroneIndex}, \"swarm_mode\": {swarmMode.ToString().ToLower()}, \"config\": \"{selectedConfigName}\"}}";
        }
    }
}