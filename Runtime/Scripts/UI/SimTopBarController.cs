using System;
using SimCore;
using UnityEngine;
using UnityEngine.UIElements;

namespace UI
{
    [DisallowMultipleComponent]
    public sealed class SimTopBarController : MonoBehaviour
    {
        [Header("Optional refs")]
        [SerializeField] private SimulationManager sim;
        [SerializeField] private MonoBehaviour droneSpawner; // optional; supports SpawnDrone() or Spawn(string)

        // Exposed hooks (wired by SimHUDController)
        public Action OnSettingsPressed;
        public Action OnControlPressed;
        public Action OnTelemetryPressed;

        private Button _btnSettings;
        private Button _btnSlower;
        private Button _btnPlayPause;
        private Button _btnStep;
        private Button _btnFaster;
        private Button _btnReset;
        private Button _btnSpawn;
        private Button _btnTelemetry;
        private Button _btnInspector;

        private Label _lblSpeed;

        private DropdownField _spawnDropdown;
        private string _spawnSelection = "Drone";

        private float _currentSpeedScale = 1.0f;

        public void Initialize(VisualElement hudRoot)
        {
            if (sim == null) sim = FindFirstObjectByType<SimulationManager>();

            // IMPORTANT:
            // All queries are done from the provided hudRoot so this controller is decoupled from scene hierarchy.
            _btnSettings  = hudRoot.Q<Button>("BtnSettings");
            _btnSlower    = hudRoot.Q<Button>("BtnSlower");
            _btnPlayPause = hudRoot.Q<Button>("BtnPlayPause");
            _btnStep      = hudRoot.Q<Button>("BtnStep");
            _btnFaster    = hudRoot.Q<Button>("BtnFaster");
            _btnReset     = hudRoot.Q<Button>("BtnReset");
            _btnSpawn     = hudRoot.Q<Button>("BtnSpawn");
            _btnTelemetry = hudRoot.Q<Button>("BtnTelemetry");
            _btnInspector = hudRoot.Q<Button>("BtnInspector");
            _lblSpeed     = hudRoot.Q<Label>("LblSpeed");

            _spawnDropdown = hudRoot.Q<DropdownField>("SpawnDropdown");
            if (_spawnDropdown != null)
            {
                if (_spawnDropdown.choices == null || _spawnDropdown.choices.Count == 0)
                    _spawnDropdown.choices = new System.Collections.Generic.List<string> { "Drone", "Robot" };

                if (string.IsNullOrEmpty(_spawnDropdown.value))
                    _spawnDropdown.value = _spawnDropdown.choices[0];

                _spawnSelection = _spawnDropdown.value;

                _spawnDropdown.RegisterValueChangedCallback(evt =>
                {
                    if (!string.IsNullOrEmpty(evt.newValue))
                        _spawnSelection = evt.newValue;
                });
            }

            if (_btnSettings == null || _btnPlayPause == null || _lblSpeed == null)
                throw new Exception("[TopBar] UXML is missing expected named elements.");

            _btnStep.tooltip = "Advance one physics step";

            // Wire up
            if (_btnSettings != null)  _btnSettings.clicked += () => OnSettingsPressed?.Invoke();
            if (_btnTelemetry != null) _btnTelemetry.clicked += () => OnTelemetryPressed?.Invoke();
            if (_btnInspector != null) _btnInspector.clicked += () => OnControlPressed?.Invoke();

            if (_btnPlayPause != null) _btnPlayPause.clicked += TogglePause;
            if (_btnStep != null)      _btnStep.clicked += () =>
            {
                sim.SetPaused(true);
                sim.StepOnce();
            };
            if (_btnSlower != null)    _btnSlower.clicked += () => SetSpeedScale(_currentSpeedScale * 0.5f);
            if (_btnFaster != null)    _btnFaster.clicked += () => SetSpeedScale(_currentSpeedScale * 2.0f);

            if (_btnReset != null) _btnReset.clicked += ResetSim;
            if (_btnSpawn != null) _btnSpawn.clicked += SpawnDrone;

            RefreshLabels();
        }

        private void TogglePause()
        {
            if (sim == null) return;
            sim.SetPaused(!sim.IsPaused);
            RefreshLabels();
        }



        private void SetSpeedScale(float newScale)
        {
            if (sim == null) return;

            // Clamp to something reasonable
            _currentSpeedScale = Mathf.Clamp(newScale, 0.05f, 32.0f);
            ApplyTimeScale();
        }

        private void ResetSim()
        {
            if (sim == null) return;

            _currentSpeedScale = 1.0f;
            sim.SetRunMode(SimulationManager.RunMode.FreeRun);
            sim.SetPaused(false);

            ApplyTimeScale();
            sim.ResetSimulation();

            RefreshLabels();
        }

        private void SpawnDrone()
        {
            var selection = string.IsNullOrEmpty(_spawnSelection) ? "Drone" : _spawnSelection;

            if (droneSpawner == null)
            {
                Debug.LogWarning($"[TopBar] Spawn requested ('{selection}') but no spawner is assigned.");
                return;
            }

            var t = droneSpawner.GetType();

            // 1) SpawnDrone()
            var mSpawnDrone = t.GetMethod("SpawnDrone", Type.EmptyTypes);
            if (selection == "Drone" && mSpawnDrone != null)
            {
                mSpawnDrone.Invoke(droneSpawner, null);
                return;
            }

            // 2) Spawn(string kind)
            var mSpawnString = t.GetMethod("Spawn", new[] { typeof(string) });
            if (mSpawnString != null)
            {
                mSpawnString.Invoke(droneSpawner, new object[] { selection });
                return;
            }

            // 3) SpawnRobot() (optional convenience)
            var mSpawnRobot = t.GetMethod("SpawnRobot", Type.EmptyTypes);
            if (selection != "Drone" && mSpawnRobot != null)
            {
                mSpawnRobot.Invoke(droneSpawner, null);
                return;
            }

            Debug.LogWarning($"[TopBar] Spawner '{t.Name}' does not implement SpawnDrone(), Spawn(string), or SpawnRobot().");
        }

        private void ApplyTimeScale()
        {
            if (sim == null) return;
            sim.SetTimeScale(_currentSpeedScale);
            RefreshLabels();
        }

        private void RefreshLabels()
        {
            if (_lblSpeed != null)
                _lblSpeed.text = $"x{_currentSpeedScale:0.00}";

            if (_btnPlayPause != null && sim != null)
                _btnPlayPause.text = sim.IsPaused ? "RESUME" : "PAUSE";
        }
    }
}
