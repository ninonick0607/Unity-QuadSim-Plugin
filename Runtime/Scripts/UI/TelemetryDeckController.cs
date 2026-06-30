using DroneCore.Controllers;
using RobotCore;
using UI.Core;
using UnityEngine;
using UnityEngine.UIElements;
// CascadedController

namespace UI
{
    [DisallowMultipleComponent]
    public sealed class TelemetryDeckController : MonoBehaviour
    {
        [Header("Classes")]
        [SerializeField] private string hiddenClass = "is-hidden";
        [SerializeField] private string activeClass = "active";

        [Header("Dock window sizing")]
        [SerializeField] private float initialDockThickness = 360f; // bottom/top height or left/right width
        [SerializeField] private float minWidth = 360f;
        [SerializeField] private float minHeight = 220f;

        private VisualElement _hudRoot;
        private VisualElement _drawer;

        private DockableWindowController _dock;
        private bool _initialized;

        // tabs
        private Button _tabGraphs, _tabState, _tabPID;
        private VisualElement _viewGraphs, _viewState, _viewPID;

        private Button _graphVel, _graphAngle, _graphAcro;
        private Button _btnClose;

        [Header("Telemetry sources")]
        [SerializeField] private SensorManager sensorManager;
        [SerializeField] private CascadedController cascadedController;
        
        [Header("Telemetry sampling (UI)")]
        [SerializeField] private float uiSampleHz = 30f;
        [SerializeField] private float graphWindowSec = 6f;

        // Graph UI elements (must exist in UXML)
        private TelemetryGraphElement _graphRoll;
        private TelemetryGraphElement _graphPitch;
        private TelemetryGraphElement _graphYaw;

        // Buffers: desired + measured for each axis
        private TelemetryRingBuffer _rollMeas, _rollDes;
        private TelemetryRingBuffer _pitchMeas, _pitchDes;
        private TelemetryRingBuffer _yawMeas, _yawDes;

        private IVisualElementScheduledItem _sampler;
        
        private enum GraphMode { Velocity, Angle, Acro }
        private GraphMode _mode = GraphMode.Acro;

        // Resize overlay + handles
        private VisualElement _resizeOverlay;
        private VisualElement _tl, _tr, _bl, _br, _l, _r, _t, _b;
        private VisualElement _titleBar;

        // STATE UI
        private ScrollView _stateScroll;
        private VisualElement _stateRoot;

        // IMU widgets
        private Label _imuValid, _imuTime, _imuFrame;
        private Label _imuAngVel, _imuAccel, _imuAtt, _imuVel;

        // GPS widgets
        private Label _gpsValid, _gpsTime;
        private Label _gpsPos;

        public void Initialize(VisualElement hudRoot)
        {
            _hudRoot = hudRoot;
            if (_hudRoot == null)
            {
                Debug.LogError("[Telemetry] Initialize called with null hudRoot.");
                return;
            }

            _drawer = _hudRoot.Q<VisualElement>("TelemetryRoot");
            //Debug.Log($"[Telemetry] classes: {string.Join(",", _drawer.GetClasses())}");
           // Debug.Log($"[Telemetry] bg resolved: {_drawer.resolvedStyle.backgroundColor}");

            if (_drawer == null)
            {
                Debug.LogError("[Telemetry] Could not find TelemetryRoot in UXML.");
                return;
            }

            // Query controls
            _btnClose = _drawer.Q<Button>("BtnTelemetryClose");

            _tabGraphs = _drawer.Q<Button>("TabGraphs");
            _tabState  = _drawer.Q<Button>("TabState");
            _tabPID    = _drawer.Q<Button>("TabPID");

            _viewGraphs = _drawer.Q<VisualElement>("GraphsView");
            _viewState  = _drawer.Q<VisualElement>("StateView");
            _viewPID    = _drawer.Q<VisualElement>("PIDView");
            _stateScroll = _drawer.Q<ScrollView>("StateContent");
            BindStateUI();
            _graphVel   = _drawer.Q<Button>("GraphTabVelocity");
            _graphAngle = _drawer.Q<Button>("GraphTabAngle");
            _graphAcro  = _drawer.Q<Button>("GraphTabAcro");

            // Dockable window elements
            _titleBar = _drawer.Q<VisualElement>("TelemetryTitleBar");
            if (_titleBar == null)
                Debug.LogError("[Telemetry] Missing TelemetryTitleBar in UXML (needed for dragging).");

            _resizeOverlay = _drawer.Q<VisualElement>("ResizeOverlay");

            _tl = _drawer.Q<VisualElement>("ResizeTL");
            _tr = _drawer.Q<VisualElement>("ResizeTR");
            _bl = _drawer.Q<VisualElement>("ResizeBL");
            _br = _drawer.Q<VisualElement>("ResizeBR");

            _l = _drawer.Q<VisualElement>("ResizeL");
            _r = _drawer.Q<VisualElement>("ResizeR");
            _t = _drawer.Q<VisualElement>("ResizeT");
            _b = _drawer.Q<VisualElement>("ResizeB");

            _dock = new DockableWindowController(_hudRoot, _drawer, _titleBar, _tl, _tr, _bl, _br, _l, _r, _t, _b);
            _dock.SetConstraints(minW: minWidth, minH: minHeight);
            _dock.SetInitialFloating(x: 40f, y: 80f, w: 620f, h: 380f);
            _dock.SetDockThickness(initialDockThickness);
            _dock.SetDock(DockSide.Bottom);
            _dock.ApplyLayout();

            // Keep overlay on top across any layout/panel changes
            _drawer.RegisterCallback<GeometryChangedEvent>(_ => BringResizeToFront());

            // Also do it once now
            ConfigureResizeOverlayPicking();
            BringResizeToFront();

            ResolveTelemetryRefsIfNeeded();
            SetupGraphs();
            StartSampler();

            // Wire behavior
            if (_btnClose != null) _btnClose.clicked += Hide;

            if (_tabGraphs != null) _tabGraphs.clicked += () => ActivateMainTab(_tabGraphs, _viewGraphs);
            if (_tabState  != null) _tabState.clicked  += () => ActivateMainTab(_tabState,  _viewState);
            if (_tabPID    != null) _tabPID.clicked    += () => ActivateMainTab(_tabPID,    _viewPID);

            if (_graphVel   != null) _graphVel.clicked   += () => ActivateSubTab(_graphVel);
            if (_graphAngle != null) _graphAngle.clicked += () => ActivateSubTab(_graphAngle);
            if (_graphAcro  != null) _graphAcro.clicked  += () => ActivateSubTab(_graphAcro);

            _initialized = true;
        }

        public void Toggle()
        {
            if (!_initialized) return;
            if (_drawer.ClassListContains(hiddenClass)) Show();
            else Hide();
        }

        public void Show()
        {
            if (!_initialized) return;
            _drawer.RemoveFromClassList(hiddenClass);
            _dock?.ApplyLayout();
        }

        public void Hide()
        {
            if (!_initialized) return;
            _drawer.AddToClassList(hiddenClass);
        }
                
        private void BindStateUI()
        {
            _imuValid  = _drawer.Q<Label>("ImuValid");
            _imuTime   = _drawer.Q<Label>("ImuTime");
            _imuFrame  = _drawer.Q<Label>("ImuFrame");
            _imuAngVel = _drawer.Q<Label>("ImuAngVel");
            _imuAccel  = _drawer.Q<Label>("ImuAccel");
            _imuAtt    = _drawer.Q<Label>("ImuAtt");
            _imuVel    = _drawer.Q<Label>("ImuVel");

            _gpsValid  = _drawer.Q<Label>("GpsValid");
            _gpsTime   = _drawer.Q<Label>("GpsTime");
            _gpsPos    = _drawer.Q<Label>("GpsPos");
        }

        private void ConfigureResizeOverlayPicking()
        {
            if (_resizeOverlay != null)
                _resizeOverlay.pickingMode = PickingMode.Ignore;

            // Ensure handles are pickable
            _tl?.SetEnabled(true); _tr?.SetEnabled(true); _bl?.SetEnabled(true); _br?.SetEnabled(true);
            _l?.SetEnabled(true);  _r?.SetEnabled(true);  _t?.SetEnabled(true);  _b?.SetEnabled(true);

            if (_tl != null) _tl.pickingMode = PickingMode.Position;
            if (_tr != null) _tr.pickingMode = PickingMode.Position;
            if (_bl != null) _bl.pickingMode = PickingMode.Position;
            if (_br != null) _br.pickingMode = PickingMode.Position;
            if (_l  != null) _l.pickingMode  = PickingMode.Position;
            if (_r  != null) _r.pickingMode  = PickingMode.Position;
            if (_t  != null) _t.pickingMode  = PickingMode.Position;
            if (_b  != null) _b.pickingMode  = PickingMode.Position;
        }

        
        private void BringResizeToFront()
        {
            // The overlay being last in UXML already helps, but UI Toolkit can reorder
            // picking/drawing under some layouts. This keeps it deterministic.
            _resizeOverlay?.BringToFront();

            // Optional extra reinforcement:
            _tl?.BringToFront();
            _tr?.BringToFront();
            _bl?.BringToFront();
            _br?.BringToFront();
            _l?.BringToFront();
            _r?.BringToFront();
            _t?.BringToFront();
            _b?.BringToFront();
        }

        private void ActivateMainTab(Button clicked, VisualElement viewToShow)
        {
            if (clicked == null) return;

            SetActive(_tabGraphs, clicked == _tabGraphs);
            SetActive(_tabState,  clicked == _tabState);
            SetActive(_tabPID,    clicked == _tabPID);

            SetHidden(_viewGraphs, viewToShow != _viewGraphs);
            SetHidden(_viewState,  viewToShow != _viewState);
            SetHidden(_viewPID,    viewToShow != _viewPID);

            _dock?.ApplyLayout();
        }
        
        private void ActivateSubTab(Button clicked)
        {
            SetActive(_graphVel,   clicked == _graphVel);
            SetActive(_graphAngle, clicked == _graphAngle);
            SetActive(_graphAcro,  clicked == _graphAcro);

            _mode = clicked == _graphVel ? GraphMode.Velocity
                : clicked == _graphAngle ? GraphMode.Angle
                : GraphMode.Acro;

            ClearGraphBuffers();
        }

        private void ClearGraphBuffers()
        {
            _rollMeas?.Clear(); _rollDes?.Clear();
            _pitchMeas?.Clear(); _pitchDes?.Clear();
            _yawMeas?.Clear(); _yawDes?.Clear();
        }

        private void SetActive(Button b, bool active)
        {
            if (b == null) return;
            if (active) b.AddToClassList(activeClass);
            else b.RemoveFromClassList(activeClass);
        }

        private void SetHidden(VisualElement ve, bool hidden)
        {
            if (ve == null) return;
            if (hidden) ve.AddToClassList(hiddenClass);
            else ve.RemoveFromClassList(hiddenClass);
        }
        
        
        private void ResolveTelemetryRefsIfNeeded()
        {
            if (sensorManager == null) sensorManager = Object.FindFirstObjectByType<SensorManager>();
            if (cascadedController == null) cascadedController = Object.FindFirstObjectByType<CascadedController>();
        }

        private void SetupGraphs()
        {

            _graphRoll  = _drawer.Q<TelemetryGraphElement>("GraphRoll");
            _graphPitch = _drawer.Q<TelemetryGraphElement>("GraphPitch");
            _graphYaw   = _drawer.Q<TelemetryGraphElement>("GraphYaw");

            if (_graphRoll == null || _graphPitch == null || _graphYaw == null)
            {
                Debug.LogError("[Telemetry] Missing GraphRoll/GraphPitch/GraphYaw elements. Add them to UXML as TelemetryGraphElement.");
                return;
            }

            int capacity = Mathf.CeilToInt(graphWindowSec * uiSampleHz) + 64;

            _rollMeas  = new TelemetryRingBuffer(capacity);
            _rollDes   = new TelemetryRingBuffer(capacity);
            _pitchMeas = new TelemetryRingBuffer(capacity);
            _pitchDes  = new TelemetryRingBuffer(capacity);
            _yawMeas   = new TelemetryRingBuffer(capacity);
            _yawDes    = new TelemetryRingBuffer(capacity);

            float yMin = -300f, yMax = 300f;
            _graphRoll.SetView(graphWindowSec, yMin, yMax);
            _graphPitch.SetView(graphWindowSec, yMin, yMax);
            _graphYaw.SetView(graphWindowSec, yMin, yMax);

            _graphRoll.ClearSeries();
            _graphRoll.AddSeries(new TelemetryGraphElement.Series
            {
                Label = "Roll Measured",
                Color = new Color(0.25f, 0.8f, 1.0f, 1f),
                Count = () => _rollMeas.Count,
                GetTime = i => _rollMeas.GetTime(i),
                GetValue = i => _rollMeas.GetValue(i)
            });
            _graphRoll.AddSeries(new TelemetryGraphElement.Series
            {
                Label = "Roll Desired",
                Color = new Color(1.0f, 0.65f, 0.2f, 1f),
                Count = () => _rollDes.Count,
                GetTime = i => _rollDes.GetTime(i),
                GetValue = i => _rollDes.GetValue(i)
            });

            _graphPitch.ClearSeries();
            _graphPitch.AddSeries(new TelemetryGraphElement.Series
            {
                Label = "Pitch Measured",
                Color = new Color(0.25f, 0.8f, 1.0f, 1f),
                Count = () => _pitchMeas.Count,
                GetTime = i => _pitchMeas.GetTime(i),
                GetValue = i => _pitchMeas.GetValue(i)
            });
            _graphPitch.AddSeries(new TelemetryGraphElement.Series
            {
                Label = "Pitch Desired",
                Color = new Color(1.0f, 0.65f, 0.2f, 1f),
                Count = () => _pitchDes.Count,
                GetTime = i => _pitchDes.GetTime(i),
                GetValue = i => _pitchDes.GetValue(i)
            });

            _graphYaw.ClearSeries();
            _graphYaw.AddSeries(new TelemetryGraphElement.Series
            {
                Label = "Yaw Measured",
                Color = new Color(0.25f, 0.8f, 1.0f, 1f),
                Count = () => _yawMeas.Count,
                GetTime = i => _yawMeas.GetTime(i),
                GetValue = i => _yawMeas.GetValue(i)
            });
            _graphYaw.AddSeries(new TelemetryGraphElement.Series
            {
                Label = "Yaw Desired",
                Color = new Color(1.0f, 0.65f, 0.2f, 1f),
                Count = () => _yawDes.Count,
                GetTime = i => _yawDes.GetTime(i),
                GetValue = i => _yawDes.GetValue(i)
            });
        }

        private void StartSampler()
        {
            if (_hudRoot == null) return;
            Debug.Log($"[Telemetry] Sampler starting at {uiSampleHz} Hz. hudRoot panel? {(_hudRoot.panel != null)}");

            void StartNow()
            {
                float hz = Mathf.Max(1f, uiSampleHz);
                long ms = Mathf.RoundToInt(1000f / hz);

                _sampler?.Pause();
                _sampler = _hudRoot.schedule.Execute(SampleTelemetry).Every(ms);
            }

            if (_hudRoot.panel != null) StartNow();
            else _hudRoot.RegisterCallback<AttachToPanelEvent>(_ => StartNow());
        }


        private void SampleTelemetry()
        {
            ResolveTelemetryRefsIfNeeded();
            if (sensorManager == null || cascadedController == null) return;

            var s = sensorManager.Latest;
            if (!s.ImuValid) return;

            float t = (float)s.ImuTimestampSec;
            if (t <= 0f) t = Time.realtimeSinceStartup;
            CascadedController.CascadeTelemetrySnapshot TelemetrySnap = cascadedController.TelemetrySnapshot;
            switch (_mode)
            {
                case GraphMode.Acro:
                {
                    // measured body rates (rad/s) -> deg/s
                    Vector3 measDeg = s.ImuAngVel * Mathf.Rad2Deg;

                    
                    //TODO: Make sure its synced up correctly, Rad2Deg or Deg2Rag or none?
                    // desired body rates (rad/s) -> deg/s
                    Vector3 desDeg = TelemetrySnap.DesiredRatesDeg;

                    _rollMeas.Push(t,  measDeg.x); _rollDes.Push(t,  desDeg.x);
                    _pitchMeas.Push(t, measDeg.y); _pitchDes.Push(t, desDeg.y);
                    _yawMeas.Push(t,   measDeg.z); _yawDes.Push(t,   desDeg.z);
                    break;
                }

                case GraphMode.Angle:
                {
                    // measured attitude in degrees (use signed -180..180)
                    Vector3 measEuler = NormalizeEulerSigned(s.ImuAttitude);

                    // desired angles: you do NOT currently expose this from your controller.
                    // Until you do, set desired to 0 so you can at least see measured change.
                    Vector3 desEuler = Vector3.zero;

                    _rollMeas.Push(t,  measEuler.x); _rollDes.Push(t,  desEuler.x);
                    _pitchMeas.Push(t, measEuler.y); _pitchDes.Push(t, desEuler.y);
                    _yawMeas.Push(t,   measEuler.z); _yawDes.Push(t,   desEuler.z);
                    break;
                }

                case GraphMode.Velocity:
                {
                    // measured velocity (m/s) in OutputFrame already
                    Vector3 meas = s.ImuVel;

                    // desired velocity: not currently exposed; placeholder 0
                    Vector3 des = Vector3.zero;

                    _rollMeas.Push(t,  meas.x); _rollDes.Push(t,  des.x);
                    _pitchMeas.Push(t, meas.y); _pitchDes.Push(t, des.y);
                    _yawMeas.Push(t,   meas.z); _yawDes.Push(t,   des.z);
                    break;
                }
            }

            _graphRoll?.MarkDirtyRepaint();
            _graphPitch?.MarkDirtyRepaint();
            _graphYaw?.MarkDirtyRepaint();
            UpdateStateUI(sensorManager.Latest);

            
        }
        
        private void UpdateStateUI(SensorData s)
        {
            ResolveTelemetryRefsIfNeeded();

            // IMU
            SetPill(_imuValid, s.ImuValid, trueText: "VALID", falseText: "NO IMU");
            if (_imuTime  != null) _imuTime.text  = s.ImuTimestampSec.ToString("F3");
            if (_imuFrame != null) _imuFrame.text = sensorManager != null ? sensorManager.outputFrame.ToString() : "--";

            if (_imuAngVel != null)
            {
                Vector3 deg = s.ImuAngVel * Mathf.Rad2Deg;
                _imuAngVel.text = FormatVec3(deg, "F2");
            }

            if (_imuAccel != null) _imuAccel.text = FormatVec3(s.ImuAccel, "F3");
            if (_imuAtt   != null) _imuAtt.text   = FormatVec3(NormalizeEulerSigned(s.ImuAttitude), "F2");
            if (_imuVel   != null) _imuVel.text   = FormatVec3(s.ImuVel, "F3");

            // GPS
            SetPill(_gpsValid, s.GpsValid, trueText: "FIX", falseText: "NO FIX");
            if (_gpsTime != null) _gpsTime.text = s.GpsTimestampSec.ToString("F3");
            if (_gpsPos  != null) _gpsPos.text  = FormatVec3(s.GpsPosition, "F3");
        }

        private static void SetPill(Label pill, bool ok, string trueText, string falseText)
        {
            if (pill == null) return;

            pill.text = ok ? trueText : falseText;

            pill.RemoveFromClassList("state-pill--ok");
            pill.RemoveFromClassList("state-pill--bad");
            pill.AddToClassList(ok ? "state-pill--ok" : "state-pill--bad");
        }
        
        private static string FormatVec3(Vector3 v, string fmt)
            => $"({v.x.ToString(fmt)}, {v.y.ToString(fmt)}, {v.z.ToString(fmt)})";

        
        
        private void PushRates(float t, Vector3 meas, Vector3 des)
        {
            _rollMeas.Push(t, meas.x);  _rollDes.Push(t, des.x);
            _pitchMeas.Push(t, meas.y); _pitchDes.Push(t, des.y);
            _yawMeas.Push(t, meas.z);   _yawDes.Push(t, des.z);
        }

        private static Vector3 NormalizeEulerSigned(Vector3 eulerDeg)
        {
            eulerDeg.x = Wrap180(eulerDeg.x);
            eulerDeg.y = Wrap180(eulerDeg.y);
            eulerDeg.z = Wrap180(eulerDeg.z);
            return eulerDeg;

            static float Wrap180(float a)
            {
                a %= 360f;
                if (a > 180f) a -= 360f;
                if (a < -180f) a += 360f;
                return a;
            }
        }

    }
}
