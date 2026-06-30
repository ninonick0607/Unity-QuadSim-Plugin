using UnityEngine;
using UnityEngine.UIElements;

namespace UI
{
    /// <summary>
    /// The compositor/controller for the whole HUD.
    /// Owns wiring between TopBar and drawers.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class SimHUDController : MonoBehaviour
    {
        [Header("Scene refs")]
        [SerializeField] private UIDocument document;

        [Header("Child controllers (same GO)")]
        [SerializeField] private SimTopBarController topBar;
        [SerializeField] private TelemetryDeckController telemetry;

        // NEW
        [SerializeField] private ControlDeckController control;

        private VisualElement _docRoot;
        private VisualElement _hudRoot;

        private void Awake()
        {
            if (document == null) document = GetComponent<UIDocument>();
            if (topBar == null) topBar = GetComponent<SimTopBarController>();
            if (telemetry == null) telemetry = GetComponent<TelemetryDeckController>();

            // NEW
            if (control == null) control = GetComponent<ControlDeckController>();

            if (document == null)
            {
                Debug.LogError("[SimHUD] Missing UIDocument.");
                enabled = false;
                return;
            }

            _docRoot = document.rootVisualElement;
            Debug.Log($"[SimHUD] root styleSheets = {_docRoot.styleSheets.count}");

            if (_docRoot == null)
            {
                Debug.LogError("[SimHUD] document.rootVisualElement is null.");
                enabled = false;
                return;
            }
            Debug.Log($"[SimHUD] UIDocument VTA = {(document.visualTreeAsset != null ? document.visualTreeAsset.name : "NULL")}");
            Debug.Log($"[SimHUD] docRoot childCount={document.rootVisualElement.childCount}");

            // MUST match: <ui:VisualElement name="HUDRoot" ...>
            _hudRoot = _docRoot.Q<VisualElement>("HUDRoot");
            if (_hudRoot == null)
            {
                Debug.LogError("[SimHUD] Could not find HUDRoot. Check Sim_Hud.uxml element name.");
                enabled = false;
                return;
            }

            ForceFullScreen(_hudRoot);

            // Initialize child controllers against the same HUD root
            if (telemetry != null) telemetry.Initialize(_hudRoot);
            if (control != null) control.Initialize(_hudRoot); // NEW
            if (topBar != null) topBar.Initialize(_hudRoot);

            // Wire TopBar -> drawers (TopBar does NOT FindFirstObjectByType).
            if (topBar != null && telemetry != null)
            {
                topBar.OnTelemetryPressed = telemetry.Toggle;
            }

            // NEW
            if (topBar != null && control != null)
            {
                topBar.OnControlPressed = control.Toggle;
            }
        }

        private static void ForceFullScreen(VisualElement ve)
        {
            ve.style.position = Position.Absolute;
            ve.style.left = 0;
            ve.style.right = 0;
            ve.style.top = 0;
            ve.style.bottom = 0;
        }
    }
}
