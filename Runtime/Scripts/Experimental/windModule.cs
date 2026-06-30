using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;
using SimCore;

namespace Experimental
{
    /// <summary>
    /// A wind/fan volume. Applies velocity-relative drag toward the zone's air
    /// velocity to any non-kinematic Rigidbody inside it, exactly ONCE per
    /// rigidbody per physics step (OnTriggerStay fires once per collider, so
    /// multi-collider rigs would otherwise multiply the force).
    ///
    /// All timing is sim-time via ClockFactory — never UnityEngine.Time.
    /// Logs applied force per step to CSV for ground-truth overlay against
    /// the Python-side d_est / phi.
    /// </summary>
    [RequireComponent(typeof(BoxCollider))]
    public class WindModule : MonoBehaviour
    {
        [Header("Base Wind Settings")]
        [Tooltip("Fan jet air speed at the source face, m/s. Blow direction is local forward (blue Z axis).")]
        [SerializeField] private float _windSpeed = 10f;

        [Tooltip("Linear drag, N per (m/s) of relative air speed.")]
        [SerializeField] private float _dragCoefficient = 0.5f;

        [Header("Spreading & Cone Settings")]
        [Range(0f, 2f)] [SerializeField] private float _spreadFactor = 0f;

        [Header("Turbulence Settings")]
        [SerializeField] private float _turbulenceStrength = 2f;
        [SerializeField] private float _turbulenceFrequency = 5f;

        [SerializeField] private bool _toggle = false;

        [Header("Diagnostics")]
        [Tooltip("Write per-step applied-force CSV to <project>/runs/wind_log_*.csv")]
        [SerializeField] private bool _logForces = true;

        private BoxCollider _boxCollider;

        // --- per-step dedupe + logging state ---
        private ulong _lastStepCount = ulong.MaxValue;
        private readonly Dictionary<Rigidbody, PendingRow> _pendingRows = new();
        private StreamWriter _log;

        private struct PendingRow
        {
            public double SimTime;
            public ulong Step;
            public string Name;
            public Vector3 Pos;
            public float TravelRatio;
            public float WindScale;
            public Vector3 AirVel;
            public Vector3 RelVel;
            public Vector3 Force;
            public int CallbackCount;   // how many trigger callbacks hit this rb this step
        }

        private void Awake()
        {
            _boxCollider = GetComponent<BoxCollider>();

            // ---- Debug logging for exact dimensions and center/origin ----
            Vector3 worldSize = Vector3.Scale(_boxCollider.size, transform.lossyScale);
            Vector3 worldCenter = transform.TransformPoint(_boxCollider.center);
            
            Debug.Log($"[WindModule] Awake - Windzone Initialized.\n" +
                      $"Dimensions (World Size): X:{worldSize.x:F3}, Y:{worldSize.y:F3}, Z:{worldSize.z:F3}\n" +
                      $"Origin/Center (World Position): {worldCenter}\n" +
                      $"Collider Center (Local Offset): {_boxCollider.center}");

            if (_logForces)
            {
                string dir = Path.Combine(Application.dataPath, "../runs");
                Directory.CreateDirectory(dir);
                string path = Path.Combine(dir,
                    $"wind_log_{System.DateTime.Now:yyyyMMdd_HHmmss}.csv");
                _log = new StreamWriter(path, false, Encoding.ASCII);
                _log.WriteLine("step,sim_t,rb,px,py,pz,travel_ratio,wind_scale," +
                               "air_vx,air_vy,air_vz,rel_vx,rel_vy,rel_vz," +
                               "fx,fy,fz,f_mag,callbacks");
                Debug.Log($"[WindModule] Logging applied forces -> {path}");
            }
        }

        private void Reset()
        {
            var col = GetComponent<BoxCollider>();
            if (col != null) col.isTrigger = true;
        }

        private void OnDestroy()
        {
            FlushPendingRows();
            _log?.Flush();
            _log?.Dispose();
        }
        // ============================================================================
        // Runtime registry + control API
        //   Self-registering so the RPC layer never scans the scene. _toggle is the
        //   force gate (see OnTriggerStay); SetEnabled flips it live so one build can
        //   serve the wind-OFF gain study and the wind-ON net study without a rebuild.
        // ============================================================================

        private static readonly List<WindModule> _active = new();
        public static IReadOnlyList<WindModule> Active => _active;

        private void OnEnable()  { if (!_active.Contains(this)) _active.Add(this); }
        private void OnDisable() { _active.Remove(this); }

        public bool  IsEnabled => _toggle;
        public float WindSpeed => _windSpeed;

        /// <summary>Turn the disturbance force on/off at runtime.</summary>
        public void SetEnabled(bool on) => _toggle = on;

        /// <summary>Override fan jet speed (m/s) at runtime. Clamped non-negative.</summary>
        public void SetWindSpeed(float metersPerSecond) => _windSpeed = Mathf.Max(0f, metersPerSecond);
        private void OnTriggerStay(Collider other)
        {
            var rb = other.attachedRigidbody;
            if (rb == null || rb.isKinematic) return;

            // ---- per-step dedupe ----
            ulong step = ClockFactory.Clock.StepCount;
            if (step != _lastStepCount)
            {
                FlushPendingRows();          // write last step's rows (with final callback counts)
                _lastStepCount = step;
            }

            if (_pendingRows.TryGetValue(rb, out var existing))
            {
                // Force already applied to this rb this step — just count the duplicate.
                existing.CallbackCount++;
                _pendingRows[rb] = existing;
                return;
            }

            // ---- geometry (world-scale aware, collider-center aware) ----
            Vector3 worldSize = Vector3.Scale(_boxCollider.size, transform.lossyScale);
            float maxRange = Mathf.Max(worldSize.z, 1e-4f);

            Vector3 boxCenter = transform.TransformPoint(_boxCollider.center);
            Vector3 sourceFacePosition = boxCenter - transform.forward * (maxRange * 0.5f);

            Vector3 toTarget = rb.position - sourceFacePosition;
            float distanceAlongFlow = Mathf.Clamp(
                Vector3.Dot(toTarget, transform.forward), 0f, maxRange);
            float travelRatio = distanceAlongFlow / maxRange;

            float distanceFalloff = 1f - travelRatio;

            Vector3 centralProjectedPoint =
                sourceFacePosition + transform.forward * distanceAlongFlow;
            float lateralDistance = Vector3.Distance(rb.position, centralProjectedPoint);
            float spreadEdgeLoss =
                Mathf.Clamp01(1f - lateralDistance * _spreadFactor * travelRatio);

            float finalWindScale = distanceFalloff * spreadEdgeLoss;
            Vector3 airVelocity = transform.forward * (_windSpeed * finalWindScale);

            // ---- turbulence: SIM time, not wall time ----
            if (_turbulenceStrength > 0f && travelRatio > 0.05f)
            {
                float simT = (float)(ClockFactory.Clock.NowNanos / 1e9);
                float timeOffset = simT * _turbulenceFrequency;

                float noiseX = Mathf.PerlinNoise(rb.position.x + timeOffset, rb.position.y);
                float noiseY = Mathf.PerlinNoise(rb.position.y + timeOffset, rb.position.z);
                float noiseZ = Mathf.PerlinNoise(rb.position.z + timeOffset, rb.position.x);

                Vector3 turbDir = new Vector3(
                    noiseX * 2f - 1f, noiseY * 2f - 1f, noiseZ * 2f - 1f).normalized;

                airVelocity += turbDir * (_turbulenceStrength * travelRatio * spreadEdgeLoss);
            }

            // ---- apply (once per rb per step) ----
            Vector3 relVel = airVelocity - rb.linearVelocity;
            Vector3 force = _toggle ? _dragCoefficient * relVel : Vector3.zero;

            if (_toggle)
                rb.AddForce(force, ForceMode.Force);

            // ---- record row ----
            if (_log != null)
            {
                _pendingRows[rb] = new PendingRow
                {
                    SimTime = ClockFactory.Clock.NowNanos / 1e9,
                    Step = step,
                    Name = rb.name,
                    Pos = rb.position,
                    TravelRatio = travelRatio,
                    WindScale = finalWindScale,
                    AirVel = airVelocity,
                    RelVel = relVel,
                    Force = force,
                    CallbackCount = 1,
                };
            }
            else
            {
                // still need the dedupe key even when not logging
                _pendingRows[rb] = default;
            }
        }

        private void FlushPendingRows()
        {
            if (_log != null)
            {
                foreach (var kv in _pendingRows)
                {
                    var r = kv.Value;
                    _log.WriteLine(
                        $"{r.Step},{r.SimTime:F4},{r.Name}," +
                        $"{r.Pos.x:F4},{r.Pos.y:F4},{r.Pos.z:F4}," +
                        $"{r.TravelRatio:F4},{r.WindScale:F4}," +
                        $"{r.AirVel.x:F4},{r.AirVel.y:F4},{r.AirVel.z:F4}," +
                        $"{r.RelVel.x:F4},{r.RelVel.y:F4},{r.RelVel.z:F4}," +
                        $"{r.Force.x:F4},{r.Force.y:F4},{r.Force.z:F4}," +
                        $"{r.Force.magnitude:F4},{r.CallbackCount}");
                }
            }
            _pendingRows.Clear();
        }

        // ---- Always draw the windzone outline ----
        private void OnDrawGizmos()
        {
            if (_boxCollider == null) _boxCollider = GetComponent<BoxCollider>();
            if (_boxCollider == null) return;

            // Save the original matrix and color to restore later
            Matrix4x4 oldMatrix = Gizmos.matrix;
            Color oldColor = Gizmos.color;

            // Align the Gizmo drawing space with the Transform's local space (handles rotation/scale)
            Gizmos.matrix = transform.localToWorldMatrix;
            
            // Draw the box outline (semi-transparent green)
            Gizmos.color = new Color(0f, 1f, 0f, 0.4f);
            Gizmos.DrawWireCube(_boxCollider.center, _boxCollider.size);

            // Restore previous Gizmo states
            Gizmos.matrix = oldMatrix;
            Gizmos.color = oldColor;
        }

        // ---- Draw the internal directional rays only when selected ----
        private void OnDrawGizmosSelected()
        {
            if (_boxCollider == null) _boxCollider = GetComponent<BoxCollider>();
            if (_boxCollider == null) return;

            Vector3 worldSize = Vector3.Scale(_boxCollider.size, transform.lossyScale);
            float length = worldSize.z;
            Vector3 center = transform.TransformPoint(_boxCollider.center);
            Vector3 startPoint = center - transform.forward * (length * 0.5f);

            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(startPoint, transform.forward * length);

            Gizmos.color = new Color(0f, 1f, 1f, 0.25f);
            Gizmos.DrawWireSphere(startPoint + transform.forward * length,
                                  _spreadFactor * length * 0.5f);
        }
    }
}