#if UNITY_EDITOR
using UnityEditor;
#endif
using DroneCore.Common;
using DroneCore.Controllers;
using DroneCore.Core;
using DroneCore.Interfaces;
using DroneCore.Api;
using UnityEngine;

namespace DroneCore
{
    /// <summary>
    /// Editor-time and runtime component assembly helper.
    /// Ensures all required components are present on a drone prefab/GameObject.
    /// 
    /// This is a "prefab builder" - it doesn't handle runtime lifecycle,
    /// that's QuadPawn's job.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class DroneRootBootstrap : MonoBehaviour
    {
#if UNITY_EDITOR
        private bool _buildQueued;
#endif

        private void Reset() => Build();
        
        private void Awake()
        {
            // Runtime only; OnValidate handles editor-time.
            if (Application.isPlaying) Build();
        }

#if UNITY_EDITOR
        private void OnValidate()
        {
            if (Application.isPlaying) return;
            if (_buildQueued) return;

            _buildQueued = true;
            EditorApplication.delayCall += () =>
            {
                _buildQueued = false;
                if (this == null) return;
                if (Application.isPlaying) return;
                Build();
            };
        }
#endif

        [ContextMenu("Build DroneRoot")]
        public void Build()
        {
            // DEDUPE FIRST (important - prevents multiple components)
            RemoveDuplicates<Rigidbody>(gameObject);
            RemoveDuplicates<QuadPawn>(gameObject);
            RemoveDuplicates<CascadedController>(gameObject);
            RemoveDuplicates<RobotCore.SensorManager>(gameObject);
            RemoveDuplicates<RobotCore.RobotCore>(gameObject);
            RemoveDuplicates<FlightCommandProxy>(gameObject);
            RemoveDuplicates<ModeCoordinator>(gameObject);
            RemoveDuplicates<QuadSimApi>(gameObject);
            // Add required components
            var rb       = GetOrAdd<Rigidbody>(gameObject);
            var body     = GetOrAdd<QuadPawn>(gameObject);
            var ctrl     = GetOrAdd<CascadedController>(gameObject);
            var sensors  = GetOrAdd<RobotCore.SensorManager>(gameObject);
            var core     = GetOrAdd<RobotCore.RobotCore>(gameObject);
            var cmd      = GetOrAdd<FlightCommandProxy>(gameObject);
            var modeCoord = GetOrAdd<ModeCoordinator>(gameObject);
            var api = GetOrAdd<QuadSimApi>(gameObject);

            // Configure Rigidbody defaults
            rb.mass = 1.28f;
            rb.useGravity = true;
            rb.isKinematic = false;
            rb.interpolation = RigidbodyInterpolation.None;
            rb.collisionDetectionMode = CollisionDetectionMode.Discrete;
            rb.maxAngularVelocity = 100f; // Prevent Unity from clamping angular velocity

            Debug.Log($"[DroneRootBootstrap] Built drone components on {gameObject.name}");
        }

        private static void RemoveDuplicates<T>(GameObject go) where T : Component
        {
            var comps = go.GetComponents<T>();
            for (int i = 1; i < comps.Length; i++)
            {
#if UNITY_EDITOR
                if (!Application.isPlaying)
                    Object.DestroyImmediate(comps[i]);
                else
                    Object.Destroy(comps[i]);
#else
                Object.Destroy(comps[i]);
#endif
            }
        }

        private static T GetOrAdd<T>(GameObject go) where T : Component
        {
            if (!go.TryGetComponent<T>(out var c))
                c = go.AddComponent<T>();
            return c;
        }
    }
}