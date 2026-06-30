using DroneCore;
using UnityEngine;

namespace RobotCore
{
    [RequireComponent(typeof(QuadPawn))]
    [DisallowMultipleComponent]
    public sealed class RobotCore : MonoBehaviour
    {
        [SerializeField] private SensorManager sensors;

        public SensorManager Sensors => sensors;

        private void Awake()
        {
            if (sensors == null)
                sensors = GetComponentInChildren<SensorManager>(includeInactive: true);
        }
    }
}