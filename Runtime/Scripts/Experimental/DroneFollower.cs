using DroneCore;
using DroneCore.Core;
using UnityEngine;

namespace Experimental
{
    public class DroneFollower : MonoBehaviour
    {
        [SerializeField] private float height = 50f;
        [SerializeField] private Vector3 offset;

        private void LateUpdate()
        {
            DroneManager manager = DroneManager.Get();

            if (manager == null)
                return;

            QuadPawn drone = manager.SelectedDrone;

            if (drone == null)
                return;

            Vector3 dronePos = drone.transform.position;

            transform.position = new Vector3(
                dronePos.x + offset.x,
                height + offset.y,
                dronePos.z + offset.z
            );

            transform.rotation = Quaternion.Euler(90f, 0f, 0f);
        }
    }
}