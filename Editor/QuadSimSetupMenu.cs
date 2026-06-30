using System.IO;
using UnityEditor;
using UnityEngine;

namespace QuadSim.EditorTools
{
    public static class QuadSimSetupMenu
    {
        [MenuItem("Tools/QuadSim/Create Basic Scene Objects")]
        public static void CreateBasicSceneObjects()
        {
            var simRoot = new GameObject("SimRoot");
            simRoot.AddComponent<SimCore.SimulationManager>();
            simRoot.AddComponent<SimCore.ControlAuthorityManager>();
            simRoot.AddComponent<SimCore.SimCoreApi>();

            Debug.Log("[QuadSim] Created SimRoot with SimulationManager, ControlAuthorityManager, and SimCoreApi.");
        }

        [MenuItem("Tools/QuadSim/Create StreamingAssets Folder")]
        public static void CreateStreamingAssetsFolder()
        {
            string path = "Assets/StreamingAssets/QuadSim/DroneModels";
            Directory.CreateDirectory(path);
            AssetDatabase.Refresh();

            Debug.Log($"[QuadSim] Created {path}");
        }
    }
}
