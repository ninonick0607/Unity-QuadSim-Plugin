using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Yaml.Drone
{
    public static class DroneConfigLoader
    {
        public static string GetConfigDirectory()
        {
            return Path.Combine(Application.streamingAssetsPath, "Configs", "Drones");
        }
        
        public static List<string> GetAvailableConfigs()
        {
            var results = new List<string>();

            string dir = GetConfigDirectory();
            if (!Directory.Exists(dir))
            {
                Debug.LogWarning($"[DroneConfigLoader] Config directory does not exist: {dir}");
                return results;
            }
            string[] files = Directory.GetFiles(dir, "*.yaml", SearchOption.TopDirectoryOnly);
            foreach (string file in files)
            {
                results.Add(Path.GetFileNameWithoutExtension(file));
            }

            return results;
        }
        public static bool LoadConfig(string ConfigName, out DroneConfig OutConfig)
        {
            OutConfig = new DroneConfig();

            string fileName = ConfigName + ".yaml";
            string fullPath = Path.Combine(GetConfigDirectory(), fileName);

            if (!File.Exists(fullPath))
            {
                Debug.LogError($"[DroneConfigLoader] Config file not found: {fullPath}");
                return false;
            }

            bool bSuccess = YamlParser.LoadDroneConfig(
                fullPath,
                out OutConfig.ModelParams,
                out OutConfig.DroneParams,
                out OutConfig.RotorParams,
                out OutConfig.CamParams,
                out OutConfig.FlightParams,
                out OutConfig.Angle,
                out OutConfig.Acro,
                out OutConfig.Velocity,
                out OutConfig.Position
            );

            if (bSuccess)
            {
                OutConfig.SourceFile = ConfigName;
                Debug.Log($"[DroneConfigLoader] Loaded drone config: {OutConfig.ModelParams.Name} ({ConfigName})");
            }

            return bSuccess;
        }

    }
}
