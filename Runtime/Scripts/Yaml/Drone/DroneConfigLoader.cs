using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Yaml.Drone
{
    public static class DroneConfigLoader
    {
        // User/project override location:
        //   Assets/StreamingAssets/Configs/Drones/<config>.yaml
        //
        // Package default location:
        //   Runtime/Resources/QuadSim/Configs/Drones/<config>.yaml
        //
        // Resources.Load paths intentionally omit the extension, so
        // Runtime/Resources/QuadSim/Configs/Drones/flightmare_quad.yaml is loaded as:
        //   Resources.Load<TextAsset>("QuadSim/Configs/Drones/flightmare_quad")
        private const string StreamingConfigFolder = "Configs/Drones";
        private const string PackageResourceFolder = "QuadSim/Configs/Drones";

        public static string GetConfigDirectory()
        {
            return Path.Combine(Application.streamingAssetsPath, "Configs", "Drones");
        }

        public static List<string> GetAvailableConfigs()
        {
            var results = new List<string>();
            var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

            // 1) User/project overrides from StreamingAssets.
            string dir = GetConfigDirectory();
            if (Directory.Exists(dir))
            {
                string[] files = Directory.GetFiles(dir, "*.yaml", SearchOption.TopDirectoryOnly);
                foreach (string file in files)
                {
                    string name = NormalizeConfigName(file);
                    if (seen.Add(name))
                        results.Add(name);
                }
            }

            // 2) Package defaults from Resources.
            TextAsset[] defaults = Resources.LoadAll<TextAsset>(PackageResourceFolder);
            foreach (TextAsset asset in defaults)
            {
                if (asset == null) continue;

                string name = NormalizeConfigName(asset.name);
                if (seen.Add(name))
                    results.Add(name);
            }

            if (results.Count == 0)
            {
                Debug.LogWarning(
                    "[DroneConfigLoader] No drone configs found.\n" +
                    $"  Checked project overrides: {dir}\n" +
                    $"  Checked package resources: Resources/{PackageResourceFolder}/*.yaml"
                );
            }

            return results;
        }

        public static bool LoadConfig(string configName, out DroneConfig outConfig)
        {
            outConfig = new DroneConfig();

            if (string.IsNullOrWhiteSpace(configName))
            {
                Debug.LogError("[DroneConfigLoader] Empty config name.");
                return false;
            }

            string cleanName = NormalizeConfigName(configName);

            // 1) User/project override. Checked first so users can tune or replace
            // package configs without modifying the installed package.
            string streamingPath = Path.Combine(GetConfigDirectory(), cleanName + ".yaml");
            if (File.Exists(streamingPath))
            {
                string yamlText;
                try
                {
                    yamlText = File.ReadAllText(streamingPath);
                }
                catch (Exception e)
                {
                    Debug.LogError($"[DroneConfigLoader] Failed to read config override: {streamingPath}\n{e}");
                    return false;
                }

                bool success = ParseYamlText(yamlText, streamingPath, cleanName, ref outConfig);
                if (success)
                    Debug.Log($"[DroneConfigLoader] Loaded project override config: {outConfig.ModelParams.Name} ({streamingPath})");

                return success;
            }

            // 2) Package default fallback. These live inside the package under:
            // Runtime/Resources/QuadSim/Configs/Drones/<name>.yaml
            string resourcePath = $"{PackageResourceFolder}/{cleanName}";
            TextAsset packageAsset = Resources.Load<TextAsset>(resourcePath);
            if (packageAsset != null)
            {
                string sourceLabel = $"Resources/{resourcePath}.yaml";
                bool success = ParseYamlText(packageAsset.text, sourceLabel, cleanName, ref outConfig);
                if (success)
                    Debug.Log($"[DroneConfigLoader] Loaded package default config: {outConfig.ModelParams.Name} ({sourceLabel})");

                return success;
            }

            Debug.LogError(
                "[DroneConfigLoader] Config file not found.\n" +
                $"  Requested: {configName}\n" +
                $"  Checked project override: {streamingPath}\n" +
                $"  Checked package default: Resources/{resourcePath}.yaml\n" +
                "  To override package defaults, create:\n" +
                $"  Assets/StreamingAssets/{StreamingConfigFolder}/{cleanName}.yaml"
            );

            return false;
        }

        private static bool ParseYamlText(string yamlText, string sourceLabel, string cleanName, ref DroneConfig outConfig)
        {
            bool success = YamlParser.LoadDroneConfigFromString(
                yamlText,
                sourceLabel,
                out outConfig.ModelParams,
                out outConfig.DroneParams,
                out outConfig.RotorParams,
                out outConfig.CamParams,
                out outConfig.FlightParams,
                out outConfig.Angle,
                out outConfig.Acro,
                out outConfig.Velocity,
                out outConfig.Position
            );

            if (success)
                outConfig.SourceFile = cleanName;

            return success;
        }

        private static string NormalizeConfigName(string configName)
        {
            string name = configName.Trim();
            name = Path.GetFileNameWithoutExtension(name);
            return name;
        }
    }
}
