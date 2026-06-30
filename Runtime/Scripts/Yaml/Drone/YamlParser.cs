using System;
using UnityEngine;
using YamlDotNet.RepresentationModel;
using System.Globalization;
using System.IO;
using System.Collections.Generic; // Required for List<>

namespace Yaml.Drone
{
    public class YamlParser
    {
        // ---------------------------------------------------------
        // Helpers
        // ---------------------------------------------------------
        
        public static bool ParseVector(YamlNode node, out Vector3 outVec)
        {
            outVec = Vector3.zero;
            if (node == null || !(node is YamlMappingNode mappingNode)) return false;

            if (TryGetFloat(mappingNode, "x", out float x)) outVec.x = x;
            if (TryGetFloat(mappingNode, "y", out float y)) outVec.y = y;
            if (TryGetFloat(mappingNode, "z", out float z)) outVec.z = z;
            return true;
        }

        public static bool ParsePID(YamlNode node, out PID outPID)
        {
            outPID = new PID(); // Assuming PID is a struct or class with empty constructor
            if (node == null || !(node is YamlMappingNode mappingNode)) return false;

            if (TryGetFloat(mappingNode, "p", out float p)) outPID.P = p;
            if (TryGetFloat(mappingNode, "i", out float i)) outPID.I = i;
            if (TryGetFloat(mappingNode, "d", out float d)) outPID.D = d;
            return true;
        }

        private static bool TryGetFloat(YamlMappingNode map, string key, out float result)
        {
            result = 0f;
            if (map.Children.TryGetValue(new YamlScalarNode(key), out YamlNode valueNode))
            {
                if (valueNode is YamlScalarNode scalar)
                    return float.TryParse(scalar.Value, NumberStyles.Float, CultureInfo.InvariantCulture, out result);
            }
            return false;
        }

        private static bool TryGetString(YamlMappingNode map, string key, out string result)
        {
            result = "";
            if (map.Children.TryGetValue(new YamlScalarNode(key), out YamlNode valueNode))
            {
                if (valueNode is YamlScalarNode scalar)
                {
                    result = scalar.Value;
                    return true;
                }
            }
            return false;
        }
        
        private static bool TryGetNode(YamlMappingNode map, string key, out YamlNode result)
        {
            return map.Children.TryGetValue(new YamlScalarNode(key), out result);
        }

        // ---------------------------------------------------------
        // Main Load Function
        // ---------------------------------------------------------

        public static bool LoadDroneConfig(
            string fullPath,
            out ModelType outModel,
            out DroneParameters outPhys, 
            out RotorModelInstance outProp, 
            out CamParameters outCam, 
            out FlightParameters outLimits,
            out AnglePID outAngle,
            out AcroPID outAcro,
            out VelocityPID outVel,
            out PositionPID outPos
        )      
        {
            // Initialize Defaults
            outModel = new ModelType();
            outPhys = new DroneParameters();
            outProp = new RotorModelInstance();
            outCam = new CamParameters();
            outLimits = new FlightParameters();
            outAngle = new AnglePID();
            outAcro = new AcroPID();
            outVel = new VelocityPID();
            outPos = new PositionPID();

            // File Check
            if (!File.Exists(fullPath))
            {
                Debug.LogError($"[Drone.YamlParser] File does not exist: {fullPath}");
                return false;
            }

            YamlMappingNode rootNode = null;

            // Load File
            try
            {
                using (var reader = new StreamReader(fullPath))
                {
                    var yaml = new YamlStream();
                    yaml.Load(reader);

                    if (yaml.Documents.Count == 0) return false;
                    if (!(yaml.Documents[0].RootNode is YamlMappingNode mapping)) return false;
            
                    rootNode = mapping;
                }
            }
            catch (System.Exception e) 
            {
                Debug.LogError($"YAML Load Exception: {e.Message}");
                return false;
            }

            // Parse Data
            try
            {
                // 1. Model Name & Mesh
                if (TryGetString(rootNode, "name", out string name)) outModel.Name = name;
                
                if (TryGetString(rootNode, "body_mesh", out string meshPath))
                {
                    // In Unity, you might load this from Resources or Addressables. 
                    // Storing the path string for now:
                    //outModel.BodyMeshPath = meshPath; 
                }

                // 2. Physical Properties
                if (TryGetNode(rootNode, "physical_properties", out YamlNode physNode) && physNode is YamlMappingNode phys)
                {
                    if (TryGetFloat(phys, "mass_kg", out float mass)) outPhys.Mass = mass;
                    
                    if (TryGetNode(phys, "com_offset", out YamlNode comNode)) 
                        ParseVector(comNode, out outPhys.CenterOfMass);
                    
                    if (TryGetNode(phys, "inertia", out YamlNode inertiaNode)) 
                        ParseVector(inertiaNode, out outPhys.Inertia);

                    if (TryGetFloat(phys, "arm_length", out float arm)) outPhys.Arm_Length = arm;
                    if (TryGetFloat(phys, "linear_drag_coef", out float lDrag)) outPhys.LinearDrag_Coeff = lDrag;
                    if (TryGetFloat(phys, "angular_drag_coef", out float aDrag)) outPhys.AngularDrag_Coeff = aDrag;
                }

                // 3. Propulsion
                if (TryGetNode(rootNode, "propulsion", out YamlNode propNode) && propNode is YamlMappingNode prop)
                {
                    // Model Params
                    if (TryGetFloat(prop, "prop_diameter", out float diam)) outProp.Model.Prop_Diameter = diam;
                    if (TryGetFloat(prop, "max_rpm", out float rpm)) outProp.Model.MaxRPM = rpm;
                    if (TryGetFloat(prop, "thrust_coef", out float thCo)) outProp.Model.ThrustCoeff = thCo;
                    if (TryGetFloat(prop, "torque_coef", out float toCo)) outProp.Model.TorqueCoeff = toCo;

                    // Propellers Array (Sequence)
                    if (TryGetNode(prop, "propellers", out YamlNode rotorsNode) && rotorsNode is YamlSequenceNode rotorSeq)
                    {
                        var rotorList = new List<RotorInstance>(rotorSeq.Children.Count);

                        foreach (YamlNode child in rotorSeq)
                        {
                            if (child is YamlMappingNode p)
                            {
                                RotorInstance r = new RotorInstance();

                                if (TryGetFloat(p, "id", out float id)) r.ID = (int)id;
                                if (TryGetFloat(p, "dir", out float dir)) r.Spin_Dir = (int)dir;

                                float rx = 0, ry = 0, rz = 0;
                                TryGetFloat(p, "x", out rx);
                                TryGetFloat(p, "y", out ry);
                                TryGetFloat(p, "z", out rz);
                                r.Location_Body = new Vector3(rx, ry, rz);

                                rotorList.Add(r);
                            }
                        }

                        outProp.Rotors = rotorList.ToArray();
                    }

                }

                // 4. Controller Gains
                if (TryGetNode(rootNode, "controller", out YamlNode ctrlNode) && ctrlNode is YamlMappingNode ctrl)
                {
                    // Position
                    if (TryGetNode(ctrl, "position_gains", out YamlNode posNode) && posNode is YamlMappingNode posMap)
                    {
                        if (TryGetNode(posMap, "x", out YamlNode px) && ParsePID(px, out PID tmpX)) outPos.PosX = tmpX;
                        if (TryGetNode(posMap, "y", out YamlNode py) && ParsePID(py, out PID tmpY)) outPos.PosY = tmpY;
                        if (TryGetNode(posMap, "z", out YamlNode pz) && ParsePID(pz, out PID tmpZ)) outPos.PosZ = tmpZ;
                    }
                    // Velocity
                    if (TryGetNode(ctrl, "velocity_gains", out YamlNode velNode) && velNode is YamlMappingNode velMap)
                    {
                        if (TryGetNode(velMap, "x", out YamlNode vx) && ParsePID(vx, out PID tmpX)) outVel.VelX = tmpX;
                        if (TryGetNode(velMap, "y", out YamlNode vy) && ParsePID(vy, out PID tmpY)) outVel.VelY = tmpY;
                        if (TryGetNode(velMap, "z", out YamlNode vz) && ParsePID(vz, out PID tmpZ)) outVel.VelZ = tmpZ;
                    }
                    // Attitude (Angle)
                    if (TryGetNode(ctrl, "attitude_gains", out YamlNode attNode) && attNode is YamlMappingNode attMap)
                    {
                        if (TryGetNode(attMap, "roll", out YamlNode ar) && ParsePID(ar, out PID tmpR)) outAngle.AngleRoll = tmpR;
                        if (TryGetNode(attMap, "pitch", out YamlNode ap) && ParsePID(ap, out PID tmpP)) outAngle.AnglePitch = tmpP;
                    }
                    // Rate (Acro)
                    if (TryGetNode(ctrl, "rate_gains", out YamlNode rateNode) && rateNode is YamlMappingNode rateMap)
                    {
                        if (TryGetNode(rateMap, "roll", out YamlNode rr) && ParsePID(rr, out PID tmpR)) outAcro.AcroRoll = tmpR;
                        if (TryGetNode(rateMap, "pitch", out YamlNode rp) && ParsePID(rp, out PID tmpP)) outAcro.AcroPitch = tmpP;
                        if (TryGetNode(rateMap, "yaw", out YamlNode ry) && ParsePID(ry, out PID tmpY)) outAcro.AcroYaw = tmpY;
                    }

                }

                // 5. Camera
                if (TryGetNode(rootNode, "camera_parameters", out YamlNode camNode) && camNode is YamlMappingNode cam)
                {
                    if (TryGetFloat(cam, "Cam_Arm_Length", out float arm)) outCam.CamArm_Length = arm;
                    if (TryGetFloat(cam, "FOV", out float fov)) outCam.FOV = fov;
                    if (TryGetFloat(cam, "Cam_Pitch", out float pitch)) outCam.CamPitch = pitch;

                    if (TryGetNode(cam, "Cam_Socket_Offset", out YamlNode offsetNode))
                    {
                        ParseVector(offsetNode, out outCam.CamSocket_Offset);
                    }
                }

                // 6. Limits
                if (TryGetNode(rootNode, "limits", out YamlNode limNode) && limNode is YamlMappingNode lim)
                {
                    if (TryGetFloat(lim, "max_velocity_xy", out float mv)) outLimits.MaxVelXY = mv;
                    if (TryGetFloat(lim, "max_velocity_z", out float mvz)) outLimits.MaxVelZ = mvz;
                    if (TryGetFloat(lim, "max_tilt_angle_deg", out float ma)) outLimits.MaxAngle = ma;
                    if (TryGetFloat(lim, "max_angular_rate_RP_deg", out float mr)) outLimits.MaxRateRollPitch = mr;
                    if (TryGetFloat(lim, "max_angular_rate_Yaw_deg", out float mry)) outLimits.MaxRateYaw = mry;
                    if (TryGetFloat(lim, "max_thrust_newtons", out float mt)) outLimits.MaxThrust = mt;
                    if (TryGetFloat(lim, "max_pid_output", out float mp)) outLimits.MaxPID = mp;
                    if (TryGetFloat(lim, "acceptable_distance", out float ad)) outLimits.AcceptableDistance = ad;
                }
                return true;
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Parsing Logic Error: {e.Message}\n{e.StackTrace}");
                return false;
            }
        }
    }
}