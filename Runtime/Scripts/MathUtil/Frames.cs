using UnityEngine;

namespace MathUtil
{
    public enum SimFrame
    {
        UnityBody, // drone local: +X forward, +Y up, +Z left
        FLU,       // +X forward, +Y left, +Z up  (ROS)
        FRD        // +X forward, +Y right, +Z down (PX4)
    }

    public static class Frames
    {
        // ====================================================================
        // Public API — Outbound (Unity → Client Frame)
        // ====================================================================

        public static void GetTargetBasisInUnity(SimFrame target,
            out Vector3 xAxis, out Vector3 yAxis, out Vector3 zAxis)
        {
            switch (target)
            {
                case SimFrame.FLU: // X=forward, Y=left(+Z), Z=up(+Y)
                    xAxis = new Vector3(1f, 0f, 0f);
                    yAxis = new Vector3(0f, 0f, 1f);
                    zAxis = new Vector3(0f, 1f, 0f);
                    break;
                case SimFrame.FRD: // X=forward, Y=right(-Z), Z=down(-Y)
                    xAxis = new Vector3(1f, 0f, 0f);
                    yAxis = new Vector3(0f, 0f, -1f);
                    zAxis = new Vector3(0f, -1f, 0f);
                    break;
                default: // UnityBody
                    xAxis = new Vector3(1f, 0f, 0f);
                    yAxis = new Vector3(0f, 1f, 0f);
                    zAxis = new Vector3(0f, 0f, 1f);
                    break;
            }
        }
        
        public static Vector3 TransformLinear(Vector3 vBodyUnity, SimFrame target)
        {
            return target switch
            {
                SimFrame.FRD => UnityBodyToFRD_Linear(vBodyUnity),
                SimFrame.FLU => UnityBodyToFLU_Linear(vBodyUnity),
                _ => vBodyUnity
            };
        }

        public static float TransformBinary(float Unity, SimFrame target)
        {
            if (target == SimFrame.FRD || target == SimFrame.FLU)
                return -Unity;
            return Unity;
        }
        
        public static Vector3 TransformFlip(Vector3 Unity, SimFrame target)
        {
            if (target == SimFrame.FRD)
            {
                return new Vector3(0, 0, Unity.z);
            }
            return Unity;
        }

        public static Vector3 TransformAcceleration(Vector3 aBodyUnity, SimFrame target)
        {
            return target switch
            {
                SimFrame.FRD => UnityBodyToFRD_Accel(aBodyUnity),
                SimFrame.FLU => UnityBodyToFLU_Accel(aBodyUnity),
                _ => aBodyUnity
            };
        }

        public static Vector3 TransformAngularVelocity(Vector3 wBodyUnity, SimFrame target)
        {
            return target switch
            {
                SimFrame.FRD => UnityBodyToFRD_AngularRate(wBodyUnity),
                SimFrame.FLU => UnityBodyToFLU_AngularRate(wBodyUnity),
                _ => wBodyUnity
            };
        }
        
        public static Vector3 TransformAttitude(Vector3 rpyDegUnityBody, SimFrame target)
        {
            return target switch
            {
                SimFrame.FLU => new Vector3(
                    -rpyDegUnityBody.x,   // roll
                    -rpyDegUnityBody.y,   // pitch
                    rpyDegUnityBody.z     // yaw
                ),
                SimFrame.FRD => new Vector3(
                    rpyDegUnityBody.x,
                    rpyDegUnityBody.y,
                    -rpyDegUnityBody.z
                ),
                _ => rpyDegUnityBody
            };
        }

        /// <summary>
        /// Convert a world-space position from Unity (Y-up) to the target frame.
        /// Used for GPS position and any other world-frame vectors.
        ///
        /// Unity world: X=east(?), Y=up, Z=north(?)  — axis meaning depends on scene,
        /// but the Y↔Z swap for up-axis is always correct.
        ///
        /// FLU world: same horizontal axes, Z=up  →  (x, z, y)
        /// FRD world: same horizontal axes, Z=down → (x, -z, -y)
        /// </summary>
        public static Vector3 TransformWorldPosition(Vector3 posUnityWorld, SimFrame target)
        {
            return target switch
            {
                SimFrame.FLU => new Vector3(posUnityWorld.x, posUnityWorld.z, posUnityWorld.y),
                SimFrame.FRD => new Vector3(posUnityWorld.x, -posUnityWorld.z, -posUnityWorld.y),
                _ => posUnityWorld
            };
        }

        public static Quaternion TransformQuaternion(Quaternion q, SimFrame target)
        {
            return target switch
            {
                // Negate all spatial components to flip chirality, swap Y and Z
                SimFrame.FLU => new Quaternion(-q.x, -q.z, -q.y, q.w),
                _ => q 
            };
        }

        // ====================================================================
        // Public API — Inbound (Client Frame → Unity)
        // ====================================================================

        /// <summary>
        /// Convert a world-space position from client frame back to Unity (Y-up).
        /// Inverse of TransformWorldPosition.
        ///
        /// FLU (x,y,z) where Z=up  →  Unity (x, z, y) where Y=up
        /// FRD (x,y,z) where Z=down → Unity (x, -z, -y) where Y=up
        /// </summary>
        public static Vector3 InverseTransformWorldPosition(Vector3 posClient, SimFrame source)
        {
            return source switch
            {
                SimFrame.FLU => new Vector3(posClient.x, posClient.z, posClient.y),
                SimFrame.FRD => new Vector3(posClient.x, -posClient.z, -posClient.y),
                _ => posClient
            };
        }

        /// <summary>
        /// Convert a torque/angular-velocity vector from client frame back to Unity body.
        /// Inverse of TransformAngularVelocity.
        ///
        /// This is the critical chirality fix: FLU is right-handed, Unity is left-handed.
        /// TransformAngularVelocity (out) does: axis swap + negate all (LH→RH).
        /// The inverse (in) is the same operation: negate all + axis swap (RH→LH).
        /// Since negate-all commutes with axis swap, the formula is identical.
        ///
        /// FLU (x,y,z) → Unity (-x, -z, -y)   [negate all + swap Y↔Z]
        /// FRD (x,y,z) → Unity (x, -z, -y)     [swap Y↔Z, negate swapped pair]
        /// </summary>
        public static Vector3 InverseTransformAngularVelocity(Vector3 wClient, SimFrame source)
        {
            return source switch
            {
                SimFrame.FLU => new Vector3(-wClient.x, -wClient.z, -wClient.y),
                SimFrame.FRD => new Vector3(-wClient.x, wClient.z, wClient.y),
                _ => wClient
            };
        }

        /// <summary>
        /// Convert a quaternion from client frame back to Unity body frame.
        /// Inverse of TransformQuaternion.
        ///
        /// TransformQuaternion does:   q_target = Q_basis^-1 * q_unity * Q_basis
        /// Inverse is:                 q_unity  = Q_basis * q_target * Q_basis^-1
        /// </summary>
        public static Quaternion InverseTransformQuaternion(Quaternion qClient,
            SimFrame source)
        {
            return source switch
            {
                SimFrame.FLU => new Quaternion(-qClient.x, -qClient.z, -qClient.y,
                    qClient.w),
                _ => qClient
            };
        }
        
        /// <summary>
        /// Convert a body-frame linear vector from client frame back to Unity body.
        /// Inverse of TransformLinear.
        ///
        /// The axis swap (x,z,y) is its own inverse for FLU.
        /// FRD uses sign flips that are also self-inverse.
        /// </summary>
        public static Vector3 InverseTransformLinear(Vector3 vClient, SimFrame source)
        {
            return source switch
            {
                SimFrame.FLU => new Vector3(vClient.x, vClient.z, vClient.y),
                SimFrame.FRD => new Vector3(vClient.x, -vClient.z, -vClient.y),
                _ => vClient
            };
        }

        // ====================================================================
        // Implementation — UnityBody → FLU/FRD
        // ====================================================================
        //
        // UnityBody basis:
        //   X = forward, Y = up, Z = left
        //
        // FLU:
        //   X = forward, Y = left, Z = up
        //   Pure vectors: (x,y,z)_body → (x, z, y)_flu
        //
        // FRD:
        //   X = forward, Y = right = -left, Z = down = -up
        //   Pure vectors: (x,y,z)_body → (x, -z, -y)_frd

        // Linear / Acceleration: pure axis relabel
        private static Vector3 UnityBodyToFLU_Linear(Vector3 v)
            => new Vector3(v.x, v.z, v.y);
        private static Vector3 UnityBodyToFRD_Linear(Vector3 v)
            => new Vector3(v.x, -v.z, -v.y);


        // Angular velocity: relabel + negate all (LH → RH chirality correction)
        private static Vector3 UnityBodyToFLU_AngularRate(Vector3 w)
            => new Vector3(-w.x, -w.z, -w.y);

        private static Vector3 UnityBodyToFRD_AngularRate(Vector3 w)
            => new Vector3(-w.x, w.z, w.y);
        

        private static Vector3 UnityBodyToFLU_Accel(Vector3 a)
            => new Vector3(a.x, a.z, a.y);
        private static Vector3 UnityBodyToFRD_Accel(Vector3 a)
            => new Vector3(a.x, -a.z, -a.y);

        // ====================================================================
        // Quaternion basis-change helpers
        // ====================================================================

        private static Quaternion GetUnityFromTargetBasis(SimFrame target)
        {
            GetTargetBasisInUnity(target, out var x, out var y, out var z);
            return QuaternionFromBasis(x, y, z);
        }

        private static Quaternion QuaternionFromBasis(Vector3 xAxisUnityBody, Vector3 yAxisUnityBody, Vector3 zAxisUnityBody)
        {
            var m = new Matrix4x4();
            m.SetColumn(0, new Vector4(xAxisUnityBody.x, xAxisUnityBody.y, xAxisUnityBody.z, 0f));
            m.SetColumn(1, new Vector4(yAxisUnityBody.x, yAxisUnityBody.y, yAxisUnityBody.z, 0f));
            m.SetColumn(2, new Vector4(zAxisUnityBody.x, zAxisUnityBody.y, zAxisUnityBody.z, 0f));
            m.SetColumn(3, new Vector4(0f, 0f, 0f, 1f));
            return QuaternionFromMatrix(m);
        }

        private static Quaternion QuaternionFromMatrix(Matrix4x4 m)
        {
            float trace = m.m00 + m.m11 + m.m22;
            if (trace > 0f)
            {
                float s = Mathf.Sqrt(trace + 1f) * 2f;
                float invS = 1f / s;
                return new Quaternion(
                    (m.m21 - m.m12) * invS,
                    (m.m02 - m.m20) * invS,
                    (m.m10 - m.m01) * invS,
                    0.25f * s
                );
            }

            if (m.m00 > m.m11 && m.m00 > m.m22)
            {
                float s = Mathf.Sqrt(1f + m.m00 - m.m11 - m.m22) * 2f;
                float invS = 1f / s;
                return new Quaternion(
                    0.25f * s,
                    (m.m01 + m.m10) * invS,
                    (m.m02 + m.m20) * invS,
                    (m.m21 - m.m12) * invS
                );
            }

            if (m.m11 > m.m22)
            {
                float s = Mathf.Sqrt(1f + m.m11 - m.m00 - m.m22) * 2f;
                float invS = 1f / s;
                return new Quaternion(
                    (m.m01 + m.m10) * invS,
                    0.25f * s,
                    (m.m12 + m.m21) * invS,
                    (m.m02 - m.m20) * invS
                );
            }

            float sZ = Mathf.Sqrt(1f + m.m22 - m.m00 - m.m11) * 2f;
            float invSZ = 1f / sZ;
            return new Quaternion(
                (m.m02 + m.m20) * invSZ,
                (m.m12 + m.m21) * invSZ,
                0.25f * sZ,
                (m.m10 - m.m01) * invSZ
            );
        }
    }
}