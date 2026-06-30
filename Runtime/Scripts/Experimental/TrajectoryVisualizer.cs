using System;
using System.Collections.Generic;
using UnityEngine;
using MathUtil;
using SimCore.Rpc;

namespace Experimental
{
    [DisallowMultipleComponent]
    public sealed class TrajectoryVisualizer : MonoBehaviour
    {
        [SerializeField] private ExternalRpcAdapter adapter; // assign in inspector
        [SerializeField] private LineRenderer pathLine;      // static ghost path
        [SerializeField] private Transform desiredGhost;     // moving desired pos
        [SerializeField] private LineRenderer velVector;     // optional (null ok)
        [SerializeField] private LineRenderer thrustVector;  // optional (null ok)
        [SerializeField] private float vectorScale = 0.05f;

        private void Start()
        {
            if (adapter == null)
            {
                Debug.LogError("[Viz] No ExternalRpcAdapter assigned.");
                enabled = false;
                return;
            }
            adapter.RegisterHandler("draw_trajectory", HandleDrawTrajectory);
            adapter.OnVizPayload += UpdateDesired;
        }

        private void OnDestroy()
        {
            if (adapter != null) adapter.OnVizPayload -= UpdateDesired;
        }

        // One-shot during setup: full path as FLU points.
        private RpcResponse HandleDrawTrajectory(RpcRequest req)
        {
            if (!req.Params.TryGetValue("points", out var raw) || raw is not object[] pts)
                return RpcResponse.Error("invalid_param: points", req.RequestId);

            var positions = new Vector3[pts.Length];
            for (int i = 0; i < pts.Length; i++)
                positions[i] = FluWorldToUnity(ToVec3(pts[i]));

            pathLine.positionCount = positions.Length;
            pathLine.SetPositions(positions);
            return RpcResponse.Ok(req.RequestId);
        }

        // Per-tick, fired off the step_with_command round trip.
        private void UpdateDesired(Dictionary<string, object> p)
        {
            if (!p.TryGetValue("viz_pos", out var posRaw)) return;  // non-viz run

            Vector3 pos = FluWorldToUnity(ToVec3(posRaw));
            if (desiredGhost != null) desiredGhost.position = pos;

            if (velVector != null && p.TryGetValue("viz_vel", out var v))
                DrawArrow(velVector, pos, FluWorldToUnity(ToVec3(v)));
            if (thrustVector != null && p.TryGetValue("viz_bz", out var bz))
                DrawArrow(thrustVector, pos, FluWorldToUnity(ToVec3(bz)));
        }

        private void DrawArrow(LineRenderer lr, Vector3 origin, Vector3 dirUnity)
        {
            lr.positionCount = 2;
            lr.SetPosition(0, origin);
            lr.SetPosition(1, origin + dirUnity * vectorScale);
        }

        // Single frame boundary. Positions AND world-direction vectors both use
        // the pure FLU->Unity world relabel (x,y,z)->(x,z,y) from Frames.cs.
        private static Vector3 FluWorldToUnity(Vector3 flu)
            => Frames.InverseTransformWorldPosition(flu, SimFrame.FLU);

        // Robust to MessagePack boxing (object[] or IList, any numeric type).
        private static Vector3 ToVec3(object o)
        {
            if (o is object[] a && a.Length >= 3)
                return new Vector3(Convert.ToSingle(a[0]), Convert.ToSingle(a[1]), Convert.ToSingle(a[2]));
            if (o is System.Collections.IList l && l.Count >= 3)
                return new Vector3(Convert.ToSingle(l[0]), Convert.ToSingle(l[1]), Convert.ToSingle(l[2]));
            return Vector3.zero;
        }
    }
}