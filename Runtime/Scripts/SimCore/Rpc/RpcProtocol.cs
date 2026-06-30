// Assets/Scripts/SimCore/Rpc/RpcProtocol.cs
// Phase 9a + 9b: Message Protocol Definitions
//
// PURPOSE:
//   Defines the wire protocol for External RPC communication.
//   All messages are MessagePack-encoded dictionaries.
//   This file provides serialization/deserialization helpers and
//   strongly-typed request/response wrappers.
//
// PROTOCOL:
//   Request:  { "method": "...", ...params }
//   Response: { "status": "ok"|"error", "data": {...}, "error_msg": "..." }
//
// NOTE ON TYPE COERCION (Phase 9b fix):
//   MessagePack-CSharp with ContractlessStandardResolver deserializes integers
//   into the smallest fitting C# type:
//     0..127      → byte
//     -32..-1     → sbyte
//     128..32767  → short (or int/long for larger)
//   The GetInt/GetFloat helpers must handle ALL numeric types, not just int/long.
//
// DEPENDENCIES:
//   - MessagePack-CSharp (NuGet: MessagePack)

using System;
using System.Collections.Generic;
using MessagePack;
using UnityEngine;

namespace SimCore.Rpc
{
    // ============================================================================
    // Wire-Level Types (MessagePack dictionaries)
    // ============================================================================

    /// <summary>
    /// Parsed request from the Python client.
    /// All fields are extracted from a MessagePack map.
    /// </summary>
    public readonly struct RpcRequest
    {
        /// <summary>The method name (e.g., "connect", "heartbeat", "set_mode").</summary>
        public readonly string Method;

        /// <summary>Raw parameters dictionary. Method handlers extract what they need.</summary>
        public readonly Dictionary<string, object> Params;

        /// <summary>Unique request ID for correlation (optional, client-assigned).</summary>
        public readonly long RequestId;

        public RpcRequest(string method, Dictionary<string, object> parameters, long requestId = 0)
        {
            Method = method;
            Params = parameters ?? new Dictionary<string, object>();
            RequestId = requestId;
        }

        /// <summary>Get a string parameter, or null if missing.</summary>
        public string GetString(string key)
        {
            if (Params.TryGetValue(key, out var val) && val is string s)
                return s;
            return null;
        }

        /// <summary>
        /// Get a float parameter, with default.
        /// Handles all numeric types that MessagePack may deserialize into.
        /// </summary>
        public float GetFloat(string key, float defaultValue = 0f)
        {
            if (!Params.TryGetValue(key, out var val)) return defaultValue;

            return val switch
            {
                float f   => f,
                double d  => (float)d,
                byte b    => b,
                sbyte sb  => sb,
                short s   => s,
                ushort us => us,
                int i     => i,
                uint ui   => ui,
                long l    => l,
                ulong ul  => ul,
                _ => defaultValue
            };
        }

        /// <summary>
        /// Get an int parameter, with default.
        /// Handles all numeric types that MessagePack may deserialize into.
        /// </summary>
        public int GetInt(string key, int defaultValue = 0)
        {
            if (!Params.TryGetValue(key, out var val)) return defaultValue;

            return val switch
            {
                byte b    => b,
                sbyte sb  => sb,
                short s   => s,
                ushort us => us,
                int i     => i,
                uint ui   => (int)ui,
                long l    => (int)l,
                ulong ul  => (int)ul,
                float f   => (int)f,
                double d  => (int)d,
                _ => defaultValue
            };
        }

        /// <summary>Get a bool parameter, with default.</summary>
        public bool GetBool(string key, bool defaultValue = false)
        {
            if (Params.TryGetValue(key, out var val) && val is bool b)
                return b;
            return defaultValue;
        }
    }

    /// <summary>
    /// Response to send back to the Python client.
    /// Serialized as a MessagePack map.
    /// </summary>
    public readonly struct RpcResponse
    {
        public readonly bool Success;
        public readonly string ErrorMessage;
        public readonly Dictionary<string, object> Data;
        public readonly long RequestId;

        private RpcResponse(bool success, string errorMessage,
                            Dictionary<string, object> data, long requestId)
        {
            Success = success;
            ErrorMessage = errorMessage;
            Data = data;
            RequestId = requestId;
        }

        public static RpcResponse Ok(long requestId = 0, Dictionary<string, object> data = null)
            => new RpcResponse(true, null, data, requestId);

        public static RpcResponse Error(string message, long requestId = 0)
            => new RpcResponse(false, message, null, requestId);
    }

    // ============================================================================
    // Serialization Helpers
    // ============================================================================

    /// <summary>
    /// Serialization utilities for the RPC wire protocol.
    /// Uses MessagePack with the typeless resolver for dictionary-based messages.
    /// </summary>
    public static class RpcSerializer
    {
        /// <summary>
        /// Deserialize a raw MessagePack byte[] into an RpcRequest.
        /// Expects a map with at least a "method" key.
        /// Returns null if deserialization fails.
        /// </summary>
        public static RpcRequest? DeserializeRequest(byte[] data)
        {
            try
            {
                var dict = MessagePackSerializer.Deserialize<Dictionary<string, object>>(
                    data, MessagePack.Resolvers.ContractlessStandardResolver.Options);

                if (dict == null || !dict.TryGetValue("method", out var methodObj) || methodObj is not string method)
                {
                    Debug.LogWarning("[RpcProtocol] Request missing 'method' field.");
                    return null;
                }

                long requestId = 0;
                if (dict.TryGetValue("request_id", out var idObj))
                {
                    requestId = idObj switch
                    {
                        byte b    => b,
                        sbyte sb  => sb,
                        short s   => s,
                        ushort us => us,
                        int i     => i,
                        uint ui   => ui,
                        long l    => l,
                        ulong ul  => (long)ul,
                        _ => 0
                    };
                }

                // Remove protocol fields, leave params
                dict.Remove("method");
                dict.Remove("request_id");

                return new RpcRequest(method, dict, requestId);
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"[RpcProtocol] Failed to deserialize request: {ex.Message}");
                return null;
            }
        }

        /// <summary>
        /// Serialize an RpcResponse to MessagePack bytes for sending over ZMQ.
        /// </summary>
        public static byte[] SerializeResponse(RpcResponse response)
        {
            var dict = new Dictionary<string, object>
            {
                ["status"] = response.Success ? "ok" : "error"
            };

            if (response.RequestId != 0)
                dict["request_id"] = response.RequestId;

            if (!response.Success && response.ErrorMessage != null)
                dict["error_msg"] = response.ErrorMessage;

            if (response.Data != null)
            {
                foreach (var kvp in response.Data)
                    dict[kvp.Key] = kvp.Value;
            }

            return MessagePackSerializer.Serialize(dict,
                MessagePack.Resolvers.ContractlessStandardResolver.Options);
        }

        /// <summary>
        /// Serialize a telemetry/sensor publish message.
        /// Topic prefix is prepended as a separate ZMQ frame by the caller.
        /// </summary>
        public static byte[] SerializePublish(string topic, Dictionary<string, object> data)
        {
            data["topic"] = topic;
            data["timestamp"] = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

            return MessagePackSerializer.Serialize(data,
                MessagePack.Resolvers.ContractlessStandardResolver.Options);
        }
    }
}