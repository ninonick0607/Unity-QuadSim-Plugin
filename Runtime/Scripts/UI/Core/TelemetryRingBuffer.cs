using UnityEngine;

namespace UI.Core
{
    /// <summary>
    /// Fixed-size ring buffer for (time,value) samples.
    /// Oldest samples are overwritten.
    /// </summary>
    public sealed class TelemetryRingBuffer
    {
        private readonly float[] _t;
        private readonly float[] _v;
        private int _head;
        private int _count;

        public int Capacity => _t.Length;
        public int Count => _count;

        public TelemetryRingBuffer(int capacity)
        {
            capacity = Mathf.Max(8, capacity);
            _t = new float[capacity];
            _v = new float[capacity];
            _head = 0;
            _count = 0;
        }

        public void Clear()
        {
            _head = 0;
            _count = 0;
        }

        public void Push(float timeSec, float value)
        {
            _t[_head] = timeSec;
            _v[_head] = value;

            _head = (_head + 1) % Capacity;
            _count = Mathf.Min(_count + 1, Capacity);
        }

        // Access in chronological order: 0 = oldest, Count-1 = newest
        public float GetTime(int i)
        {
            int idx = (_head - _count + i);
            while (idx < 0) idx += Capacity;
            return _t[idx % Capacity];
        }

        public float GetValue(int i)
        {
            int idx = (_head - _count + i);
            while (idx < 0) idx += Capacity;
            return _v[idx % Capacity];
        }
    }
}