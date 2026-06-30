// Assets/Scripts/SimCore/FrequencyLimiter.cs

using System;

namespace SimCore
{
    /// <summary>
    /// Clock-driven frequency limiter (sim-time), similar to your UE FrequencyLimiter.
    /// Use this to run sensors/controllers at subrates relative to the base sim dt.
    /// </summary>
    public sealed class FrequencyLimiter
    {
        private readonly double _periodSec;
        private long _nextTickNanos;
        private bool _initialized;

        public double PeriodSec => _periodSec;

        public FrequencyLimiter(double rateHz, long startNowNanos = 0)
        {
            if (rateHz <= 0) throw new ArgumentOutOfRangeException(nameof(rateHz), "rateHz must be > 0.");
            _periodSec = 1.0 / rateHz;

            if (startNowNanos != 0)
            {
                Reset(startNowNanos);
            }
        }

        /// <summary>
        /// Returns true if it's time to run at the given nowNanos (does not advance schedule).
        /// </summary>
        public bool ShouldRun(long nowNanos)
        {
            if (!_initialized)
            {
                // First call: run immediately, then schedule next tick.
                _nextTickNanos = nowNanos;
                _initialized = true;
            }

            return nowNanos >= _nextTickNanos;
        }

        /// <summary>
        /// Call after you run. Schedules the next tick.
        /// Uses "catch-up" stepping to avoid drift if you miss periods.
        /// </summary>
        public void Consume(long nowNanos)
        {
            if (!_initialized)
            {
                _nextTickNanos = nowNanos;
                _initialized = true;
            }

            long periodNanos = (long)(_periodSec * 1.0e9);

            // If we're behind, jump forward by as many whole periods as needed (no drift).
            if (nowNanos >= _nextTickNanos)
            {
                long behind = nowNanos - _nextTickNanos;
                long periodsMissed = behind / periodNanos;
                _nextTickNanos += (periodsMissed + 1) * periodNanos;
            }
        }

        public void Reset(long nowNanos)
        {
            _initialized = true;
            _nextTickNanos = nowNanos;
        }

        /// <summary>
        /// Convenience: should run now, and if so consumes.
        /// </summary>
        public bool ShouldRunAndConsume(long nowNanos)
        {
            if (!ShouldRun(nowNanos)) return false;
            Consume(nowNanos);
            return true;
        }
    }
}
