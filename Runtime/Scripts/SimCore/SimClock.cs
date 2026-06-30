// Assets/Scripts/SimCore/SimClock.cs

using System;

namespace SimCore
{
    /// <summary>
    /// Authoritative simulation clock. Use this everywhere (controllers, sensors, limiters, IO).
    /// Do not use UnityEngine.Time for simulation timing.
    /// </summary>
    public interface IClock
    {
        long NowNanos { get; }
        long StartNanos { get; }
        ulong StepCount { get; }

        /// <summary>Base fixed step size in seconds (e.g. 0.004 for 250 Hz).</summary>
        double StepSizeSec { get; }

        /// <summary>Advance exactly one fixed step.</summary>
        long Step();

        /// <summary>Advance by an arbitrary amount (used for scaled clocks / accumulators).</summary>
        long StepBy(double seconds);

        /// <summary>Reset the clock back to start (and clears step count).</summary>
        void Reset(long? newStartNanos = null);

        double ElapsedSince(long sinceNanos);
    }

    public abstract class ClockBase : IClock
    {
        private ulong _stepCount;

        public abstract long NowNanos { get; }
        public abstract long StartNanos { get; }
        public virtual double StepSizeSec => 0.0;
        public ulong StepCount => _stepCount;

        public virtual long Step()
        {
            _stepCount++;
            return NowNanos;
        }

        public virtual long StepBy(double seconds)
        {
            // default behavior: count as "a step" even if dt isn't fixed
            _stepCount++;
            return NowNanos;
        }

        public virtual void Reset(long? newStartNanos = null)
        {
            _stepCount = 0;
        }

        public double ElapsedSince(long sinceNanos)
        {
            return ElapsedBetween(NowNanos, sinceNanos);
        }

        public static double ElapsedBetween(long secondNanos, long firstNanos)
        {
            return (secondNanos - firstNanos) / 1.0e9;
        }

        protected static long AddTo(long tNanos, double dtSec)
        {
            // Clamp tiny negatives (can happen with scale/pause edge cases)
            if (dtSec < 0) dtSec = 0;
            return tNanos + (long)(dtSec * 1.0e9);
        }
    }

    /// <summary>
    /// Deterministic fixed-step clock. Default start time = 0 (recommended).
    /// </summary>
    public sealed class SteppableClock : ClockBase
    {
        private long _current;
        private long _start;
        private readonly double _stepSec;

        public SteppableClock(double stepSec, long startNanos = 0)
        {
            if (stepSec <= 0) throw new ArgumentOutOfRangeException(nameof(stepSec), "Step must be > 0.");
            _stepSec = stepSec;

            _start = startNanos;
            _current = _start;
        }

        public override long NowNanos => _current;
        public override long StartNanos => _start;
        public override double StepSizeSec => _stepSec;

        public override long Step()
        {
            base.Step(); // increments StepCount
            _current = AddTo(_current, _stepSec);
            return _current;
        }

        public override long StepBy(double seconds)
        {
            base.StepBy(seconds);
            _current = AddTo(_current, seconds);
            return _current;
        }

        public override void Reset(long? newStartNanos = null)
        {
            base.Reset(newStartNanos);
            _start = newStartNanos ?? _start;
            _current = _start;
        }

        public void SetNowNanos(long nowNanos)
        {
            _current = nowNanos;
        }
    }

    /// <summary>
    /// A wrapper that can pause and apply time scaling to an underlying clock.
    /// This does NOT change physics dt; it changes how much sim time you choose to advance
    /// per rendered frame (your SimulationManager will decide how many base steps to run).
    /// </summary>
    public sealed class ScaledClock
    {
        public bool IsPaused { get; private set; }
        public double TimeScale { get; private set; } = 1.0;

        public void SetPaused(bool paused) => IsPaused = paused;

        public void SetTimeScale(double scale)
        {
            // Allow >1 for fast-forward; clamp negatives to 0.
            TimeScale = (scale < 0) ? 0 : scale;
        }

        /// <summary>
        /// Convert "real frame delta" to "sim seconds to advance" (used by SimulationManager accumulator).
        /// </summary>
        public double ComputeAdvanceSeconds(double realDeltaSeconds)
        {
            if (IsPaused) return 0.0;
            return realDeltaSeconds * TimeScale;
        }
    }
}
