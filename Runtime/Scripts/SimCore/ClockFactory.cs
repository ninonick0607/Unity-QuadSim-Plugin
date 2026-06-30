// Assets/Scripts/SimCore/ClockFactory.cs

using System;

namespace SimCore
{
    /// <summary>
    /// Central clock registry. Everything in the sim should reference time through here.
    /// </summary>
    public static class ClockFactory
    {
        private static SteppableClock _clock;
        private static readonly ScaledClock _scaled = new ScaledClock();
        private static bool _initialized;

        public static bool IsInitialized => _initialized;

        /// <summary>Fixed base dt for simulation (seconds). Default is 250 Hz.</summary>
        public static double BaseDtSec { get; private set; } = 1.0 / 250.0;

        /// <summary>Authoritative sim clock (fixed step).</summary>
        public static SteppableClock Clock
        {
            get
            {
                EnsureInitialized();
                return _clock;
            }
        }

        /// <summary>Pause/scale wrapper used by SimulationManager to decide how much sim-time to advance per frame.</summary>
        public static ScaledClock Scaled => _scaled;

        /// <summary>
        /// Initialize the sim clock system. Safe to call multiple times.
        /// If called again with different params, it will re-init and reset time.
        /// </summary>
        public static void Initialize(double baseHz = 250.0, long startNanos = 0, bool paused = false, double timeScale = 1.0)
        {
            if (baseHz <= 0) throw new ArgumentOutOfRangeException(nameof(baseHz), "baseHz must be > 0.");

            BaseDtSec = 1.0 / baseHz;
            _clock = new SteppableClock(BaseDtSec, startNanos);
            _clock.Reset(startNanos);

            _scaled.SetPaused(paused);
            _scaled.SetTimeScale(timeScale);

            _initialized = true;
        }

        public static void Reset(long? newStartNanos = null)
        {
            EnsureInitialized();
            _clock.Reset(newStartNanos);
        }

        /// <summary>Advance exactly one fixed step (dt = BaseDtSec).</summary>
        public static long StepOnce()
        {
            EnsureInitialized();
            return _clock.Step();
        }

        public static void SetPaused(bool paused)
        {
            _scaled.SetPaused(paused);
        }

        public static void SetTimeScale(double scale)
        {
            _scaled.SetTimeScale(scale);
        }

        private static void EnsureInitialized()
        {
            if (_initialized) return;
            // Default init: 250 Hz, start at 0, running
            Initialize();
        }
    }
}
