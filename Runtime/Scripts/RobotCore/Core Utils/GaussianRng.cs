namespace RobotCore.Core_Utils
{
    public sealed class GaussianRng
    {
        private readonly System.Random _rng;
        private bool   _hasSpare;
        private double _spare;

        public GaussianRng(int seed) { _rng = new System.Random(seed); }

        public float Next()
        {
            if (_hasSpare) { _hasSpare = false; return (float)_spare; }

            double u1, u2, s;
            do {
                u1 = 2.0 * _rng.NextDouble() - 1.0;
                u2 = 2.0 * _rng.NextDouble() - 1.0;
                s  = u1 * u1 + u2 * u2;
            } while (s >= 1.0 || s == 0.0);

            double mul = System.Math.Sqrt(-2.0 * System.Math.Log(s) / s);
            _spare    = u2 * mul;
            _hasSpare = true;
            return (float)(u1 * mul);
        }
    }
}