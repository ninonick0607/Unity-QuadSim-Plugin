using UnityEngine;

namespace DroneCore.Common
{
    [System.Serializable]
    public sealed class QuadPIDController
    {
        public float Kp = 0f;
        public float Ki = 0f;
        public float Kd = 0f;

        public float MinOutput = -1f;
        public float MaxOutput = 1f;

        public float IntegralLimit = 10f;
        public float DecayFactor = 0.99f;

        private float _integralSum = 0f;
        private float _prevError = 0f;
        private bool _hasLastError = false;

        public void SetGains(float p, float i, float d) { Kp = p; Ki = i; Kd = d; }
        public void SetLimits(float min, float max) { MinOutput = min; MaxOutput = max; }
        //public void SetIntegralLimits(float limit) { IntegralLimit = Mathf.Abs(limit); }
        public void SetDecayFactor(float decay) { DecayFactor = Mathf.Clamp01(decay); }

        public void Reset()
        {
            _integralSum = 0f;
            _prevError = 0f;
            _hasLastError = false;
        }

        public float Calculate(float desired, float measured, float dt)
        {
            if (dt <= Mathf.Epsilon) return 0f;

            float error = desired - measured;

            // P
            float pTerm = Kp * error;

            // I (decay + clamp)
            _integralSum = DecayFactor * _integralSum + error * dt;
            _integralSum = Mathf.Clamp(_integralSum, -IntegralLimit, IntegralLimit);
            float iTerm = Ki * _integralSum;

            // D
            float dTerm = 0f;
            if (_hasLastError)
            {
                float derivative = (error - _prevError) / dt;
                dTerm = Kd * derivative;
            }
            else
            {
                _hasLastError = true;
            }

            _prevError = error;

            // Combine + clamp
            float output = pTerm + iTerm + dTerm;
            output = Mathf.Clamp(output, MinOutput, MaxOutput);

            // Back-calculation anti-windup
            if (Ki > Mathf.Epsilon)
            {
                float maxITerm = MaxOutput - pTerm - dTerm;
                float minITerm = MinOutput - pTerm - dTerm;
                float clampedITerm = Mathf.Clamp(iTerm, minITerm, maxITerm);

                if (!Mathf.Approximately(iTerm, clampedITerm))
                    _integralSum = clampedITerm / Ki;
            }

            return output;
        }
    }
}
