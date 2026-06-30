using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

namespace UI.Core
{
    /// <summary>
    /// Lightweight time-series plotter for UI Toolkit.
    /// Expects time in seconds (monotonic), values in chosen units.
    /// </summary>
    public sealed partial class TelemetryGraphElement : VisualElement
    {
        [Obsolete("Obsolete")]
        public new class UxmlFactory : UxmlFactory<TelemetryGraphElement, VisualElement.UxmlTraits> { }

        public struct Series
        {
            public string Label;
            public Color Color;
            public Func<int> Count;
            public Func<int, float> GetTime;
            public Func<int, float> GetValue;
        }

        
        
        private readonly List<Series> _series = new();
        
        private bool _autoScroll = true;
        private float _timeOffsetSec = 0f;

        public TelemetryGraphElement()
        {
            generateVisualContent += OnGenerateVisualContent;
            pickingMode = PickingMode.Ignore;
            RegisterCallback<WheelEvent>(OnWheel);
        }
        
        private float _timeZoom = 1f;   // 1 = default window
        private float _valueZoom = 1f;  // 1 = default range
        private float _valuePan = 0f;   // for later
        private float _baseTimeWindowSec = 5f;
        private float _baseYMin = -200f;
        private float _baseYMax = 200f;
        
        public void SetView(float timeWindowSec, float yMin, float yMax)
        {
            _baseTimeWindowSec = Mathf.Max(0.1f, timeWindowSec);
            _baseYMin = yMin;
            _baseYMax = Mathf.Max(yMin + 1e-3f, yMax);
            MarkDirtyRepaint();
        }
        private void OnWheel(WheelEvent evt)
        {
            // UI Toolkit wheel delta is inverted compared to some systems; tune sign to taste.
            float d = evt.delta.y;

            const float step = 0.12f;

            if (evt.ctrlKey)
            {
                // Ctrl+wheel: vertical zoom (value scale)
                _valueZoom = Mathf.Clamp(_valueZoom * (1f + d * step * 0.01f), 0.1f, 20f);
                evt.StopPropagation();
                MarkDirtyRepaint();
                return;
            }

            if (evt.altKey)
            {
                // Alt+wheel: time zoom (x scale)
                _timeZoom = Mathf.Clamp(_timeZoom * (1f + d * step * 0.01f), 0.1f, 20f);
                evt.StopPropagation();
                MarkDirtyRepaint();
                return;
            }

            // otherwise, let ScrollView handle normal scrolling
        }

        public void SetAutoScroll(bool enabled) => _autoScroll = enabled;

        // Optional for future: allow history offset
        public void SetTimeOffset(float offsetSec)
        {
            _timeOffsetSec = offsetSec;
            MarkDirtyRepaint();
        }

        public void ClearSeries() => _series.Clear();

        public void AddSeries(Series s) => _series.Add(s);
        private bool _showLegend = true;
        public void SetShowLegend(bool show) { _showLegend = show; MarkDirtyRepaint(); }

        private void OnGenerateVisualContent(MeshGenerationContext ctx)
        {
            var r = contentRect;
            if (r.width <= 2 || r.height <= 2) return;

            // Background (simple solid)
            var bg = new Rect(r.x, r.y, r.width, r.height);
            DrawSolidRect(ctx, bg, new Color(0.02f, 0.02f, 0.03f, 0.9f));

            float latestT = 0f;
            for (int si = 0; si < _series.Count; si++)
            {
                int n = _series[si].Count();
                if (n > 0) latestT = Mathf.Max(latestT, _series[si].GetTime(n - 1));
            }

            float timeWindow = _baseTimeWindowSec * _timeZoom;
            float endT = latestT + (_autoScroll ? 0f : _timeOffsetSec);
            float startT = endT - timeWindow;

            float yMin = _baseYMin;
            float yMax = _baseYMax;

            float yMid = (yMin + yMax) * 0.5f;
            float half = (yMax - yMin) * 0.5f * _valueZoom;

            float zyMin = yMid - half + _valuePan;
            float zyMax = yMid + half + _valuePan;
            float zyRange = Mathf.Max(1e-6f, zyMax - zyMin);
            
            if (_showLegend && _series.Count > 0)
            {
                float x = r.xMin + 8f;
                float y = r.yMin + 8f;

                for (int i = 0; i < _series.Count; i++)
                {
                    var s = _series[i];

                    // colored swatch line
                    DrawLine(ctx, new Vector2(x, y + 6f), new Vector2(x + 16f, y + 6f), s.Color, 2f);

                    // label
                    ctx.painter2D.fillColor = Color.white;
                    //ctx.painter2D.lineWidth = 11;
                    // Painter2D text APIs vary by Unity version; safest is Label overlay.
                    // So instead: store legend text and draw via IMGUIContainer OR use a Label in UXML.
                }
            }

            
            if (zyMin < 0f && zyMax > 0f)
            {
                float yZeroRatio = (0f - zyMin) / zyRange;
                float yZero = Mathf.Lerp(r.yMax, r.yMin, yZeroRatio);
                DrawLine(ctx, new Vector2(r.xMin, yZero), new Vector2(r.xMax, yZero),
                    new Color(1f, 1f, 1f, 0.15f), 1.0f);
            }
            
            // Series polylines
            for (int si = 0; si < _series.Count; si++)
            {
                var s = _series[si];
                int n = s.Count();
                if (n < 2) continue;

                // Build points in view window
                // (Avoid allocations if you care later; keep it simple for now.)
                List<Vector2> pts = new List<Vector2>(256);

                for (int i = 0; i < n; i++)
                {
                    float t = s.GetTime(i);
                    if (t < startT) continue;
                    if (t > endT) break;

                    float v = s.GetValue(i);

                    float rx = (t - startT) / timeWindow;
                    float ry = (v - zyMin) / zyRange;

                    float x = Mathf.Lerp(r.xMin, r.xMax, rx);
                    float y = Mathf.Lerp(r.yMax, r.yMin, ry);

                    pts.Add(new Vector2(x, y));
                }

                if (pts.Count >= 2)
                    DrawPolyline(ctx, pts, s.Color, 2.0f);
            }
        }

        private static void DrawSolidRect(MeshGenerationContext ctx, Rect rect, Color color)
        {
            var p = ctx.painter2D;
            p.fillColor = color;
            p.BeginPath();
            p.MoveTo(new Vector2(rect.xMin, rect.yMin));
            p.LineTo(new Vector2(rect.xMax, rect.yMin));
            p.LineTo(new Vector2(rect.xMax, rect.yMax));
            p.LineTo(new Vector2(rect.xMin, rect.yMax));
            p.ClosePath();
            p.Fill();
        }

        private static void DrawLine(MeshGenerationContext ctx, Vector2 a, Vector2 b, Color color, float width)
        {
            var p = ctx.painter2D;
            p.strokeColor = color;
            p.lineWidth = width;
            p.BeginPath();
            p.MoveTo(a);
            p.LineTo(b);
            p.Stroke();
        }

        private static void DrawPolyline(MeshGenerationContext ctx, List<Vector2> pts, Color color, float width)
        {
            var p = ctx.painter2D;
            p.strokeColor = color;
            p.lineWidth = width;
            p.BeginPath();
            p.MoveTo(pts[0]);
            for (int i = 1; i < pts.Count; i++) p.LineTo(pts[i]);
            p.Stroke();
        }
    }
}
