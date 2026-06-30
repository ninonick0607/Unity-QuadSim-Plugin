using UnityEngine;
using System;
using System.Collections;

namespace RobotCore.Core_Utils
{
    /// <summary>
    /// WGS84 georeferencing. Converts between a local ENU tangent frame (meters,
    /// relative to a configured geodetic origin) and geodetic coordinates
    /// (latitude/longitude in degrees, ellipsoidal height in meters).
    ///
    /// Mirrors the role of Unreal's AGeoReferencingSystem (flat-planet / ENU mode).
    /// Local "up" (U) is along the ellipsoid normal at the origin; for local-area
    /// flight that is effectively true vertical.
    ///
    /// All math is double precision — ECEF magnitudes are ~6.4e6 m and float would
    /// discard ~0.5 m of position.
    /// </summary>
    public class GeoReferencingSystem
    {
        // WGS84 ellipsoid
        private const double A  = 6378137.0;               // semi-major axis [m]
        private const double F  = 1.0 / 298.257223563;     // flattening
        private const double E2 = F * (2.0 - F);           // first eccentricity squared
        private const double B  = A * (1.0 - F);           // semi-minor axis [m]

        public double OriginLatDeg { get; private set; }
        public double OriginLonDeg { get; private set; }
        public double OriginHeight { get; private set; }    // ellipsoidal height [m]

        private double _x0, _y0, _z0;       // origin in ECEF
        private double _sinLat, _cosLat, _sinLon, _cosLon;
        
        public GeoReferencingSystem(double originLatDeg, double originLonDeg, double originHeightM)
            => SetOrigin(originLatDeg, originLonDeg, originHeightM);

        public void SetOrigin(double latDeg, double lonDeg, double heightM)
        {
            OriginLatDeg = latDeg;
            OriginLonDeg = lonDeg;
            OriginHeight = heightM;

            double lat = latDeg * Math.PI / 180.0;
            double lon = lonDeg * Math.PI / 180.0;
            _sinLat = Math.Sin(lat); _cosLat = Math.Cos(lat);
            _sinLon = Math.Sin(lon); _cosLon = Math.Cos(lon);

            GeodeticToEcef(latDeg, lonDeg, heightM, out _x0, out _y0, out _z0);
        }

        // --- Public conversions ------------------------------------------------

        /// <summary>Local ENU (m) -> geodetic (lat°, lon°, ellipsoidal height m).</summary>
        public void EnuToGeodetic(double e, double n, double u,
                                  out double latDeg, out double lonDeg, out double heightM)
        {
            EnuToEcef(e, n, u, out double x, out double y, out double z);
            EcefToGeodetic(x, y, z, out latDeg, out lonDeg, out heightM);
        }

        /// <summary>Geodetic -> local ENU (m).</summary>
        public void GeodeticToEnu(double latDeg, double lonDeg, double heightM,
                                  out double e, out double n, out double u)
        {
            GeodeticToEcef(latDeg, lonDeg, heightM, out double x, out double y, out double z);
            EcefToEnu(x, y, z, out e, out n, out u);
        }

        /// <summary>Exact ellipsoidal height [m] of a local ENU point (for the baro).</summary>
        public double EnuToEllipsoidalHeight(double e, double n, double u)
        {
            EnuToGeodetic(e, n, u, out _, out _, out double h);
            return h;
        }

        // --- ECEF <-> ENU ------------------------------------------------------

        private void EnuToEcef(double e, double n, double u, out double x, out double y, out double z)
        {
            x = _x0 + (-_sinLon * e) + (-_sinLat * _cosLon * n) + (_cosLat * _cosLon * u);
            y = _y0 + ( _cosLon * e) + (-_sinLat * _sinLon * n) + (_cosLat * _sinLon * u);
            z = _z0 +                  ( _cosLat          * n) + (_sinLat          * u);
        }

        private void EcefToEnu(double x, double y, double z, out double e, out double n, out double u)
        {
            double dx = x - _x0, dy = y - _y0, dz = z - _z0;
            e = (-_sinLon * dx) + (_cosLon * dy);
            n = (-_sinLat * _cosLon * dx) + (-_sinLat * _sinLon * dy) + (_cosLat * dz);
            u = ( _cosLat * _cosLon * dx) + ( _cosLat * _sinLon * dy) + (_sinLat * dz);
        }

        // --- Geodetic <-> ECEF -------------------------------------------------

        private static void GeodeticToEcef(double latDeg, double lonDeg, double h,
                                           out double x, out double y, out double z)
        {
            double lat = latDeg * Math.PI / 180.0, lon = lonDeg * Math.PI / 180.0;
            double sLat = Math.Sin(lat), cLat = Math.Cos(lat);
            double sLon = Math.Sin(lon), cLon = Math.Cos(lon);

            double N = A / Math.Sqrt(1.0 - E2 * sLat * sLat);
            x = (N + h) * cLat * cLon;
            y = (N + h) * cLat * sLon;
            z = (N * (1.0 - E2) + h) * sLat;
        }

        private static void EcefToGeodetic(double x, double y, double z,
                                           out double latDeg, out double lonDeg, out double h)
        {
            // Bowring closed-form (sub-mm near the surface, no iteration)
            double lon = Math.Atan2(y, x);
            double p   = Math.Sqrt(x * x + y * y);
            double th  = Math.Atan2(z * A, p * B);
            double sT  = Math.Sin(th), cT = Math.Cos(th);
            double ep2 = (A * A - B * B) / (B * B);   // second eccentricity squared

            double lat = Math.Atan2(z + ep2 * B * sT * sT * sT,
                                    p - E2 * A * cT * cT * cT);
            double sLat = Math.Sin(lat);
            double N = A / Math.Sqrt(1.0 - E2 * sLat * sLat);
            h = p / Math.Cos(lat) - N;

            latDeg = lat * 180.0 / Math.PI;
            lonDeg = lon * 180.0 / Math.PI;
        }
    }
}