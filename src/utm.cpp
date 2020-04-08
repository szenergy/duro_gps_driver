// include folder headers
#include "UTM.h"

// Converts degrees to radians.
FLOAT CoordinateTransition::DegToRad(FLOAT deg)
{
    return (deg / 180.0 * pi);
}

// Determines the central meridian for the given UTM zone.
FLOAT CoordinateTransition::UTMCentralMeridian(int zone)
{
    FLOAT cmeridian;
    cmeridian = DegToRad(-183.0 + ((FLOAT)zone * 6.0));

    return cmeridian;
}

// Converts a latitude/longitude pair to x and y coordinates in the Transverse Mercator projection
void CoordinateTransition::MapLatLonToXY(FLOAT phi, FLOAT lambda, FLOAT lambda0, FLOAT &x, FLOAT &y)
{
    FLOAT N, nu2, ep2, t, t2, l;
    FLOAT l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;
    /* Precalculate ep2 */
    ep2 = (POW(sm_a, 2.0) - POW(sm_b, 2.0)) / POW(sm_b, 2.0);
    /* Precalculate nu2 */
    nu2 = ep2 * POW(COS(phi), 2.0);
    /* Precalculate N */
    N = POW(sm_a, 2.0) / (sm_b * SQRT(1 + nu2));
    /* Precalculate t */
    t = TAN(phi);
    t2 = t * t;
    /* Precalculate l */
    l = lambda - lambda0;

    /* Precalculate coefficients for l**n in the equations below
       so a normal human being can read the expressions for easting
       and northing
       -- l**1 and l**2 have coefficients of 1.0 */
    l3coef = 1.0 - t2 + nu2;
    l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
    l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
    l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
    l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
    l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);
    /* Calculate easting (x) */
    x = N * COS(phi) * l + (N / 6.0 * POW(COS(phi), 3.0) * l3coef * POW(l, 3.0)) + (N / 120.0 * POW(COS(phi), 5.0) * l5coef * POW(l, 5.0)) + (N / 5040.0 * POW(COS(phi), 7.0) * l7coef * POW(l, 7.0));
    /* Calculate northing (y) */
    y = ArcLengthOfMeridian(phi) + (t / 2.0 * N * POW(COS(phi), 2.0) * POW(l, 2.0)) + (t / 24.0 * N * POW(COS(phi), 4.0) * l4coef * POW(l, 4.0)) + (t / 720.0 * N * POW(COS(phi), 6.0) * l6coef * POW(l, 6.0)) + (t / 40320.0 * N * POW(COS(phi), 8.0) * l8coef * POW(l, 8.0));
    return;
}

// Computes the ellipsoidal distance from the equator to a point at a given latitude.
FLOAT CoordinateTransition::ArcLengthOfMeridian(FLOAT phi)
{
    FLOAT alpha, beta, gamma, delta, epsilon, n;
    FLOAT result;
    /* Precalculate n */
    n = (sm_a - sm_b) / (sm_a + sm_b);
    /* Precalculate alpha */
    alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (POW(n, 2.0) / 4.0) + (POW(n, 4.0) / 64.0));
    /* Precalculate beta */
    beta = (-3.0 * n / 2.0) + (9.0 * POW(n, 3.0) / 16.0) + (-3.0 * POW(n, 5.0) / 32.0);
    /* Precalculate gamma */
    gamma = (15.0 * POW(n, 2.0) / 16.0) + (-15.0 * POW(n, 4.0) / 32.0);
    /* Precalculate delta */
    delta = (-35.0 * POW(n, 3.0) / 48.0) + (105.0 * POW(n, 5.0) / 256.0);
    /* Precalculate epsilon */
    epsilon = (315.0 * POW(n, 4.0) / 512.0);
    /* Now calculate the sum of the series and return */
    result = alpha * (phi + (beta * SIN(2.0 * phi)) + (gamma * SIN(4.0 * phi)) + (delta * SIN(6.0 * phi)) + (epsilon * SIN(8.0 * phi)));
    return result;
}

// Converts a latitude/longitude pair to x and y coordinates in the Universal Transverse Mercator projection.
int CoordinateTransition::LatLonToUTMXY(FLOAT lat, FLOAT lon, int zone, FLOAT &x, FLOAT &y)
{
    if ((zone < 1) || (zone > 60))
        zone = FLOOR((lon + 180.0) / 6) + 1;

    MapLatLonToXY(DegToRad(lat), DegToRad(lon), UTMCentralMeridian(zone), x, y);

    /* Adjust easting and northing for UTM system. */
    x = x * UTMScaleFactor + 500000.0;
    y = y * UTMScaleFactor;
    if (y < 0.0)
        y = y + 10000000.0;
    return zone;
}
