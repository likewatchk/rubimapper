"""Convert from latitude and longitude values to XYZ values"""
from math import *
from geoid import *


def geocentric_forward(lat, lon, h, geoid=wgs84_geoid):
    """
    Parameters
    ----------
    lat : float, latitude in degrees [-90, 90]
    lon : float, longitude in degrees [-180, 180]
    h : float, height in meters
    geoid : the ellipsoid model

    Returns
    -------
    x : float, meters
    y : float, meters
    z : float, meters
    """
    sphi = sin(radians(lat))
    cphi = cos(radians(lat))
    slam = sin(radians(lon))
    clam = cos(radians(lon))
    n = geoid.a / sqrt(1. - geoid.e2 * sphi**2)
    Z = (geoid.e2m * n + h) * sphi
    X = (n + h) * cphi
    Y = X * slam
    X *= clam
    return (X, Y, Z)


def geocentric_reverse(x, y, z, geoid=wgs84_geoid):
    """
    Parameters
    ----------
    x : float, meters
    y : float, meters
    z : float, meters
    geoid : the ellipsoid model

    Returns
    -------
    latitude : float, latitude in degrees [-90, 90]
    longitude : float, longitude in degrees [-180, 180]
    height : float, height in meters
    """
    R = hypot(x, y)
    slam = y / R if R else 0.
    clam = x / R if R else 1.
    # Distance to center of earth
    h = hypot(R, z)
    if h > geoid.max_rad:
        #  We really far away (> 12 million light years); treat the earth as a
        #  point and h, above, is an acceptable approximation to the height.
        #  This avoids overflow, e.g., in the computation of disc below.  It's
        #  possible that h has overflowed to inf; but that's OK.
        #
        #  Treat the case x, y finite, but R overflows to +inf by scaling by 2.
        R = hypot(x / 2., y / 2.)
        slam = (y / 2.) / R if R else 0.
        clam = (x / 2.) / R if R else 1.
        H = hypot(z / 2., R)
        sphi = (z / 2.) / H
        cphi = R / H
    elif geoid.e4a == 0.:
        #  Treat the spherical case.  Dealing with underflow in the general case
        #  with _e2 = 0 is difficult.  Origin maps to N pole same as with
        #  ellipsoid.
        H = hypot(1. if h == 0. else z, R)
        sphi = (1. if h == 0. else z) / H
        cphi = R / H
        h -= geoid.a
    else:
        #  Treat prolate spheroids by swapping R and z here and by switching
        #  the arguments to phi = atan2(...) at the end.
        p = (R / geoid.a)**2
        q = geoid.e2m * (z / geoid.a)**2
        r = (p + q - geoid.e4a) / 6.
        if geoid.f < 0:
            p, q = q, p
        if not (geoid.e4a * q == 0 and r <= 0):
            #  Avoid possible division by zero when r = 0 by multiplying
            #  equations for s and t by r^3 and r, resp.
            S = geoid.e4a * p * q / 4.
            r2 = r**2
            r3 = r * r2
            disc = S * (2. * r3 + S)
            u = r
            if disc >= 0:
                T3 = S + r3
                #  Pick the sign on the sqrt to maximize abs(T3).  This minimizes
                #  loss of precision due to cancellation.  The result is unchanged
                #  because of the way the T is used in definition of u.
                T3 += -sqrt(disc) if T3 < 0 else sqrt(disc)
                #  N.B. cbrt always returns the real root.  cbrt(-8) = -2.
                T = T3**(1. / 3.)
                #  T can be zero; but then r2 / T -> 0.
                u += T + (r2 / T if T else 0.)
            else:
                #  T is complex, but the way u is defined the result is real.
                ang = atan2(sqrt(-disc), -(S + r3))
                #  There are three possible cube roots.  We choose the root which
                #  avoids cancellation.  Note that disc < 0 implies that r < 0.
                u += 2. * r * cos(ang / 3.)
            #  guaranteed positive
            v = sqrt(u**2 + geoid.e4a * q)
            #  Avoid loss of accuracy when u < 0.  Underflow doesn't occur in
            #  e4 * q / (v - u) because u ~ e^4 when q is small and u < 0.
            #  u+v, guaranteed positive
            uv = geoid.e4a * q / (v - u) if u < 0 else u + v
            # Need to guard against w going negative due to roundoff in uv - q.
            w = max(0., geoid.e2a * (uv - q) / (2. * v))
            #  Rearrange expression for k to avoid loss of accuracy due to
            #  subtraction.  Division by 0 not possible because uv > 0, w >= 0.
            k = uv / (sqrt(uv + w**2) + w)
            k1 = k if geoid.f >= 0 else k - geoid.e2
            k2 = k + geoid.e2 if geoid.f >= 0 else k
            d = k1 * R / k2
            H = hypot(z / k1, R / k2)
            sphi = (z / k1) / H
            cphi = (R / k2) / H
            h = (1. - geoid.e2m / k1) * hypot(d, z)
        else:
            #  This leads to k = 0 (oblate, equatorial plane) and k + e^2 = 0
            #  (prolate, rotation axis) and the generation of 0/0 in the general
            #  formulas for phi and h.  using the general formula and division by 0
            #  in formula for h.  So handle this case by taking the limits:
            #  f > 0: z -> 0, k      ->   e2 * sqrt(q)/sqrt(e4 - p)
            #  f < 0: R -> 0, k + e2 -> - e2 * sqrt(q)/sqrt(e4 - p)
            zz = sqrt((geoid.e4a - p if geoid.f >= 0 else p) / geoid.e2m)
            xx = sqrt(geoid.e4a - p if geoid.f < 0 else p)
            H = hypot(zz, xx)
            sphi = zz / H
            cphi = xx / H
            #  for tiny negative z (not for prolate)
            if z < 0:
                sphi = -sphi
            h = -geoid.a * (geoid.e2m if geoid.f >= 0 else 1) * H / geoid.e2a
    lat = atan2(sphi, cphi)
    lon = atan2(slam, clam)

    return (degrees(lat), degrees(lon), h)
