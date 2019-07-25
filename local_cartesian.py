"""Convert to and from a georeferenceable local cartesian coordinates system"""
from math import *
import numpy as np
from geocentric import *
from geoid import *


def get_lla_to_enu_R_matrix(lat, lng):
    """
    Get a East, North, Up rotation matrix

    Parameters
    ----------
    lat : float, latitude in degrees [-90, 90]
    lng : float, longitude in degrees [-180, 180]
    h : float, height in meters
    geoid : the ellipsoid model

    Returns
    -------
    rotation_matrix : np.array, 3x3
    """
    sphi = sin(radians(lat))
    cphi = cos(radians(lat))
    slam = sin(radians(lng))
    clam = cos(radians(lng))
    R_c_to_o = np.array(
        [[-slam, clam, 0.], [-clam * sphi, -slam * sphi, cphi], [clam * cphi, slam * cphi, sphi]])
    return R_c_to_o


def local_cartesian_forward(origin_lla, ref_lla, geoid=wgs84_geoid):
    """
    Get the East, North, Up coordinates relative to an origin

    Parameters
    ----------
    origin_lla : (float, float, float), (latitude, longitude, height) angles are in degrees. Height is in meters
    ref_lla : (float, float, float), (latitude, longitude, height) angles are in degrees. Height is in meters
    geoid : the ellipsoid model

    Returns
    -------
    x : float, Easting
    y : float, Northing
    z : float, Height
    """
    if len(origin_lla) == 2:
        origin_lla = (origin_lla[0], origin_lla[1], 0.0)
    if len(ref_lla) == 2:
        ref_lla = (ref_lla[0], ref_lla[1], 0.0)
    origin_xyz = geocentric_forward(
        origin_lla[0], origin_lla[1], origin_lla[2], wgs84_geoid)
    geocentric_ref_xyz = geocentric_forward(
        ref_lla[0], ref_lla[1], ref_lla[2], wgs84_geoid)
    R_c_to_o = get_lla_to_enu_R_matrix(origin_lla[0], origin_lla[1])
    pt_at_center = np.array(geocentric_ref_xyz) - np.array(origin_xyz)
    t = np.matmul(R_c_to_o, pt_at_center)
    return t


def local_cartesian_reverse(origin_lla, ref_xyz, geoid=wgs84_geoid):
    """
    Get the Latitude, Longitude, Height coordinates given an origin and cartesian offset

    Parameters
    ----------
    origin_lla : (float, float, float), (latitude, longitude, height) angles are in degrees. Height is in meters
    xyz : (float, float, float), (Easting, Northing, Height)
    geoid : the ellipsoid model

    Returns
    -------
    latitude : float, latitude in degrees
    longitude : float, longitude in degrees
    height : float, height in meters
    """
    if len(origin_lla) == 2:
        origin_lla = (origin_lla[0], origin_lla[1], 0.0)
    if len(ref_xyz) == 2:
        ref_xyz = (ref_xyz[0], ref_xyz[1], 0.0)
    origin_xyz = geocentric_forward(
        origin_lla[0], origin_lla[1], origin_lla[2], wgs84_geoid)
    R_c_to_o = get_lla_to_enu_R_matrix(origin_lla[0], origin_lla[1])
    R_o_to_c = R_c_to_o.T
    geocentric_ref_xyz = np.array(
        np.matmul(R_o_to_c, ref_xyz)) + np.array(origin_xyz)
    return geocentric_reverse(geocentric_ref_xyz[0], geocentric_ref_xyz[1], geocentric_ref_xyz[2], wgs84_geoid)

