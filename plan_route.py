# coding=utf-8
# Tested in Python 2.7.13
"""
It calculates the direction to travel in to get to a destination on the
road map, given the coordinates of the start and end points.
"""

import json
import math
import requests


def main(start, end):
    """
    It calculates the shortest route on the road map from the start
    position to the end position, and returns the compass bearing
    required to start the journey.
    """
    get_request_string_for_route, err = make_get_request_string_for_route(
        start, end)
    if err is not None:
        return None, err
    raw_request = requests.get(get_request_string_for_route)
    if raw_request.status_code != 200:
        return (
            None,
            "Router server gave status code {}".format(
                raw_request.status_code))
    return calculate_direction(raw_request.content.decode('utf-8'))


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


def vector_angle_to_north(v):
    """
    It calculates the angle of the vector relative to (0, 1).  The angle
    is between 0 and 2π radians.
    """
    magnitude = (v['longitude']**2 + v['latitude']**2)**0.5
    if isclose(magnitude, 0):
        return None, "Vector has zero magnitude."
    angle = math.atan2(v['longitude'], v['latitude'])
    if angle < 0:
        angle = angle + 2 * math.pi
    return angle, None


EARTH_RADIUS = 6371e3


def distance_between(a, b):
    """
    It calculates the distance in metres between two map points, each
    specified by a latitude and longitude.

    This formula is taken from

    https://www.movable-type.co.uk/scripts/latlong.html

    A copy of the html of the page is in the file
    DistanceBetweenPoints.html.
    """
    phi1 = math.radians(a['latitude'])
    phi2 = math.radians(b['latitude'])
    delta_phi = phi2 - phi1
    delta_lambda = math.radians(b['longitude'] - a['longitude'])
    aa = (
        math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2)
        * math.sin(delta_lambda/2)**2)
    c = 2 * math.atan2(math.sqrt(aa), math.sqrt(1 - aa))
    return EARTH_RADIUS * c


def segment_30_metres_away(points):
    """
    Given a list of the coordinates of the route, it finds the ones that
    are the start and end points of the segment that is 30 metres away.
    """
    len_points = len(points)
    if len_points < 2:
        return None, None, 'Less than two points in the list.'
    distance = 0
    for i, point in enumerate(points):
        start = {'latitude': point[1], 'longitude': point[0]}
        stop = {'latitude': points[i+1][1], 'longitude': points[i+1][0]}
        length = distance_between(start, stop)
        distance += length
        if distance > 30:
            break
    return start, stop, None


def parse_route(raw_route):
    """
    It decodes the json string containing the route plan, and extracts
    the coordinates of the ends of the segment of it that is 30 metres
    away from the start.
    """
    route_as_dict = json.loads(raw_route)
    if 'code' not in route_as_dict:
        return None, None, "No 'code' key in route string."
    if route_as_dict['code'] != 'Ok':
        return (None,
                None,
                'Route code is {}'.format(route_as_dict['code']))
    points = route_as_dict['routes'][0]['geometry']['coordinates']
    result = segment_30_metres_away(points)
    return result


def calculate_direction(raw_route):
    """
    It calculates the bearing along the first segment of the route.
    """
    start, end, err = parse_route(raw_route)
    if err is not None:
        return err, None
    diff = {
        'longitude': end['longitude'] - start['longitude'],
        'latitude': end['latitude'] - start['latitude']}
    angle, err = vector_angle_to_north(diff)
    if err is not None:
        return None, "The first two nodes in the route are identical."
    return angle, None


def valid_map_position(m):
    """ It checks that a MapPosition is a two-tuple of floats. """
    return (len(m) == 2 and
            isinstance(m['latitude'], float) and
            isinstance(m['longitude'], float))


def make_get_request_string_for_route(start, destination):
    """
    It creates an SQL query string that gets the shortest route between
    the start and end nodes.
    """
    if not valid_map_position(start):
        return None, "Start position is not valid."
    if not valid_map_position(destination):
        return None, "Destination position is not valid."
    return ((
        "http://localhost:5000/route/v1/driving/{},{};{},{}?geometries="
        "geojson".format(start['longitude'], start['latitude'],
                         destination['longitude'], destination['latitude'])),
        None)
