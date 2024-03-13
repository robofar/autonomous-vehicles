# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Data Access Object (DAO) taken from Carla examples slightly adopted to our needs.
"""

import numpy as np
from typing import Tuple, Dict

import carla


class GlobalRoutePlannerDAO:
    """
    This class is the data access layer for fetching data from the carla server instance for GlobalRoutePlanner
    """

    def __init__(self, wmap: carla.World, sampling_resolution: float):
        """
        Initialize the data access object.

        :param wmap : carla.world object
        :param sampling_resolution : sampling distance between waypoints
        """
        self._sampling_resolution = sampling_resolution
        self._wmap = wmap

    def get_topology(self) -> Dict:
        """
        Accessor for topology.

        This function retrieves topology from the server as a list of
        road segments as pairs of waypoint objects, and processes the
        topology into a list of dictionary objects.

        :return topology: list of dictionary objects with the following attributes
            entry   -   waypoint of entry point of road segment
            entryxyz-   (x,y,z) of entry point of road segment
            exit    -   waypoint of exit point of road segment
            exitxyz -   (x,y,z) of exit point of road segment
            path    -   list of waypoints separated by 1m from entry
                        to exit
        """
        topology = []
        # Retrieving waypoints to construct a detailed topology
        for segment in self._wmap.get_topology():
            wp1, wp2 = segment[0], segment[1]
            l1, l2 = wp1.transform.location, wp2.transform.location
            # Rounding off to avoid floating point imprecision
            x1, y1, z1, x2, y2, z2 = np.round([l1.x, l1.y, l1.z, l2.x, l2.y, l2.z], 0)
            wp1.transform.location, wp2.transform.location = l1, l2
            seg_dict = dict()
            seg_dict["entry"], seg_dict["exit"] = wp1, wp2
            seg_dict["entryxyz"], seg_dict["exitxyz"] = (x1, y1, z1), (x2, y2, z2)
            seg_dict["path"] = []
            endloc = wp2.transform.location
            if wp1.transform.location.distance(endloc) > self._sampling_resolution:
                w = wp1.next(self._sampling_resolution)[0]
                while w.transform.location.distance(endloc) > self._sampling_resolution:
                    seg_dict["path"].append(w)
                    w = w.next(self._sampling_resolution)[0]
            else:
                seg_dict["path"].append(wp1)
            topology.append(seg_dict)

        return topology

    def get_waypoint(self, location: carla.Location) -> carla.Waypoint:
        """
        The method returns waypoint at given location

        :param location: vehicle location
        :return waypoint: generated waypoint close to location
        """
        return self._wmap.get_waypoint(location)

    def get_resolution(self):
        """Accessor for self._sampling_resolution"""
        return self._sampling_resolution
