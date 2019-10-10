# Author : Sungwoo Koo, Soongsil University, Smart Systems Software 20120584
# Professor : Kanghee Kim

"""
Copyright (c) <2019> <Sungwoo Koo, softwkoo>
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseEvent

from math import *
import numpy as np
import itertools

import sys
import csv
import rosbag
import os

from line_maker import *
from local_cartesian import *
from shapely.geometry import Point as Pt  # to avoid overlapping names
from shapely.geometry.polygon import Polygon  # shapely 를 사용할 수 없으면 LaneSet의 in_area_of_interest를 주석처리 할 것.

# csv.field_size_limit(sys.maxsize)  # Amount of WKT data is so large.

# changed name, Vector to Point
# Point -> line(=point_list) -> line_list
# Type of point_list & line_list is list

# *** LaneSet defined for connection or sequence
# Lane(point_list) _ point_list==line
# LaneSet(line_list)
# Endpoint(point, idx) _ idx has meaning only in LaneSet
# EndpointSet(LaneSet)

# *** PathSet defined for direction of lane
# Pathpoint
# Path(Lane)
# PathSet(LaneSet)

# *** WaySet defined for writing csv files
# Waypoint
# Waypiece(Waypoint1, Waypoint2)
# Way(Waypoint_list)
# WaySet(Way_list)

INTERVAL = 0.99
START_ID = 10001

""" Define Classes """


class Point:
    def __init__(self, x, y, z):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __str__(self):
        return '<Vector %s, %s, %s>' % (self.x, self.y, self.z)

    def __copy__(self):
        return Point(self.x, self.y, self.z)

    # negative
    def __neg__(self):
        return Point(-self.x, -self.y, -self.z)

    # operator *
    def __mul__(self, coef):
        return Point(self.x * coef, self.y * coef, self.z * coef)

    def __rmul__(self, coef):
        return self.__mul__(coef)

    # operator /
    def __truediv__(self, number):
        return self.__copy__() * (number ** -1)

    # operator +
    def __add__(self, operand):
        return Point(self.x + operand.x, self.y + operand.y, self.z + operand.z)

    # operator -
    def __sub__(self, operand):
        return self.__copy__() + -operand

    # Cross product
    # cross = a ** b
    def __pow__(self, operand):
        return Point(self.y * operand.z - self.z * operand.y,
                     self.z * operand.x - self.x * operand.z,
                     self.z * operand.y - self.y * operand.x)

    # Dot Project
    # dp = a & b
    def __and__(self, operand):
        return (self.x * operand.x) + (self.y * operand.y) + (self.z * operand.z)

    def normal(self):
        return self.__copy__() / self.magnitude()

    def magnitude(self):
        return (self.x ** 2 + self.y ** 2 + self.z ** 2) ** .5

    def magnitude_xy(self):
        return (self.x ** 2 + self.y ** 2) ** .5

    def __ge__(self, number):
        return self.magnitude() >= number

    def __gt__(self, number):
        return self.magnitude() > number

    def __le__(self, number):
        return self.magnitude() <= number

    def __lt__(self, number):
        return self.magnitude() < number

    def __eq__(self, operand):
        return self.x == operand.x and self.y == operand.y  # and self.z == operand.z  #todo z축 값 맞추기


class Endpoint:
    def __init__(self, **kwargs):
        self.x = float(kwargs['x']) if kwargs['point'] is None else float(kwargs['point'].x)
        self.y = float(kwargs['y']) if kwargs['point'] is None else float(kwargs['point'].y)
        self.z = float(kwargs['z']) if kwargs['point'] is None else float(kwargs['point'].z)
        self.lane_idx = int(kwargs['idx'])
        self.valid = True

    def __str__(self):
        return '<Endpoint %s, %s, %s, valid = %s>' % (self.x, self.y, self.z, self.valid)

    def __eq__(self, operand):  # different line and same coordinates
        return self.x == operand.x and self.y == operand.y and self.lane_idx != operand.lane_idx
        #  and self.z == operand.z


class EndpointSet:
    def __init__(self, lane_set):  # lane_set is LaneSet obj
        self.endpoint_list = []
        idx = 0
        for lane in lane_set.lane_list:
            lane.set_idx(idx)
            self.endpoint_list.extend([lane.endp1, lane.endp2])
            idx += 1

    def __str__(self):
        return '<EndpointSet, length : %s>' % (len(self.endpoint_list))


class Lane:
    def __init__(self, point_list=None, valid=True, startpoint=None, start_dir=None, endpoint=None, end_dir=None,
                 straight=False, curve=False):
        self.point_list = [] if point_list is None else point_list  # point_list = line
        self.valid = valid

        if len(self.point_list) == 0 and startpoint is None and endpoint is None:
            self.endp1 = None
            self.endp2 = None
        else:
            self.endp1 = Endpoint(point=self.point_list[0], idx=9999999999) if startpoint is None else Endpoint(
                point=startpoint, idx=9999999999)  # first_point
            self.endp2 = Endpoint(point=self.point_list[-1], idx=9999999999) if endpoint is None else Endpoint(
                point=endpoint, idx=9999999999)  # last_point

        if startpoint is not None and endpoint is not None:
            if straight:  # start and endpoint are Point
                diff = endpoint - startpoint
                dx = diff.x
                dy = diff.y
                dz = diff.z
                distance = int(diff.magnitude())
                unitx = dx/distance
                unity = dy/distance
                unitz = dz/distance
                for i in range(distance-1):
                    j = i + 1
                    self.point_list.append(Point(startpoint.x + j * unitx, startpoint.y + j * unity,
                                                 startpoint.z + j * unitz))
                self.point_list.insert(0, startpoint)
                self.point_list.append(endpoint)
            elif curve:  # Needs Points and Directions of them.
                diff = endpoint - startpoint
                diff_dir = end_dir - start_dir
                rotate_angle = start_dir - 1.5707963267  # the angle should be radian
                print("s : ", start_dir, "e : ", end_dir, "rot_dir : ", rotate_angle)
                rot_diff = rotate(diff, rotate_angle)
                # initial_curve = make_curve(rot_diff, -diff_dir)
                initial_curve_no_z = make_curve2(rot_diff, -diff_dir)
                dz = diff.z
                unitz = dz / (len(initial_curve_no_z) -1)
                initial_curve = []
                for i in range(len(initial_curve_no_z)):
                    point = Point(initial_curve_no_z[i].x, initial_curve_no_z[i].y, startpoint.z + i*unitz)
                    initial_curve.append(point)
                # re_rotate to original state.
                out_point_list = []
                for i in range(len(initial_curve)):
                    point = rotate(initial_curve[i], -rotate_angle)
                    point = translate(point, startpoint.x, startpoint.y)
                    out_point_list.append(point)
                self.point_list = out_point_list

    def __str__(self):
        return '<Line object, vectors : %s, valid : %s>' % (len(self.point_list), True if self.valid else False)

    def __add__(self, operand):
        if self.point_list[-1] == operand.point_list[0]:
            point_list = self.point_list + operand.point_list[1:]
        else:
            point_list = self.point_list + operand.point_list
        return Lane(point_list)

    def z_list(self):
        out_z_list = []
        for point in self.point_list:
            out_z_list.append(point.z)
        return out_z_list

    def invalidate(self):
        self.valid = False
        self.endp1.valid = False
        self.endp2.valid = False

    def set_idx(self, idx):
        self.endp1.lane_idx = idx
        self.endp2.lane_idx = idx

    def end_points(self):
        return [self.endp1, self.endp2]

    def plot(self, bo=False):
        plt.xlabel("coordinate X")
        plt.ylabel("coordinate Y")
        plt.title("A Lane")
        x = []
        y = []
        for point in self.point_list:
            x.append(point.x)
            y.append(point.y)
        if bo:
            plt.plot(x, y, 'bo')
        else:
            plt.plot(x, y)

    def show(self, bo=False):
        self.plot(bo=bo)
        plt.show()

    def make_uniform_intervals(self, meter):
        idx1 = 0
        idx2 = 1
        to_be_erased_idx = []
        out_point_list = []
        while self.point_list[idx1] != (self.point_list[-1]):
            diff = self.point_list[idx1] - self.point_list[idx2]
            while diff <= meter:
                if not idx2 == len(self.point_list) - 1:
                    to_be_erased_idx.append(idx2)
                else:
                    break
                idx2 += 1
                diff = self.point_list[idx1] - self.point_list[idx2]
            idx1 = idx2
            idx2 += 1
        to_keep_idx = [idx for idx in range(len(self.point_list)) if idx not in to_be_erased_idx]
        for idx in to_keep_idx:
            out_point_list.append(self.point_list[idx])
        return Lane(out_point_list)

    def connect(self, other):  # only when self and other are adjacent
        if self.point_list[0] == other.point_list[0]:
            other.point_list.reverse()
            lane = self + other
        elif self.point_list[-1] == other.point_list[0]:
            lane = self + other
        elif self.point_list[0] == other.point_list[-1]:
            lane = other + self
        elif self.point_list[-1] == other.point_list[-1]:
            self.point_list.reverse()
            lane = self + other
        self.invalidate()
        other.invalidate()
        return lane

    def translate(self, dx, dy):
        out_point_list = []
        for point in self.point_list:
            p = Point(point.x+dx, point.y+dy, point.z)
            out_point_list.append(p)
        return Lane(out_point_list)


class LaneSet:
    def __init__(self, line_list=None, lane_list=None):
        self.lane_list = [] if lane_list is None else lane_list
        if line_list is not None:
            idx = 0  # make lane_list and initialize idx on end_p
            for line in line_list:
                lane = Lane(point_list=line)
                lane.set_idx(idx)
                self.lane_list.append(lane)
                idx += 1
        self.endpointset = EndpointSet(self)

    def __add__(self, other):
        out_lane_list = self.lane_list + other.lane_list
        return LaneSet(lane_list=out_lane_list)

    def min_max_xy(self):
        maximum_x = self.lane_list[0].point_list[0].x
        for lane in self.lane_list:
            maxx = max(point.x for point in lane.point_list)
            if maxx > maximum_x:
                maximum_x = maxx
        maximum_y = self.lane_list[0].point_list[0].y
        for lane in self.lane_list:
            maxy = max(point.y for point in lane.point_list)
            if maxy > maximum_y:
                maximum_y = maxy
        minimum_x = self.lane_list[0].point_list[0].x
        for lane in self.lane_list:
            minx = min(point.x for point in lane.point_list)
            if minx < minimum_x:
                minimum_x = minx
        minimum_y = self.lane_list[0].point_list[0].y
        for lane in self.lane_list:
            miny = min(point.y for point in lane.point_list)
            if miny < minimum_y:
                minimum_y = miny
        return minimum_x, minimum_y, maximum_x, maximum_y

    def closest_point(self, point):
        closest_point = self.lane_list[0].point_list[0]
        min_l = (point - closest_point).magnitude_xy()
        for lane in self.lane_list:
            for p in lane.point_list:
                l = (point - p).magnitude_xy()
                if l < min_l:
                    min_l = l
                    closest_point = p
        return closest_point

    def eliminate_duplication(self):
        remove_list = []
        for lane1 in self.lane_list:
            for lane2 in self.lane_list:
                if id(lane1)!=id(lane2) and lane1.point_list == lane2.point_list:
                    remove_list.append(lane2)
        for l in remove_list:
            self.lane_list.remove(l)

    def adjust_z_coordinate_to(self, reference_laneset):
        out_point_list = []
        out_lane_list = []
        n = 1
        for lane in self.lane_list:
            for point in lane.point_list:
                p = reference_laneset.closest_point(point)
                print(float(int((n / len(lane.point_list)) * 10000))/100, "%")
                n += 1
                out_point = Point(point.x, point.y, p.z)  # change z_coordinate with closest point's Z
                out_point_list.append(out_point)
            out_lane_list.append(Lane(point_list=out_point_list))
            out_point_list = []

        return LaneSet(lane_list=out_lane_list)

    def lanes_in_area_of_interest(self, point_list):
        out_line_list = []  # not Lane, it is line!
        poly = Polygon(point_list)
        x, y = poly.exterior.xy
        plt.plot(x, y)  # show together with the LaneSet
        self.show()
        for lane in self.lane_list:
            endpoint_1 = Pt(lane.point_list[0].x, lane.point_list[0].y)
            endpoint_2 = Pt(lane.point_list[-1].x, lane.point_list[-1].y)
            if poly.contains(endpoint_1) and poly.contains(endpoint_2):
                out_line_list.append(lane.point_list)
        return LaneSet(line_list=out_line_list)

    def points_in_area_of_interest(self, point_list):
        out_point_list = []  # not Lane, it is line!
        poly = Polygon(point_list)
        x, y = poly.exterior.xy
        plt.plot(x, y)  # show together with the LaneSet
        self.show()
        for lane in self.lane_list:
            for point in lane.point_list:
                pt = Pt(point.x, point.y)
                if poly.contains(pt):
                    out_point_list.append(Point(point.x, point.y, point.z))
        return Lane(point_list=out_point_list)

    def remove_lanes(self, idx_list):
        out_lane_list = []
        for i in range(len(self.lane_list)):
            if i not in idx_list:
                out_lane_list.append(self.lane_list[i])
        return LaneSet(lane_list=out_lane_list)

    def remove_points_in_area(self, point_list):
        poly = Polygon(point_list)
        x, y = poly.exterior.xy
        plt.plot(x, y)  # show together with the LaneSet
        self.show()
        remove_list = []
        for lane in self.lane_list:
            for point in lane.point_list:
                p = Pt(point.x, point.y)
                if poly.contains(p):
                    remove_list.append(point)
            for rp in remove_list:
                lane.point_list.remove(rp)
            remove_list = []

    def make_uniform_intervals(self, meter):
        out_lane_list = []
        for lane in self.lane_list:
            out_lane_list.append(lane.make_uniform_intervals(meter))
        return LaneSet(lane_list=out_lane_list)

    def sub_plot(self, ax, bo=False, ep=False):  # in Lane
        x = []
        y = []
        for lane in self.lane_list:
            for point in lane.point_list:
                x.append(point.x)
                y.append(point.y)
            if bo:
                # plt.text(x[0], y[0], "{},{}".format(x[0],y[0]), fontsize=8)
                # plt.text(x[-1], y[-1], "{},{}".format(x[-1],y[-1]), fontsize=8)
                ax.plot(x, y, 'bo')
            elif ep:
                plt.plot(x, y, 'bo')
                plt.text(x[0], y[0], "{}".format("S"), fontsize=8)
                plt.text(x[-1], y[-1], "{}".format("E"), fontsize=8)
            else:
                ax.plot(x, y)
            x = []
            y = []

    def sub_plot_idx(self, ax, bo=False, reverse=False):
        x = []
        y = []
        idx = 0
        for lane in self.lane_list:
            for point in lane.point_list:
                x.append(point.x)
                y.append(point.y)
            if bo:
                ax.plot(x, y, 'bo')
                plt.text(x[len(x) // 2], y[len(y) // 2], "{}".format(idx), fontsize=10)
            elif reverse:
                ax.plot(y, x)
                plt.text(y[0], x[0], "{}".format("S"), fontsize=8)
                plt.text(y[-1], x[-1], "{}".format("E"), fontsize=8)
                plt.text(y[len(y) // 2], x[len(x) // 2], "{}".format(idx), fontsize=11)
            else:
                ax.plot(x, y)
                plt.text(x[len(x) // 2], y[len(y) // 2], "{}".format(idx), fontsize=11)
            idx += 1
            x = []
            y = []

    def sub_plot2(self, ax, bo=False):  # in Lanes
        x = []
        y = []
        for lane in self.lane_list:
            for point in lane.point_list:
                x.append(point.x)
                y.append(point.y)
        if bo:
            # plt.text(x[0], y[0], "{},{}".format(x[0],y[0]), fontsize=8)
            # plt.text(x[-1], y[-1], "{},{}".format(x[-1],y[-1]), fontsize=8)
            line, = ax.plot(x, y, 'bo')
        else:
            line, = ax.plot(x, y)
        return line

    def sub_plot3(self, ax, bo=False):  # in Draggable lane
        dls = []
        x = []
        y = []
        for lane in self.lane_list:
            for point in lane.point_list:
                x.append(point.x)
                y.append(point.y)
            if bo:
                # plt.text(x[0], y[0], "{},{}".format(x[0],y[0]), fontsize=8)
                # plt.text(x[-1], y[-1], "{},{}".format(x[-1],y[-1]), fontsize=8)
                line, = ax.plot(x, y, 'bo')
                dl = DraggaleLine(line)
                dl.connect()
                dls.append(dl)
            else:
                line, = ax.plot(x, y)
                dl = DraggaleLine(line)
                dl.connect()
                dls.append(dl)
            x = []
            y = []
        return dls

    def sub_plot4(self, ax, bo=False):  # in plot lane
        out_plot_line_list = []
        x = []
        y = []
        for lane in self.lane_list:
            for point in lane.point_list:
                x.append(point.x)
                y.append(point.y)
            if bo:
                # plt.text(x[0], y[0], "{},{}".format(x[0],y[0]), fontsize=8)
                # plt.text(x[-1], y[-1], "{},{}".format(x[-1],y[-1]), fontsize=8)
                line, = ax.plot(x, y, 'bo', picker=5)
                out_plot_line_list.append(line)
            else:
                line, = ax.plot(x, y, picker=5)
                out_plot_line_list.append(line)
            x = []
            y = []
        return out_plot_line_list

    def plot(self, bo=False, reverse=False, ep=False):
        plt.xlabel("coordinate X")
        plt.ylabel("coordinate Y")
        plt.title("Line_list")
        x = []
        y = []
        for lane in self.lane_list:
            for point in lane.point_list:
                x.append(point.x)
                y.append(point.y)
            if bo:
                # plt.text(x[0], y[0], "{},{}".format(x[0],y[0]), fontsize=8)
                # plt.text(x[-1], y[-1], "{},{}".format(x[-1],y[-1]), fontsize=8)
                plt.plot(x, y, 'bo')
            elif reverse:
                plt.plot(y, x)
            elif ep:
                plt.plot(x, y)
                plt.text(x[0], y[0], "{}".format("S"), fontsize=8)
                plt.text(x[-1], y[-1], "{}".format("E"), fontsize=8)
            else:
                plt.plot(x, y)
            x = []
            y = []

    def plot_idx(self, bo=False, reverse=False):
        plt.xlabel("coordinate X")
        plt.ylabel("coordinate Y")
        plt.title("Line_list")
        x = []
        y = []
        idx = 0
        for lane in self.lane_list:
            for point in lane.point_list:
                x.append(point.x)
                y.append(point.y)
            if bo:
                plt.plot(x, y, 'bo')
                plt.text(x[len(x) // 2], y[len(y) // 2], "{}".format(idx), fontsize=10)
            elif reverse:
                plt.plot(y, x)
                plt.text(y[0], x[0], "{}".format("S"), fontsize=8)
                plt.text(y[-1], x[-1], "{}".format("E"), fontsize=8)
                plt.text(y[len(y) // 2], x[len(x) // 2], "{}".format(idx), fontsize=11)
            else:
                plt.plot(x, y)
                plt.text(x[len(x) // 2], y[len(y) // 2], "{}".format(idx), fontsize=11)
            idx += 1
            x = []
            y = []

    def show(self, bo=False, reverse=False):
        self.plot(bo=bo, reverse=reverse)
        print("(**SHOW**) executed : LaneSet.show()")
        plt.show()

    def show_ep(self, bo=False, reverse=False):
        self.plot(bo=bo, reverse=reverse)
        print("(**SHOW**) executed : LaneSet.show()")
        plt.show()

    def show_idx(self,bo=False, reverse=False):
        self.plot_idx(bo=bo, reverse=reverse)
        print("(**SHOW**) executed : LaneSet.show()")
        plt.show()

    def get_valid_lanes(self):
        valid_line_list = []
        for lane in self.lane_list:
            if lane.valid:
                valid_line_list.append(lane.point_list)
        return LaneSet(line_list=valid_line_list)  # needs point_list type so that re-indexing

    def split_connected_lanes(self, meter=7):
        for lane in self.lane_list:
            for i in range(len(lane.point_list) - 1):
                diff = lane.point_list[i] - lane.point_list[i + 1]
                if diff > meter:
                    lane.invalidate()
                    l1 = Lane(lane.point_list[:i + 1])
                    l1.set_idx(len(self.lane_list))
                    self.lane_list.append(l1)
                    l2 = Lane(lane.point_list[i + 1:])
                    l2.set_idx(len(self.lane_list))
                    self.lane_list.append(l2)
        out_laneset = self.get_valid_lanes()
        return out_laneset

    def connect_split_lanes(self):
        new_endpoints = []
        for ep1 in self.endpointset.endpoint_list:
            if ep1.valid is False:
                continue
            lane1 = self.lane_list[ep1.lane_idx]
            for ep2 in self.endpointset.endpoint_list:
                if ep2.valid is False:
                    continue
                lane2 = self.lane_list[ep2.lane_idx]
                if ep1 == ep2:  # 이중루프 전체 한 루프에 한 번만 걸림. break 해도 됨.
                    lane = lane1.connect(lane2)  # lane1.connect(lane2[1:])
                    # lane1.connect(lane2[0:-2].reverse)
                    # (lane2.reverse).connect(lane1)
                    self.lane_list.append(lane)
                    lane.set_idx((self.lane_list.index(lane)))
                    new_endpoints = lane.end_points()  # new_endpoints.extend(lane.end_points()) 로 해도 됨.
                    break
            self.endpointset.endpoint_list.extend(new_endpoints)
            new_endpoints = []
        out_laneset = self.get_valid_lanes()
        print("executed : connect_split_lanes")
        return out_laneset

    def translate(self, dx, dy):
        out_lane_list = []
        for lane in self.lane_list:
            _lane = lane.translate(dx, dy)
            out_lane_list.append(_lane)
        return LaneSet(lane_list=out_lane_list)

    def add_lane(self, in_lane):
        self.lane_list.append(in_lane)


class Pathpoint:
    def __init__(self, point, direction, r):
        self.x = point.x
        self.y = point.y
        self.z = point.z
        self.dir = direction
        self.r = r

    def __str__(self):
        return '<Pathpoint %s, %s, %s, %s, %s>' % (self.x, self.y, self.z, self.dir, self.r)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and id(self) != id(other)  # and self.z == other.z

    def point(self):
        return Point(self.x, self.y, self.z)


class Path:
    def __init__(self, lane, up_down='up'):
        """
        :param up_down: 위로 올라가는 방향 or 밑으로 내려가는 방향
        :return: point_list, direction_list, r_list를 가진 Path.
        """
        self.point_list = lane.point_list
        self.direction_list = []  # direction 은 두개의 point 간의 차이를 이용
        self.r_list = []  # r은 두개의 direction 간의 차이을 이용

        # step 1 make direction_list
        direction = 0
        for i in range(len(lane.point_list) - 1):  # khkim: up/down 구별을 생성자 호출 이전에 정의하고, 여기서는 구별하지 말 것
            if up_down == 'up':  # [i+1] - [i] 의 결과를 [i+1] 에 적용
                vector = lane.point_list[i + 1] - lane.point_list[i]  # khkim: vector -> diff
            elif up_down == 'down':  # [i] - [i+1] 의 결과를 [i]에 적용
                vector = lane.point_list[i] - lane.point_list[i + 1]
            dx = vector.x
            dy = vector.y
            """
            atan(dx/dy)를 기준으로 한다. _ ROS 좌표를 사용, 왼쪽을 향하는 Y축과 겹치는 방향이 0 radian(dtlane의 정체 file참조)

            x  |  y  |  dir
            +0    +      atan(dx/dy)                        (1사분면)
            +     -0            atan(-dy/dx) + 1.5707963267 (2사분면)
            -0    -      atan(-dx/-dy) + 3.1415926535       (3사분면)
            -     +0            atan(dy/-dx) + 4.7123889802 (4사분면)

           0도  (dx = 0, dy > 0) -> 0            (1사분면 포함)
           90도 (dx > 0, dy = 0) -> 1.5707963267 (2사분면 포함)
           180도(dx = 0, dy < 0) -> 3.1415926535 (3사분면 포함)
           270도(dx < 0, dy = 0) -> 4.7123889802 (4사분면 포함)
           360도(dx = 0, dy > 0) -> 0            (1사분면)
          """
            # khkim: direction = atan2(dx/dy)
            # khkim: if (direction < 0) direction += 2*pi
            if dx >= 0 and dy > 0:  # 1사분면 and 0도
                direction = atan(dx / dy)
            elif dx > 0 and dy <= 0:  # 2사분면 and 90도
                direction = atan(-dy / dx) + 1.5707963267
            elif dx <= 0 and dy < 0:  # 3사분면 and 180도
                direction = atan(-dx / -dy) + 3.1415926535
            elif dx < 0 and dy >= 0:  # 4사분면 and 270도
                direction = atan(dy / -dx) + 4.7123889802

            if up_down == 'up':  # [i+1] - [i] 의 결과를 [i+1] 에 적용
                if i == 0:
                    self.direction_list.extend([direction, direction])  # [0]의 direction 을 [1] 의 direction 과 동일하게.
                else:
                    self.direction_list.append(direction)
            elif up_down == 'down':  # [i] - [i+1] 의 결과를 [i]에 적용
                if i == len(lane.point_list) - 2:
                    self.direction_list.extend([direction, direction])  # [-1]의 direction 을 [-2] 의 direction 과 동일하게.
                else:
                    self.direction_list.append(direction)

        # step 2 make r_list
        r = 0
        for i in range(len(lane.point_list) - 1):  # khkim: up/down 구별을 생성자 호출 이전에 정의하고, 여기서는 구별하지 말 것
            if up_down == 'up':  # [i+1] - [i] 의 결과를 [i+1] 에 적용
                delta_dir = self.direction_list[i + 1] - self.direction_list[i]
            elif up_down == 'down':  # [i] - [i+1] 의 결과를 [i]에 적용
                delta_dir = self.direction_list[i] - self.direction_list[i + 1]

            if delta_dir == 0:
                r = 90000000000
            else:
                r = 1 / delta_dir

            if up_down == 'up':  # [i+1] - [i] 의 결과를 [i+1] 에 적용
                if i == 0:
                    self.r_list.extend([r, r])  # [0]의 direction 을 [1] 의 direction 과 동일하게.
                else:
                    self.r_list.append(r)
            elif up_down == 'down':  # [i] - [i+1] 의 결과를 [i]에 적용
                if i == len(lane.point_list) - 2:
                    self.r_list.extend([r, r])  # [-1]의 direction 을 [-2] 의 direction 과 동일하게.
                else:
                    self.r_list.append(r)

    def __str__(self):
        return '<Path obj, points : %s>' % (len(self.point_list))

    def pathpoint_list(self):
        # pathpoint_list
        pathpoint_list = []
        for i in range(len(self.point_list)):
            pathpoint_list.append(Pathpoint(self.point_list[i], self.direction_list[i], self.r_list[i]))
        return pathpoint_list

    def smoothing(self, threshold_degree=0.02, threshold_counts=8, meter=0.99, up_down='up'):  # 직선구간과 곡선구간으로 구분, 직선구간은 직선 lane 새로 생성, 곡선구간도..?
        n = 0
        flag = False
        straight_section_start_idx = 0
        straight_section_end_idx = 0
        straight_section_list = []
        # 직선 구간 탐색 및 기록
        pathpoint_list = self.pathpoint_list()
        for i in range(len(pathpoint_list) - 1):
            if abs(pathpoint_list[i + 1].dir - pathpoint_list[i].dir) <= threshold_degree:
                n += 1
            else:
                n = 0
            if n == threshold_counts:
                straight_section_start_idx = i - (threshold_counts - 1)  # to keep the startpoint not replaced
                straight_section_end_idx = i + 1  # to keep the endpoint not replaced
                flag = True
            if n > threshold_counts:
                straight_section_end_idx += 1
            if n < threshold_counts:
                if flag:
                    straight_section_end_idx = i
                    straight_section_list.append([straight_section_start_idx, straight_section_end_idx])
                    flag = False
        # create smoothed(straight) lane_list
        straight_lane_list = []
        for section in straight_section_list:
            startpoint = self.point_list[section[0]]
            endpoint = self.point_list[section[1]]
            lane = Lane(startpoint=startpoint, endpoint=endpoint, straight=True)
            straight_lane_list.append(lane)
        # extract existing curve(not straight) section  #todo curve smoothing
        existing_curve_lane_list = []
        point_list = self.point_list
        for i in range(len(straight_section_list) + 1):  # straight_section + 1 = curve_section
            if len(straight_section_list) == 0:  # there is no straight section
                existing_curve_lane_list.append(Lane(point_list))
            elif i == 0:
                existing_curve_lane_list.append(Lane(point_list[:(straight_section_list[i][0] + 1)]))
            elif i == len(straight_section_list):
                existing_curve_lane_list.append(Lane(point_list[straight_section_list[i - 1][1]:]))
            else:
                existing_curve_lane_list.append(
                    Lane(point_list[straight_section_list[i - 1][1]:straight_section_list[i][0] + 1]))
        # existing lanes + smoothed lanes
        out_point_list = []
        for i in range(len(existing_curve_lane_list) - 1):
            _point_list = existing_curve_lane_list[i].point_list + straight_lane_list[i].point_list
            out_point_list += _point_list
        out_point_list += existing_curve_lane_list[-1].point_list
        _lane = Lane(out_point_list)
        _lane = _lane.make_uniform_intervals(meter)
        return Path(_lane)

    def waypoint_list(self):
        # pathpoint_list
        waypoint_list = []
        for i in range(len(self.point_list)):
            waypoint_list.append(Waypoint(self.point_list[i], self.direction_list[i], self.r_list[i]))
        return waypoint_list

    def way(self):
        return Way(self.waypoint_list())

    def lane(self):
        return Lane(point_list=self.point_list)

    def plot(self, bo=False):
        plt.xlabel("coordinate X")
        plt.ylabel("coordinate Y")
        plt.title("A Path")
        x = []
        y = []
        for point in self.point_list:
            x.append(point.x)
            y.append(point.y)
        if bo:
            plt.plot(x, y, 'bo')
        else:
            plt.plot(x, y)

    def show(self, bo=False):
        self.plot(bo=bo)
        print("(**SHOW**) executed : Path.show()")
        plt.show()


class PathSet:
    def __init__(self, path_list=None, laneset=None, up_down='up'):
        self.path_list = [] if path_list is None else path_list
        if laneset is not None:
            if up_down == 'up':
                for lane in laneset.lane_list:
                    self.path_list.append(Path(lane, up_down='up'))
            elif up_down == 'down':
                for lane in laneset.lane_list:
                    self.path_list.append(Path(lane, up_down='down'))

    def __str__(self):
        return '<PathSet obj, Paths : %s>' % (len(self.path_list))

    def laneset(self):
        out_lane_list = []
        for path in self.path_list:
            out_lane_list.append(path.lane())
        return LaneSet(lane_list=out_lane_list)

    def smoothing(self):
        global INTERVAL
        out_path_list = []
        for path in self.path_list:
            out_path_list.append(path.smoothing(meter=INTERVAL))
        return PathSet(path_list=out_path_list)

    def plot(self, bo=False, reverse=False):
        plt.xlabel("coordinate X")
        plt.ylabel("coordinate Y")
        plt.title("Pathset")
        x = []
        y = []
        idx = 0
        for path in self.path_list:
            for point in path.point_list:
                x.append(point.x)
                y.append(point.y)
            if bo:
                plt.plot(x, y, 'bo')
                plt.text(x[len(x) // 2], y[len(y) // 2], "{}".format(idx), fontsize=10)
            elif reverse:
                plt.plot(y, x)
                plt.text(y[0], x[0], "{}".format("S"), fontsize=8)
                plt.text(y[-1], x[-1], "{}".format("E"), fontsize=8)
                plt.text(y[len(y) // 2], x[len(x) // 2], "{}".format(idx), fontsize=11)
            else:
                plt.plot(x, y)
                plt.text(x[len(x) // 2], y[len(y) // 2], "{}".format(idx), fontsize=11)
            idx += 1
            x = []
            y = []
        return idx

    def show(self, bo=False, reverse=False):
        idx = self.plot(bo=bo, reverse=reverse)
        print("(**SHOW**) executed : PathSet.show()", " lines = {}".format(idx - 1))
        plt.show()

    def connecting_lanes_straight(self, connection_list):
        out_lane_list = []
        for connection in connection_list:
            beforepath = self.path_list[connection[0]]
            afterpath = self.path_list[connection[1]]
            connecting_lane = Lane(startpoint=beforepath.point_list[-1], endpoint=afterpath.point_list[0],
                                   straight=True)
            out_lane_list.append(connecting_lane)
        out_laneset = LaneSet(lane_list=out_lane_list)
        return PathSet(laneset=out_laneset)

    def connecting_lanes_curved(self, connection_list):
        out_lane_list = []
        for connection in connection_list:
            beforepath = self.path_list[connection[0]]
            afterpath = self.path_list[connection[1]]
            connecting_lane = Lane(startpoint=beforepath.point_list[-1], start_dir=beforepath.direction_list[-1],
                                   endpoint=afterpath.point_list[0], end_dir=afterpath.direction_list[0], curve=True)
            connecting_lane = connecting_lane.make_uniform_intervals(0.99)
            out_lane_list.append(connecting_lane)
        out_laneset = LaneSet(lane_list=out_lane_list)
        return PathSet(laneset=out_laneset)

    def wayset(self):
        out_way_list = []
        for path in self.path_list:
            out_way_list.append(path.way())
        return WaySet(out_way_list)


class DraggaleLine:
    lock = None  # only one can be animated at a time

    def __init__(self, line):
        print('__init')
        self.line = line
        self.press = None
        self.background = None

    def connect(self):
        'connect to all the events we need'
        self.cidpress = self.line.figure.canvas.mpl_connect(
            'button_press_event', self.on_press)
        self.cidrelease = self.line.figure.canvas.mpl_connect(
            'button_release_event', self.on_release)
        self.cidmotion = self.line.figure.canvas.mpl_connect(
            'motion_notify_event', self.on_motion)

    def on_press(self, event):
        'on button press we will see if the mouse is over us and store some data'
        if event.inaxes != self.line.axes: return
        if DraggaleLine.lock is not None: return
        contains, attrd = self.line.contains(event)
        if not contains: return
        # print('event contains')
        x0 = list(self.line.get_xdata())
        y0 = list(self.line.get_ydata())
        self.press = x0, y0, event.xdata, event.ydata
        DraggaleLine.lock = self

        # draw everything but the selected rectangle and store the pixel buffer
        canvas = self.line.figure.canvas
        axes = self.line.axes
        self.line.set_animated(True)
        canvas.draw()
        self.background = canvas.copy_from_bbox(self.line.axes.bbox)

        # now redraw just the rectangle
        axes.draw_artist(self.line)

        # and blit just the redrawn area
        canvas.blit(axes.bbox)

    def on_motion(self, event):
        'on motion we will move the rect if the mouse is over us'
        if DraggaleLine.lock is not self:
            return
        if event.inaxes != self.line.axes: return
        x0, y0, xpress, ypress = self.press
        dx = event.xdata - xpress
        dy = event.ydata - ypress
        dx_list = [dx] * len(x0)
        dy_list = [dy] * len(y0)
        new_x = [x+dx for x, dx in zip(x0, dx_list)]
        new_y = [y+dy for y, dy in zip(y0, dy_list)]
        self.line.set_data(new_x, new_y)

        canvas = self.line.figure.canvas
        axes = self.line.axes
        # restore the background region
        canvas.restore_region(self.background)

        # redraw just the current rectangle
        axes.draw_artist(self.line)

        # blit just the redrawn area
        canvas.blit(axes.bbox)

    def on_release(self, event):
        'on release we reset the press data'
        if DraggaleLine.lock is not self:
            return

        self.press = None
        DraggaleLine.lock = None

        # turn off the rect animation property and reset the background
        self.line.set_animated(False)
        self.background = None

        # redraw the full figure
        self.line.figure.canvas.draw()

    def disconnect(self):
        'disconnect all the stored connection ids'
        self.line.figure.canvas.mpl_disconnect(self.cidpress)
        self.line.figure.canvas.mpl_disconnect(self.cidrelease)
        self.line.figure.canvas.mpl_disconnect(self.cidmotion)


class LineBuilder:
    def __init__(self, line):
        self.line = line
        self.xs = list(line.get_xdata())
        self.ys = list(line.get_ydata())
        self.cid = line.figure.canvas.mpl_connect('button_press_event', self)

    def __call__(self, event):
        print('click', event)
        if event.inaxes!=self.line.axes: return
        self.xs.append(event.xdata)
        self.ys.append(event.ydata)
        self.line.set_data(self.xs, self.ys)  # plot
        self.line.figure.canvas.draw()        # show

    def disconnect(self):
        self.line.figure.canvas.mpl_disconnect(self.cid)


class EditablePoints:
    def __init__(self, fig, ax, line):
        self.figure, self.ax, self.line = fig, ax, line
        self.inner_line = None
        self.dragging_point = None
        self.points = {}
        xdata = list(line.get_xdata())
        ydata = list(line.get_ydata())
        for x, y in zip(xdata, ydata):
            self.points[y] = x
        self._connect()
        self._init_plot(xdata, ydata)

    def __del__(self):
        self.disconnect()
        print("deleted!!!!")

    def _connect(self):
        self.pcid = self.figure.canvas.mpl_connect('button_press_event', self._on_press)
        self.rcid = self.figure.canvas.mpl_connect('button_release_event', self._on_release)
        self.mcid = self.figure.canvas.mpl_connect('motion_notify_event', self._on_motion)

    def _init_plot(self, xs, ys):
        xmin = min(xs)
        xmax = max(xs)
        ymin = min(ys)
        ymax = max(ys)
        self.ax.set_xlim(xmin - 10, xmax + 10)
        self.ax.set_ylim(ymin - 10, ymax + 10)
        self.ax.grid(which='both')
        self._update_plot()

    def _update_plot(self):
        if not self.points:
            self.inner_line.set_data([], [])
        else:
            y, x = zip(*sorted(self.points.items()))
            if not self.inner_line:
                self.inner_line, = self.ax.plot(x, y, 'b', marker="o", markersize=10)
            else:
                self.inner_line.set_data(x, y)
        xline = self.inner_line.get_xdata()
        yline = self.inner_line.get_ydata()
        self.line.set_data(xline, yline)
        self.figure.canvas.draw()

    def _add_point(self, x, y=None):
        if isinstance(x, MouseEvent):
            x, y = int(x.xdata), int(x.ydata)
        self.points[y] = x
        return y, x

    def _remove_point(self, y, _):
        if y in self.points:
            self.points.pop(y)

    def _find_neighbor_point(self, event):
        distance_threshold = 3.0
        nearest_point = None
        min_distance = sqrt(2 * (100 ** 2))
        for y, x in self.points.items():
            distance = hypot(event.xdata - x, event.ydata - y)
            if distance < min_distance:
                min_distance = distance
                nearest_point = (y, x)
        if min_distance < distance_threshold:
            return nearest_point
        return None

    def _on_press(self, event):
        # left click
        if event.button == 1 and event.inaxes in [self.ax]:
            point = self._find_neighbor_point(event)
            if point:
                self._add_point(event)
                self.dragging_point = point
                self._remove_point(*point)
            else:
                self._add_point(event)
            self._update_plot()
        # right click
        elif event.button == 3 and event.inaxes in [self.ax]:
            point = self._find_neighbor_point(event)
            if point:
                self._remove_point(*point)
                self._update_plot()

    def _on_release(self, event):
        if event.button == 1 and event.inaxes in [self.ax] and self.dragging_point:
            self._add_point(event)
            self.dragging_point = None
            self._update_plot()

    def _on_motion(self, event):
        if not self.dragging_point:
            return
        self._remove_point(*self.dragging_point)
        self.dragging_point = self._add_point(event)
        self._update_plot()

    def disconnect(self):
        self.figure.canvas.mpl_disconnect(self.pcid)
        self.figure.canvas.mpl_disconnect(self.rcid)
        self.figure.canvas.mpl_disconnect(self.mcid)


class LineBrowser:
    def __init__(self, laneset, fig, ax1, ax2):
        self.figure, self.ax1, self.ax2, self.line_list = fig, ax1, ax2, []
        self.lastline = None
        self.selected = None
        self.laneset = laneset
        self.focused = None
        self._init_plot()

    def __del__(self):
        del self.focused

    def _connect(self):
        self.pick_cid = self.figure.canvas.mpl_connect('pick_event', self._on_pick)

    def _init_plot(self):
        min_max_xy = self.laneset.min_max_xy()
        self.ax1.set_xlim(min_max_xy[0] - 20, min_max_xy[2] + 20)
        self.ax1.set_ylim(min_max_xy[1] - 20, min_max_xy[3] + 20)
        self.ax2.set_xlim(0, 40)
        self.ax2.set_ylim(0, 40)
        self.line_list = self.laneset.sub_plot4(self.ax1)
        self._connect()

    def _on_pick(self, event):
        if self.lastline is not None and self.selected is not None:
            self.lastline.set_visible(True)
            self.selected.set_visible(False)
        if self.focused is not None:
            self.focused.disconnect()
            del self.focused
        thisline = event.artist
        self.lastline = thisline
        thisline.set_visible(False)
        x = thisline.get_xdata()
        y = thisline.get_ydata()
        self.selected, = self.ax1.plot(x, y, color='yellow')
        self._updata(event)

    def _updata(self, event):
        self.ax2.cla()
        plot_bag_reference2(self.ax2, ro=False, dash=True)
        plot_bag_reference2_addition(self.ax2, ro=False, dash=True)
        plot_bag_reference_clicked_point_ax(self.ax2, ro=False, dash=True)
        if self.lastline is not None:
            x = self.lastline.get_xdata()
            y = self.lastline.get_ydata()
        else:
            self.lastline = event.artist
            x = self.lastline.get_xdata()
            y = self.lastline.get_ydata()
        self.focused = EditablePoints(self.figure, self.ax2, self.lastline)

    def disconnect(self):
        self.figure.canvas.mpl_disconnect(self.pick_cid)


    #  Waypoint/Waypiece/Way/WaySet for making csv

class Waypoint:
    def __init__(self, point, direction, r, pid=-1, valid=True):
        self.x = point.x
        self.y = point.y
        self.z = point.z
        self.dir = direction
        self.r = r
        self.pid = pid
        self.valid = valid

    def __str__(self):
        return '<Waypoint %s, %s, %s, %s, %s>' % (self.x, self.y, self.z, self.dir, self.r)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and id(self) != id(other)  # and self.z == other.z

    def point(self):
        return Point(self.x, self.y, self.z)

    def set_pid(self, pid):
        self.pid = pid

    def invalidate(self):
        self.valid = False


class Waypiece:
    def __init__(self, front_waypoint, back_waypoint, lid=-1, before_waypiece=None, after_waypiece=None):
        self.front_pt = front_waypoint
        self.back_pt = back_waypoint
        self.lid = lid
        self.before_piece = before_waypiece
        self.after_piece = after_waypiece

    def __str__(self):
        return '<Waypiece lid:%s, flid:%s, lid:%s, bnid:%s, fnid:%s>' \
               % (self.lid, self.front_pt, self.back_pt, self.before_piece, self.after_piece)

    def set_lid(self, lid):
        self.lid = lid


class Way:
    def __init__(self, waypoint_list):  # circle:bp=ep, af=fp / branch: bp, ap
        self.waypoint_list = waypoint_list
        self.waypiece_list = []

        for i in range(len(self.waypoint_list)-1):  # WayPoints
            wpiece = Waypiece(self.waypoint_list[i], self.waypoint_list[i+1])
            self.waypiece_list.append(wpiece)

        for i in range(len(self.waypiece_list)-1):  # WayPieces   /// it breaks if point_list has sequence problem
            if self.waypiece_list[i].back_pt == self.waypiece_list[i+1].front_pt:
                self.waypiece_list[i].after_piece = self.waypiece_list[i+1]
                self.waypiece_list[i+1].before_piece = self.waypiece_list[i]

    def valid_points(self):
        if not self.waypoint_list[0].valid:
            del self.waypoint_list[0]
        if not self.waypoint_list[-1].valid:
            del self.waypoint_list[-1]

    def set_pid(self, next_pid):    # in WaySet
        for wp in self.waypoint_list:
            wp.set_pid(next_pid)
            next_pid += 1
        return next_pid

    def set_lid(self, next_lid):    # in WaySet
        for wpiece in self.waypiece_list:
            wpiece.set_lid(next_lid)
            next_lid += 1
        return next_lid


class WaySet:
    def __init__(self, way_list):
        global START_ID
        self.way_list = way_list
        self.next_pid = START_ID
        self.next_lid = START_ID
        self._figure_circle()  # may invalidate last point of circle            and set circle
        self._figure_branch()  # may invalidate first and last point of branch  and set branch
        self._valid_points()
        for way in self.way_list:
            self.next_pid = way.set_pid(self.next_pid)  # change next_pid
        for way in self.way_list:
            self.next_lid = way.set_lid(self.next_lid)  # change next_lid

    def __add__(self, operand):
        out_way_list = self.way_list + operand.way_list
        return WaySet(out_way_list)

    def num_of_points(self):
        return self.next_pid - 1

    def num_of_lines(self):
        return self.next_lid - 1

    def _figure_circle(self):
        for way in self.way_list:  # figure circle lanes
            if way.waypiece_list[-1].back_pt == way.waypiece_list[0].front_pt:  # circle, drop last point
                way.waypiece_list[-1].back_pt.invalidate()
                way.waypiece_list[-1].back_pt = way.waypiece_list[0].front_pt
                way.waypiece_list[-1].after_piece = way.waypiece_list[0]
                way.waypiece_list[0].before_piece = way.waypiece_list[-1]

    def _figure_branch(self):  #except first and last point
        for way1 in self.way_list:
            for way2 in self.way_list:
                for i in range(len(way2.waypiece_list)-1):  # except Last and first point
                    # use i, except last point of way2
                    if way1.waypiece_list[0].front_pt == way2.waypiece_list[i].back_pt:
                        way1.waypiece_list[0].front_pt.invalidate()
                        way1.waypiece_list[0].front_pt = way2.waypiece_list[i].back_pt
                        way1.waypiece_list[0].before_piece = way2.waypiece_list[i]
                        # do not change way2.waypiece_list[i].after_piece. it has already proper after_piece
                    # use i+1, except first point of way2
                    if way1.waypiece_list[-1].back_pt == way2.waypiece_list[i+1].front_pt:
                        way1.waypiece_list[-1].back_pt.invalidate()
                        way1.waypiece_list[-1].back_pt = way2.waypiece_list[i+1].front_pt
                        way1.waypiece_list[-1].after_piece = way2.waypiece_list[i+1]

    def _valid_points(self):
        for way in self.way_list:
            way.valid_points()


""" 
     Functions 
                """


def read_wkt_csv_file(wkt_csv_file):  # read WKT whose format is LineString ZM(lat lon alt, lat lon alt ... 0) not ECEF xyz.
    with open(wkt_csv_file, mode='r', encoding='utf-8') as f:
        line_list = []
        line = []
        reader = csv.reader(f, delimiter=',')  # read csv_file(wkt LineString data)
        reader.__next__()  # skip the first line
        for row in reader:
            ls = row[0].replace("LINESTRING ZM (", "")  # There may be other case, 'LineString Z ('
            ls = ls[:-3]  # There may be other case, [:-1]
            points_in_ls = ls.split(",")
            for point_string in points_in_ls:
                point_xyz = point_string.split(" ")
                # point = Vector(float(point_xyz[0]), float(point_xyz[1]), float(point_xyz[2]))  # 소수점 3자리로 고정하면 라인스트링이 이상한 모습
                point = Point(float(point_xyz[1]), float(point_xyz[0]),
                              float(point_xyz[2]))  # to make in the order of 'lat, lon, alt' [0]=lon, [1]=lat, [2]=alt
                line.append(point)
            line_list.append(line)
            line = []
        # _line_num = reader.line_num - 1                               # remove the first line
    print("(step 1) executed : build_line_list")
    return line_list


def show_line_list(line_list):
    plt.xlabel("coordinate X")
    plt.ylabel("coordinate Y")
    plt.title("vector_lists")
    x_list = []
    y_list = []
    for line in line_list:
        for vector in line:
            x_list.append(vector.x)
            y_list.append(vector.y)
        plt.plot(x_list, y_list)
        x_list = []
        y_list = []
    print("(**print**) executed : show_line_lists")
    plt.show()


def _change_autoware_tf_to_normal_tf(in_matrix):
    # change autoware-based matrix to normal cartesian
    _out_matrix = in_matrix
    for row in _out_matrix:
        _temp = row[0]
        row[0] = row[1]
        row[1] = _temp
    return _out_matrix


def locate_wgs84_in_kcity_map(point):  # Contributor : Hannah.B
    tf_matrix = [[-0.0952712880335145617660685957162, 0.0356186920600441853101969513773, -1.32788380636286018621206039825, 5593785.56023157481104135513306],
                 [-0.813633188344384805645859159995, -0.639655226726892789379519399517, -0.168181160269529478279437739729, 2011961.50533704226836562156677],
                 [1.04092967825777282797616862808, -0.735162337170987334467042728647, 0.97708460208008451886030343303, -10227112.8726466950029134750366 ]]
    tf_matrix = _change_autoware_tf_to_normal_tf(tf_matrix)
    ecef_point = geocentric_forward(point.x, point.y, point.z)  # in the order of 'lat, lon, alt' (this is handled in read_wkt_csv_file, x=lat, y=lon, z=alt)
    kcity_point_x = tf_matrix[0][0] * ecef_point[0] + tf_matrix[0][1] * ecef_point[1] + tf_matrix[0][2] * ecef_point[2] + tf_matrix[0][3] - 35
    kcity_point_y = tf_matrix[1][0] * ecef_point[0] + tf_matrix[1][1] * ecef_point[1] + tf_matrix[1][2] * ecef_point[2] + tf_matrix[1][3] - 2
    kcity_point_z = tf_matrix[2][0] * ecef_point[0] + tf_matrix[2][1] * ecef_point[1] + tf_matrix[2][2] * ecef_point[2] + tf_matrix[2][3]
    return Point(kcity_point_y, kcity_point_x, kcity_point_z)  # Autoware requires to swap x and y!!!


def transform_wgs84_map_to_kcity_map(in_line_list):
    _out_line_list = []
    _out_line = []
    for line in in_line_list:
        for point in line:
            _out_point = locate_wgs84_in_kcity_map(point)
            _out_line.append(_out_point)
        _out_line_list.append(_out_line)
        _out_line = []
    print("(step 1) executed : transform_wgs84_map_to_kcity_map")
    return _out_line_list


def rotate(point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians.
    """
    px = point.x
    py = point.y
    qx = math.cos(angle) * px - math.sin(angle) * py
    qy = math.sin(angle) * px + math.cos(angle) * py
    return Point(qx, qy, point.z)


def translate(point, x, y):
    outx = point.x + x
    outy = point.y + y
    return Point(outx, outy, point.z)


def make_curve(endpoint, end_radian):
    k0 = 0.0
    states = [[endpoint.x, endpoint.y, end_radian]]
    result = generate_path(states, k0)
    xc, yc, yawc = motion_model.generate_trajectory(
        result[0][3], result[0][4], result[0][5], k0)

    plt.plot(xc, yc, 'bo')
    plt.grid(True)
    plt.axis("equal")
    plt.show()

    unit_z = endpoint.z / (len(xc) - 1)
    zc = []
    for i in range(len(xc)):
        zc.append(i * unit_z)
    point_list = []
    for i in range(len(xc)):
        point_list.append(Point(xc[i], yc[i], zc[i]))
    point_list.insert(0, Point(0, 0, 0))  # 첫점 일치
    return point_list


def calc_x_delta_wrt_line(nominal_endp, actual_endp1, actual_endp2):
    x1 = actual_endp1.x
    y1 = actual_endp1.y
    x2 = actual_endp2.x
    y2 = actual_endp2.y
    slope = (y2 - y1) / (x2 - x1)
    x = (nominal_endp.y + slope * x1 - y1) / slope
    delta = nominal_endp.x - x
    return delta


def make_curve2(endpoint, end_radian):
    k0 = 0.0
    states = [[endpoint.x, endpoint.y, end_radian]]
    result = generate_path(states, k0)
    xc, yc, yawc = motion_model.generate_trajectory(
        result[0][3], result[0][4], result[0][5], k0)

    delta = calc_x_delta_wrt_line(endpoint, Point(xc[-1], yc[-1], 0), Point(xc[-2], yc[-2], 0))
    print("delta = " + str(delta))
    delta_list = [delta] * len(xc)
    new_xc = [x1 + x2 for x1, x2 in zip(xc, delta_list)]  # fix to meet first and last point of original
    xy = [(x, y) for x, y in zip(new_xc, yc)]
    xy = [tup for tup in xy if tup[0] > 0]
    xy = [tup for tup in xy if abs(tup[1]) < abs(endpoint.y)]
    new_xc = [tup[0] for tup in xy]
    yc = [tup[1] for tup in xy]
    if planner.show_animation:
        plt.plot(new_xc, yc, 'bo')
        plt.grid(True)
        plt.axis("equal")
        plt.show()

    point_list = []
    for i in range(len(new_xc)):
        point_list.append(Point(new_xc[i], yc[i], 0))
    point_list.insert(0, Point(0, 0, 0))  # 첫점 일치
    point_list.append(Point(endpoint.x, endpoint.y, 0)) # 마지막점 일치
    return point_list


def plot_bag_reference(ro=True, dash=False):
    path_dir = '/home/softkoo/dtlane_ws/rosbag'
    file_list = os.listdir(path_dir)
    file_list.sort()
    x = []
    y = []
    for bag_file in file_list:
        bag = rosbag.Bag("rosbag/" + bag_file)
        for msg in bag.read_messages(topics=['ndt_pose']):
            x.append(msg[1].pose.position.x)
            y.append(msg[1].pose.position.y)
        # plt.text(y[0], x[0], "{},{}".format(y[0], x[0]), fontsize=8)
        # plt.text(y[-1], x[-1], "{},{}".format(y[-1], x[-1]), fontsize=8)
        if ro:
            plt.plot(y, x, 'ro')
        elif dash:
            plt.plot(y, x, linestyle=':')
        bag.close()
        x = []
        y = []


def plot_bag_reference_addition(ro=True, dash=False):
    path_dir = '/home/softkoo/dtlane_ws/rosbag2'
    file_list = os.listdir(path_dir)
    file_list.sort()
    x = []
    y = []
    for bag_file in file_list:
        bag = rosbag.Bag("rosbag2/" + bag_file)
        for msg in bag.read_messages(topics=['ndt_pose']):
            x.append(msg[1].pose.position.x)
            y.append(msg[1].pose.position.y)
        # plt.text(y[0], x[0], "{},{}".format(y[0], x[0]), fontsize=8)
        # plt.text(y[-1], x[-1], "{},{}".format(y[-1], x[-1]), fontsize=8)
        if ro:
            plt.plot(y, x, 'b')
        elif dash:
            plt.plot(y, x, linestyle=':')
        bag.close()
        x = []
        y = []


def plot_bag_reference_addition2(ro=True, dash=False):
    path_dir = '/home/softkoo/dtlane_ws/rosbag3'
    file_list = os.listdir(path_dir)
    file_list.sort()
    x = []
    y = []
    for bag_file in file_list:
        bag = rosbag.Bag("rosbag3/" + bag_file)
        for msg in bag.read_messages(topics=['ndt_pose']):
            x.append(msg[1].pose.position.x)
            y.append(msg[1].pose.position.y)
        # plt.text(y[0], x[0], "{},{}".format(y[0], x[0]), fontsize=8)
        # plt.text(y[-1], x[-nn
        if ro:
            plt.plot(y, x, 'g')
        elif dash:
            plt.plot(y, x, linestyle=':')
        bag.close()
        x = []
        y = []


def plot_bag_reference2(ax, ro=True, dash=False):   # it has 'ax' field in addition to ver.1
    path_dir = '/home/softkoo/dtlane_ws/rosbag'
    file_list = os.listdir(path_dir)
    file_list.sort()
    x = []
    y = []
    for bag_file in file_list:
        bag = rosbag.Bag("rosbag/" + bag_file)
        for msg in bag.read_messages(topics=['ndt_pose']):
            x.append(msg[1].pose.position.x)
            y.append(msg[1].pose.position.y)
        # plt.text(y[0], x[0], "{},{}".format(y[0], x[0]), fontsize=8)
        # plt.text(y[-1], x[-1], "{},{}".format(y[-1], x[-1]), fontsize=8)
        if ro:
            ax.plot(y, x, 'ro')
        elif dash:
            ax.plot(y, x, linestyle=':')
        bag.close()
        x = []
        y = []


def plot_bag_reference2_addition(ax, ro=True, dash=False):   # it has 'ax' field in addition to ver.1
    path_dir = '/home/softkoo/dtlane_ws/rosbag3'
    file_list = os.listdir(path_dir)
    file_list.sort()
    x = []
    y = []
    for bag_file in file_list:
        bag = rosbag.Bag("rosbag3/" + bag_file)
        for msg in bag.read_messages(topics=['ndt_pose']):
            x.append(msg[1].pose.position.x)
            y.append(msg[1].pose.position.y)
        # plt.text(y[0], x[0], "{},{}".format(y[0], x[0]), fontsize=8)
        # plt.text(y[-1], x[-1], "{},{}".format(y[-1], x[-1]), fontsize=8)
        if ro:
            ax.plot(y, x, 'g')
        elif dash:
            ax.plot(y, x, linestyle=':')
        bag.close()
        x = []
        y = []


def plot_bag_reference_clicked_point_ax(ax, ro=True, dash=False):
    path_dir = '/home/softkoo/dtlane_ws/clicked_points_rosbag'
    file_list = os.listdir(path_dir)
    file_list.sort()
    x = []
    y = []
    for bag_file in file_list:
        bag = rosbag.Bag("clicked_points_rosbag/" + bag_file)
        for msg in bag.read_messages(topics=['clicked_point']):
            x.append(msg[1].point.x)
            y.append(msg[1].point.y)
        # plt.text(y[0], x[0], "{},{}".format(y[0], x[0]), fontsize=8)
        # plt.text(y[-1], x[-nn
        if ro:
            ax.plot(y, x, 'g')
        elif dash:
            ax.plot(y, x, marker='.', linestyle=':')
        bag.close()
        x = []
        y = []


def plot_bag_reference_clicked_point(ro=True, dash=False):
    path_dir = '/home/softkoo/dtlane_ws/clicked_points_rosbag'
    file_list = os.listdir(path_dir)
    file_list.sort()
    x = []
    y = []
    for bag_file in file_list:
        bag = rosbag.Bag("clicked_points_rosbag/" + bag_file)
        for msg in bag.read_messages(topics=['clicked_point']):
            x.append(msg[1].point.x)
            y.append(msg[1].point.y)
        # plt.text(y[0], x[0], "{},{}".format(y[0], x[0]), fontsize=8)
        # plt.text(y[-1], x[-nn
        if ro:
            plt.plot(y, x, 'g')
        elif dash:
            plt.plot(y, x, marker='.', linestyle=':')
        bag.close()
        x = []
        y = []


def write_autoware_dtlane(wayset):
    # write point.csv
    with open('Result/point.csv', 'w', newline='') as point_csv:
        point_writer = csv.writer(point_csv, delimiter=',',
                                  quotechar='|', quoting=csv.QUOTE_MINIMAL)
        point_writer.writerow(
            ['PID'] + ['B(Lat)'] + ['L(Long)'] + ['H'] + ['Bx'] + ['Ly'] + ['ReF'] + ['MCODE1'] + ['MCODE2'] + [
                'MCODE3'])
        for way in wayset.way_list:
            for wp in way.waypoint_list:
                point_writer.writerow([wp.pid] + [0] + [0] + [(float(int(wp.z * 1000)) / 1000)] +
                                      [float(int(wp.x * 1000)) / 1000] + [float(int(wp.y * 1000)) / 1000] +
                                      [0] + [0] + [0] + [0])
    # write node.csv
    with open('Result/node.csv', 'w', newline='') as node_csv:
        node_writer = csv.writer(node_csv, delimiter=',',
                                 quotechar='|', quoting=csv.QUOTE_MINIMAL)
        node_writer.writerow(['NID'] + ['PID'])
        for way in wayset.way_list:  # LineString 개수
            for wp in way.waypoint_list:  # Point 개수
                node_writer.writerow([wp.pid] + [wp.pid])

    # write dtlane.csv
    with open('Result/dtlane.csv', 'w', newline='') as dtlane_csv:
        dtlane_writer = csv.writer(dtlane_csv, delimiter=',',
                                   quotechar='|', quoting=csv.QUOTE_MINIMAL)
        dtlane_writer.writerow(
            ['DID'] + ['Dist'] + ['PID'] + ['Dir'] + ['Apara'] + ['r'] + ['slope'] + ['cant'] + ['LW'] + ['RW'])
        for i in range(len(wayset.way_list)):
            for j in range(len(wayset.way_list[i].waypoint_list)):
                dtlane_writer.writerow([wayset.way_list[i].waypoint_list[j].pid] + [j] +
                                       [wayset.way_list[i].waypoint_list[j].pid] +
                                       [wayset.way_list[i].waypoint_list[j].dir] + [0] +
                                       [wayset.way_list[i].waypoint_list[j].r] + [0] + [0] + [2] + [2])

    # write lane.csv
    with open('Result/lane.csv', 'w', newline='') as lane_csv:  # use only Waypieces
        lane_writer = csv.writer(lane_csv, delimiter=','
                                 , quotechar='|', quoting=csv.QUOTE_MINIMAL)
        lane_writer.writerow(
            ['LnID'] + ['DID'] + ['BLID'] + ['FLID'] + ['BNID'] + ['FNID'] + ['JCT'] + ['BLID2'] + ['BLID3'] + [
                'BLID4'] + ['FLID2'] + ['FLID3'] + ['FLID4'] + ['CrossID'] + ['Span'] + ['LCnt'] + ['Lno'] + [
                'LaneType'] + ['LimitVel'] + ['RefVel'] + ['RoadSecID'] + ['LaneChgFG'])

        for way in wayset.way_list:
            for waypiece in way.waypiece_list:
                lane_writer.writerow([waypiece.lid] + [waypiece.front_pt.pid]
                                     + [0 if waypiece.before_piece is None else waypiece.before_piece.lid]
                                     + [0 if waypiece.after_piece is None else waypiece.after_piece.lid]
                                     + [waypiece.front_pt.pid]
                                     + [waypiece.back_pt.pid] + [0] + [0] + [0] + [0]
                                     + [0] + [0] + [0] + [0] + [1.000] + [1]
                                     + [1] + [0] + [20] + [20] + [0] + [0] + [0])


def choose_loop(title, in_laneset, mode, bo=False, reverse=True):
    control = 'y'
    out_laneset = in_laneset
    while control == 'y':
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title(title)
        out_laneset.sub_plot(ax, bo=True)
        draw_line, = ax.plot([], [])
        line_builder = LineBuilder(draw_line)
        plt.show()

        verts = zip(line_builder.xs, line_builder.ys)
        if len(line_builder.xs) == 0:
            out_laneset.show(bo=bo, reverse=reverse)
        else:
            if mode == 't':
                out_laneset = out_laneset.lanes_in_area_of_interest(verts)
                out_laneset.show(bo=bo, reverse=reverse)
            elif mode == 'r':
                out_laneset.remove_points_in_area(verts)
                out_laneset.show(bo=bo, reverse=reverse)
            else:
                print("Error!!! wrong parameter Take_Remove")
                raise EnvironmentError
        control = input("more " + title + " [y/n] ")
        if control != 'y':
            line_builder.disconnect()
            del line_builder
    return out_laneset


def extract_lanes(title, in_laneset, bo=False, reverse=True):
    control = 'y'
    add_lane_list = []
    add_laneset = None
    while control == 'y':
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title(title)
        in_laneset.sub_plot(ax, ep=True)
        draw_line, = ax.plot([], [])
        line_builder = LineBuilder(draw_line)
        plt.show()

        verts = zip(line_builder.xs, line_builder.ys)
        if len(line_builder.xs) == 0:
            pass
        else:
            extracted_lane = in_laneset.points_in_area_of_interest(verts)
            extracted_lane.show()
            add_lane_list.append(extracted_lane)
            add_laneset = LaneSet(lane_list=add_lane_list)
            add_laneset.show()
        control = input('more ' + title + "? [y/n]")
        if control != 'y':
            line_builder.disconnect()
            del line_builder
    return add_laneset


def drop_lanes(in_laneset):
    control = 'y'
    out_laneset = in_laneset
    while control == 'y':
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title("drop lanes")
        out_laneset.sub_plot_idx(ax, reverse=True)
        plt.show()

        idx_list = []
        control2 = 'y'
        while control2 == 'y':
            idx = input("enter the index of lane you want to remove")
            idx_list.append(int(idx))

            undo = input("Ah!, undo! [y/n]")
            if undo == 'y':
                del idx_list[-1]

            control2 = input("more index? [y/n]")
        out_laneset = out_laneset.remove_lanes(idx_list)
        out_laneset.show_idx(reverse=True)

        control = input("more? [y/n]")
    return out_laneset


def read_rosbag(path, folder):
    f_list = os.listdir(path)
    f_list.sort()
    rosbag_point_list = []
    rosbag_lane_list = []
    for bag_file in f_list:
        bag = rosbag.Bag(folder + "/" + bag_file)
        for msg in bag.read_messages(topics=['ndt_pose']):
            p = Point(msg[1].pose.position.y, msg[1].pose.position.x, msg[1].pose.position.z)
            rosbag_point_list.append(p)
        for msg in bag.read_messages(topics=['/ndt_pose']):
            p = Point(msg[1].pose.position.y, msg[1].pose.position.x, msg[1].pose.position.z)
            rosbag_point_list.append(p)
        rosbag_lane_list.append(Lane(point_list=rosbag_point_list))
        rosbag_point_list = []
        bag.close()
    return LaneSet(lane_list=rosbag_lane_list)


def read_point_csv(path, folder):
    f_list = os.listdir(path)
    f_list.sort()
    point_csv_point_list = []
    point_csv_lane_list = []
    for point_csv in f_list:
        with open(folder+'/'+point_csv, mode='r', encoding='utf-8') as f:
            reader = csv.reader(f, delimiter=',')
            reader.__next__()
            for row in reader:
                p = Point(float(row[4]), float(row[5]), float(row[3]))
                point_csv_point_list.append(p)
            point_csv_lane_list.append(Lane(point_list=point_csv_point_list))
            point_csv_point_list = []
    return LaneSet(lane_list=point_csv_lane_list)


""" execution """

# wgs84_line_list        :     list of wgs84(lat, lon, alt)
# line = point_list      :     list of points
# lane                   :     Lane obj
# laneset                :     LaneSet obj


