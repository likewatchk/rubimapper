"""
Copyright [2019] [Sungwoo Koo(likewatchk@gmail.com), Kanghee Kim(kim.kanghee@gmail.com), Soongsil Univ.]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

from rubimapperold import *

PATH = '/home/softkoo/dtlane_ws/'
INTERVAL = 0.99

# drag_and_pick = True
drag_and_pick = False
adjusting_z = False

""" step 1 : extract points ( WKT(Well-Known-Text) : LineString file whose coordinates are extracted from shp ) """
wgs84_line_list = read_wkt_csv_file('A3_LINK_2019new.csv')
line_list = transform_wgs84_map_to_kcity_map(wgs84_line_list)
# show_line_list(line_list)

""" step 2 : connect adjacent lanes and make intervals """
laneset = LaneSet(line_list=line_list)
connected_laneset = laneset.connect_split_lanes()
connected_laneset.show(reverse=True)

""" step 3 : cut area with mouse """
connected_laneset = choose_loop("take interesting part", in_laneset=connected_laneset, mode='t')
connected_laneset = choose_loop("remove points", in_laneset=connected_laneset, mode='r', bo=True)
connected_laneset = connected_laneset.split_connected_lanes()
connected_laneset = drop_lanes(connected_laneset)


# folder = 'rosbag_test_0707_2'
# current_pose_laneset = read_rosbag(PATH + folder, folder)
# current_pose_laneset = extract_lanes("take interesting part", in_laneset=current_pose_laneset)
# if current_pose_laneset is None:
#     pass
# else:
#     connected_laneset = connected_laneset + current_pose_laneset
#     connected_laneset.show(reverse=True)

# folder = 'rosbag_merge5'
# current_pose_laneset = read_rosbag(PATH + folder, folder)
# current_pose_laneset = extract_lanes("take interesting part", in_laneset=current_pose_laneset)
# if current_pose_laneset is None:
#     pass
# else:
#     connected_laneset = connected_laneset + current_pose_laneset
#     connected_laneset.show(reverse=True)

# folder = 'workzone'
# point_csv_laneset = read_point_csv(PATH + folder, folder)
# point_csv_laneset.show(reverse=True)
# point_csv_laneset = extract_lanes("take interesting part", in_laneset=point_csv_laneset)
# # point_csv_laneset = point_csv_laneset.split_connected_lanes(meter=25)
# # point_csv_laneset.eliminate_duplication()
# point_csv_laneset.show(reverse=True)
# if point_csv_laneset is None:
#     pass
# else:
#     connected_laneset = connected_laneset + point_csv_laneset
#     connected_laneset.show(reverse=True)

folder = 'point2'
point_csv_laneset = read_point_csv(PATH + folder, folder)
point_csv_laneset.show(reverse=True)
point_csv_laneset = extract_lanes("take interesting part", in_laneset=point_csv_laneset)
# point_csv_laneset = point_csv_laneset.split_connected_lanes(meter=25)
# point_csv_laneset.eliminate_duplication()
point_csv_laneset.show(reverse=True)
if point_csv_laneset is None:
    pass
else:
    connected_laneset = connected_laneset + point_csv_laneset
    connected_laneset.show(reverse=True)

folder = 'point1'
point_csv_laneset = read_point_csv(PATH + folder, folder)
point_csv_laneset.show(reverse=True)
point_csv_laneset = extract_lanes("take interesting part", in_laneset=point_csv_laneset)
# point_csv_laneset = point_csv_laneset.split_connected_lanes(meter=25)
# point_csv_laneset.eliminate_duplication()
point_csv_laneset.show(reverse=True)
if point_csv_laneset is None:
    pass
else:
    connected_laneset = connected_laneset + point_csv_laneset
    connected_laneset.show(reverse=True)

# folder = 'taewook_rosbag2'
# current_pose_laneset = read_rosbag(PATH + folder, folder)
# current_pose_laneset = extract_lanes("take interesting part", in_laneset=current_pose_laneset)
# if current_pose_laneset is None:
#     pass
# else:
#     connected_laneset = connected_laneset + current_pose_laneset
#     connected_laneset.show(reverse=True)


connected_laneset = choose_loop("remove points", in_laneset=connected_laneset, mode='r', bo=True)
connected_laneset = connected_laneset.split_connected_lanes(meter=25)
connected_laneset = connected_laneset.make_uniform_intervals(INTERVAL)
connected_laneset = drop_lanes(connected_laneset)


""" step 4 : Drag and fit """

if drag_and_pick:
    # in Lanes
    control2_temp = 'y'
    while control2_temp == 'y':
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title("drag and fix in LaneSet")
        plot_bag_reference()
        plot_bag_reference_addition()
        plot_bag_reference_addition2()
        plot_bag_reference_clicked_point(ro=False, dash=True)
        line = connected_laneset.sub_plot2(ax, bo=True)
        draggable_line = DraggaleLine(line)
        draggable_line.connect()

        plt.show()

        x = connected_laneset.lane_list[0].point_list[0].x
        y = connected_laneset.lane_list[0].point_list[0].y
        new_x = list(line.get_xdata())[0]
        new_y = list(line.get_ydata())[0]
        dx = new_x - x
        dy = new_y - y
        connected_laneset = connected_laneset.translate(dx, dy)

        # plot_bag_reference()
        # connected_laneset.show(bo=True)

        control2_temp = input('more? : [y/n]')
        if control2_temp != 'y':
            draggable_line.disconnect()


    # in Lane
    control3_temp = 'y'
    while control3_temp == 'y':
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title("drag and fix in Lane")
        plot_bag_reference(ro=False, dash=True)
        plot_bag_reference_clicked_point(ro=False, dash=True)
        plot_bag_reference_addition2()
        dls = connected_laneset.sub_plot3(ax)

        plt.show()
        out_lane_list = []
        for dl, lane in itertools.zip_longest(dls, connected_laneset.lane_list):
            x = lane.point_list[0].x
            y = lane.point_list[0].y
            new_x = list(dl.line.get_xdata())[0]
            new_y = list(dl.line.get_ydata())[0]
            dx = new_x - x
            dy = new_y - y
            out_lane_list.append(lane.translate(dx, dy))
        connected_laneset = LaneSet(lane_list=out_lane_list)

        plot_bag_reference()
        connected_laneset.show()

        control3_temp = input('more? : [y/n]')
        if control3_temp != 'y':
            for dl in dls:
                dl.disconnect()
                del dl

    # in Point
    control4_temp = 'y'
    while control4_temp == 'y':
        fig = plt.figure()
        ax1 = fig.add_subplot(121)
        ax2 = fig.add_subplot(122)
        plot_bag_reference2(ax1, ro=False, dash=True)
        plot_bag_reference2_addition(ax1, ro=False, dash=True)
        plot_bag_reference_clicked_point_ax(ax1, ro=False, dash=True)
        lb = LineBrowser(connected_laneset, fig, ax1, ax2)
        plt.show()

        plot_line_list = lb.line_list
        out_lane_list = []
        for pl, lane in itertools.zip_longest(plot_line_list, connected_laneset.lane_list):
            xdata = list(pl.get_xdata())
            ydata = list(pl.get_ydata())
            zdata = lane.z_list()
            out_point_list = []
            ran = min(len(xdata), len(zdata))
            for i in range(ran):
                pt = Point(xdata[i], ydata[i], zdata[i])
                out_point_list.append(pt)
            out_lane = Lane(point_list=out_point_list)
            out_lane_list.append(out_lane)
        connected_laneset = LaneSet(lane_list=out_lane_list)

        plot_bag_reference()
        connected_laneset.show(bo=True)

        control4_temp = input('more? : [y/n]')
        if control4_temp != 'y':
            lb.disconnect()
            del lb


""" step 5 : fix direction of lane and connect intersections """
connected_laneset = drop_lanes(connected_laneset)

pathset = PathSet(laneset=connected_laneset)
# pathset.show(reverse=True)

control5_temp = 'y'   # apply reverse
while control5_temp == 'y':
    fig = plt.figure()
    ax = fig.add_subplot(111)
    pathset.plot(reverse=True)
    ax.set_title("reverse lane")
    plt.show()

    control5_1 = 'y'
    while control5_1 != 'q':
        reverse_idx = input('Enter the idx to reverse')
        laneset = pathset.laneset()
        laneset.lane_list[int(reverse_idx)].point_list.reverse()
        # pathset.path_list[int(reverse_idx)] = Path(lane=Lane(pathset.path_list[int(reverse_idx)].point_list.reverse()))
        pathset = PathSet(laneset=laneset)
        pathset.show(reverse=True)
        control5_1 = input('quit? (q)')

    control5_temp = input('more? (y/n)')


control6_temp = 'y'  # enter two index to making connecting lane between the two lane registered.
while control6_temp == 'y':
    fig = plt.figure()
    ax = fig.add_subplot(111)
    pathset.plot(reverse=True)
    ax.set_title("make connection_idx_list")
    plt.show()

    control6_choice_loop = True
    while control6_choice_loop is True:
        choice = input('(s)traight or (c)urve')
        if choice == 'c':
            control6_choice_loop = False
            curve_connection_list = []
            control6_1 = ''
            while control6_1 != 'q' and control6_1 != 'Q':
                before = input('Enter before lane index')
                after = input('Enter after lane index')
                curve_connection_list.append([int(before), int(after)])
                control6_1 = input("quit? (enter q/Q)")

            pathset2 = pathset.connecting_lanes_curved(curve_connection_list)
            pathset = PathSet(laneset=pathset.laneset() + pathset2.laneset())
            pathset.show(reverse=True)
        elif choice == 's':
            control6_choice_loop = False
            straight_connection_list = []
            control6_2 = ''
            while control6_2 != 'q' and control6_2 != 'Q':
                before = input('Enter before lane index')
                after = input('Enter after lane index')
                straight_connection_list.append([int(before), int(after)])
                control6_2 = input("quit? (enter q/Q)")

            pathset2 = pathset.connecting_lanes_straight(straight_connection_list)
            pathset = PathSet(laneset=pathset.laneset() + pathset2.laneset())
            pathset.show(reverse=True)
        elif choice == 'p':
            control6_choice_loop = False
        else:
            control6_choice_loop = True

    plot_bag_reference()
    plot_bag_reference_clicked_point(ro=False, dash=True)
    pathset.show()

    curve_connection_list = []
    straight_connection_list = []
    control6_temp = input('more? : [y/n]')


print('connecting split lanes')
pathset = PathSet(laneset=pathset.laneset().connect_split_lanes())
print('connected split lanes')

if adjusting_z:
    print("adjusting Z_coordinate")
    folder = 'rosbag'
    current_pose_laneset = read_rosbag(PATH + folder, folder)
    connected_laneset = pathset.laneset()
    connected_laneset = connected_laneset.adjust_z_coordinate_to(current_pose_laneset)
    pathset = PathSet(laneset=connected_laneset)
    print("adjusted Z_coordinate")

connected_laneset = pathset.laneset()
connected_laneset = drop_lanes(connected_laneset)
pathset = PathSet(laneset=connected_laneset)

print('smoothing')
pathset.show(bo=True)
pathset = pathset.smoothing()
print('smoothed')
pathset.show(bo=True)

plot_bag_reference(ro=False, dash=True)
pathset.show()


""" Final step : make dtlane.csv, point.csv, node.csv, lane.csv """

wayset = pathset.wayset()  # figure circles and branchs / initialize ID of points and line
write_autoware_dtlane(wayset)
