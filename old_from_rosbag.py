from rubimapperold import *

PATH = '/home/softkoo/project/rubimapper/rubimapper-master/'
INTERVAL = 0.99
control_picking = 'y'
# control_picking = 'n'
control_reverse = 'y'   # apply reverse
# control_reverse = 'n'   # not apply reverse
control_connecting = 'y'  # enter two index to making connecting lane between the two lane registered.
# control_connecting = 'n'  # do not use connecting

## RUBIS saved vetormap loading
folder = 'rosbag_hayeon'
current_pose_laneset = read_rosbag(PATH + folder)
current_pose_laneset.show(reverse=True)
current_pose_laneset = extract_lane_points("take interesting part", in_laneset=current_pose_laneset)
current_pose_laneset = current_pose_laneset.make_uniform_intervals(INTERVAL)


while control_picking == 'y':
    fig = plt.figure()
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)
    # plot_bag_reference2(ax1, ro=False, dash=True)
    # plot_bag_reference2_addition(ax1, ro=False, dash=True)
    # plot_bag_reference_clicked_point_ax(ax1, ro=False, dash=True)
    lb = LineBrowser(current_pose_laneset, fig, ax1, ax2)
    plt.show()
    plot_line_list = lb.line_list
    out_lane_list = []
    for pl, lane in itertools.zip_longest(plot_line_list, current_pose_laneset.lane_list):
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
    current_pose_laneset = LaneSet(lane_list=out_lane_list)
    # plot_bag_reference()
    current_pose_laneset.show(bo=True)
    control_picking = input('more? : [y/n]')
    if control_picking != 'y':
        lb.disconnect()
        del lb

pathset = PathSet(laneset=current_pose_laneset)
pathset.show(reverse=True)


while control_reverse == 'y':
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

    control_reverse = input('more? (y/n)')

while control_connecting == 'y':
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

    # plot_bag_reference()
    # plot_bag_reference_clicked_point(ro=False, dash=True)
    pathset.show()

    curve_connection_list = []
    straight_connection_list = []
    control_connecting = input('more? : [y/n]')


print('connecting split lanes')
pathset = PathSet(laneset=pathset.laneset().connect_split_lanes())
print('connected split lanes')

pathset.show(reverse=True)


wayset = pathset.wayset()  # figure circles and branchs / initialize ID of points and line
write_autoware_dtlane(wayset)
