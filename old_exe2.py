from rubimapperold import *

PATH = '/home/softkoo/dtlane_ws/'
INTERVAL = 0.99

folder = 'rosbag_test'
current_pose_laneset = read_rosbag(PATH + folder, folder)
current_pose_laneset.show(reverse=True)
current_pose_laneset = extract_lanes("take interesting part", in_laneset=current_pose_laneset)

pathset = PathSet(laneset=current_pose_laneset)
pathset.show(reverse=True)

wayset = pathset.wayset()  # figure circles and branchs / initialize ID of points and line
write_autoware_dtlane(wayset)
