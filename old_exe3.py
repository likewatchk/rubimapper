from rubimapperold import *

PATH = '/home/softkoo/dtlane_ws/'
INTERVAL = 0.99

folder = 'point'
point_csv_laneset = read_point_csv(PATH + folder, folder)
point_csv_laneset.show(reverse=True)
point_csv_laneset = extract_lanes("take interesting part", in_laneset=point_csv_laneset)

pathset = PathSet(laneset=point_csv_laneset)
pathset.show(reverse=True)


wayset = pathset.wayset()  # figure circles and branchs / initialize ID of points and line
write_autoware_dtlane(wayset)
