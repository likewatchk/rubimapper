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

folder = 'point'
point_csv_laneset = read_point_csv(PATH + folder, folder)
point_csv_laneset.show(reverse=True)
point_csv_laneset = extract_lanes("take interesting part", in_laneset=point_csv_laneset)

pathset = PathSet(laneset=point_csv_laneset)
pathset.show(reverse=True)


wayset = pathset.wayset()  # figure circles and branchs / initialize ID of points and line
write_autoware_dtlane(wayset)
