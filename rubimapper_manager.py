from rubimapper import *
import tkinter
from tkinter import filedialog
import os

INTERVAL = 0.99
working_list = []
working_laneset = LaneSet(line_list=None, lane_list=None)
working_list.append(working_laneset)
working_idx = 0

# Manager main window
rb_manager = tkinter.Tk()
rb_manager.title("Rubimapper++")
rb_manager.geometry("800x450+300+300")
rb_manager.resizable(False, False)

# Frame #1 of main window : Buttons for Funcs
f1_func_window=tkinter.Frame(rb_manager, relief='solid', bd=1)
f1_func_window.pack(side="left", expand=False)

# Frame #2 of main window : Text box for Terminal
f2_terminal=tkinter.Frame(rb_manager, relief='solid', bd=1)
f2_terminal.pack(side="right", expand=False)


terminal_text = tkinter.Text(f2_terminal, width=40, height=23)
terminal_text.tag_add("강조", "end-1c linestart", "end")
terminal_text.tag_config("강조", background="purple")
terminal_text.pack()


def get_info(arg):
    pos = terminal_text.index("end")
    x = tkinter.StringVar()
    x = terminal_text.get("end-1c linestart", "end")
    terminal_text.insert("end", "\n")
    for line in os.popen(x, 'r'):
        terminal_text.insert("end", line)


terminal_text.bind("<Return>", get_info)


def check_linestring():
    cwd = os.getcwd()
    linestring_name = filedialog.askopenfilenames(parent=rb_manager,
                                                  initialdir=cwd,
                                                  initialfile='',
                                                  filetypes=[("CSV", "*.csv")])
    if len(linestring_name) < 1:
        return
    wgs84_line_list = read_wkt_csv_file(linestring_name[0])
    line_list = transform_wgs84_map_to_kcity_map(wgs84_line_list)
    # show_line_list(line_list)
    """ step 2 : connect adjacent lanes and make intervals """
    current_open_laneset = LaneSet(line_list=line_list)
    current_open_laneset = current_open_laneset.connect_split_lanes()
    terminal_text.insert("end", "check LineString \n")
    current_open_laneset.show(reverse=True)


def check_point_csv():
    cwd = os.getcwd()
    pdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(pdir_name) < 1:
        return
    current_open_laneset = read_point_csv(pdir_name)
    terminal_text.insert("end", "check point.csv\n")
    current_open_laneset.show(reverse=True)


def check_rosbag():
    cwd = os.getcwd()
    rdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(rdir_name) < 1:
        return
    current_open_laneset = read_rosbag(rdir_name)
    terminal_text.insert("end", "check rosbag\n")
    current_open_laneset.show(reverse=True)


def check_current_lanes():
    global working_laneset
    terminal_text.insert("end", "check current lanes\n")
    working_laneset.show(reverse=True)


def check_current_paths():
    global working_laneset
    path_set = PathSet(laneset=working_laneset)
    terminal_text.insert("end", "check current paths\n")
    path_set.show(reverse=True)


def open_linestring():
    """ step 1 : extract points ( WKT(Well-Known-Text) : LineString file whose coordinates are extracted from shp ) """
    global INTERVAL
    global working_laneset
    cwd = os.getcwd()
    linestring_name = filedialog.askopenfilenames(parent=rb_manager,
                                      initialdir=cwd,
                                      initialfile='',
                                      filetypes=[("CSV", "*.csv")])
    if len(linestring_name) < 1:
        return
    wgs84_line_list = read_wkt_csv_file(linestring_name[0])
    line_list = transform_wgs84_map_to_kcity_map(wgs84_line_list)
    # show_line_list(line_list)
    """ step 2 : connect adjacent lanes and make intervals """
    current_open_laneset = LaneSet(line_list=line_list)
    current_open_laneset = current_open_laneset.connect_split_lanes()
    terminal_text.insert("end", "open/take LineString \n")
    current_open_laneset.show(reverse=True)
    current_open_laneset = choose_area2("loaded LineString, extract lanes in the specified area.",
                                        in_laneset=current_open_laneset, mode='t')
    current_open_laneset = choose_area2("loaded LineString, Deletes points in the specified area.",
                                        in_laneset=current_open_laneset, mode='r', bo=True)
    current_open_laneset = current_open_laneset.split_connected_lanes()
    current_open_laneset = current_open_laneset.make_uniform_intervals(INTERVAL)
    working_laneset = working_laneset + current_open_laneset
    working_laneset.show(reverse=True)


def open_point_csv():
    global INTERVAL
    global working_laneset
    cwd = os.getcwd()
    pdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(pdir_name) < 1:
        return
    current_open_laneset = read_point_csv(pdir_name)
    # current_open_laneset.show(reverse=True)
    terminal_text.insert("end", "open/take point.csv \n")
    current_open_laneset = extract_lane_points2("loaded point.csv, extract points in the specified area",
                                                in_laneset=current_open_laneset)
    current_open_laneset = current_open_laneset.make_uniform_intervals(INTERVAL)
    working_laneset = working_laneset + current_open_laneset
    working_laneset.show(reverse=True)
    # pathset = PathSet(laneset=working_laneset)
    # pathset.show(reverse=True)
    # working_laneset = pathset.laneset()
    # working_laneset.show(reverse=True)


def open_rosbag():
    global INTERVAL
    global working_laneset
    cwd = os.getcwd()
    rdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(rdir_name) < 1:
        return
    current_open_laneset = read_rosbag(rdir_name)
    # current_open_laneset.show(reverse=True)
    terminal_text.insert("end", "open/take rosbag \n")
    current_open_laneset = extract_lane_points2("loaded rosbag, extract points in the specified area",
                                                in_laneset=current_open_laneset)
    current_open_laneset = current_open_laneset.make_uniform_intervals(INTERVAL)
    working_laneset = working_laneset + current_open_laneset
    working_laneset.show(reverse=True)


def exec_drag_lanes():
    global working_laneset
    terminal_text.insert("end", "drag lanes\n")
    working_laneset = drag_lanes(working_laneset)


def exec_drag_each_lane():
    global working_laneset
    terminal_text.insert("end", "drag each lane \n")
    working_laneset = drag_each_lane(working_laneset)


def exec_pick_each_point():
    global working_laneset
    terminal_text.insert("end", "pick each point \n")
    working_laneset = pick_each_point(working_laneset)


def exec_drop_lanes():
    global working_laneset
    terminal_text.insert("end", "drop lanes \n")
    working_laneset = drop_lanes2(working_laneset)


def exec_reverse_lanes():
    global working_laneset
    terminal_text.insert("end", "reverse lanes \n")
    working_laneset = reverse_lanes(working_laneset)


def exec_connect_lanes():
    global working_laneset
    terminal_text.insert("end", "connect lanes \n")
    working_laneset = connect_lanes(working_laneset)


def save_dtlane():
    global working_laneset
    pathset = PathSet(laneset=working_laneset)
    wayset = pathset.wayset()
    terminal_text.insert("end", "save dtlane \n")
    write_autoware_dtlane(wayset)


def smoothing():
    global working_laneset
    pathset = PathSet(laneset=working_laneset)
    pathset.smoothing()
    terminal_text.insert("end", "smooting \n")
    working_laneset = pathset.laneset()


def adjust_z_coordinate():
    global working_laneset
    cwd = os.getcwd()
    rdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(rdir_name) < 1:
        return
    current_pose_laneset = read_rosbag(rdir_name)
    terminal_text.insert("end", "adjust z coordinate with reference \n")
    working_laneset = working_laneset.adjust_z_coordinate_to(current_pose_laneset, textbox=terminal_text)
    terminal_text.insert("end", "adjust z coordinate with reference, complete \n")
    working_laneset.show(reverse=True)


def force_unblock():
    global control
    control = 'y'
    block.set(False)
    plt.close("all")
    wi = tkinter.Tk()
    wi.title("testing_dynamic")
    wi.geometry("640x400+400+100")
    wi.resizable(True, True)
    wi.quit()
    wi.destroy()


##############################################
block = tkinter.BooleanVar(f2_terminal, False)


def show():  # plt.show() graph closed 이후에 tk창 하나를 quit해야 blocking이 풀림.
    control_window = tkinter.Tk()
    control_window.title("작업을 마치신 후에 다음 버튼을 눌러주세요")
    control_window.geometry("400x50+800+600")
    control_window.resizable(False, False)

    def btnext():
        plt.close("all")
        control_window.quit()
        control_window.destroy()

    button = tkinter.Button(control_window, text="다음", command=btnext)
    button.pack()
    plt.show()


def choose_area2(title, in_laneset, mode, bo=False, reverse=True):
    control = 'y'
    out_laneset = in_laneset

    def unblocking_y():
        nonlocal control
        control = 'y'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    def unblocking_n():
        nonlocal control
        control = 'n'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    while control == 'y':
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title(title)
        out_laneset.sub_plot(ax, bo=True)
        draw_line, = ax.plot([], [])
        line_builder = LineBuilder(draw_line)
        show()

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
        control_box = tkinter.Tk()
        control_box.title("control box")
        control_box.geometry("100x100+800+400")
        control_box.resizable(False, False)
        ybt = tkinter.Button(control_box, text="take more", command=unblocking_y)
        nbt = tkinter.Button(control_box, text="stop", command=unblocking_n)
        ybt.pack()
        nbt.pack()
        block.set(True)
        rb_manager.wait_variable(block)
        if control != 'y':
            line_builder.disconnect()
            del line_builder
    return out_laneset


def extract_lane_points2(title, in_laneset, bo=False, reverse=True):
    control = 'y'
    add_lane_list = []
    add_laneset = None

    def unblocking_y():
        nonlocal control
        control = 'y'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    def unblocking_n():
        nonlocal control
        control = 'n'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    while control == 'y':
        print("DEBUG:loop / control : ", control)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title(title)
        in_laneset.sub_plot(ax, ep=True)
        draw_line, = ax.plot([], [])
        line_builder = LineBuilder(draw_line)
        show()

        verts = zip(line_builder.xs, line_builder.ys)
        if len(line_builder.xs) == 0:
            pass
        else:
            extracted_lane = in_laneset.points_in_area_of_interest(verts)
            extracted_lane.show()
            add_lane_list.append(extracted_lane)
            add_laneset = LaneSet(lane_list=add_lane_list)
            add_laneset.show()

        control_box = tkinter.Tk()
        control_box.title("control box")
        control_box.geometry("100x100+800+400")
        control_box.resizable(False, False)
        ybt = tkinter.Button(control_box, text="take more", command=unblocking_y)
        nbt = tkinter.Button(control_box, text="stop", command=unblocking_n)
        ybt.pack()
        nbt.pack()
        block.set(True)
        rb_manager.wait_variable(block)

        if control != 'y':
            line_builder.disconnect()
            del line_builder
    return add_laneset


def drag_lanes(laneset):
    control = 'y'
    reference = False
    cwd = os.getcwd()
    rdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(rdir_name) > 1:
        rosbag_reference = read_rosbag(rdir_name)
        reference = True

    def unblocking_y():
        nonlocal control
        control = 'y'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    def unblocking_n():
        nonlocal control
        control = 'n'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    while control == 'y':
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title("drag and fix in LaneSet")
        if reference:
            rosbag_reference.sub_plot(ax)
        line = laneset.sub_plot2(ax, bo=True)
        draggable_line = DraggaleLine(line)
        draggable_line.connect()
        show()

        x = laneset.lane_list[0].point_list[0].x
        y = laneset.lane_list[0].point_list[0].y
        new_x = list(line.get_xdata())[0]
        new_y = list(line.get_ydata())[0]
        dx = new_x - x
        dy = new_y - y
        out_laneset = laneset.translate(dx, dy)

        control_box = tkinter.Tk()
        control_box.title("control box")
        control_box.geometry("100x100+800+400")
        control_box.resizable(False, False)
        ybt = tkinter.Button(control_box, text="take more", command=unblocking_y)
        nbt = tkinter.Button(control_box, text="stop", command=unblocking_n)
        ybt.pack()
        nbt.pack()
        block.set(True)
        rb_manager.wait_variable(block)

        if reference:
            rosbag_reference.plot(ro=True)
        out_laneset.show(dash=True)

        if control != 'y':
            draggable_line.disconnect()
    return out_laneset


def drag_each_lane(laneset):
    control = 'y'
    reference = False
    cwd = os.getcwd()
    rdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(rdir_name) > 1:
        rosbag_reference = read_rosbag(rdir_name)
        reference = True

    def unblocking_y():
        nonlocal control
        control = 'y'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    def unblocking_n():
        nonlocal control
        control = 'n'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    while control == 'y':
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title("drag and fix in Lane")
        if reference:
            rosbag_reference.sub_plot(ax, ro=True)
        dls = laneset.sub_plot3(ax)
        show()

        out_lane_list = []
        for dl, lane in itertools.zip_longest(dls, laneset.lane_list):
            x = lane.point_list[0].x
            y = lane.point_list[0].y
            new_x = list(dl.line.get_xdata())[0]
            new_y = list(dl.line.get_ydata())[0]
            dx = new_x - x
            dy = new_y - y
            out_lane_list.append(lane.translate(dx, dy))
        out_laneset = LaneSet(lane_list=out_lane_list)

        control_box = tkinter.Tk()
        control_box.title("control box")
        control_box.geometry("200x100+800+400")
        control_box.resizable(False, False)
        ybt = tkinter.Button(control_box, text="take more", command=unblocking_y)
        nbt = tkinter.Button(control_box, text="stop", command=unblocking_n)
        ybt.pack()
        nbt.pack()
        block.set(True)
        rb_manager.wait_variable(block)

        if reference:
            rosbag_reference.plot(ro=True)
        out_laneset.show(dash=True)

        if control != 'y':
            for dl in dls:
                dl.disconnect()
                del dl
    return out_laneset


def pick_each_point(laneset):
    control = 'y'
    reference = False
    cwd = os.getcwd()
    rdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(rdir_name) > 1:
        rosbag_reference = read_rosbag(rdir_name)
        reference = True

    def unblocking_y():
        nonlocal control
        control = 'y'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    def unblocking_n():
        nonlocal control
        control = 'n'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    while control == 'y':
        fig = plt.figure()
        ax1 = fig.add_subplot(121)
        ax2 = fig.add_subplot(122)
        # plot_bag_reference2(ax1, ro=False, dash=True)
        if reference:
            lb = LineBrowser(laneset, fig, ax1, ax2, reference=rosbag_reference)
        else:
            lb = LineBrowser(laneset, fig, ax1, ax2)
        show()

        plot_line_list = lb.line_list
        out_lane_list = []
        for pl, lane in itertools.zip_longest(plot_line_list, laneset.lane_list):
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
        out_laneset = LaneSet(lane_list=out_lane_list)

        control_box = tkinter.Tk()
        control_box.title("control box")
        control_box.geometry("200x100+800+400")
        control_box.resizable(False, False)
        ybt = tkinter.Button(control_box, text="take more", command=unblocking_y)
        nbt = tkinter.Button(control_box, text="stop", command=unblocking_n)
        ybt.pack()
        nbt.pack()
        block.set(True)
        rb_manager.wait_variable(block)

        if reference:
            rosbag_reference.plot(ro=True)
        out_laneset.show(bo=True)

        if control != 'y':
            lb.disconnect()
            del lb
    return out_laneset


def drop_lanes2(laneset):
    control = 'y'
    out_laneset = laneset

    def unblocking_y():
        nonlocal control
        control = 'y'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    def unblocking_n():
        nonlocal control
        control = 'n'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    while control == 'y':
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title("drop lanes")
        out_laneset.sub_plot_idx(ax, reverse=True)
        idx_list = []

        control_window = tkinter.Tk()
        control_window.title("삭제할 레인의 인덱스를 적고 enter키를 누르세요 / 누적됩니다")
        control_window.geometry("400x100+800+600")
        control_window.resizable(False, False)
        idx_entry = tkinter.Entry(control_window)
        idx_list_label = tkinter.Label(control_window)

        def btnext():
            plt.close("all")
            control_window.quit()
            control_window.destroy()

        def append_idx(event):
            nonlocal idx_list
            idx_list.append(int(idx_entry.get()))
            idx_list_label.config(text=str(idx_list))
            idx_entry.delete(0, 'end')

        def remove_idx():
            nonlocal idx_list
            del idx_list[-1]
            idx_list_label.config(text=str(idx_list))
            idx_entry.delete(0, 'end')

        idx_entry.bind("<Return>", append_idx)
        idx_entry.pack()
        idx_list_label.pack()
        remove_button = tkinter.Button(control_window, text="되돌리기", command=remove_idx)
        next_button = tkinter.Button(control_window, text="완료", command=btnext)
        remove_button.pack()
        next_button.pack()
        plt.show()

        out_laneset = out_laneset.remove_lanes(idx_list)
        out_laneset.show_idx(reverse=True)

        control_box = tkinter.Tk()
        control_box.title("control box")
        control_box.geometry("100x100+800+400")
        control_box.resizable(False, False)
        ybt = tkinter.Button(control_box, text="take more", command=unblocking_y)
        nbt = tkinter.Button(control_box, text="stop", command=unblocking_n)
        ybt.pack()
        nbt.pack()
        block.set(True)
        rb_manager.wait_variable(block)

        out_laneset.show(reverse=True)

    return out_laneset


def reverse_lanes(laneset):
    pathset = PathSet(laneset=laneset)
    control = 'y'
    idx_list = []

    def unblocking_y():
        nonlocal control
        control = 'y'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    def unblocking_n():
        nonlocal control
        control = 'n'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    while control == 'y':
        fig = plt.figure()
        ax = fig.add_subplot(111)
        pathset.plot(reverse=True)
        ax.set_title("reverse lane")

        control_window = tkinter.Tk()
        control_window.title("방향을 뒤집고 싶은 레인의 인덱스를 적고 enter키를 누르세요 / 누적됩니다")
        control_window.geometry("400x100+800+600")
        control_window.resizable(False, False)
        idx_entry = tkinter.Entry(control_window)
        idx_list_label = tkinter.Label(control_window)

        def btnext():
            plt.close("all")
            control_window.quit()
            control_window.destroy()

        def append_idx(event):
            nonlocal idx_list
            idx_list.append(int(idx_entry.get()))
            idx_list_label.config(text=str(idx_list))
            idx_entry.delete(0, 'end')

        def remove_idx():
            nonlocal idx_list
            del idx_list[-1]
            idx_list_label.config(text=str(idx_list))
            idx_entry.delete(0, 'end')

        idx_entry.bind("<Return>", append_idx)
        idx_entry.pack()
        idx_list_label.pack()
        remove_button = tkinter.Button(control_window, text="되돌리기", command=remove_idx)
        next_button = tkinter.Button(control_window, text="완료", command=btnext)
        remove_button.pack()
        next_button.pack()
        plt.show()

        out_laneset = pathset.laneset()
        for idx in idx_list:
            out_laneset.lane_list[idx].point_list.reverse()
            # pathset.path_list[int(reverse_idx)] = Path(lane=Lane(pathset.path_list[int(reverse_idx)].point_list.reverse()))

        pathset = PathSet(laneset=out_laneset)
        pathset.show(reverse=True)

        control_box = tkinter.Tk()
        control_box.title("control box")
        control_box.geometry("100x100+800+400")
        control_box.resizable(False, False)
        ybt = tkinter.Button(control_box, text="take more", command=unblocking_y)
        nbt = tkinter.Button(control_box, text="stop", command=unblocking_n)
        ybt.pack()
        nbt.pack()
        block.set(True)
        rb_manager.wait_variable(block)

        pathset.show(reverse=True)

    out_laneset = pathset.laneset()
    return out_laneset


def connect_lanes(laneset):
    pathset = PathSet(laneset=laneset)
    control = 'y'  # enter two index to making connecting lane between the two lane registered.
    connection_list = []
    lane_style = ''

    def unblocking_y():
        nonlocal control
        control = 'y'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    def unblocking_n():
        nonlocal control
        control = 'n'
        block.set(False)
        control_box.quit()
        control_box.destroy()

    while control == 'y':
        fig = plt.figure()
        ax = fig.add_subplot(111)
        pathset.plot(reverse=True)
        ax.set_title("make connection_idx_list")
        control_window = tkinter.Tk()
        control_window.title("연결시키고 싶은 before lane 인덱스와 after lane 인덱스를 넣고 '추가'하세요")
        control_window.geometry("400x300+800+600")
        control_window.resizable(False, False)
        text_label = tkinter.Label(control_window)
        idx_entry1 = tkinter.Entry(control_window)
        idx_entry2 = tkinter.Entry(control_window)
        connection_list_label = tkinter.Label(control_window)
        dump = tkinter.IntVar()

        # radio_straight.select()

        def lane_style_curve():
            nonlocal lane_style
            lane_style = 'c'

        def lane_style_straight():
            nonlocal lane_style
            lane_style = 's'

        def btnext():
            plt.close("all")
            control_window.quit()
            control_window.destroy()

        def append_idx():
            nonlocal connection_list
            connection_list.append([int(idx_entry1.get()), int(idx_entry2.get())])
            connection_list_label.config(text=str(connection_list))
            idx_entry1.delete(0, 'end')
            idx_entry2.delete(0, 'end')

        def remove_idx():
            nonlocal connection_list
            del connection_list[-1]
            connection_list_label.config(text=str(connection_list))
            idx_entry1.delete(0, 'end')
            idx_entry2.delete(0, 'end')

        radio_curve = tkinter.Radiobutton(control_window, text="curve", value=1, variable=dump, command=lane_style_curve)
        radio_curve.pack()
        radio_straight = tkinter.Radiobutton(control_window, text="straight", value=2, variable=dump, command=lane_style_straight)
        radio_straight.pack()

        text_label.config(text="from idx, next idx 입력 후 추가를 누르세요")
        text_label.pack()
        idx_entry1.pack()
        idx_entry2.pack()
        connection_list_label.pack()
        append_button = tkinter.Button(control_window, text="추가", command=append_idx)
        remove_button = tkinter.Button(control_window, text="되돌리기", command=remove_idx)
        next_button = tkinter.Button(control_window, text="완료", command=btnext)
        append_button.pack()
        remove_button.pack()
        next_button.pack()
        plt.show()

        if lane_style is 'c':  #curve
            pathset2 = pathset.connecting_lanes_curved(connection_list)
            pathset = PathSet(laneset=pathset.laneset() + pathset2.laneset())
            print('DEBUG1')
        elif lane_style is 's':  #straight
            pathset2 = pathset.connecting_lanes_straight(connection_list)
            pathset = PathSet(laneset=pathset.laneset() + pathset2.laneset())
            print('DEBUG2')

        connection_list = []
        pathset.show(reverse=True)

        control_box = tkinter.Tk()
        control_box.title("control box")
        control_box.geometry("100x100+800+400")
        control_box.resizable(False, False)
        ybt = tkinter.Button(control_box, text="take more", command=unblocking_y)
        nbt = tkinter.Button(control_box, text="stop", command=unblocking_n)
        ybt.pack()
        nbt.pack()
        block.set(True)
        rb_manager.wait_variable(block)

        pathset.show(reverse=True)

    out_laneset = pathset.laneset().connect_split_lanes()
    out_laneset.show(reverse=True)
    return out_laneset


bt_check_linestring = tkinter.Button(f1_func_window, text="peek lineString", command=check_linestring)
bt_check_point_csv = tkinter.Button(f1_func_window, text="peek point.csv", command=check_point_csv)
bt_check_rosbag = tkinter.Button(f1_func_window, text="peek rosbag", command=check_rosbag)
bt_check_current_lanes = tkinter.Button(f1_func_window, text="check/show current lanes", command=check_current_lanes)
bt_check_current_paths = tkinter.Button(f1_func_window, text="check/show current paths", command=check_current_paths)
bt_open_linestring = tkinter.Button(f1_func_window, text="open LineString", command=open_linestring)
bt_open_point_csv = tkinter.Button(f1_func_window, text="open point.csv", command=open_point_csv)
# openBT_rosbag = tkinter.Button(f1_func_window, text="open rosbag", command=unblocking_t)
bt_open_rosbag = tkinter.Button(f1_func_window, text="open rosbag", command=open_rosbag)
bt_drag_lanes = tkinter.Button(f1_func_window, text="drag lanes", command=exec_drag_lanes)
bt_drag_each_lanes = tkinter.Button(f1_func_window, text="drag each lane", command=exec_drag_each_lane)
bt_pick_each_point = tkinter.Button(f1_func_window, text="pick each point", command=exec_pick_each_point)
bt_drop_lanes = tkinter.Button(f1_func_window, text="drop lanes", command=exec_drop_lanes)
bt_reverse_lanes = tkinter.Button(f1_func_window, text="reverse lanes", command=exec_reverse_lanes)
bt_connect_lanes = tkinter.Button(f1_func_window, text="connect lanes", command=exec_connect_lanes)
bt_save_dtlane = tkinter.Button(f1_func_window, text="create new window", command=save_dtlane)
bt_emergency_next = tkinter.Button(f1_func_window, text="emergency unblocking", command=force_unblock)

bt_check_linestring.pack()
bt_check_point_csv.pack()
bt_check_rosbag.pack()
bt_check_current_lanes.pack()
bt_check_current_paths.pack()
bt_open_linestring.pack()
bt_open_point_csv.pack()
bt_open_rosbag.pack()
bt_drag_lanes.pack()
bt_drag_each_lanes.pack()
bt_pick_each_point.pack()
bt_drop_lanes.pack()
bt_reverse_lanes.pack()
bt_connect_lanes.pack()
bt_save_dtlane.pack()
bt_emergency_next.pack()

# mainloop window
rb_manager.mainloop()
