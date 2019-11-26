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

from rubimapper import *
import tkinter
from tkinter import filedialog
import tkinter.font
import numpy as np
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.backend_bases import button_press_handler
from matplotlib.figure import Figure
import os
import copy

INTERVAL = 0.99

working_list = []  # for undoing/redoing
working_laneset = LaneSet(line_list=None, lane_list=None) # initial laneset
working_list.append(copy.deepcopy(working_laneset))
working_idx = 0

# Manager main window
rb_manager = tkinter.Tk()
rb_manager.title("Rubimapper++")
rb_manager.geometry("1200x700+300+300")
rb_manager.resizable(True, True)

# Frame #1 of main window : Buttons for Funcs
f1_menu_window=tkinter.Frame(rb_manager, relief='solid', bd=1)
f1_menu_window.pack(side="top", fill="x")

# Frame #2 of main window : Text box for Terminal/working process monitor
f2_terminal=tkinter.Frame(rb_manager, relief='solid', bd=1)
f2_terminal.pack(side='bottom', fill="x")
terminal_text = tkinter.Text(f2_terminal, height="2")
terminal_text.tag_add("강조", "end-1c linestart", "end")
terminal_text.tag_config("강조", background="purple")
terminal_text.pack(side='bottom', fill="x")


def get_info(arg):
    pos = terminal_text.index("end")
    x = tkinter.StringVar()
    x = terminal_text.get("end-1c linestart", "end")
    terminal_text.insert("end", "\n")
    for line in os.popen(x, 'r'):
        terminal_text.insert("end", line)


terminal_text.bind("<Return>", get_info)

# Frame #3 of main window : control buttons
f3_control=tkinter.Frame(rb_manager, relief='solid', bd=1)
f3_control.pack(side="right", anchor="n", fill="y")

# Frame #4 of main window : Canvas to draw present laneset/pathset
f4_canvas=tkinter.Frame(rb_manager, relief='solid', bd=1)
f4_canvas.pack(expand=True, fill="both")

present_fig = Figure()
present_ax = present_fig.add_subplot(111)
working_laneset.sub_plot(present_ax) # empty start
present_ax.set_title("Current work")
present_ax.set_aspect('equal', 'datalim') # equal ratio on graph

main_canvas = FigureCanvasTkAgg(present_fig, master=f4_canvas)  # A tk.DrawingArea.
main_canvas.draw()
main_canvas.get_tk_widget().pack(expand=True, fill="both")

main_toolbar = NavigationToolbar2Tk(main_canvas, f4_canvas)
main_toolbar.update()
main_canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)


def on_key_press(event): # for matplotlib's short cut
    key_press_handler(event, main_canvas, main_toolbar)


key_event_id = main_canvas.mpl_connect("key_press_event", on_key_press)


def open_linestring():
    global INTERVAL
    global working_laneset
    global working_idx
    cwd = os.getcwd()
    linestring_name = filedialog.askopenfilenames(parent=rb_manager,
                                      initialdir=cwd,
                                      initialfile='',
                                      filetypes=[("CSV", "*.csv")])
    if len(linestring_name) < 1:
        return
    wgs84_line_list = read_wkt_csv_file(linestring_name[0])
    line_list = transform_wgs84_map_to_kcity_map(wgs84_line_list)
    current_open_laneset = LaneSet(line_list=line_list)
    current_open_laneset = current_open_laneset.connect_split_lanes()
    terminal_text.insert("end", "open/take LineString \n")
    terminal_text.see("end")

    # new window
    current_open_laneset = current_open_laneset.make_uniform_intervals(INTERVAL)
    extract_lanes_window(current_open_laneset)


def open_point_csv():
    global INTERVAL
    global working_laneset
    global working_idx
    cwd = os.getcwd()
    pdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(pdir_name) < 1:
        return
    current_open_laneset = read_point_csv(pdir_name)
    # current_open_laneset.show(reverse=True)
    terminal_text.insert("end", "open/take point.csv \n")
    terminal_text.see("end")

    # new window
    current_open_laneset = current_open_laneset.make_uniform_intervals(INTERVAL)
    extract_points_window(current_open_laneset, "point.csv")


def open_rosbag():
    global INTERVAL
    global working_laneset
    global working_idx
    cwd = os.getcwd()
    rdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(rdir_name) < 1:
        return
    current_open_laneset = read_rosbag(rdir_name)
    # current_open_laneset.show(reverse=True)
    terminal_text.insert("end", "open/take rosbag \n")
    terminal_text.see("end")

    # new window
    current_open_laneset = current_open_laneset.make_uniform_intervals(INTERVAL)
    extract_points_window(current_open_laneset, "rosbag")


def exec_main_complete():  # used
    global case, lb, dl, dls, working_laneset, working_list, working_idx
    out_laneset = working_laneset.symmetric_laneset()
    if case is 1:
        verts = zip(lb.xs, lb.ys)
        if len(lb.xs) == 0:
            pass
        else:
            out_laneset.remove_points_in_area(verts)
            out_laneset = out_laneset.split_connected_lanes()
            working_laneset = out_laneset.symmetric_laneset()
        del working_list[working_idx + 1:]
        working_list.append(copy.deepcopy(working_laneset))
        working_idx = len(working_list) - 1
        refresh_canvas()
        bt_main_complete.config(state='disabled', text="")
        lb.disconnect()
        del lb
    if case is 2:
        temp_laneset = working_laneset.symmetric_laneset()
        x = temp_laneset.lane_list[0].point_list[0].x
        y = temp_laneset.lane_list[0].point_list[0].y
        new_x = list(dl.line.get_xdata())[0]
        new_y = list(dl.line.get_ydata())[0]
        dx = new_x - x
        dy = new_y - y
        out_laneset = temp_laneset.translate(dx, dy)
        working_laneset = out_laneset.symmetric_laneset()
        del working_list[working_idx + 1:]
        working_list.append(copy.deepcopy(working_laneset))
        working_idx = len(working_list) - 1
        refresh_canvas()
        bt_main_complete.config(state='disabled', text="")
        dl.disconnect()
        del dl
    if case is 3:
        temp_laneset = working_laneset.symmetric_laneset()
        out_lane_list = []
        for dl, lane in itertools.zip_longest(dls, temp_laneset.lane_list):
            x = lane.point_list[0].x
            y = lane.point_list[0].y
            new_x = list(dl.line.get_xdata())[0]
            new_y = list(dl.line.get_ydata())[0]
            dx = new_x - x
            dy = new_y - y
            out_lane_list.append(lane.translate(dx, dy))
        out_laneset = LaneSet(lane_list=out_lane_list)
        working_laneset = out_laneset.symmetric_laneset()
        del working_list[working_idx + 1:]
        working_list.append(copy.deepcopy(working_laneset))
        working_idx = len(working_list) - 1
        refresh_canvas()
        bt_main_complete.config(state='disabled', text="")
        for dl in dls:
            dl.disconnect()
            del dl
    if case is 4:
        working_laneset = working_laneset.connect_split_lanes()
        del working_list[working_idx + 1:]
        working_list.append(copy.deepcopy(working_laneset))
        working_idx = len(working_list) - 1
        refresh_canvas_with_paths()
        bt_main_complete.config(state='disabled', text="")


def exec_drag_lanes():
    global case, dl, working_laneset, working_list, working_idx
    terminal_text.insert("end", "drag lanes\n")
    terminal_text.see("end")

    cwd = os.getcwd()
    rdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    reference = False
    rosbag_reference = None
    if len(rdir_name) > 1:
        rosbag_reference = read_rosbag(rdir_name)
        reference = True

    present_ax.cla()
    temp_laneset = working_laneset.symmetric_laneset()
    if reference:
        rosbag_reference.symmetric_laneset().sub_plot(present_ax, dash=True)
    line = temp_laneset.sub_plot2(present_ax, bo=True)
    dl = DraggaleLine(line)
    dl.connect()
    main_canvas.draw()

    case = 2
    bt_main_complete.config(state='normal', text='drag completed', fg='red')


def exec_drag_each_lane():
    global case, dls, working_laneset, working_list, working_idx
    terminal_text.insert("end", "drag lanes\n")
    terminal_text.see("end")

    cwd = os.getcwd()
    rdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    reference = False
    rosbag_reference = None
    if len(rdir_name) > 1:
        rosbag_reference = read_rosbag(rdir_name)
        reference = True

    present_ax.cla()
    temp_laneset = working_laneset.symmetric_laneset()
    if reference:
        rosbag_reference.symmetric_laneset().sub_plot(present_ax, dash=True)
    dls = temp_laneset.sub_plot_each_draggable_line(present_ax)
    # line = temp_laneset.sub_plot2(present_ax, bo=True)
    # dl = DraggaleLine(line)
    # dl.connect()
    main_canvas.draw()

    case = 3
    bt_main_complete.config(state='normal', text='drag completed', fg='red')


def exec_pick_each_point():
    global working_laneset
    global working_list
    global working_idx
    terminal_text.insert("end", "pick each point \n")
    terminal_text.see("end")

    working_laneset = pick_each_point(working_laneset)
    del working_list[working_idx + 1:]
    working_list.append(copy.deepcopy(working_laneset))
    working_idx = len(working_list) - 1
    refresh_canvas()


def exec_drop_lanes():  # use
    global case, lb, cid, working_laneset, working_list, working_idx
    terminal_text.insert("end", "drop lanes \n")
    terminal_text.see("end")

    refresh_canvas_with_paths()
    control_window = tkinter.Tk()
    control_window.title("삭제할 레인의 인덱스를 적고 enter키를 누르세요 / 누적됩니다")
    control_window.geometry("400x100+800+600")
    control_window.resizable(False, False)
    idx_entry = tkinter.Entry(control_window)
    idx_list_label = tkinter.Label(control_window)
    idx_list = []

    def btnext():
        nonlocal idx_list
        global working_laneset, working_idx
        working_laneset = working_laneset.remove_lanes(idx_list)
        del working_list[working_idx + 1:]
        working_list.append(copy.deepcopy(working_laneset))
        working_idx = len(working_list) - 1
        refresh_canvas()
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

    control_window.mainloop()


def exec_drop_points():   # used
    global case, lb, working_laneset, working_list, working_idx
    terminal_text.insert("end", "drop lanes \n")
    terminal_text.see("end")

    lb = choose_area("delete points (if it make some distance, we seperate it)",
                          working_laneset.symmetric_laneset(), main_canvas, present_ax, main_toolbar)
    main_canvas.draw()
    case = 1
    bt_main_complete.config(state='normal', text="drop points", fg='red')


def exec_reverse_lanes():  # used
    global working_laneset, working_list, working_idx
    terminal_text.insert("end", "reverse lanes \n")
    terminal_text.see("end")

    refresh_canvas_with_paths()
    control_window = tkinter.Tk()
    control_window.title("방향을 뒤집고 싶은 레인의 인덱스를 적고 enter키를 누르세요 / 누적됩니다")
    control_window.geometry("400x100+800+600")
    control_window.resizable(False, False)
    idx_entry = tkinter.Entry(control_window)
    idx_list_label = tkinter.Label(control_window)
    idx_list = []

    def btnext():
        global working_laneset, working_list ,working_idx
        nonlocal idx_list
        for idx in idx_list:
            working_laneset.lane_list[idx].point_list.reverse()
        del working_list[working_idx + 1:]
        working_list.append(copy.deepcopy(working_laneset))
        working_idx = len(working_list) - 1
        refresh_canvas_with_paths()
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

    control_window.mainloop()


def exec_connect_lanes():  # used
    global working_laneset, working_list ,working_idx
    terminal_text.insert("end", "connect lanes \n")
    terminal_text.see("end")

    refresh_canvas_with_paths()
    control_window = tkinter.Tk()
    control_window.title("연결시키고 싶은 before lane 인덱스와 after lane 인덱스를 넣고 '추가'하세요")
    control_window.geometry("400x300+800+600")
    control_window.resizable(False, False)
    text_label = tkinter.Label(control_window)
    idx_entry1 = tkinter.Entry(control_window)
    idx_entry2 = tkinter.Entry(control_window)
    connection_list_label = tkinter.Label(control_window)
    dump = tkinter.IntVar()
    connection_list = []
    lane_style = ''

    def lane_style_curve():
        nonlocal lane_style
        lane_style = 'c'

    def lane_style_straight():
        nonlocal lane_style
        lane_style = 's'

    def btnext():
        global working_laneset, working_idx, case
        nonlocal lane_style, connection_list
        pathset = PathSet(laneset=working_laneset)
        if lane_style is 'c':  # curve
            pathset2 = pathset.connecting_lanes_curved(connection_list)
            pathset = PathSet(laneset=pathset.laneset() + pathset2.laneset())
        elif lane_style is 's':  # straight
            pathset2 = pathset.connecting_lanes_straight(connection_list)
            pathset = PathSet(laneset=pathset.laneset() + pathset2.laneset())

        working_laneset = pathset.laneset()
        case=4
        bt_main_complete.config(state='normal', text='tie-up', fg='red')
        del working_list[working_idx + 1:]
        working_list.append(copy.deepcopy(working_laneset))
        working_idx = len(working_list) - 1
        refresh_canvas_with_paths()
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
    radio_straight = tkinter.Radiobutton(control_window, text="straight", value=2, variable=dump,
                                         command=lane_style_straight)
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

    control_window.mainloop()


def save_dtlane():
    global working_laneset
    pathset = PathSet(laneset=working_laneset)
    wayset = pathset.wayset()
    terminal_text.insert("end", "save dtlane \n")
    write_autoware_dtlane(wayset)
    terminal_text.insert("end", "saved dtlane \n")
    terminal_text.see("end")


def refresh_canvas():
    global present_fig
    global present_ax
    global main_canvas
    global working_laneset
    present_ax.cla()
    present_ax.set_title("Current Work")
    # tl = working_laneset.symmetric_laneset()
    # tl.sub_plot(present_ax)
    working_laneset.sub_plot(present_ax, reverse=True)
    main_canvas.draw()
    terminal_text.insert("end", "check laneset \n")
    terminal_text.see("end")


def refresh_canvas_with_paths():
    global working_laneset
    global main_canvas
    global present_fig
    global present_ax
    path_set = PathSet(laneset=working_laneset)
    terminal_text.insert("end", "check current paths\n")
    terminal_text.see("end")
    present_ax.cla()
    present_ax.set_title("Current Work")
    path_set.sub_plot(present_ax, reverse=True)
    main_canvas.draw()


def extract_lanes_window(current_open_laneset):  # use in 'open'
    open_ls_window = tkinter.Tk()
    open_ls_window.wm_title("SHP를 변환한 LineString(csv)으로 작업합니다.")

    open_ls_fig = Figure()
    open_ls_ax = open_ls_fig.add_subplot(111)
    open_ls_ax.set_title("LineString")
    open_ls_ax.set_aspect('equal', 'datalim')

    canvas = FigureCanvasTkAgg(open_ls_fig, master=open_ls_window)
    current_open_laneset = current_open_laneset.symmetric_laneset()  # 대칭!
    current_open_laneset.sub_plot(ax=open_ls_ax)
    canvas.draw()
    canvas.get_tk_widget().pack(expand=True, fill='both')

    toolbar = NavigationToolbar2Tk(canvas, open_ls_window)
    toolbar.update()

    frame = tkinter.Frame(open_ls_window, relief='solid', bd=1)
    frame.pack(side="bottom")

    line_builder = None

    def take():
        nonlocal current_open_laneset, open_ls_ax, canvas, toolbar, line_builder
        line_builder= choose_area("loaded LineString, extract lanes in the specified area.",
                                       current_open_laneset, canvas, open_ls_ax, toolbar)
        take_button.config(state="disabled")
        complete_button.config(state="normal")
        canvas.draw()

    def complete():
        global working_laneset, working_list, working_idx
        nonlocal current_open_laneset, open_ls_ax, canvas, toolbar, line_builder
        verts = zip(line_builder.xs, line_builder.ys)
        if len(line_builder.xs) == 0:
            pass
        else:
            out_laneset = current_open_laneset.lanes_in_area_of_interest(verts)
            working_laneset = working_laneset + out_laneset.symmetric_laneset()
            working_list.append(copy.deepcopy(working_laneset))
            working_idx += 1
            refresh_canvas()
        line_builder.disconnect()
        del line_builder
        open_ls_window.quit()
        open_ls_window.destroy()

    take_button = tkinter.Button(frame, text="take", command=take, width='10')
    complete_button = tkinter.Button(frame, text="complete", command=complete, width='10')

    take_button.grid(row=0, column=0)
    complete_button.grid(row=0, column=1)
    complete_button.config(state="disabled")

    open_ls_window.mainloop()


def extract_points_window(current_open_laneset, title):   # use in 'open'
    open_ls_window = tkinter.Tk()
    open_ls_window.wm_title("polygon을 그려서 point.csv 혹은 rosbag으로 부터 점을 추출합니다.")

    open_ls_fig = Figure()
    open_ls_ax = open_ls_fig.add_subplot(111)
    open_ls_ax.set_title(title)
    open_ls_ax.set_aspect('equal', 'datalim')

    canvas = FigureCanvasTkAgg(open_ls_fig, master=open_ls_window)
    current_open_laneset = current_open_laneset.symmetric_laneset()  # 대칭!
    current_open_laneset.sub_plot(ax=open_ls_ax)
    canvas.draw()
    canvas.get_tk_widget().pack(expand=True, fill='both')

    toolbar = NavigationToolbar2Tk(canvas, open_ls_window)
    toolbar.update()

    frame = tkinter.Frame(open_ls_window, relief='solid', bd=1)
    frame.pack(side="bottom")

    line_builder = None

    def take():
        nonlocal current_open_laneset, open_ls_ax, canvas, toolbar, line_builder
        line_builder = choose_area("loaded LineString, extract lanes in the specified area.",
                                       current_open_laneset, canvas, open_ls_ax, toolbar)
        take_button.config(state="disabled")
        complete_button.config(state="normal")
        canvas.draw()

    def complete():
        global working_laneset
        global working_list
        global working_idx
        nonlocal current_open_laneset, open_ls_ax, canvas, toolbar, line_builder

        verts = zip(line_builder.xs, line_builder.ys)
        if len(line_builder.xs) == 0:
            pass
        else:
            extracted_lane = current_open_laneset.points_in_area_of_interest(verts)
            working_laneset.lane_list.append(extracted_lane.symmetric_lane())
            working_list.append(copy.deepcopy(working_laneset))
            working_idx += 1
            refresh_canvas()
        line_builder.disconnect()
        del line_builder
        open_ls_window.quit()
        open_ls_window.destroy()

    take_button = tkinter.Button(frame, text="take", command=take, width='10')
    complete_button = tkinter.Button(frame, text="complete", command=complete, width='10')

    take_button.grid(row=0, column=0)
    complete_button.grid(row=0, column=1)
    complete_button.config(state="disabled")

    open_ls_window.mainloop()


def smoothing():  # needs a button
    global working_laneset
    global working_list
    global working_idx
    pathset = PathSet(laneset=working_laneset)
    pathset.smoothing()
    terminal_text.insert("end", "smooting \n")
    terminal_text.see("end")
    working_laneset = pathset.laneset()
    del working_list[working_idx + 1:]
    working_list.append(copy.deepcopy(working_laneset))
    working_idx = len(working_list) - 1
    refresh_canvas()


def adjust_z_coordinate():  # needs a button
    global working_laneset
    global working_list
    global working_idx
    cwd = os.getcwd()
    rdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(rdir_name) < 1:
        return
    current_pose_laneset = read_rosbag(rdir_name)
    terminal_text.insert("end", "adjust z coordinate with reference \n")
    working_laneset = working_laneset.adjust_z_coordinate_to(current_pose_laneset, textbox=terminal_text)
    terminal_text.insert("end", "adjust z coordinate with reference, complete \n")
    terminal_text.see("end")
    working_laneset.show(reverse=True)
    del working_list[working_idx + 1:]
    working_list.append(copy.deepcopy(working_laneset))
    working_idx = len(working_list) - 1
    refresh_canvas()


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


def exec_undo():  # used
    global working_laneset
    global working_list
    global working_idx
    if working_idx > 0:
        working_idx -= 1
        working_laneset = copy.deepcopy(working_list[working_idx])
    refresh_canvas()


def exec_redo():  # used
    global working_laneset
    global working_list
    global working_idx
    if working_idx < len(working_list)-1:
        working_idx += 1
        working_laneset = copy.deepcopy(working_list[working_idx])
    refresh_canvas()

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


def choose_area(title, in_laneset, canvas, ax, toolbar, bo=False, reverse=True):
    ax.cla()
    out_laneset = in_laneset

    ax.set_title(title)
    out_laneset.sub_plot(ax, bo=True)
    draw_line, = ax.plot([], [])
    line_builder = LineBuilder(draw_line)

    return line_builder


def pick_each_point(laneset):
    control = 'y'
    temp_laneset = laneset.symmetric_laneset()
    reference = False
    cwd = os.getcwd()
    rdir_name = filedialog.askdirectory(parent=rb_manager, initialdir=cwd)
    if len(rdir_name) > 1:
        rosbag_reference = read_rosbag(rdir_name)
        rosbag_reference = rosbag_reference.symmetric_laneset()
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
        # plot_bag_reference2(present_ax, ro=False, dash=True)
        if reference:
            lb = LineBrowser(temp_laneset, fig, ax1, ax2, reference=rosbag_reference)
        else:
            lb = LineBrowser(temp_laneset, fig, ax1, ax2)
        show()

        plot_line_list = lb.line_list
        out_lane_list = []
        for pl, lane in itertools.zip_longest(plot_line_list, temp_laneset.lane_list):
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
    return out_laneset.symmetric_laneset()


# f1_menu_buttons
# bt_drop_lanes = tkinter.Button(f1_menu_window, text="drop", command=exec_drop_lanes, width='10')
bt_reverse_lanes = tkinter.Button(f1_menu_window, text="reverse", command=exec_reverse_lanes, width='10')
bt_connect_lanes = tkinter.Button(f1_menu_window, text="connect", command=exec_connect_lanes, width='10')
bt_emergency_next = tkinter.Button(f1_menu_window, text="emergency unblocking", command=force_unblock)

add_menu_button = tkinter.Menubutton(f1_menu_window, text="add", relief="raised", direction="right", width='10')
add_menu=tkinter.Menu(add_menu_button, tearoff=0)
add_menu.add_command(label='linestring', command=open_linestring)
add_menu.add_command(label='point.csv(folder)', command=open_point_csv)
add_menu.add_command(label='rosbag(folder)', command=open_rosbag)
add_menu_button['menu'] = add_menu

drag_menu_button = tkinter.Menubutton(f1_menu_window, text="drag", relief="raised", direction="right", width='10')
drag_menu=tkinter.Menu(drag_menu_button, tearoff=0)
drag_menu.add_command(label='drag lanes', command=exec_drag_lanes)
drag_menu.add_command(label='drag each lane', command=exec_drag_each_lane)
drag_menu.add_command(label='picking each points', command=exec_pick_each_point)
drag_menu_button['menu'] = drag_menu

drop_menu_button = tkinter.Menubutton(f1_menu_window, text="drop", relief="raised", direction="right", width='10')
drop_menu=tkinter.Menu(drop_menu_button, tearoff=0)
drop_menu.add_command(label='drop lanes', command=exec_drop_lanes)
drop_menu.add_command(label='drop points', command=exec_drop_points)
drop_menu_button['menu'] = drop_menu

add_menu_button.grid(row=0, column=0)
drag_menu_button.grid(row=0, column=1)
drop_menu_button.grid(row=0, column=2)
bt_reverse_lanes.grid(row=0, column=3)
bt_connect_lanes.grid(row=0, column=4)

# f3_control_buttons
bt_refresh = tkinter.Button(f3_control, text="Refresh", command=refresh_canvas, width='10')
bt_undo = tkinter.Button(f3_control, text="←Undo", command=exec_undo, width='10')
bt_redo = tkinter.Button(f3_control, text="Redo→", command=exec_redo, width='10')
bt_check_current_paths = tkinter.Button(f3_control, text="show dir/idx", command=refresh_canvas_with_paths)
bt_main_complete = tkinter.Button(f3_control, text="complete", command=exec_main_complete)
bt_save_dtlane = tkinter.Button(f3_control, text="save", command=save_dtlane, width='10', bg='yellow')

bt_refresh.pack()
bt_undo.pack()
bt_redo.pack()
bt_check_current_paths.pack()
bt_main_complete.pack()
bt_main_complete.config(state='disabled', text="만능키")
bt_save_dtlane.pack(side="bottom")

# mainloop window
rb_manager.mainloop()
