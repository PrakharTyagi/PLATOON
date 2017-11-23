#!/usr/bin/env python

# Class for GUI that plots the truck trajectories. Subscribes to a topic.

# TODO
# Handle plotting of multiple trucks.
# Add label indicating that recording is underway. Display elapsed time.
# Display servertime. Currently displaying self._ts_placeholder = 0.
# Support fixed displayed tail length.

from modeltruck_platooning.msg import *
import rospy
import time
import Tkinter as tk
import path
import math
import os


class TruckPlot():
    """Class for GUI that plots the truck trajectories. """
    def __init__(self, root, filename = 'record', width = 5, height = 5,
                display_tail = False, win_size = 600,
                node_name = 'truckplot_sub', topic_name = 'truck2',
                topic_type = truckmocap):
        self.root = root
        self._width = float(width)       # Real width (meters) of the window.
        self._height = float(height)
        self._win_height = win_size      # Graphical window height.
        self._win_width = int(self._win_height*self._width/self._height)
        self._save_filename = filename
        self._display_tail = display_tail
        self._node_name = node_name      # Subscriber node name.
        self._topic_name = topic_name    # Subscriber topic name.
        self._topic_type = topic_type    # Subscriber topic type.

        self.pt = path.Path()       # A fixed path to draw.
        self._saved_trajectory = []    # Recorded path of a truck for plotting.
        self._recorded_path = []        # Recorded path for saving to file.
        self._recording = False
        self._ts_placeholder = 0     # Elapsed time placeholder.
        self._timestamp = 0
        self._rec_start_time = 0

        # Stuff for canvas.
        bg_color = 'SlateGray2'
        w1 = 10
        ypad = 10
        self._truckl = 0.2
        self._truckw = 0.1

        # Setup subscriber node.
        rospy.init_node(self._node_name, anonymous = True)
        rospy.Subscriber(self._topic_name, self._topic_type, self._callback)

        # Base frame.
        s_frame = tk.Frame(self.root, background = bg_color)
        s_frame.pack()

        # Create canvas frame with a canvas for drawing in.
        canv_frame = tk.Frame(self.root)
        canv_frame.pack(in_ = s_frame, side= tk.LEFT)
        self._canv = tk.Canvas(self.root, width = self._win_width,
                            height = self._win_height, background='#FFFFFF',
                            borderwidth = 0, relief = tk.RAISED)
        self._canv.pack(in_ = canv_frame)
        self._canv.bind('<Button-1>', self._left_click)

        self._tr2 = self._canv.create_polygon(0, 0, 0, 0, 0, 0, fill = 'green')

        # Create frame next to the canvas for buttons, labels etc.
        right_frame = tk.Frame(self.root, background = bg_color)
        right_frame.pack(in_ = s_frame, side = tk.RIGHT, anchor = tk.N)

        # Create button frame.
        button_frame = tk.Frame(self.root, background = bg_color)
        button_frame.pack(in_ = right_frame, side = tk.TOP, anchor = tk.N,
                            pady = (0, ypad))

        traj_frame = tk.Frame(self.root, background = bg_color)
        traj_frame.pack(in_ = right_frame, side = tk.TOP, anchor = tk.N,
                            pady = (0, ypad))

        # Create frame for recording widgets.
        record_frame = tk.Frame(self.root, background = bg_color)
        record_frame.pack(in_ = right_frame, side = tk.TOP,
                            anchor = tk.N, pady = (0, ypad))

        # Create bottom frame for other stuff.
        bottom_frame = tk.Frame(self.root, background = bg_color)
        bottom_frame.pack(in_ = right_frame, side = tk.TOP, anchor = tk.N,
                            pady = (3*ypad, 0))

        # Button for quitting the program.
        quit_button = tk.Button(self.root, text = 'Quit',
                            command = self._quit1,
                            width = w1, height = 2, background = 'red2',
                            activebackground = 'red3')
        quit_button.pack(in_ = button_frame)

        # Checkbox for displaying trajectories.
        self.traj_button_var = tk.IntVar()
        self.traj_button = tk.Checkbutton(self.root,
            text = 'Display\ntrajectories', variable = self.traj_button_var,
            command = self._traj_btn_callback, width = w1, height = 2,
            background = bg_color)
        if self._display_tail:
            self.traj_button.toggle()
        self.traj_button.pack(in_ = traj_frame)

        # Button for clearing trajectories.
        self.clear_button = tk.Button(self.root, text = 'Clear trajectories',
                            command = self._clear_trajectories,
                            width = w1, height = 2, background = 'orange',
                            activebackground = 'dark orange')
        self.clear_button.pack(in_ = traj_frame)

        # Buttons for recording trajectories.
        self.start_record_button = tk.Button(
            self.root, text = 'Start new\nrecording',
            command = self._start_record, width = w1, height = 2)
        self.start_record_button.pack(in_ = record_frame)
        self.stop_record_button = tk.Button(self.root, text = 'Stop\nrecording',
            command = self._stop_record, width = w1, height = 2,
            state = tk.DISABLED)
        self.stop_record_button.pack(in_ = record_frame)

        # Label displaying elapsed recording time.
        self.rec_time_text_var = tk.StringVar()
        self.rec_time_label = tk.Label(self.root,
            textvariable = self.rec_time_text_var, anchor = tk.W,
            justify = tk.LEFT, width = 13, height = 2, background = bg_color,
            foreground = 'grey')
        self.rec_time_text_var.set('Not recording\n')
        self.rec_time_label.pack(in_ = record_frame)

        # Label displaying the elapsed server time.
        self.time_text_var = tk.StringVar()
        self.time_label = tk.Label(self.root, textvariable = self.time_text_var,
                                    anchor = tk.W, justify = tk.LEFT,
                                    width = 13, height = 2,
                                    background = bg_color)
        self.time_text_var.set('')
        self.time_label.pack(in_ = bottom_frame)

        # Actions for closing the window and pressing ctrl-C on the window.
        self.root.protocol('WM_DELETE_WINDOW', self._quit1)
        self.root.bind('<Control-c>', self._quit2)

        # Draw the coordinate arrows and coordinate labels in corners.
        self._draw_cf()


    def _left_click(self, event):
        """Action when left clicking on the canvas. """
        xreal, yreal = self._pixel_to_real(event.x, event.y)
        print('Clicked at ({:07.4f}, {:07.4f})'.format(xreal, yreal))


    def _quit1(self):
        """Quits the GUI. """
        print('Quitting.')
        self.root.quit()


    def _quit2(self, event):
        """Quits the GUI. """
        print('Quitting.')
        self.root.quit()


    def _callback(self, data):
        """Called when subscriber receives data. Calls all methods to run at
        each update step. """
        try:
            self._saved_trajectory.append([data.x, data.y])
            self._draw_canvas(data)
            self._timestamp = data.timestamp
            self.time_text_var.set(
                'Server time: \n{:.1f}'.format(self._timestamp))
            self._record_data(data)
        except Exception as e:
            print('Error in callback: {}'.format(e))


    def _draw_cf(self):
        """Draw lines for the origin and create text displaying the coordinates
        in the corners. """
        cftag = 'cf'
        # Create origin coordinate arrows.
        self._canv.create_line(int(self._win_width/2), int(self._win_height/2),
                            int(self._win_width/2), int(self._win_height/2) - 50,
                            width = 2, arrow = 'last', tag = cftag)
        self._canv.create_line(int(self._win_width/2), int(self._win_height/2),
                            int(self._win_width/2) + 50, int(self._win_height/2),
                            width = 2, arrow = 'last', tag = cftag)

        # Add coordinates to the corners.
        d = 6
        self._canv.create_text(
            d, d,
            text = '({:.1f}, {:.1f})'.format(
                -self._width/2, self._height/2),
            anchor = 'nw', tag = cftag)
        self._canv.create_text(
            d, self._win_height - d,
            text = '({:.1f}, {:.1f})'.format(
                -self._width/2, -self._height/2),
            anchor = 'sw', tag = cftag)
        self._canv.create_text(
            self._win_width - d, self._win_height - d,
            text = '({:.1f}, {:.1f})'.format(
                self._width/2, -self._height/2),
            anchor = 'se', tag = cftag)
        self._canv.create_text(
            self._win_width - d, d,
            text = '({:.1f}, {:.1f})'.format(
                self._width/2, self._height/2),
            anchor = 'ne', tag = cftag)


    def _draw_canvas(self, resp):
        """Draws things that should be on the canvas. Coordinate frame, corner
        coordinate texts, path, truck trajectory, truck position. """
        if self._display_tail:
            self._plot_sequence(
                self._tailn(self._saved_trajectory, 2), join = False,
                clr = 'green', tag = 'traj')

        self._move_truck(self._tr2, resp.x, resp.y, resp.yaw)


    def _plot_sequence(self, seq, join = False, clr = 'blue', width = 2,
                        tag = 'line'):
        """Plots a sequence, a list on the form [[x0, y0], [x1, y1], ...],
        where x1 and y1 are real coordinates. Joins the beginning and end
        if join = True. """
        if len(seq) > 0:
            if join:        # Join the first and the last points.
                starti = 0
            else:
                starti = 1
            try:
                for i in range(starti,len(seq)):
                    x1, y1 = self._real_to_pixel(seq[i - 1][0], seq[i - 1][1])
                    x2, y2 = self._real_to_pixel(seq[i][0], seq[i][1])
                    self._canv.create_line(x1, y1, x2, y2,
                                        fill = clr, width = width, tag = tag)
            except Exception as e:
                print('Error when plotting sequence: {}'.format(e))


    def _move_truck(self, truck, xreal, yreal, yaw):
        """Moves a truck triangle to the new position. """
        l = self._truckl # Truck length in meters.
        w = self._truckw

        # Coordinates for frontal corner.
        xf, yf = self._real_to_pixel(
            xreal + l/2*math.cos(yaw),
            yreal + l/2*math.sin(yaw))

        # Coordinates for rear right corner.
        xr, yr = self._real_to_pixel(
            xreal - l/2*math.cos(yaw) + w/2*math.cos(yaw - math.pi/2),
            yreal - l/2*math.sin(yaw) + w/2*math.sin(yaw - math.pi/2))

        # Coordinates for rear left corner.
        xl, yl = self._real_to_pixel(
            xreal - l/2*math.cos(yaw) + w/2*math.cos(yaw + math.pi/2),
            yreal - l/2*math.sin(yaw) + w/2*math.sin(yaw + math.pi/2))

        self._canv.tag_raise(truck)  # Move to top level of canvas.
        self._canv.coords(truck, xf, yf, xr, yr, xl, yl) # Move polygon.


    def _traj_btn_callback(self):
        """Callback for trajectory check button. Enable/disaple plottinf of
        trajectories. """
        if self.traj_button_var.get() == 1:
            self._display_tail = True
            self._plot_sequence(self._saved_trajectory, clr = 'green',
                tag = 'traj')
            self.clear_button.config(state = 'normal')
        else:
            self._display_tail = False
            self._canv.delete('traj')
            #self.clear_button.config(state = 'disabled')


    def _record_data(self, response):
        """Appends data to the list recording the trajectory."""
        if self._recording:
            self._recorded_path.append([response.x, response.y, response.yaw,
                                    self._timestamp])
            self.rec_time_text_var.set(
                'Recording: \n{:.1f}'.format(
                    time.time() - self._rec_start_time))


    def _start_record(self):
        """Starts a new recording of trajectories."""
        if self._recording:
            print('Already recording.')
        else:
            self._recorded_path = []
            self._recording = True
            self.start_record_button.config(state = 'disabled')
            self.stop_record_button.config(state = 'normal')
            self._rec_start_time = time.time()
            self.rec_time_label.config(foreground = 'black')
            print('Recording started.')


    def _stop_record(self):
        """Stops current recording of trajectories."""
        if not self._recording:
            print('No recording is running.')
        else:
            self._recording = False
            self.start_record_button.config(state = 'normal')
            self.stop_record_button.config(state = 'disabled')
            self.rec_time_text_var.set('Not recording\n')
            self.rec_time_label.config(foreground = 'grey')
            print('Recording stopped.')
            self._save_recorded()


    def _save_recorded(self):
        """Saves recorded path to file. Writes each line on the format
        x,y,yaw,servertime"""
        try:
            __location__ = os.path.realpath(os.path.join(os.getcwd(),
                                            os.path.dirname(__file__)))

            i = 0
            while os.path.exists(os.path.join(__location__, '{}{}{}'.format(
                self._save_filename, i, '.txt'))):
                i += 1

            filename = self._save_filename + str(i) + '.txt'
            #filename = filename_in

            fl = open(os.path.join(__location__, filename), 'w');

            for xy in self._recorded_path:
                fl.write('{},{},{},{}\n'.format(xy[0], xy[1], xy[2], xy[3]))

            fl.close()
            print('Saved as {}'.format(filename))

        except Exception as e:
            print('\nError when saving to file: {}'.format(e))


    def _real_to_pixel(self, xreal, yreal):
        """Transform from real to pixel coordinates. """
        xpixel = int(self._win_width/self._width * xreal + self._win_width/2)
        ypixel = int(-self._win_height/self._height * yreal + self._win_height/2)
        return xpixel, ypixel


    def _pixel_to_real(self, xp, yp):
        """Transform from pixel to real coordinates. """
        xreal = float((xp - self._win_width/2) * self._width/self._win_width)
        yreal = float((self._win_height/2 - yp) * self._height/self._win_height)
        return xreal, yreal


    def _tailn(self, seq, n):
        """Returns the last n of a list. If n < 0 return whole list. """
        l = len(seq)
        if n < 0 or n > l:
            return seq
        elif n == 0:
            return []
        else:
            return seq[l - n:l]


    def _clear_trajectories(self):
        """Clear the saved truck trajectories. """
        self._saved_trajectory = []
        self._canv.delete('traj')


    def load_path(self, filename):
        """Loads a path from a file. """
        self.pt.load(filename)
        self._plot_sequence(self.pt.path, join = True, tag = 'path',
            clr = 'blue', width = 2)


    def gen_circle_path(self, radius, points, center = [0, 0]):
        """Generates a circle/ellipse path. """
        self.pt.gen_circle_path(radius, points, center = center)
        self._plot_sequence(self.pt.path, join = True, tag = 'path',
            clr = 'blue', width = 2)


def main():
    width = 6                   # Width in meters of displayed area.
    height = 6                  # Height in meters.
    x_radius = 1.6              # Ellipse x-radius.
    y_radius = 1.2              # Ellipse y-radius.
    center = [0.3, -0.5]        # The coordinates of the center of the ellipse.
    pts = 200                   # Number of points on displayed reference path.
    display_tail = True         # If truck trajectories should be displayed.
    filename = 'record'         # Recorded files are saved as filenameNUM.txt
    node_name = 'truckplot_sub' # Name of subscriber node.
    topic_name = 'truck2'       # Name of topic it subscribes to.
    topic_type = truckmocap     # The type of the topic.

    root = tk.Tk()
    try:
        truckplot = TruckPlot(root, filename = filename, width = width,
            height = height, display_tail = display_tail,
            node_name = node_name, topic_name = topic_name,
            topic_type = topic_type)

        truckplot.gen_circle_path([x_radius, y_radius], pts, center = center)

        try:
            root.mainloop()
        except KeyboardInterrupt:
            print('Interrupted with keyboard.')

    except RuntimeError as e:
        print('Error when running GUI: {}'.format(e))


if __name__ == '__main__':
    main()
