#!/usr/bin/env python

# Class for GUI that plots the truck trajectories.

import rospy
from modeltruck_platooning.srv import *
import time
import Tkinter as tk
import path
import math

class TruckPlot():
    """Class for GUI that plots the truck trajectories. """
    def __init__(self, root, width = 5, height = 5, update_ts = 0.1):
        self.width = float(width)       # Real width (meters) of the window.
        self.height = float(height)
        self.win_height = 800           # Graphical window height.
        self.win_width = int(self.win_height*self.width/self.height)
        self.update_ts = update_ts  # Update interval of the plot.
        self.service_found = True
        self.pt = path.Path()       # A fixed path to draw.
        self.recorded_path = []     # Recorded path of a truck.
        self.tail_time = 4          # How many seconds back to plot trajectory.

        # Try to connect to the server. Quit application if failed.
        try:
            rospy.wait_for_service('test_plot', timeout = 2)
            self.test_plot = rospy.ServiceProxy('test_plot', TestPlot)
        except Exception as e:
            print('Service connection failed: {}'.format(e))
            self.service_found = False

        if not self.service_found:
            print('No service found.')
            self._quit1()
            raise RuntimeError('Could not connect to server.')

        bg_color = 'SlateGray2'
        w1 = 10
        ypad = 10
        # Base frame.
        s_frame = tk.Frame(root, background = bg_color)
        s_frame.pack()

        # Create canvas frame with a canvas for drawing in.
        canv_frame = tk.Frame(root)
        canv_frame.pack(in_ = s_frame, side= tk.LEFT)
        self.canv = tk.Canvas(root, width = self.win_width,
                            height = self.win_height, background='#FFFFFF',
                            borderwidth = 0, relief = tk.RAISED)
        self.canv.pack(in_ = canv_frame)
        self.canv.bind('<Button-1>', self._left_click)

        # Create frame next to the canvas for buttons, labels etc.
        right_frame = tk.Frame(root, background = bg_color)
        right_frame.pack(in_ = s_frame, side = tk.RIGHT, anchor = tk.N)

        # Create button frame.
        button_frame = tk.Frame(root, background = bg_color)
        button_frame.pack(in_ = right_frame, side = tk.TOP, anchor = tk.N,
                            pady = (0, ypad))

        # Create frame for recording widgets.
        record_frame = tk.Frame(root, background = bg_color)
        record_frame.pack(in_ = right_frame, side = tk.TOP,
                            anchor = tk.N, pady = (0, ypad))

        # Create bottom frame for other stuff.
        bottom_frame = tk.Frame(root, background = bg_color)
        bottom_frame.pack(in_ = right_frame, side = tk.TOP, anchor = tk.N,
                            pady = (0, ypad))

        # Button for quitting the program.
        quit_button = tk.Button(root, text = 'Quit',
                            command = self._quit1,
                            width = w1, height = 2, background = 'red2',
                            activebackground = 'red3')
        quit_button.pack(in_ = button_frame)

        # Button for clearing trajectories.
        clear_button = tk.Button(root, text = 'Clear trajectories',
                            command = self._clear_trajectories,
                            width = w1, height = 2, background = 'orange',
                            activebackground = 'dark orange')
        clear_button.pack(in_ = button_frame)

        # Buttons for recording trajectories.
        start_record_button = tk.Button(root, text = 'Start new\nrecording',
            command = self._start_record, width = w1, height = 2)
        start_record_button.pack(in_ = record_frame)
        stop_record_button = tk.Button(root, text = 'Stop\nrecording',
            command = self._stop_record, width = w1, height = 2)
        stop_record_button.pack(in_ = record_frame)

        # Label displaying the elapsed server time.
        self.time_text_var = tk.StringVar()
        self.time_label = tk.Label(root, textvariable = self.time_text_var,
                                    anchor = tk.W, justify = tk.LEFT,
                                    width = 13, height = 2,
                                    background = bg_color)
        self.time_text_var.set('')
        self.time_label.pack(in_ = bottom_frame)

        # Actions for closing the window and pressing ctrl-C on the window.
        root.protocol("WM_DELETE_WINDOW", self._quit1)
        root.bind('<Control-c>', self._quit2)

        self._refresher()   # Start the refresher that draws everything.


    def _left_click(self, event):
        """Action when left clicking on the canvas. """
        xreal, yreal = self._pixel_to_real(event.x, event.y)
        print('Clicked at ({:07.4f}, {:07.4f})'.format(xreal, yreal))


    def _quit1(self):
        """Quits the GUI. """
        print('Quitting.')
        root.quit()


    def _quit2(self, event):
        """Quits the GUI. """
        print('Quitting.')
        root.quit()


    def _refresher(self):
        """Runs every self.update_ts seconds. Calls all methods to run at each
        update step. """
        try:
            # Get data from the server.
            response = self.test_plot()
            self.recorded_path.append([response.x, response.y])
            self._draw_canvas(response)
            self.time_text_var.set(
                'Server time: \n{:.1f}'.format(response.timestamp))
        except rospy.ServiceException as e:
            print('Service call failed: {}'.format(e))

        root.after(int(self.update_ts*1000), self._refresher)


    def _draw_cf(self):
        """Draw lines for the origin and create text displaying the coordinates
        in the corners. """
        # Create origin coordinate arrows.
        self.canv.create_line(int(self.win_width/2), int(self.win_height/2),
                            int(self.win_width/2), int(self.win_height/2) - 50,
                            width = 2, arrow = 'last')
        self.canv.create_line(int(self.win_width/2), int(self.win_height/2),
                            int(self.win_width/2) + 50, int(self.win_height/2),
                            width = 2, arrow = 'last')

        # Add coordinates to the corners.
        self.canv.create_text(
            2, 2, text = '({}, {})'.format(-self.width/2, self.height/2),
            anchor = 'nw')
        self.canv.create_text(
            2, self.win_height - 2, text = '({}, {})'.format(
                -self.width/2, -self.height/2),
            anchor = 'sw')
        self.canv.create_text(
            self.win_width - 2, self.win_height - 2, text = '({}, {})'.format(
                self.width/2, -self.height/2),
            anchor = 'se')
        self.canv.create_text(self.win_width - 2, 2, text = '({}, {})'.format(
                self.width/2, self.height/2),
            anchor = 'ne')


    def _draw_canvas(self, resp):
        """Draws things that should be on the canvas. Coordinate frame, corner
        coordinate texts, path, truck trajectory, truck position. """
        self.canv.delete('all')
        self._draw_cf()
        self._plot_sequence(self.pt.path, True)
        self._plot_sequence(
            self._tailn(self.recorded_path, int(self.tail_time/self.update_ts)),
            False, 'green')
        self._draw_truck(resp.x, resp.y, resp.yaw, 'green')


    def _plot_sequence(self, seq, join, clr = 'blue', wid = 2):
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
                    self.canv.create_line(x1, y1, x2, y2,
                                        fill = clr, width = wid)
            except Exception as e:
                print('Error when plotting sequence: {}'.format(e))


    def _draw_truck(self, xreal, yreal, yaw, clr = 'black'):
        """Draws a triangle centered at the truck position. """
        l = 0.2 # Truck length in meters.
        w = 0.1

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

        self.canv.create_polygon(xf, yf, xr, yr, xl, yl, fill = clr)


    def _start_record(self):
        """Starts a new recording of trajectories."""
        print('TODO: write start recording method.')


    def _stop_record(self):
        """Stops current recording of trajectories."""
        print('TODO: write stop recording method.')


    def _save_record(self):
        """Saves the recorded trajectories to a file."""
        print('TODO: write save recording method.')


    def _real_to_pixel(self, xreal, yreal):
        """Transform from real to pixel coordinates. """
        xpixel = int(self.win_width/self.width * xreal + self.win_width/2)
        ypixel = int(-self.win_height/self.height * yreal + self.win_height/2)
        return xpixel, ypixel


    def _pixel_to_real(self, xp, yp):
        """Transform from pixel to real coordinates. """
        xreal = float((xp - self.win_width/2) * self.width/self.win_width)
        yreal = float((self.win_height/2 - yp) * self.height/self.win_height)
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
        self.recorded_path = []


    def load_path(self, filename):
        """Loads a path from a file. """
        self.pt.load(filename)


    def gen_circle_path(self, radius, points):
        """Generates a circle/ellipse path. """
        self.pt.gen_circle_path(radius, points)


if __name__ == '__main__':
    width = 5
    height = 5
    update_ts = 0.01

    root = tk.Tk()
    try:
        pp = TruckPlot(root, width, height, update_ts)
        pp.gen_circle_path([1.7, 2.2], 200)

        try:
            root.mainloop()
        except KeyboardInterrupt:
            print('Interrupted with keyboard.')

    except RuntimeError as e:
        print('Error when running GUI: {}'.format(e))
