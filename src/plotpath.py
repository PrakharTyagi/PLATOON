#!/usr/bin/env python

import sys
import rospy
from modeltruck_platooning.srv import *
import time
import Tkinter as tk
import path

class PlotPath():

    def __init__(self, root, width = 5, height = 5, update_ts = 0.1):
        self.width = float(width)       # Real width (meters) of the window.
        self.height = float(height)
        self.win_height = 800           # Graphical window height
        self.win_width = int(self.win_height*self.width/self.height)
        self.update_ts = update_ts
        self.service_found = True
        self.has_path = False
        self.pt = path.Path()

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

        # Base frame.
        s_frame = tk.Frame(root, background = 'aquamarine')
        s_frame.pack()
        # Create canvas frame with a canvas for drawing in.
        canv_frame = tk.Frame(root)
        canv_frame.pack(in_ = s_frame, side= tk.LEFT)
        self.canv = tk.Canvas(root, width = self.win_width,
                    height = self.win_height, background='#FFFFFF',
                    borderwidth = 0, relief = tk.RAISED)
        self.canv.pack(in_ = canv_frame)
        self.canv.bind('<Button-1>', self._left_click)

        # Create frame next to the canvas for putting buttons etc in.
        right_frame = tk.Frame(root, background = 'aquamarine')
        right_frame.pack(in_ = s_frame, side = tk.RIGHT, anchor = tk.N)
        # Button for quitting the program.
        quit_button = tk.Button(root, text = 'Quit',
                            command = self._quit1,
                            width = 10, height = 1, background = 'red3',
                            activebackground = 'red4')
        quit_button.pack(in_ = right_frame)

        root.protocol("WM_DELETE_WINDOW", self._quit1)  # Window close action.
        root.bind('<Control-c>', self._quit2)   # Ctrl-C when GUI is selected.

        self._draw_cf()

        self._refresher()

    def _draw_cf(self):
        """Draw lines for the origin and create text displaying the coordinates
        in the corners."""
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


    def _left_click(self, event):
        """Action when left clicking on the canvas."""
        xreal, yreal = self._pixel_to_real(event.x, event.y)
        print('Clicked at ({:07.4f}, {:07.4f})'.format(xreal, yreal))


    def _plot_sequence(self, seq, clr = 'blue', wid = 2):
        """Plots a sequence, a list on the form [[x0, y0], [x1, y1], ...], where
        x1 and y1 are real coordinates."""
        if len(seq) > 0:
            try:
                for i in range(len(seq)):
                    x1, y1 = self._real_to_pixel(seq[i - 1][0], seq[i - 1][1])
                    x2, y2 = self._real_to_pixel(seq[i][0], seq[i][1])
                    self.canv.create_line(x1, y1, x2, y2,
                                        fill = clr, width = wid)
            except Exception as e:
                print('Error when plotting sequence: {}'.format(e))


    def _quit1(self):
        """Quits the GUI."""
        print('Quitting.')
        root.quit()


    def _quit2(self, event):
        """Quits the GUI."""
        print('Quitting.')
        root.quit()


    def _refresher(self):
        """Runs every self.update_ts seconds. Calls all methods to run at each
        update step."""
        try:
            # Get data from the server.
            resp = self.test_plot()
            self._draw_truck(resp.x, resp.y, resp.yaw)
        except rospy.ServiceException, e:
            print('Service call failed: {}'.format(e))

        root.after(int(self.update_ts*1000), self._refresher)


    def _draw_truck(self, xreal, yreal, yaw):
        """Draws the truck at the given position."""
        xp, yp = self._real_to_pixel(xreal, yreal)
        # self.canv.delete('all')
        # self._draw_cf()
        # self._plot_sequence(self.pt.path)
        self.canv.create_oval(xp - 3, yp - 3, xp + 3, yp + 3, fill = 'green')


    def _real_to_pixel(self, xreal, yreal):
        """Transform from real to pixel coordinates."""
        xpixel = int(self.win_width/self.width * xreal + self.win_width/2)
        ypixel = int(-self.win_height/self.height * yreal + self.win_height/2)
        return xpixel, ypixel


    def _pixel_to_real(self, xp, yp):
        """Transform from pixel to real coordinates."""
        xreal = float((xp - self.win_width/2) * self.width/self.win_width)
        yreal = float((self.win_height/2 - yp) * self.height/self.win_height)
        return xreal, yreal


    def load_path(self, filename):
        """Loads a path from a file."""
        self.pt.load(filename)
        self._plot_sequence(self.pt.path)


    def gen_circle_path(self, radius, points):
        """Generates a circle/ellipse path."""
        self.pt.gen_circle_path(radius, points)
        self._plot_sequence(self.pt.path)


if __name__ == '__main__':
    root = tk.Tk()
    try:
        pp = PlotPath(root)
        pp.gen_circle_path(1.9, 200)

        try:
            root.mainloop()
        except KeyboardInterrupt:
            print('Interrupted with keyboard.')

    except RuntimeError as e:
        print('Error when running GUI: {}'.format(e))
