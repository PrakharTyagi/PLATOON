#!/usr/bin/env python

# Class for describing a path.

# TODO handle parametrization of path?
# gamma and gamma derivative (curvature?)

import rospy
from Tkinter import *
import os
import math


class Path:
    """Class for a path. Path is described by a series of coordinate pairs."""
    def __init__(self):
        self.path = []
        self.gamma = []
        self.gammap = []
        self.gammapp = []
        self.lp = True


    def save(self, filename):
        """Saves path to file. Writes each line on the format x,y"""
        try:
            __location__ = os.path.realpath(os.path.join(os.getcwd(),
                                            os.path.dirname(__file__)))
            fl = open(os.path.join(__location__, filename), 'w');

            for xy in self.path:
                fl.write('{},{}\n'.format(xy[0], xy[1]))

            fl.close()
            print('Saved path as {}'.format(filename))

        except Exception as e:
            print('\nError when saving path to file: '),
            print(e)


    def load(self, filename):
        """Loads path from file. Assumes lines to be on the format x,y"""
        self.path = []
        try:
            __location__ = os.path.realpath(os.path.join(os.getcwd(),
                                            os.path.dirname(__file__)))
            fl = open(os.path.join(__location__, filename), 'r');

            for line in fl:
                line_list = line.strip().split(',')

                if len(line_list) > 1:
                    x = float(line_list[0])
                    y = float(line_list[1])
                    self.path.append((x, y))

            self.calc_gammas()

        except Exception as e:
            print('\nError when loading path from file: '),
            print(e)

        try:
            fl.close()
        except:
            pass


    def interpolate(self):
        """Interpolates the path. Returns a denser list that has added one set
        of intermediate points to the original path."""
        N = len(self.path)
        interlist = [[0, 0] for i in range(N - 1)]
        for i in range(N - 1):
            interlist[i][0] = (self.path[i][0] + self.path[i + 1][0])/2
            interlist[i][1] = (self.path[i][1] + self.path[i + 1][1])/2

        newlist = [[0, 0] for i in range(2*N - 1)]
        for i in range(N - 1):
            newlist[i*2] = self.path[i]
            newlist[i*2 + 1] = interlist[i]
        newlist[2*N - 2] = self.path[N - 1]

        self.path = newlist


    def plot(self, realh, realw):
        """Plots the path in a Tkinter window. Arguments are the width and
        height of the real path area in meters."""
        root = Tk()
        h = 600     # Tkinter canvas height.
        w = 600

        s_frame = Frame(root, background = 'aquamarine')
        s_frame.pack()

        canv_frame = Frame(root)
        canv_frame.pack(in_ = s_frame, side=LEFT)

        canv = Canvas(root, width = w, height = h, background='#FFFFFF',
                    borderwidth = 0, relief = RAISED)
        canv.pack(in_ = canv_frame)

        right_frame = Frame(root, background = 'aquamarine')
        right_frame.pack(in_ = s_frame, side = RIGHT, anchor = N)
        quit_button = Button(root, text = 'Close', command = self.quitm,
                            width = 10, background = 'coral',
                            activebackground = 'red')
        quit_button.pack(in_ = right_frame)

        canv.create_line(int(w/2), int(h/2), int(w/2), int(h/2) - 50,
                        width = 2, arrow = 'last')
        canv.create_line(int(w/2), int(h/2), int(w/2) + 50, int(h/2),
                        width = 2, arrow = 'last')

        canv.create_text(2, 2, text = '({}, {})'.format( -realw, realh),
                            anchor = 'nw')
        canv.create_text(2, h - 2, text = '({}, {})'.format(-realw, -realh),
                            anchor = 'sw')
        canv.create_text(w - 2, h - 2, text = '({}, {})'.format(realw, -realh),
                            anchor = 'se')
        canv.create_text(w - 2, 2, text = '({}, {})'.format(realw, realh),
                            anchor = 'ne')

        root.protocol("WM_DELETE_WINDOW", self.quitm)

        try:
            for i in range(len(self.path)):
                xy1 = self.pixelv(i - 1, realh, realw, h, w)
                xy2 = self.pixelv(i, realh, realw, h, w)
                canv.create_line(xy1[0], xy1[1], xy2[0], xy2[1],
                                    fill = 'blue', width = 2)
                canv.create_oval(xy2[0] - 3, xy2[1] - 3,
                                        xy2[0] + 3, xy2[1] + 3, fill = 'green')
        except Exception as e:
            print(e)

        while self.lp:
            try:
                if 'normal' == root.state():
                    root.update()
            except:
                pass

        root.destroy()


    def pixelv(self, index, hreal, wreal, hpixel, wpixel):
        """Transforms the path in real coordinates to pixel coordinates."""
        try:
            xpixel = int(wpixel/wreal * self.path[index][0] + wpixel/2)
            ypixel = int(- hpixel/hreal * self.path[index][1] + hpixel/2)
            return [xpixel, ypixel]
        except:
            return [0, 0]

    def quitm(self):
        self.lp = False


    def split(self):
        """Returns two lists, one containing x and one containing y."""
        xlist = [a for a,b in self.path]
        ylist = [b for a,b in self.path]

        return xlist, ylist


    def printp(self):
        """Prints the path in the terminal."""
        for xy in self.path:
            print('{:07.4f}, {:07.4f}'.format(xy[0], xy[1]))


    def get_xy(self, index):
        """# Returns x and y at the given index."""
        try:
            x = self.path[index][0]
            y = self.path[index][1]
            return [x, y]

        except Exception as e:
            print('\nError when retrieving x and y: '),
            print(e)
            return [0, 0]


    def get_closest(self, xy):
        """Return the closest x and y of the path to the given coordinates,
        as well as the index of the path list it is found on."""
        try:
            closest = min(self.path,
                        key = lambda a: (a[0] - xy[0])**2 + (a[1] - xy[1])**2)
            index = self.path.index(closest)
            return index, closest

        except Exception as e:
            print('\nError when retrieving closest point on path: '),
            print(e)
            return 0, [0, 0]


    def get_tangent(self, index):
        """Returns a unit vector approximating the tangent direction at the
        given index"""
        try:
            vec = [self.path[index + 1][0] - self.path[index - 1][0],
                    self.path[index + 1][1] - self.path[index - 1][1]]
            vec_norm = math.sqrt(vec[0]**2 + vec[1]**2)
            vec[0] = vec[0]/vec_norm
            vec[1] = vec[1]/vec_norm
            return vec
        except Exception as e:
            print('\nError when calculating tangent: '),
            print(e)
            return [0, 0]


    def get_derivative(self, index):
        """Returns a approximation of the derivative of y w.r.t. x at the
        given index."""
        try:
            return (self.path[index + 1][1] - self.path[index - 1][1])/(
                    self.path[index + 1][0] - self.path[index - 1][0])
        except Exception as e:
            print('\nError when calculating derivative: '),
            print(e)
            return 0


    def get_angle(self, index):
        """Returns the angle of the tangent of the path in radians."""

        return 0


    def gen_circle_path(self, radius, points):
        """Generates a circle path with specified radius and number of
        points."""
        newpath = []
        for i in range(points):
            x = radius*math.cos(2*math.pi*i/points)
            y = radius*math.sin(2*math.pi*i/points)
            newpath.append([x, y])

        self.path = newpath
        self.calc_gammas()

    def calc_gammas(self):
        """Used by class to calculate gammas."""
        self.calc_gamma()
        self.calc_gammap()
        self.calc_gammapp()

    def get_gamma(self, index):
        try:
            return self.gamma[index]
        except Exception as e:
            print('\nError when getting gamma: '),
            print(e)
            return 0

    def get_gammap(self, index):
        try:
            return self.gammap[index]
        except Exception as e:
            print('\nError when getting gamma prime: '),
            print(e)
            return 0

    def get_gammapp(self, index):
        try:
            return self.gammapp[index]
        except Exception as e:
            print('\nError when getting gamma prime prime: '),
            print(e)
            return 0


    def calc_gamma(self):
        """Used by class to calculate gamma values and save them."""
        self.gamma = [0 for i in range(len(self.path))]

        for i in range(len(self.path)):
            x1 = self.path[i - 1][0]
            y1 = self.path[i - 1][1]
            if i == len(self.path) - 1:
                x2 = self.path[0][0]
                y2 = self.path[0][1]
            else:
                x2 = self.path[i + 1][0]
                y2 = self.path[i + 1][1]
            try:
                rad = math.atan((y2 - y1)/(x2 - x1))
                if x2 < x1:
                    if y2 > y1:
                        rad = rad - math.pi
                    else:
                        rad = rad + math.pi
                self.gamma[i] = rad
            except ZeroDivisionError:
                if y2 > y1:
                    self.gamma[i] = math.pi/2
                else:
                    self.gamma[i] = 3*math.pi/2



    def calc_gammap(self):
        """Used by class to calculate gamma prime."""
        self.gammap = [0 for i in range(len(self.path))]

        for i in range(len(self.gamma)):
            x1 = self.path[i - 1][0]
            y1 = self.path[i - 1][1]
            x2 = self.path[i][0]
            y2 = self.path[i][1]
            if i == len(self.path) - 1:
                x3 = self.path[0][0]
                y3 = self.path[0][1]
                self.gammap[i] = (self.gamma[0] - self.gamma[i - 1]) / (
                                math.sqrt((x3 - x2)**2 + (y3 - y2)**2) + math.sqrt(
                                (x2 - x1)**2 + (y2 - y1)**2))
            else:
                x3 = self.path[i + 1][0]
                y3 = self.path[i + 1][1]

                self.gammap[i] = (self.gamma[i + 1] - self.gamma[i - 1]) / (
                                math.sqrt((x3 - x2)**2 + (y3 - y2)**2) + math.sqrt(
                                (x2 - x1)**2 + (y2 - y1)**2))

    def calc_gammapp(self):
        """Used by class to calculate gamma prime prime."""
        self.gammapp = [0 for i in range(len(self.path))]

        for i in range(len(self.gamma)):
            x1 = self.path[i - 1][0]
            y1 = self.path[i - 1][1]
            x2 = self.path[i][0]
            y2 = self.path[i][1]
            if i == len(self.path) - 1:
                x3 = self.path[0][0]
                y3 = self.path[0][1]
                self.gammapp[i] = (self.gammap[0] - self.gammap[i - 1]) / (
                            math.sqrt((x3 - x2)**2 + (y3 - y2)**2) + math.sqrt(
                            (x2 - x1)**2 + (y2 - y1)**2))
            else:
                x3 = self.path[i + 1][0]
                y3 = self.path[i + 1][1]

                self.gammapp[i] = (self.gammap[i + 1] - self.gammap[i - 1]) / (
                            math.sqrt((x3 - x2)**2 + (y3 - y2)**2) + math.sqrt(
                            (x2 - x1)**2 + (y2 - y1)**2))



if __name__ == '__main__':
    pt = Path()
    pt.gen_circle_path(1, 100)
    #a = pt.get_gamma(4)
    #pt.printp()
    #pt.load('hej2.txt')
    #pt.plot(4, 4)
    #pt.printp()
    #pt.interpolate()


    pt.save('hej2.txt')
