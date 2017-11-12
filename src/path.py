#!/usr/bin/env python

# Class for describing a path.

# TODO handle parametrization of path?

import rospy
import matplotlib.pyplot as plt
from Tkinter import *
import os


class Path:
    """Class for a path. Path is described by a series of coordinate pairs."""
    def __init__(self):
        self.path = []
        self.newpath = []
        # TODO add variables characterizing the area the path is in?


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


    def visualize(self):
        """Plots the path."""
        xlist, ylist = self.split()
        plt.plot(xlist, ylist, xlist, ylist, 'ro')
        plt.show()


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
        pass


    def get_derivative(self, index):
        """Returns a approximation of the derivative of y w.r.t. x at the
        given index."""
        pass



if __name__ == '__main__':
    pt = Path()
    pt.load('hej3.txt')
    pt.visualize()
    #pt.printp()
    #pt.interpolate()
    #pt.visualize()

    pt.save('hej2.txt')
