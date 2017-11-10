#!/usr/bin/env python

# Class for describing a path.

# TODO handle parametrization of path?

import rospy
import matplotlib.pyplot as plt

from Tkinter import *


class Path:
    """Class for a path. Path is described by a series of coordinate pairs."""
    def __init__(self):
        self.path = []
        # TODO add variables characterizing the area the path is in?


    def save(self, filename):
        """Saves path to file."""
        fl = open(filename, 'w')

        if len(self.path) == 0:
            print('\nWarning: writing empty path to file.')

        for xy in self.path:
            fl.write('{},{}\n'.format(xy[0], xy[1]))

        fl.close()


    def load(self, filename):
        """Loads path from file."""
        self.path = []

        try:
            fl = open(filename, 'r')

            for line in fl:
                line_list = line.strip().split(',')

                if len(line_list) > 1:
                    x = float(line_list[0])
                    y = float(line_list[1])
                    self.path.append((x, y))

        except Exception as e:
            print('\nError when loading file: '),
            print(e)

        try:
            fl.close()
        except:
            pass

    def interpolate(self):
        """Interpolates the path. Returns a denser list."""
        pass


    def visualize(self):
        """Plots the path."""
        x, y = self.split()
        plt.plot(x, y)
        plt.show()


    def visualize2(self):
        root = Tk()

        s_frame = Frame(root, background = 'aquamarine')
        s_frame.pack()

        canv_frame = Frame(root)
        canv_frame.pack(in_ = s_frame, side=LEFT)

        canv = Canvas(root, width = 500, height = 500, background='#FFFFFF',
                    borderwidth = 5, relief = RAISED)
        canv.pack(in_ = canv_frame)
        canv.configure(scrollregion = (-250, -250, 250, 250))
        canv.bind('<Button-1>', self.redraw)

        right_frame = Frame(root, background = 'aquamarine')
        right_frame.pack(in_ = s_frame, side = RIGHT, anchor = N)
        quit_button = Button(root, text = 'Quit', command = quit,
                            width = 10, background = 'coral',
                            activebackground = 'red')
        quit_button.pack(in_ = right_frame)

        while True:
            root.update()
            inpt = eval(raw_input('\nEnter 0 to exit: '))
            if inpt == 0 or 'normal' != root.state():
                root.quit()
                break
            else:
                pass

    def quit():
        root.quit()

    def redraw(self, event):
        pass


    def split(self):
        """Returns two lists, one containing x and one containing y."""
        x = [a for a,b in self.path]
        y = [b for a,b in self.path]

        return x, y


    def printp(self):
        """Prints the path in the terminal."""
        for xy in self.path:
            print('{:07.4f}, {:07.4f}'.format(xy[0], xy[1]))


    def get_xy(self, index):
        """# Returns x and y at the given index."""
        try:
            x = self.path[index][0]
            y = self.path[index][1]
            return x, y

        except Exception as e:
            print('\nError when retrieving x and y: '),
            print(e)
            return 0, 0



if __name__ == '__main__':
    pt = Path()
    pt.load('hej1.txt')
    pt.save('hej2.txt')
    #pt.visualize2()
