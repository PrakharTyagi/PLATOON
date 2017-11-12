#!/usr/bin/env python

from Tkinter import *
import rospy
import os

class NewPath:
    def __init__(self, root, filename):
        self.newpath = []
        self.graphicpath = []
        self.win_height = 800
        self.win_width = self.win_height
        self.height = 4.0      # Width of area in meters.
        self.width = self.height

        s_frame = Frame(root, background = 'aquamarine')
        s_frame.pack()

        canv_frame = Frame(root)
        canv_frame.pack(in_ = s_frame, side=LEFT)

        self.canv = Canvas(root, width = self.win_width,
                    height = self.win_height, background='#FFFFFF',
                    borderwidth = 0, relief = RAISED)
        self.canv.pack(in_ = canv_frame)
        #self.canv.configure(scrollregion = (-self.win_size/2, -self.win_size/2,
        #                                    self.win_size/2, self.win_size/2))
        self.canv.bind('<Button-1>', self.append_new_path)

        right_frame = Frame(root, background = 'aquamarine')
        right_frame.pack(in_ = s_frame, side = RIGHT, anchor = N)
        quit_button = Button(root, text = 'Quit and \nsave path', command = self.myquit,
                            width = 10, height = 2, background = 'coral',
                            activebackground = 'red')
        quit_button.pack(in_ = right_frame)

        self.canv.create_line(int(self.win_width/2), int(self.win_height/2),
                            int(self.win_width/2),
                            int(self.win_height/2) - 50, width = 2,
                            arrow = 'last')
        self.canv.create_line(int(self.win_width/2), int(self.win_height/2),
                            int(self.win_width/2) + 50,
                            int(self.win_height/2), width = 2,
                            arrow = 'last')

        self.canv.create_text(2, 2,
                            text = '({}, {})'.format(
                            -self.width/2, self.height/2),
                            anchor = 'nw')
        self.canv.create_text(2, self.win_height - 2,
                            text = '({}, {})'.format(-self.width/2,
                            -self.height/2),
                            anchor = 'sw')
        self.canv.create_text(self.win_width - 2, self.win_height - 2,
                            text = '({}, {})'.format(self.width/2,
                            -self.height/2),
                            anchor = 'se')
        self.canv.create_text(self.win_width - 2, 2,
                            text = '({}, {})'.format(self.width/2,
                            self.height/2),
                            anchor = 'ne')


    def myquit(self):
        #self.path = self.newpath
        #self.newpath = []
        self.save(filename)
        root.quit()

    def save(self, filename):
        """Saves path to file. Writes each line on the format x,y"""
        try:
            __location__ = os.path.realpath(os.path.join(os.getcwd(),
                                            os.path.dirname(__file__)))
            fl = open(os.path.join(__location__, filename), 'w');

            for xy in self.newpath:
                fl.write('{},{}\n'.format(xy[0], xy[1]))

            fl.close()
            print('Saved path as {}'.format(filename))

        except Exception as e:
            print('\nError when saving path to file: '),
            print(e)


    def append_new_path(self, event):
        if (0 <= event.x <= self.win_width) and (
            0 <= event.y <= self.win_height):
            x = event.x
            y = event.y
            y2 = self.win_height - event.y
            print('Clicked at:     x: ' + str(x) + '    y: ' +  str(y))

            self.draw(x, y)
            self.newpath.append(self.transform([x, y]))
            self.graphicpath.append([x, y])


    def draw(self, x, y):
        try:
            self.canv.create_line(self.graphicpath[-1][0],
                                self.graphicpath[-1][1],
                                    x, y, fill = 'blue', width = 2)
        except:
            pass
        self.canv.create_oval(x - 3, y - 3, x + 3, y + 3, fill = 'green')
        #self.canv.delete('all')

    def transform(self, xy):
        x = float((xy[0] - self.win_width/2) * self.width/self.win_width)
        y = float((self.win_height/2 - xy[1]) * self.height/self.win_height)
        return [x, y]

    def printp(self):
        """Prints the path in the terminal."""
        for xy in self.newpath:
            print('{:07.4f}, {:07.4f}'.format(xy[0], xy[1]))


if __name__ == '__main__':

    filename = 'hej2.txt'

    root = Tk()

    newpath = NewPath(root, filename)
    root.title('Record path')
    root.mainloop()
