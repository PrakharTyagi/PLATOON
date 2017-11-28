#!/usr/bin/env python

import rospy
from platoon.msg import drive_param
import curses
#import signal
#TIMEOUT = 0.1 # number of seconds your want for timeout
forward = 1500;
left = 90;
gear = 1;
gearval = 50;
# def interrupted(signum, frame):
#     "called when read times out"
#     global forward
#     forward = 0
#     global left
#     left = 0
#     stdscr.addstr(2, 20, "Stop")
#     stdscr.addstr(2, 25, '%.2f' % forward)
#     stdscr.addstr(3, 20, "Stop")
#     stdscr.addstr(3, 25, '%.2f' % left)
# signal.signal(signal.SIGALRM, interrupted)

# def input():
#     try:
#             foo = stdscr.getch()
#             return foo
#     except:
#             # timeout
#             return

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_talker', anonymous=True)
pub = rospy.Publisher('/vehicle_2/drive_parameters', drive_param, queue_size=10)

# set alarm
#signal.alarm(TIMEOUT)
#s = input()
# disable the alarm after success
#signal.alarm(0)
#print 'You typed', s

stdscr.refresh()

step = 10

key = ''
while key != ord('q'):
#	signal.setitimer(signal.ITIMER_REAL,0.05)
#	key = input()
	key = stdscr.getch()
	stdscr.refresh()
#	signal.alarm(0)

	#stdscr.addstr(1, 10, key)

	if key == curses.KEY_UP:
		forward = forward - step
		stdscr.addstr(2, 20, "Up  ")
		stdscr.addstr(2, 25, '%.2f' % forward)
		stdscr.addstr(6, 20, "    ")
	elif key == curses.KEY_DOWN:
		forward = forward + step;
		stdscr.addstr(2, 20, "Down")
		stdscr.addstr(2, 25, '%.2f' % forward)
		stdscr.addstr(6, 20, "    ")
	if key == curses.KEY_LEFT:
		left = left - step;
		stdscr.addstr(3, 20, "left")
		stdscr.addstr(3, 25, '%.2f' % left)
		stdscr.addstr(6, 20, "    ")
	elif key == curses.KEY_RIGHT:
		left = left + step;
		stdscr.addstr(3, 20, "rgt ")
		stdscr.addstr(3, 25, '%.2f' % left)
		stdscr.addstr(6, 20, "    ")
	"""if key == curses.KEY_NPAGE:
		if(gear > 60):
			gear = gear - 60;
		temp = (gear / 60)
		stdscr.addstr(4, 20, "gear %d " % temp)
		#stdscr.addstr(4, 25, '%.2f' % gear)
		stdscr.addstr(6, 20, "    ")
	elif key == curses.KEY_PPAGE:
		if(gear < 180):
			gear = gear + 60;
		temp = (gear /60)
		stdscr.addstr(4, 20, "gear %d " % temp)
		#stdscr.addstr(4, 25, '%.2f' % gear)
		stdscr.addstr(6, 20, "    ")"""

	if key == curses.KEY_NPAGE:
		if(gear > 1):
			gear = gear - 1;

		if (gear == 1):
			gearval = 60

		elif (gear == 2):
			gearval = 140 	#120

		else:
			gearval = 220	#200

		stdscr.addstr(4, 20, "gear %d " % gear)
		#stdscr.addstr(4, 25, '%.2f' % gear)
		stdscr.addstr(6, 20, "    ")

	elif key == curses.KEY_PPAGE:
		if(gear < 3):
			gear = gear + 1;

		if (gear == 1):
			gearval = 60

		elif (gear == 2):
			gearval = 140	#120

		else:
			gearval = 220	#200

		stdscr.addstr(4, 20, "gear %d " % gear)
		#stdscr.addstr(4, 25, '%.2f' % gear)
		stdscr.addstr(6, 20, "    ")

	if key == curses.KEY_DC:
		left = 0
		forward = 0
		stdscr.addstr(6, 20, "Stop")
	msg = drive_param()
	msg.velocity = forward
	msg.angle = left
	msg.gear = gearval
	pub.publish(msg)
curses.endwin()
