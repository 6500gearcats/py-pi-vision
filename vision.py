import grip, datetime, math
import cv2
import numpy as np
import pixy
from ctypes import *
from pixy import *

import time
import math
from networktables import NetworkTables


NetworkTables.initialize(server='10.87.24.43')
table = NetworkTables.getTable('SmartDashboard')
subtable = table.getSubTable('vision')

targetdistance = 11.5

pixy.init()
pixy.change_prog("line")

middleLeftX = 35
middleRightX = 45

def iscentered(num):
	return ((num <= middleRightX) and (num >= middleLeftX))

def isleft(num):
	return (num < middleLeftX)

def isright(num):
	return (num > middleRightX)


class Vector(Structure):
	_fields_ = [("m_x0", c_uint), ("m_y0", c_uint), ("m_x1", c_uint), ("m_y1", c_uint), ("m_index", c_uint), ("m_flags", c_uint)]
class IntersectionLine(Structure):
	_fields_ = [("m_index", c_uint), ("m_reserved", c_uint), ("m_angle", c_uint)]

vectors = VectorArray(100)
intersections = IntersectionLineArray(100)
frame = 0
distanceaveragelist = []
angleaveragelist = []


debug = True
def dbgprint(*args, **kwargs):
	if debug:
		print(' '.join(str(i) for i in args), **kwargs)


# CAMERA SETTINGS!
#                     brightness (int)    : min=0 max=255 step=1 default=-8193 value=            5
#                       contrast (int)    : min=0 max=255 step=1 default=57343 value=          128
#                     saturation (int)    : min=0 max=255 step=1 default=57343 value=          200
# white_balance_temperature_auto (bool)   : default=1 value=                                     0
#                           gain (int)    : min=0 max=255 step=1 default=57343 value=            0
#           power_line_frequency (menu)   : min=0 max=2 default=2 value=                         1
#                                0: Disabled
#                                1: 50 Hz
#                                2: 60 Hz
#      white_balance_temperature (int)    : min=0 max=10000 step=10 default=61432 value=         0
#                      sharpness (int)    : min=0 max=255 step=1 default=57343 value=            0
#         backlight_compensation (int)    : min=0 max=1 step=1 default=57343 value=              1
#                  exposure_auto (menu)   : min=0 max=3 default=0 value=                         1
#                                1: Manual Mode
#                                3: Aperture Priority Mode
#              exposure_absolute (int)    : min=1 max=10000 step=1 default=166 value=          205
#         exposure_auto_priority (bool)   : default=0 value=                                     1 



"""Finds the radius and angle as s

:param a: side length to the left of the robot
:type a: float
:param b: side length to the right of the robot
:type b: float
:param c: the constant distance between the vision targets
:type c: float
:returns: a tuple of length 2, with the radius first and the angle second
:rtype: both floats
"""
def radiusAndAngle(a, b, c):
    spin = -1;
    #flip values and spin
    if(b < a):
        a,b = b,a
        spin = 1
    # Use the law of cosines to find cos(A)
    angle = ( ( (a * a) - (b * b) - (c * c) )
                        / (-2 * b * c) )
    # Halve the length of the constant line and use the law of cosines to find the radius
    a = math.sqrt( (b * b)
                              + ((c / 2) * (c / 2))
                              - (2 * b * (c / 2) * angle ) )
    # Find the angle instead of acos(A)
    angle = math.acos(math.radians(angle))
    # Get the angle on the opposite using law of sines
    angle = math.degrees(( math.asin( ( b *  math.sin(math.radians(angle)) ) / a ) ) * spin)
    # Get the side we actually want
    angle = ( 90 - ( ( ( angle * 180 ) / math.pi ) * spin ) ) * spin * math.pi
    return (a, angle)


def distanceToContour(contour):
	x,y,w,h = cv2.boundingRect(contour)
	hnumerator = ((23/4) / 12) * 240
	hdenominator = 2 * h * math.tan(math.radians(29.6))
	#wnumerator = 0.5 * 240
	#wdenominator = 2 * w * math.tan(math.radians(50.6))
	dis = (((hnumerator / hdenominator) * 12)) * 2# + ((wnumerator / wdenominator) * 12))
	return dis


pipeline = grip.GripPipeline()

dbgprint("pipeline created")

source = cv2.VideoCapture(0)

dbgprint("Camera setup, starting main loop")


def calculateAverageDistance(importantDistance):
	if len(distanceaveragelist) == 5:
		distanceaveragelist.reverse()
		distanceaveragelist.pop()
		distanceaveragelist.reverse()
	
	distanceaveragelist.append(importantDistance)
	return sum(distanceaveragelist)

def calculateAverageAngle(importantAngle):
	if len(angleaveragelist) == 5:
		angleaveragelist.reverse()
		angleaveragelist.pop()
		angleaveragelist.reverse()
	
	angleaveragelist.append(importantAngle)
	return sum(angleaveragelist)


def targetVision():
	have_frame, frame = source.read()
	if have_frame:
		pipeline.process(frame)
		contours = pipeline.filter_contours_output
		distances = []
		
		subtable.putNumber("# of contours", len(contours))
		
		if len(contours) != 2:
			return
		
		leftcontour = None
		rightcontour = None
		x0,y0,w0,h0 = cv2.boundingRect(contours[0])
		x1,y1,w1,h1 = cv2.boundingRect(contours[1])
		if x0 > x1:
			leftcontour = contours[1]
			rightcontour = contours[0]
		else:
			leftcontour = contours[0]
			rightcontour = contours[1]
		
		leftdistance = distanceToContour(leftcontour)
		rightdistance = distanceToContour(rightcontour)
		
		importantDistance, importantAngle = radiusAndAngle(leftdistance, rightdistance, targetdistance)
		averageDistance = calculateAverageDistance(importantDistance)
		averageAngle = calculateAverageAngle(importantAngle)
		dbgprint(importantDistance, importantAngle, averageDistance, averageAngle)
		
		subtable.putNumber("rawDistance", importantDistance)
		subtable.putNumber("rawAngle", importantAngle)
		subtable.putNumber("distance", averageDistance)
		subtable.putNumber("angle", averageAngle)

def lineVision():
	feature = line_get_main_features()
	if (feature is not None):
		subtable.putString("error", "Error " + str(datetime.datetime.now()) + ": Cannot find any lines!")
		return
	
	v_count = line_get_vectors(100, vectors)
	dbgprint(v_count)
	subtable.putNumber("# of lines", v_count)
	x0 = vectors[0].m_x0
	y0 = vectors[0].m_y0
	x1 = vectors[0].m_x1
	y1 = vectors[0].m_y1
	
	if (y1 < y0): # 0 should be on top
		tx = x0
		ty = y0
		x0 = x1
		y0 = y1
		x1 = tx
		y1 = ty
	
	
	instruction = "None"
	
	if (iscentered(x0) and iscentered(x1)):
		instruction = "MF"
	elif (isleft(x0) and isleft(x1)):
		instruction = "ML"
	elif (isright(x0) and isright(x1)):
		instruction = "MR"
	
	elif (isright(x0) and (iscentered(x1) or isleft(x1))):
		instruction = "TR"
	elif (isleft(x0) and (iscentered(x1) or isright(x1))):
		instruction = "TL"
	elif (isright(x1) and (iscentered(x0) or isleft(x0))):
		instruction = "TL"
	elif (isleft(x1) and (iscentered(x0) or isright(x0))):
		instruction = "TR"
	
	dbgprint(x0, y0, x1, y1, instruction)
	subtable.putString("instruction", instruction)


while(True):
	visionMode = subtable.getString("mode", "None")
	if visionMode == "None":
		time.sleep(0.25)
		continue
	elif visionMode == "Far":
		targetVision()
	elif visionMode == "Near":
		lineVision()

source.release()