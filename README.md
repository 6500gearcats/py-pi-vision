# 6500's python 3 code to do vision stuff on a Raspberry Pi

The pipeline.grip exports to grip.py

grip.py is then imported by vision.py, vision.py is where all of our actual code is written



To run this without segmentation faults, first install opencv for python, then plug in the Logitech C270 webcam and then the PixyCam 2.  The camera should have a ring of leds around it set to rgb (20, 150, 0).
Then run `sudo python3 vision.py`



Note 1: this has only been tested on a Raspberry Pi 3B, no 3B+ or 2B/2B+ so use it on those at your own risk.

Note 2: pixy libraries are included here because they are super fickle to get to compile for python 3 on the pi.

Note 3: make sure to properly configure your networktables server ip in vision.py + set debug to False.