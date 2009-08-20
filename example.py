import math
import operator
import pyglet
import sys
pyglet.options['debug_gl'] = False	# TURN ON FOR DEBUGGIN GL!!!
from random import random
from pyglet.window import key
from pyglet.window import mouse
from pyglet import resource
from pyglet.gl import *
from pyglet import clock
import copy

from euclid import *
from rigid_body import *

# Set Up window
window = pyglet.window.Window(800, 600,"Pynewt Test",True)
keys = key.KeyStateHandler()
window.push_handlers(keys)
window.set_vsync (False)
window.set_fullscreen (False)
window.clear()

# Define a simple function to create ctypes arrays of floats:
def vec(*args):
	return (GLfloat * len(args))(*args) 
	
class Light:
	def __init__ (self, pos=[0,20,20]):
		self.pos=pos
		glEnable(GL_LIGHT0)
		glLightfv(GL_LIGHT0, GL_POSITION, vec(pos[0], pos[1], pos[2], 1))
		glLightfv(GL_LIGHT0, GL_DIFFUSE, vec(1, 1, 1, 1))
		glLightfv(GL_LIGHT0, GL_AMBIENT, vec(0.2, 0.2, 0.2, 0.2))
		glMaterialfv(GL_FRONT, GL_DIFFUSE, vec(0.75, 0.75, 0.75, 1))
		glMaterialfv(GL_FRONT, GL_AMBIENT, vec(0.1, 0.1, 0.1, 1))
		glMaterialfv(GL_FRONT, GL_SPECULAR, vec(0.3, 0.3, 0.3, 1))
		glMaterialfv(GL_FRONT, GL_SHININESS, GLfloat(100.))
	def newpos (self, pos):
		self.pos = pos
		glLightfv(GL_LIGHT0, GL_POSITION, vec ( pos[0], pos[1], pos[2], 1) )
		
class Camera:
	def __init__ (self, pos=[0,20,20], target=[0,0,0], up=[0,0,1] ):
		self.pos = pos
		self.target = target
		self.up = up

@window.event
def on_draw():
	window.clear()
	
	glMatrixMode (GL_MODELVIEW)
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
	glAlphaFunc ( GL_GREATER, 0.1 ) 
	glLoadIdentity()
	
	# do camera view
	gluLookAt (camera.pos[0], camera.pos[1], camera.pos[2], camera.target[0], camera.target[1], camera.target[2], camera.up[0], camera.up[1], camera.up[2])
	light0.newpos ( light0.pos )
	
	# PyNewt rendering
	sim.draw()
	
@window.event
def on_resize(width, height):
    #Override the default on_resize handler to create a 3D projection
	glViewport(0, 0, width, height)
	glMatrixMode(GL_PROJECTION)
	glEnable (GL_LINE_SMOOTH)
	glEnable (GL_BLEND)
	glEnable (GL_DEPTH_TEST)
	glHint (GL_LINE_SMOOTH_HINT, GL_NICEST)
	glLineWidth(1)
	glLoadIdentity()
	gluPerspective(60., width / float(height), .1, 1000.)
	glMatrixMode(GL_MODELVIEW)
	
	glEnable(GL_LIGHTING) 
	
	return pyglet.event.EVENT_HANDLED
	

def update (dt):
	if keys[key.SPACE]:
		pos = yrot ( light0.pos, dt )
		light0.newpos ( pos )
		
	if keys[key.D]:
		box1.position[0]-=dt*5
	if keys[key.A]:
		box1.position[0]+=dt*5
	if keys[key.Q]:
		# angular momentum is in object space - create a temp angular momentum from object and rotate it to world space using conjugate of object orientation unit quaternion
		ang_mom_temp = box1.orientation.conjugated().get_matrix() * box1.angular_momentum
		# add world space angular momentum 
		ang_mom_temp[2]+=dt*1000
		# replace new angular momentum into object, rotated back into object space
		box1.angular_momentum = box1.orientation.get_matrix() * ang_mom_temp
	if keys[key.E]:
		# angular momentum is in object space - create a temp angular momentum from object and rotate it to world space using conjugate of object orientation unit quaternion
		ang_mom_temp = box1.orientation.conjugated().get_matrix() * box1.angular_momentum
		# add world space angular momentum 
		ang_mom_temp[1]+=dt*1000
		# replace new angular momentum into object, rotated back into object space
		box1.angular_momentum = box1.orientation.get_matrix() * ang_mom_temp

	sim.update(dt)

# Globals
camera = Camera()
light0 = Light()

# PyNewt object creation
sim = Sim(gravity=Vector3(0,0,-10), ground=True)
box1 = Rigid_Body(x=2,y=2,z=0.5,dynamic=True, simulation=sim)
box1.position.z = 10

pyglet.clock.schedule(update)
#pyglet.clock.schedule_interval(update,1/60.)	
pyglet.app.run()
