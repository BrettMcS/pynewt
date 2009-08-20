from euclid import *
from pyglet.gl import *
from copy import copy
from math import sin
import sys

class Sim:
	""" Main simulation class.   Contains universal forces and list of all rigid bodies """
	def __init__ (self, gravity=Vector3(0,0,-10), ground=False):
		self.gravity = gravity
		self.bodies=[]
		if ground:
			self.ground = Rigid_Body(x=100, y=100, z=0.1, dynamic=False, simulation=self)
		else:
			self.ground = None
		
	def update (self, dt):
		""" Progress simulation forward by dt seconds """
		if dt>0:
			for body in self.bodies:
				if body.dynamic:
					body.compute_force_and_torque()
					body.update(dt)

			
	def draw(self):
		for body in self.bodies:
			if body.dynamic:
				body.draw()

			
class Box:
	""" Box class - used as mesh for rigid body box collision detection.  
	Also used for display purposes. """
	def __init__ (self, x,y,z):
		# CALCULATE VERTEX COORDINATES
		self.v = []
		self.v.append  (Vector3( x/2.0, -y/2.0, z/2.0 ))
		self.v.append  (Vector3( -x/2.0, -y/2.0, z/2.0 ))
		self.v.append  (Vector3( -x/2.0, -y/2.0, -z/2.0 ))
		self.v.append  (Vector3( x/2.0, -y/2.0, -z/2.0 ))
		self.v.append  (Vector3( x/2.0, y/2.0, z/2.0 ))
		self.v.append  (Vector3( -x/2.0, y/2.0, z/2.0 ))
		self.v.append  (Vector3( -x/2.0, y/2.0, -z/2.0 ))
		self.v.append  (Vector3( x/2.0, y/2.0, -z/2.0 ))
		self.faces=[]
		self.normals=[]
		#f
		self.faces.append ((self.v[0],self.v[1],self.v[2],self.v[3]))
		self.normals.append ( (0,-1,0) )
		#b
		self.faces.append ((self.v[5],self.v[4],self.v[7],self.v[6]))
		self.normals.append ( (0,1,0) )
		#l
		self.faces.append ((self.v[1],self.v[5],self.v[6],self.v[2]))
		self.normals.append ( (-1,0,0) )
		#r
		self.faces.append ((self.v[4],self.v[0],self.v[3],self.v[7]))
		self.normals.append ( (1,0,0) )
		#t
		self.faces.append ((self.v[4],self.v[5],self.v[1],self.v[0]))
		self.normals.append ( (0,0,1) )
		#b
		self.faces.append ((self.v[3],self.v[2],self.v[6],self.v[7]))
		self.normals.append ( (0,0,-1) )
	
	def draw (self):
		glBegin (GL_QUADS)
		fc = 0
		for face in self.faces:
			for v in face:
				glNormal3f (self.normals[fc][0], self.normals[fc][1], self.normals[fc][2])
				glVertex3f (v.x, v.y, v.z )
			fc+=1
		glEnd()



	
def integrate_euler (body, dt):
	""" Integrate values for time span of dt seconds """
	# compute primaries
	# P = F*t   = F = ma
	body.linear_momentum += body.force * dt
	body.angular_momentum += body.torque * dt
	
	# compute auxiliaries
	# P = mv, so v = P/m 	- v(t) linear velocity
	body.velocity = body.linear_momentum * body.inverse_mass
	# I(t) = R(t)IbodyR(t)^T	 - Rotation Quaternion * Body space inertia tensor * Inverse of Rotation Quaternion
	body.inertia = body.inertia_tensor * body.orientation.get_matrix()
	#body.inertia = body.orientation.get_matrix() * body.inertia_tensor * body.orientation.conjugated().get_matrix()
	body.inverse_inertia = body.inertia.inverse()
	# w(t) - angular velocity - I(t)^-1 * L	- Inverse of inertia * Angular Momentum - referred to as omega
	body.angular_velocity = body.inverse_inertia * body.angular_momentum 
	
	# integrate velocity to get position
	body.position += body.velocity * dt
	# integrate angular velocity to get new orientation (Rdot as it is referred to)
	rot_speed = abs(body.angular_velocity)
	if rot_speed<>0:
		body.orientation *= Quaternion.new_rotate_axis (rot_speed * dt, (1/rot_speed) * body.angular_velocity )
		body.orientation.normalize()

class Contact:
	""" Class used to contain information regarding body-body contact / collision """
	def __init__ (self, a, b, p, n, ea, eb, vf):
		self.a = a		# body a
		self.b = b		# body b
		self.p = p		# world space vertex location
		self.n = n		# outwards pointing normal of face
		self.ea = ea	# edge direction for a
		self.eb = eb	# edge direction for b
		self.vf = vf	# true if vertex/face contact.
		
		# DEBUG
		#print "Contact between: ",a,b
		#print "At: ", p
		#print "Normal: ",n
		#print "Vertex/Face contact: ", vf

	
	def get_relative_velocities_of_points (self, v1, v2):
		return self.n.dot (v1 - v2)
		
	def get_velocity_of_vertex (self, body):
		# np(t) = v(t) + w(t) X ( p(t) - x(t))
		if body.dynamic:
			return body.velocity + (body.angular_velocity.cross ( self.p - body.position ))
		else:
			return Vector3(0,0,0)
	
	def resolve(self):
		a_v = self.get_velocity_of_vertex (self.a)
		b_v = self.get_velocity_of_vertex (self.b)
		Vrel = self.get_relative_velocities_of_points(a_v, b_v)
		epsilon = 0.0001
		#DEBUG
		#print "a_v: ",a_v," b_v: ",b_v, " VREL: ",Vrel
		
		if Vrel>epsilon:
			# MOVING APART
			return
		if Vrel>-epsilon:
			print "AT REST!"
			# RESTING CONTACT - add epsilon
			return
			
		ra = self.p - self.a.position
		rb = self.p = self.b.position
		
		numerator = -(1.0 + epsilon) * Vrel
		
		# calc denominator in 4 parts
		term1 = self.a.inverse_mass
		term2 = self.b.inverse_mass
		term3 = self.n.dot ((self.a.inverse_inertia * (ra.cross(self.n))).cross(ra))
		term4 = self.n.dot ((self.b.inverse_inertia * (rb.cross(self.n))).cross(rb))
		
		# compute impulse magnitude
		j = numerator / (term1 + term2 + term3 + term4)
		force = j * self.n
		
		# DEBUG
		#print "Force:",force
		#sys.exit()
		
		# apply impulse to the bodies
		if self.a.dynamic:
			self.a.linear_momentum += force * self.a.bounce
			self.a.angular_momentum += ra.cross(force * self.a.bounce)
		if self.b.dynamic:
			self.b.linear_momentum -= force
			self.b.angular_momentum -= rb.cross(force)
		
		# compute auxiliaries
		if self.a.dynamic:
			self.a.velocity = self.a.linear_momentum * self.a.inverse_mass
			self.a.angular_velocity = self.a.inverse_inertia * self.a.angular_momentum 
		if self.b.dynamic:
			self.b.velocity = self.b.linear_momentum * self.b.inverse_mass
			self.b.angular_velocity = self.b.inverse_inertia * self.b.angular_momentum 

			
class Rigid_Body:
	def __init__ (self, x=2, y=2, z=2, bounce=0.4, dynamic=True, simulation=None):
		# MESH
		self.mesh = Box(x,y,z)
		self.dynamic = dynamic
		# CONSTANTS
		if dynamic:
			self.mass = x*y*z*40 							# mass M
			self.inverse_mass = 1.0 / self.mass				# inv mass 1/M
		else:
			self.mass = 99999999.
			self.inverse_mass = 0.
		self.cog = Vector3(0,0,0)					# centre of gravity - not *really* a constant, but is for now :)									
		self.inertia_tensor = Matrix4()						# inertia 
		self.inertia_tensor.a = (1.0/12.0)*self.mass*(y*y+z*z)
		self.inertia_tensor.f = (1.0/12.0)*self.mass*(x*x+z*z)
		self.inertia_tensor.k = (1.0/12.0)*self.mass*(x*x+y*y)	
		self.inverse_inertia_tensor = self.inertia_tensor.inverse()		# inverse inertia
		self.bounce = bounce						# coefficent of restitution
		
		#  PRIMARY (only ones required for integration later)
		self.position = Vector3(0,0,0)			# m
		self.linear_momentum = Vector3(0,0,0)			# kg/m/s
		self.orientation = Quaternion() 		# unit
		self.angular_momentum = Vector3(0,0,0)
		
		# SECONDARY
		self.velocity = Vector3(0,0,0)			# m/s (derived from momentum)
		self.spin = Quaternion()				# rate of change in orientation 
		self.angular_velocity = Vector3(0,0,0)	# derived from angular_momentum
		self.body_to_world = Matrix4()			# matrix
		self.world_to_body = Matrix4()			# matrix
		self.inertia = Matrix4()				
		self.inverse_inertia = Matrix4()
		
		self.force = Vector3(0,0,0)
		self.torque = Vector3(0,0,0)
		
		#self.recalculate()
		self.previous = copy(self)
		self.sim = simulation
		self.sim.bodies.append (self)
		
	def recalculate (self):
		pass
	
	def compute_force_and_torque(self):
		self.force = self.mass * self.sim.gravity	# F = ma
		self.ground_collision()
		
	def ground_collision(self):
		for v in self.mesh.v:
			nv = self.orientation.get_matrix() * v
			nv += self.position
			d = 0 + nv[2]
			if d<=0:
				self.position[2]+= -d
				nv[2]=0.	# move contact point to ground
				# a, b, p, n, ea, eb, vf
				contact = Contact (self, self.sim.ground, nv, Vector3(0,0,1.), None, None, True)
				contact.resolve()
				return

	def update (self, dt):
		self.previous = copy(self)
		integrate_euler (self, dt)
		
	def draw (self):
		glPushMatrix()
		glTranslatef (self.position[0], self.position[1], self.position[2])
		#get angle from orientation quaternion
		angle, axis = self.orientation.get_angle_axis()
		angle= angle * 57.2957795	# convert angle to degrees for opengl
		glRotatef (angle, axis.x, axis.y, axis.z)
		self.mesh.draw()
		glPopMatrix()
		
	
		
		
		
		
		
