
---

## A pure python implementation of rigid body dynamics. ##

---


### Primary Aims ###
  * Rigid body class with inital support for box shapes.
  * Simple euler integration for updating state of bodies.
  * Collision detection and resolution.
  * Simple simulation interface.

### Secondary Aims ###

Include better integration (RK4), better collision and response methods, more shapes (including triangle mesh), better performance.

### Other Aims ###

To help me, and others like me, understand how physics engines actually work!
It's all new to me, and something like this would have helped me when I started out investigating the subject.

### Dependencies ###

Uses http://www.pyglet.org/index.html pyglet for rendering, but can easily adapted / extended to any library.

Heavily dependant on http://code.google.com/p/pyeuclid for all vector/quaternion maths.