#
# primitives.py
# Bart Trzynadlowski, 2024
#
# Primitive shapes.
#

import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *

def draw_sphere(radius: float, longitudinal_slices: int = 32, latitudinal_slices: int = 32):
    quadric = gluNewQuadric()
    gluQuadricNormals(quadric, GLU_SMOOTH)  # Enable smooth shading
    gluSphere(quadric, radius, longitudinal_slices, latitudinal_slices)
    gluDeleteQuadric(quadric)

def draw_box(size: np.ndarray):
    xx = size[0] * 0.5
    yy = size[1] * 0.5
    zz = size[2] * 0.5

    vertices = np.array([
        # Front-top-left (V0)
        [-xx,  yy,  zz],
        # Front-top-right (V1)
        [ xx,  yy,  zz],
        # Front-bottom-right (V2)
        [ xx, -yy,  zz],
        # Front-bottom-left (V3)
        [-xx, -yy,  zz],
        # Back-top-left (V4)
        [-xx,  yy, -zz],
        # Back-top-right (V5)
        [ xx,  yy, -zz],
        # Back-bottom-right (V6)
        [ xx, -yy, -zz],
        # Back-bottom-left (V7)
        [-xx, -yy, -zz]
    ])

    vertex_normals = np.array([
        # Vertex 0
        [-1,  1,  1],  # Front-left-top
        # Vertex 1
        [ 1,  1,  1],  # Front-right-top
        # Vertex 2
        [ 1, -1,  1],  # Front-right-bottom
        # Vertex 3
        [-1, -1,  1],  # Front-left-bottom
        # Vertex 4
        [-1,  1, -1],  # Back-left-top
        # Vertex 5
        [ 1,  1, -1],  # Back-right-top
        # Vertex 6
        [ 1, -1, -1],  # Back-right-bottom
        # Vertex 7
        [-1, -1, -1]   # Back-left-bottom
    ])

    vertex_normals = vertex_normals / np.linalg.norm(vertex_normals, axis=1)[:, np.newaxis]

    indices = [
        (0, 1, 2, 3),  # Front face
        (4, 5, 6, 7),  # Back face
        (0, 3, 7, 4),  # Left face
        (1, 2, 6, 5),  # Right face
        (0, 1, 5, 4),  # Top face
        (3, 2, 6, 7)   # Bottom face
    ]

    glBegin(GL_QUADS)
    for face in indices:
        for vertex in face:
            glNormal3fv(vertex_normals[vertex])
            glVertex3fv(vertices[vertex])
    glEnd()

def draw_cylinder(radius: float, height: float, slices: int = 32):
    glPushMatrix()
    glTranslate(0, 0.5 * height, 0) # pivot point: center
    glRotate(90, 1, 0, 0)           # upright cylinder


    quadric = gluNewQuadric()
    gluQuadricNormals(quadric, GLU_SMOOTH)
    gluCylinder(quadric, radius, radius, height, slices, 1)

    # Draw the top cap
    glPushMatrix()
    glTranslatef(0.0, 0.0, height)
    draw_circle(radius, slices)
    glPopMatrix()

    # Draw the bottom cap
    glPushMatrix()
    glRotatef(180, 1.0, 0.0, 0.0)
    draw_circle(radius, slices)
    glPopMatrix()

    gluDeleteQuadric(quadric)

    glPopMatrix()

# Function to draw a circle (used for cylinder caps)
def draw_circle(radius: float, slices: int = 32):
    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(0.0, 0.0, 1.0)
    glVertex3f(0.0, 0.0, 0.0)
    for i in range(slices + 1):
        angle = 2.0 * np.pi * i / slices
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        glVertex3f(x, y, 0.0)
    glEnd()

def draw_axes_gizmo(length=1.0, arrow_head_length=0.2, arrow_head_radius=0.05):
    def draw_arrow(direction):
        glPushMatrix()
        if direction[0] != 0:
            glRotatef(90, 0, 1, 0) if direction[0] > 0 else glRotatef(-90, 0, 1, 0)
        elif direction[1] != 0:
            glRotatef(-90, 1, 0, 0) if direction[1] > 0 else glRotatef(90, 1, 0, 0)
        elif direction[2] < 0:
            glRotatef(180, 0, 1, 0)

        # Draw cylinder (shaft)
        quad = gluNewQuadric()
        gluCylinder(quad, arrow_head_radius/2, arrow_head_radius/2, length - arrow_head_length, 32, 1)

        # Draw cone (arrowhead)
        glTranslatef(0, 0, length - arrow_head_length)
        gluCylinder(quad, arrow_head_radius, 0, arrow_head_length, 32, 1)

        gluDeleteQuadric(quad)
        glPopMatrix()

    #glPushAttrib(GL_CURRENT_BIT | GL_LIGHTING_BIT)
    #glEnable(GL_LIGHTING)
    #glEnable(GL_LIGHT0)

    # X-axis (red)
    glColor3f(1.0, 0.0, 0.0)
    #glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, [1.0, 0.0, 0.0, 1.0])
    draw_arrow([1, 0, 0])

    # Y-axis (green)
    glColor3f(0.0, 1.0, 0.0)
    #glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, [0.0, 1.0, 0.0, 1.0])
    draw_arrow([0, 1, 0])

    # Z-axis (blue)
    glColor3f(0.0, 0.0, 1.0)
    #glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, [0.0, 0.0, 1.0, 1.0])
    draw_arrow([0, 0, 1])

    #glPopAttrib()