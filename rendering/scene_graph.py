#
# scene_graph.py
# Bart Trzynadlowski, 2024
#
# Scene graph classes for rendering. The scene graph is an n-way tree.
#

from abc import abstractmethod
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *

from .primitives import draw_box, draw_cylinder, draw_sphere, draw_axes_gizmo

def x_rotation(degrees: float) -> np.ndarray:
    """The 3x3 rotation matrix for a rotation of `theta` radians about the x-axis."""
    theta = np.deg2rad(degrees)
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])


def y_rotation(degrees: float) -> np.ndarray:
    """The 3x3 rotation matrix for a rotation of `theta` radians about the y-axis."""
    theta = np.deg2rad(degrees)
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])


def z_rotation(degrees: float) -> np.ndarray:
    """The 3x3 rotation matrix for a rotation of `theta` radians about the z-axis."""
    theta = np.deg2rad(degrees)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

def euler_rotation_matrix(euler_degrees: np.ndarray) -> np.ndarray:
    """The 3x3 rotation matrix in order of roll (about x, as in URDF convention), pitch (y), yaw (z, which is up in robot coordinate system) """
    x_rot = x_rotation(euler_degrees[0])
    y_rot = y_rotation(euler_degrees[1])
    z_rot = z_rotation(euler_degrees[2])
    return np.matmul(z_rot, np.matmul(y_rot, x_rot))

class SceneGraphNode:
    def __init__(self, position: np.ndarray = [ 0, 0, 0 ], euler_degrees: np.ndarray = [ 0, 0, 0 ], **kwargs):
        self._matrix = np.eye(4)
        self._children = []

        if "matrix" in kwargs:
            self._matrix = kwargs["matrix"]
        else:
            # Compute matrix from position and Euler angles
            self._matrix[0:3, 0:3] = euler_rotation_matrix(euler_degrees=euler_degrees)
            self._matrix[0:3, 3] = position

    @property
    def position(self) -> np.ndarray:
        return self._matrix[0:3, 3]

    def add_children(self, *children):
        for child in children:
            if isinstance(child, SceneGraphNode):
                self._children.append(child)
            else:
                try:
                    # Detect iterable: https://stackoverflow.com/questions/1952464/python-how-to-determine-if-an-object-is-iterable
                    _ = iter(child)
                    for obj in child:
                        self.add_children(obj)
                except TypeError:
                    raise ValueError("Argument is neighter a SceneGraphNode nor iterable")

    def draw(self):
        return

    def render(self):
        glPushMatrix()
        glMultMatrixf(self._matrix.T)   # transpose to put in OpenGL column-major format

        # Draw
        self.draw()

        # Walk the graph
        for child in self._children:
            child.render()

        glPopMatrix()

class Translation(SceneGraphNode):
    def __init__(self, amount: np.ndarray):
        super().__init__(position=amount, euler_degrees=[0,0,0])

class Rotation(SceneGraphNode):
    def __init__(self, axis: np.ndarray, degrees: float):
        ux, uy, uz = axis / np.linalg.norm(axis)
        theta = np.deg2rad(degrees)
        cost = np.cos(theta)
        sint = np.sin(theta)
        m = 1.0 - cost

        matrix = np.array([
            [ ux*ux*m+cost, ux*uy*m-uz*sint, ux*uz*m+uy*sint, 0 ],
            [ ux*uy*m+uz*sint, uy*uy*m+cost, uy*uz*m-ux*sint, 0 ],
            [ ux*uz*m-uy*sint, uy*uz*m+ux*sint, uz*uz*m+cost, 0 ],
            [0, 0, 0, 1]
        ])

        super().__init__(matrix=matrix)

    def draw(self):
        return

# https://gamedev.stackexchange.com/questions/202398/how-does-unitys-fromtorotation-work-a-major-part-of-its-behavior-seems-undocum
class FromToRotation(Rotation):
    def __init__(self, from_direction: np.ndarray, to_direction: np.ndarray):
        dir_from = from_direction / np.linalg.norm(from_direction)
        dir_to = to_direction / np.linalg.norm(to_direction)
        axis = np.cross(dir_from, dir_to)

        epsilon = 1e-6
        if np.linalg.norm(axis) < epsilon:
            # TODO: math needs to be checked here
            # Special case where from and to are coincident
            right = np.cross([0,1,0], dir_from)
            if np.linalg.norm(right) < epsilon:
                axis = [0,0,1]
            else:
                axis = np.cross(dir_from, right)

        radians = np.acos(np.dot(dir_from, dir_to))

        super().__init__(axis=axis, degrees=np.rad2deg(radians))



class Sphere(SceneGraphNode):
    def __init__(self, radius: float, position: np.ndarray, euler_degrees: np.ndarray = np.array([ 0, 0, 0 ]), color: np.ndarray = np.array([ 1, 1, 1 ])):
        super().__init__(position=position, euler_degrees=euler_degrees)
        self._radius = radius
        self._color = color

    def draw(self):
        glColor3f(self._color[0], self._color[1], self._color[2])
        draw_sphere(radius=self._radius)

class Box(SceneGraphNode):
    def __init__(self, width: float, height: float, depth: float, position: np.ndarray, euler_degrees: np.ndarray = [ 0, 0, 0 ], color: np.ndarray = [ 1, 1, 1 ]):
        super().__init__(position=position, euler_degrees=euler_degrees)
        self._size = np.array([ width, height, depth ])
        self._color = color

    def draw(self):
        glColor3f(self._color[0], self._color[1], self._color[2])
        draw_box(size=self._size)

class Cylinder(SceneGraphNode):
    def __init__(self, radius: float, height: float, position: np.ndarray, euler_degrees: np.ndarray = [ 0, 0, 0 ], color: np.ndarray = [ 1, 1, 1 ]):
        super().__init__(position=position, euler_degrees=euler_degrees)
        self._radius = radius
        self._height = height
        self._color = color

    def draw(self):
        glColor3f(self._color[0], self._color[1], self._color[2])
        draw_cylinder(radius=self._radius, height=self._height)

class AxesGizmo(SceneGraphNode):
    def __init__(self, scale: float):
        super().__init__()
        self._scale = scale

    def draw(self):
        glPushMatrix()
        glScale(self._scale, self._scale, self._scale)
        draw_axes_gizmo()
        glPopMatrix()