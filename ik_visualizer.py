#
# ik_visualizer.py
# Bart Trzynadlowski, 2024
#
# Renders CRANE-X7 joints obtained from IK solutions.
#
# TODO:
# -----
# - Orbit, zoom, pan functionality.
# - Auto-detect robot size and select an appropriate view point.
# - Allow IK target to be moved in real-time.
# - URDF support for arbitrary robots.
#

from typing import Tuple

import numpy as np
import pygame
from pygame.locals import DOUBLEBUF, OPENGL
from OpenGL.GL import *
from OpenGL.GLU import *

from rendering import *


def lights():
    glEnable(GL_NORMALIZE)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)

    ambient_light = [0.25, 0.25, 0.25, 1.0]
    diffuse_light = [0.2, 0.2, 0.2, 1.0]
    specular_light = [1.0, 1.0, 1.0, 1.0]
    light_position = [1, 1, 0, 0]
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_light)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_light)
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular_light)
    glLightfv(GL_LIGHT0, GL_POSITION, light_position)

    #global_ambient = [0, 0, 0, 1.0]
    #glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient)

    # See: https://www.sjbaker.org/steve/omniv/opengl_lighting.html
    glEnable(GL_COLOR_MATERIAL) # the effect glColor will have on material properties
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE)

    glMaterialfv(GL_FRONT, GL_SPECULAR, specular_light)
    glMateriali(GL_FRONT, GL_SHININESS, 25)

def init_opengl(resolution: Tuple[int, int]):
    pygame.init()
    pygame.display.set_mode(resolution, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("IK Visualizer")
    glEnable(GL_DEPTH_TEST)
    glShadeModel(GL_SMOOTH)  # Enable smooth shading
    glClearColor(0.0, 0.0, 0.0, 1.0)  # Set background color to black
    glClearDepth(1.0)  # Set clear depth
    glDepthFunc(GL_LEQUAL)  # Set depth function
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)  # Nice perspective corrections
    glFrontFace(GL_CW)

def init_viewport(resolution: Tuple[int, int]):
    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (resolution[0] / resolution[1]), 0.1, 50.0)


camera_distance = 5.0
camera_azimuth = 0.0
camera_elevation = 0.0
camera_target = [ 0, 0, 0 ]
mouse_x, mouse_y = 0, 0

def camera():
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glScalef(1, 1, 1)

    x = camera_distance * np.cos(camera_elevation) * np.sin(camera_azimuth)
    y = camera_distance * np.sin(camera_elevation)
    z = camera_distance * np.cos(camera_elevation) * np.cos(camera_azimuth)

    camera_pos = [x + camera_target[0], y + camera_target[1], z + camera_target[2]]
    look_at = camera_target
    up = [0, 1, 0]

    gluLookAt(
        camera_pos[0], camera_pos[1], camera_pos[2],
        look_at[0], look_at[1], look_at[2],
        up[0], up[1], up[2]
    )

    # camera_pos = [ 0, 1, 0 ] # off to the side of robot, and above a bit
    # look_at = [ 0, 0, 0 ]
    # up = [ 0, 0, 1 ]            # z is up in robot frame
    # gluLookAt(
    #     camera_pos[0], camera_pos[1], camera_pos[2],
    #     look_at[0], look_at[1], look_at[2],
    #     up[0], up[1], up[2]
    # )

def action(scene_graph: SceneGraphNode):
    # Draw the scene
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    scene_graph.render()

def joint(position: np.ndarray, rpy: np.ndarray = [0,0,0], rotation_axis: np.ndarray = [0,0,1], rotation_degrees: float = 0) -> Tuple[SceneGraphNode, SceneGraphNode]:
    orange = [1,0.5,0]

    joint = SceneGraphNode(position=position, euler_degrees=rpy)
    joint_rotation = Rotation(axis=rotation_axis, degrees=rotation_degrees)
    joint.add_children(joint_rotation)

    gizmo = AxesGizmo(scale=0.04)
    rotate = FromToRotation(from_direction=[0,1,0], to_direction=rotation_axis) # rotate cylinder (default y is up) so that axis is angle of rotation
    gizmo.add_children(rotate)
    node = Cylinder(radius=0.02, height=0.06, position=[0,0,0], color=orange)
    rotate.add_children(node)
    joint_rotation.add_children(gizmo)

    return joint, joint_rotation

def crane_x7(joint_angles: np.ndarray) -> SceneGraphNode:
    root = SceneGraphNode(position=[0,0,0], euler_degrees=[0,0,0])

    # Big gizmo for world
    world_gizmo = AxesGizmo(scale=0.2)
    root.add_children(world_gizmo)

    # Create all joints and link them together.
    # joint1_root is the root node of the joint sub-graph, which will apply the joint translation
    # and rotation (rpy). But because the joint is then rotated (as part of forward kinematics)
    # with another rotation node, we have to attach to a descendant node, joint1_end.
    joint1_root, joint1_end = joint(position=[0,0,0.041], rotation_axis=[0,0,1], rotation_degrees=joint_angles[0])
    root.add_children(joint1_root)

    joint2_root, joint2_end = joint(position=[0,0,0.064], rotation_axis=[0,-1,0], rotation_degrees=joint_angles[1])
    joint1_end.add_children(joint2_root)

    joint3_root, joint3_end = joint(position=[0,0,0.065], rotation_axis=[0,0,1], rotation_degrees=joint_angles[2])
    joint2_end.add_children(joint3_root)

    joint4_root, joint4_end = joint(position=[0,0,0.185], rotation_axis=[0,-1,0], rotation_degrees=joint_angles[3])
    joint3_end.add_children(joint4_root)

    joint5_root, joint5_end = joint(position=[0,0,0.121], rotation_axis=[0,0,1], rotation_degrees=joint_angles[4])
    joint4_end.add_children(joint5_root)

    joint6_root, joint6_end = joint(position=[0,0,0.129], rotation_axis=[0,-1,0], rotation_degrees=joint_angles[5])
    joint5_end.add_children(joint6_root)

    joint7_root, joint7_end = joint(position=[0,0,0.019], rotation_axis=[0,0,1], rotation_degrees=joint_angles[6])
    joint6_end.add_children(joint7_root)

    # Create linkages between joints
    joint_roots = [ joint1_root, joint2_root, joint3_root, joint4_root, joint5_root, joint6_root, joint7_root ]
    joint_ends = [ joint1_end, joint2_end, joint3_end, joint4_end, joint5_end, joint6_end, joint7_end ]
    for i in range(len(joint_roots)):
        current = joint_roots[i]
        linkage_translation = current.position - [0,0,0]    # remember, we are in local space, so previous position is origin
        linkage_direction = linkage_translation
        linkage_length = np.linalg.norm(linkage_translation)

        # Cylinders are rendered such that +y is up. We want to render linkage as cylinder along
        # linkage_direction, so we rotate it first. Then we translate by half of linkage_translation
        # from previous joint in order to position it midway.
        translate = Translation(amount=0.5 * linkage_translation)
        rotate = FromToRotation(from_direction=[0,1,0], to_direction=linkage_direction)
        linkage = Cylinder(radius=0.01, height=linkage_length, position=[0,0,0])
        translate.add_children(rotate)
        rotate.add_children(linkage)

        # Add linkage as child of previous joint
        if i > 0:
            joint_ends[i-1].add_children(translate)
        else:
            root.add_children(translate)

    return root

def handle_mouse_button(event: pygame.event.Event):
    global mouse_x, mouse_y
    if event.type == pygame.MOUSEBUTTONDOWN:
        mouse_x, mouse_y = pygame.mouse.get_pos()

def handle_mouse_wheel(event: pygame.event.Event):
    global camera_distance
    if event.type == pygame.MOUSEWHEEL:
        camera_distance = max(0.1, camera_distance - event.y * 0.1)

def handle_mouse_motion(event: pygame.event.Event):
    global mouse_x, mouse_y, camera_azimuth, camera_elevation, camera_target
    if event.type == pygame.MOUSEMOTION:
        dx, dy = event.pos[0] - mouse_x, event.pos[1] - mouse_y
        mouse_x, mouse_y = event.pos

        if event.buttons[2]:  # Right mouse button
            camera_azimuth += dx * 0.01
            camera_elevation += dy * 0.01
            camera_elevation = max(min(camera_elevation, np.pi / 2), -np.pi / 2)
        elif event.buttons[1]:  # Middle mouse button
            forward = [
                np.cos(camera_elevation) * np.sin(camera_azimuth),
                np.sin(camera_elevation),
                np.cos(camera_elevation) * np.cos(camera_azimuth)
            ]
            right = [
                np.cos(camera_azimuth),
                0,
                -np.sin(camera_azimuth)
            ]
            up = [
                np.sin(camera_elevation) * np.sin(camera_azimuth),
                np.cos(camera_elevation),
                np.sin(camera_elevation) * np.cos(camera_azimuth)
            ]

            for i in range(3):
                camera_target[i] += (-dx * right[i] - dy * up[i]) * 0.01

def main():
    resolution = (800, 600)
    init_opengl(resolution=resolution)
    init_viewport(resolution=resolution)

    joint_angles=[np.float64(3.1664190366910066e-14), np.float64(-42.62156391594923), np.float64(6.613471424999982), np.float64(-51.56352162191282), np.float64(38.91598287604014), np.float64(-26.996998632826443), np.float64(101.9329623596963), np.float64(-20.26359190544769), np.float64(0.0)]
    scene_graph = crane_x7(joint_angles=joint_angles)

    lights()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            handle_mouse_button(event=event)
            handle_mouse_wheel(event=event)
            handle_mouse_motion(event=event)
        camera()
        action(scene_graph=scene_graph)
        pygame.display.flip()
        pygame.time.wait(int(1000 * 1.0 / 60.0))

if __name__ == "__main__":
    main()
