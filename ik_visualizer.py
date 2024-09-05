#
# ik_visualizer.py
# Bart Trzynadlowski, 2024
#
# Renders CRANE-X7 joints obtained from IK solutions.
#
# TODO:
# -----
# - Auto-detect robot size and select an appropriate view point.
# - URDF support for arbitrary robots.
#

import timeit
from typing import Tuple

from ikpy.chain import Chain
import numpy as np
import pygame
from pygame.locals import DOUBLEBUF, OPENGL
from OpenGL.GL import *
from OpenGL.GLU import *

from rendering import *
from math_helpers import euler_rotation_matrix4


####################################################################################################
# Graphics
####################################################################################################

camera_position = [ 0, 0, -1.5 ]    # in world coordinate system
camera_target = [ 0, 0, 0 ]         # in world coordinate system
mouse_x, mouse_y = 0, 0
camera_rotation_matrix= euler_rotation_matrix4(euler_degrees=[-90,180,0])

def init_graphics(resolution: Tuple[int, int]):
    # OpenGL
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

    # Viewport
    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (resolution[0] / resolution[1]), 0.1, 50.0)

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

def camera():
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glScalef(1, 1, 1)

    gluLookAt(
        camera_position[0], camera_position[1], camera_position[2],
        camera_target[0], camera_target[1], camera_target[2],
        0, 1, 0 # up vector
    )

    # Orbit around the scene by rotation the scene itself
    glMultMatrixf(camera_rotation_matrix.T)

def rotate_camera(elevation_degrees: float, azimuth_degrees: float):
    global camera_rotation_matrix
    degrees = np.array([ elevation_degrees, azimuth_degrees, 0 ])
    delta_rotation_matrix = euler_rotation_matrix4(euler_degrees=degrees)
    camera_rotation_matrix = delta_rotation_matrix @ camera_rotation_matrix

def draw_scene(scene_graph: SceneGraphNode):
    # Draw the scene
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    scene_graph.render()


####################################################################################################
# CRANE-X7 Rendering
#
# Produces a scene graph of the CRANE-X7 arm with given joint rotations.
####################################################################################################

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

def crane_x7(joint_angles: np.ndarray, ik_target_position: np.ndarray) -> SceneGraphNode:
    root = SceneGraphNode(position=[0,0,0], euler_degrees=[0,0,0])

    # Big gizmo for world
    world_gizmo = AxesGizmo(scale=0.2)
    root.add_children(world_gizmo)

    # IK target
    target = Sphere(radius=0.02, position=ik_target_position, color=[1,0,1])
    root.add_children(target)

    # Ensure axes are correct
    # cubex = Box(width=0.1, height=0.1, depth=0.1, position=[0.3,0,0], color=[1,0,0])
    # cubey = Box(width=0.1, height=0.1, depth=0.1, position=[0,0.3,0], color=[0,1,0])
    # cubez = Box(width=0.1, height=0.1, depth=0.1, position=[0,0,0.3], color=[0,0,1])
    # root.add_children(cubex, cubey, cubez)
    # return root

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

####################################################################################################
# Main Loop
####################################################################################################

def handle_mouse_button(event: pygame.event.Event):
    global mouse_x, mouse_y
    if event.type == pygame.MOUSEBUTTONDOWN:
        mouse_x, mouse_y = pygame.mouse.get_pos()

def handle_mouse_wheel(event: pygame.event.Event):
    global camera_position
    if event.type == pygame.MOUSEWHEEL:
        camera_position[2] = max(0.1, camera_position[2] - event.y * 0.1)

def handle_mouse_motion(event: pygame.event.Event):
    global mouse_x, mouse_y, camera_target, ik_target_position
    if event.type == pygame.MOUSEMOTION:
        dx, dy = event.pos[0] - mouse_x, event.pos[1] - mouse_y
        mouse_x, mouse_y = event.pos
        if event.buttons[2]:    # right mouse button
            rotate_camera(elevation_degrees=dy * 0.1, azimuth_degrees=dx * 0.1)
        elif event.buttons[1]:  # middle mouse button
            camera_target[1] += dy * 0.001
            camera_target[0] += -dx * 0.001
        elif event.buttons[0]:  # left mouse button
            inverse_camera_rotation_matrix = np.linalg.inv(camera_rotation_matrix)
            camera_right = inverse_camera_rotation_matrix[0:3,0]
            camera_up = inverse_camera_rotation_matrix[0:3,1]
            ik_target_position += 0.001 * dx * camera_right - 0.001 * dy * camera_up

ik_target_position = np.array([ 0.1, 0.2, .50 ])

def main():
    # Load Crane X7 (must be consistent with hard-coded definition)
    #TODO: construct scene graph directly from URDF file
    joint_mask = [ True ] * 9
    joint_mask[0] = False   # first is not a real joint (it is a link)
    joint_mask[-1] = False  # last not a real joint
    kinematic_chain = Chain.from_urdf_file(urdf_file="crane_x7_simple.urdf", active_links_mask=joint_mask)
    joint_angles = [ 0 ] * 9

    # Init graphics
    init_graphics(resolution=(800,600))

    # Render loop
    target_frame_rate = 60
    target_frame_time = 1.0 / target_frame_rate
    lights()
    while True:
        frame_start = timeit.default_timer()

        # Process inputs
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            handle_mouse_button(event=event)
            handle_mouse_wheel(event=event)
            handle_mouse_motion(event=event)

        # Perform IK
        joint_angles = kinematic_chain.inverse_kinematics(target_position=ik_target_position, initial_position=joint_angles) #, target_orientation=[1,0,0], orientation_mode="X")
        joint_degrees = [ np.rad2deg(rads) for rads in joint_angles ][1:-1] # get the 7 middle joints

        # Produce scene graph by running forward kinematics
        scene_graph = crane_x7(joint_angles=joint_degrees, ik_target_position=ik_target_position)

        # Draw
        camera()
        draw_scene(scene_graph=scene_graph)

        # Display and wait until next frame
        pygame.display.flip()
        frame_end = timeit.default_timer()
        frame_time_elapsed = frame_end - frame_start
        frame_time_remaining = target_frame_time - frame_time_elapsed
        if frame_time_remaining > 0:
            pygame.time.wait(int(frame_time_remaining * 1000))

if __name__ == "__main__":
    main()
