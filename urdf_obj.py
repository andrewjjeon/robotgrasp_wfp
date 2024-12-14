import pybullet as p
import trimesh
import numpy as np
import os
import math

# Link Indices and Names:
# Index: 0, Link Name: panda_link1
# Index: 1, Link Name: panda_link2
# Index: 2, Link Name: panda_link3
# Index: 3, Link Name: panda_link4
# Index: 4, Link Name: panda_link5
# Index: 5, Link Name: panda_link6
# Index: 6, Link Name: panda_link7
# Index: 7, Link Name: panda_link8
# Index: 8, Link Name: panda_hand
# Index: 9, Link Name: panda_leftfinger
# Index: 10, Link Name: panda_rightfinger
# Index: 11, Link Name: panda_grasptarget

# joint_configuration = {
#     "panda_joint1": -0.08258942509847775,
#     "panda_joint2": -0.9423399551273665,
#     "panda_joint3": 0.019490906304476946,
#     "panda_joint4": -2.7832563950388054,
#     "panda_joint5": 0.03141476552378789,
#     "panda_joint6": 1.92817867580201,
#     "panda_joint7": 0.8466345183220174,
#     "panda_finger_joint1": 0.040408313274383545,
#     "panda_finger_joint2": 0.040408313274383545
# }




# initalize pybullet, no GUI
p.connect(p.DIRECT)


urdf_path = "/home/andrewjjeon/FoundationPose/demo_data/panda.urdf"
visual_directory = "/home/andrewjjeon/FoundationPose/demo_data/meshes/visual/"
collision_directory = "/home/andrewjjeon/FoundationPose/demo_data/meshes/collision/"
# robotid is 0 for 1 robot, useFixedBase prevent robot from moving
robot_id = p.loadURDF(urdf_path, useFixedBase=True)

# for joint_index in range(p.getNumJoints(robot_id)): # p.getNumJoints is tuple 12
#     # p.getJointInfo returns a 16 element tuple containing various information, index 1 is joint name 
#     joint_name = p.getJointInfo(robot_id, joint_index)[1].decode("utf-8")
#     if joint_name in joint_configuration:
#         joint_position = joint_configuration[joint_name] # set joint position if it matches our joint_configuration dictionary above
#         p.resetJointState(robot_id, joint_index, joint_position)

meshes = []

for shape in p.getVisualShapeData(robot_id): # p.getVisualShapeData is tuple 11
    print(shape)
    link_index = shape[1]
    mesh_file = shape[4].decode("utf-8") # index 4 points to the .obj mesh

    # fixing the paths of .obj meshes to an actual file system path
    if mesh_file.startswith("package://meshes/visual/"):
        mesh_path = mesh_file.replace("package://meshes/visual/", visual_directory)
    elif mesh_file.startswith("package://meshes/collision/"):
        mesh_path = mesh_file.replace("package://meshes/collision/", collision_directory)
    else:
        mesh_path = mesh_file

    # Load the mesh if it exists
    if os.path.exists(mesh_path):
        # load the .obj path as a trimesh object
        loaded_mesh = trimesh.load(mesh_path)

        # if the loaded mesh is a scene, loop through the individual meshes in the scene then process
        if isinstance(loaded_mesh, trimesh.Scene):
            for name, mesh in loaded_mesh.geometry.items():
                
                # handle left finger separately
                if link_index == 9: # left finger
                    print("got left finger")

                    link_state = p.getLinkState(robot_id, link_index)
                    link_translation = np.array(link_state[4]) # x, y, z
                    # 3x3 rotation matrix r11, r12, r13, r21, ...
                    link_rotation = np.array(p.getMatrixFromQuaternion(link_state[5])).reshape(3, 3)

                    # fill in with information
                    # [r11, r12, r13, tx]
                    # [r21, r22, r23, ty]
                    # [r31, r32, r33, tz]
                    # [0  , 0  , 0  ,  1]

                    #________________________________
                    transformation_matrix = np.eye(4)
                    transformation_matrix[:3, :3] = link_rotation
                    transformation_matrix[:3, 3] = link_translation

                    print(f"left transform matrix {transformation_matrix}")

                    #transformation_matrix[:3, 3] += [0, 0, 0.0584] # for hand
                    # transformation_matrix[:3, 3] += [0, 0.01, 0.02]
                    #________________________________


                    print(f"transforming {name} mesh")
                    mesh.apply_transform(transformation_matrix)
                    
                    output_dir = "transformed_scene_meshes"
                    output_path = os.path.join(output_dir, f"left{name}_transformed.obj")
                    mesh.export(output_path)
                    meshes.append(mesh)

                # handle right finger separately
                elif link_index == 10: # right finger
                    print("got right finger")

                    link_state = p.getLinkState(robot_id, link_index)
                    link_translation = np.array(link_state[4]) # x, y, z
                    # 3x3 rotation matrix r11, r12, r13, r21, ...
                    link_rotation = np.array(p.getMatrixFromQuaternion(link_state[5])).reshape(3, 3)

                    # fill in with information
                    # [r11, r12, r13, tx]
                    # [r21, r22, r23, ty]
                    # [r31, r32, r33, tz]
                    # [0  , 0  , 0  ,  1]
                    transformation_matrix = np.eye(4)
                    transformation_matrix[:3, :3] = link_rotation
                    transformation_matrix[:3, 3] = link_translation

                    print(f"right transform matrix {transformation_matrix}")

                    #transformation_matrix[:3, 3] += [0, 0, 0.0584] # for hand
                    # transformation_matrix[:3, 3] += [0, -0.01, 0.02]

                    # Add 180° rotation about the Z-axis
                    rotation_quaternion = trimesh.transformations.quaternion_from_euler(0, 0, np.pi)
                    rotation_matrix = trimesh.transformations.quaternion_matrix(rotation_quaternion)
                    transformation_matrix = transformation_matrix @ rotation_matrix

                    print(f"transforming {name} mesh")
                    mesh.apply_transform(transformation_matrix)
                    
                    output_dir = "transformed_scene_meshes"
                    output_path = os.path.join(output_dir, f"right{name}_transformed.obj")
                    mesh.export(output_path)
                    meshes.append(mesh)

                # if not fingers
                else:
                    link_state = p.getLinkState(robot_id, link_index)
                    link_translation = np.array(link_state[4]) # x, y, z
                    # 3x3 rotation matrix r11, r12, r13, r21, ...
                    link_rotation = np.array(p.getMatrixFromQuaternion(link_state[5])).reshape(3, 3)

                    # fill in with information
                    # [r11, r12, r13, tx]
                    # [r21, r22, r23, ty]
                    # [r31, r32, r33, tz]
                    # [0  , 0  , 0  ,  1]
                    transformation_matrix = np.eye(4)
                    transformation_matrix[:3, :3] = link_rotation
                    transformation_matrix[:3, 3] = link_translation
                    # transformation_matrix now describes the pose of the link, 
                    # its position in the world (x,y,z) and its orientation (3x3 rotation)

                    print(f"transforming {name} mesh")
                    mesh.apply_transform(transformation_matrix)
                    
                    output_dir = "transformed_scene_meshes"
                    output_path = os.path.join(output_dir, f"{name}_transformed.obj")
                    mesh.export(output_path)
                    meshes.append(mesh)




        # if the loaded mesh is a mesh not a scene, just process mesh directly
        elif isinstance(loaded_mesh, trimesh.Trimesh): 

            mesh = loaded_mesh
            if not mesh.is_empty:
                # handle base separately
                if link_index == -1:  # base link
                    print('got base')
                    transformation_matrix = np.eye(4)
                    
                # if not base link, then fill in transformation matrix, these matrices 
                # can be multiplied with other matrices to compute overall pose of robot
                else:
                    link_state = p.getLinkState(robot_id, link_index)
                    link_translation = np.array(link_state[4])
                    link_rotation = np.array(p.getMatrixFromQuaternion(link_state[5])).reshape(3, 3)

                    # fill in with information
                    # [r11, r12, r13, tx]
                    # [r21, r22, r23, ty]
                    # [r31, r32, r33, tz]
                    # [0  , 0  , 0  ,  1]
                    transformation_matrix = np.eye(4)
                    transformation_matrix[:3, :3] = link_rotation
                    transformation_matrix[:3, 3] = link_translation
                    # transformation_matrix now describes the pose of the link, 
                    # its position in the world(x,y,z) and its orientation (3x3 rotation)

                # now apply transformation matrix to every point in each respective mesh
                mesh.apply_transform(transformation_matrix)

                # Export transformed mesh for inspection
                output_dir = "transformed_meshes"
                os.makedirs(output_dir, exist_ok=True)
                output_path = os.path.join(output_dir, f"{os.path.splitext(os.path.basename(mesh_path))[0]}_transformed.obj")
                mesh.export(output_path)
                meshes.append(mesh)





# # Force add finger meshes if they are not in the scene
# finger_mesh_path = os.path.join(visual_directory, "finger.obj")
# if os.path.exists(finger_mesh_path):
#     finger_mesh = trimesh.load(finger_mesh_path)

#     if isinstance(finger_mesh, trimesh.Scene):
#         print(f"'finger.obj' is a Scene with {len(finger_mesh.geometry)} sub-meshes.")
#         debug_dir = "debug_finger_parts"
#         os.makedirs(debug_dir, exist_ok=True)

#         # Base transform for the `panda_hand`
#         hand_transform = np.eye(4)
#         hand_transform[:3, 3] = [0, 0, 0.0584]  # Position of `panda_hand` from URDF

#         # # Retrieve the joint indices for the prismatic finger joints
#         # joint_index_1 = [
#         #     p.getJointInfo(robot_id, i) for i in range(p.getNumJoints(robot_id))
#         #     if p.getJointInfo(robot_id, i)[1].decode('utf-8') == "panda_finger_joint1"
#         # ][0][0]

#         # joint_index_2 = [
#         #     p.getJointInfo(robot_id, i) for i in range(p.getNumJoints(robot_id))
#         #     if p.getJointInfo(robot_id, i)[1].decode('utf-8') == "panda_finger_joint2"
#         # ][0][0]

#         # # Get the joint state for the finger joints
#         # joint_state_1 = p.getJointState(robot_id, joint_index_1)[0]  # Linear position of joint1
#         # joint_state_2 = p.getJointState(robot_id, joint_index_2)[0]  # Linear position of joint2

#         for name, sub_mesh in finger_mesh.geometry.items():
#             print(f"Processing sub-mesh name: {name}")

#             ### Left Finger Transformation ###
#             left_finger_transform = hand_transform.copy()
#             left_finger_transform[:3, 3] += [0, 0.01, 0.02]
#             # left_finger_transform[:3, 3] += [0, 0.01 + joint_state_1, 0.02]

#             # Apply transformation to the sub-mesh
#             left_sub_mesh = sub_mesh.copy()
#             left_sub_mesh.apply_transform(left_finger_transform)
#             left_sub_mesh.export(os.path.join(debug_dir, f"left_finger_{name}.obj"))
#             meshes.append(left_sub_mesh)


#             ### Right Finger Transformation ###
#             right_finger_transform = hand_transform.copy()
#             # right_finger_transform[:3, 3] += [0, -0.01 - joint_state_2, 0.02]
#             right_finger_transform[:3, 3] += [0, -0.01, 0.02]

#             # Add 180° rotation about the Z-axis
#             rotation_quaternion = trimesh.transformations.quaternion_from_euler(0, 0, np.pi)
#             rotation_matrix = trimesh.transformations.quaternion_matrix(rotation_quaternion)
#             right_finger_transform = right_finger_transform @ rotation_matrix

#             # Apply transformation to the sub-mesh
#             right_sub_mesh = sub_mesh.copy()
#             right_sub_mesh.apply_transform(right_finger_transform)
#             right_sub_mesh.export(os.path.join(debug_dir, f"right_finger_{name}.obj"))
#             meshes.append(right_sub_mesh)


# print(meshes)

# Combine all individual transformed meshes into a single mesh using trimesh.Scene
scene = trimesh.Scene()
for idx, mesh in enumerate(meshes):
    if isinstance(mesh, trimesh.Trimesh):
        # print(f"Adding Trimesh to scene: mesh_{idx}, Vertices: {len(mesh.vertices)}, Faces: {len(mesh.faces)}")
        scene.add_geometry(mesh, node_name=f"mesh_{idx}")

# Export the entire scene as a single OBJ file
scene.export("panda_scene.obj")
print(f"Exported full robot scene as .obj")

# Alternative export as STL for testing
scene.export("panda_scene.stl")
print("Exported scene as STL format for testing")

# Disconnect from PyBullet
p.disconnect()



# flattened_scene = trimesh.util.concatenate([g for g in scene.geometry.values()])
# flattened_scene.export(combined_output_path)
# print(f"Exported full robot scene as flattened .obj to {combined_output_path}")




#______________________________________________________________________
#Specific handling for left and right fingers
# finger_paths = [
#     ("panda_leftfinger", visual_directory + "finger.obj"),
#     ("panda_rightfinger", visual_directory + "finger.obj")
# ]

# for finger_name, finger_path in finger_paths:
#     finger_index = [p.getJointInfo(robot_id, i)[12].decode("utf-8") for i in range(p.getNumJoints(robot_id))].index(finger_name)
#     link_state = p.getLinkState(robot_id, finger_index)
#     link_translation = np.array(link_state[4])
#     link_rotation = np.array(p.getMatrixFromQuaternion(link_state[5])).reshape(3, 3)

#     transformation_matrix = np.eye(4)
#     transformation_matrix[:3, :3] = link_rotation
#     transformation_matrix[:3, 3] = link_translation

#     if os.path.exists(finger_path):
#         finger_mesh = trimesh.load(finger_path)
#         finger_mesh.apply_transform(transformation_matrix)
#         meshes.append(finger_mesh)
#         print(f"Finger {finger_name} added successfully")
