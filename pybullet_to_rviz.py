import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import JointState
import tf2_ros

import pybullet as p
import pybullet_data
import time

import numpy as np
import math

speed, strength = 5.0, 1.0
gravity = -9.81

defaults = [0] * 18

def rotate_vector_z(vector, theta_degrees):
    """
    Rotates a single 3D vector [x, y, z] around the Z-axis.
    """
    # 1. Unpack coordinates
    x, y = vector
    
    # 2. Convert angle to radians
    theta_rad = np.radians(theta_degrees)
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    
    # 3. Calculate new X and Y coordinates (Z stays the same)
    x_new = x * c - y * s
    y_new = x * s + y * c
    
    return np.array([x_new, y_new])

def project_point_on_plane(target_point, plane_point, plane_normal):
    p_target = np.array(target_point, dtype=float)
    p_plane = np.array(plane_point, dtype=float)
    n_plane = np.array(plane_normal, dtype=float)
    
    n_norm = np.linalg.norm(n_plane)
    if n_norm == 0:
        raise ValueError("Normal vector cannot be zero.")
    n_plane = n_plane / n_norm
    
    v = p_target - p_plane
    distance = np.dot(v, n_plane)
    projected_point = p_target - (distance * n_plane)
    
    return projected_point

def get_joint_dictionary(robot_id):
    joint_map = {}
    num_joints = p.getNumJoints(robot_id)
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        
        if joint_type != p.JOINT_FIXED:
            joint_map[joint_name] = i
            
    return joint_map

def get_link_dictionary(robot_id):
    index_to_name = {}
    name_to_index = {}
    
    base_name = p.getBodyInfo(robot_id)[0].decode("utf-8")
    index_to_name[-1] = base_name
    name_to_index[base_name] = -1
    
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        link_name = joint_info[12].decode("utf-8")
        
        index_to_name[i] = link_name
        name_to_index[link_name] = i
        
    return index_to_name, name_to_index

def get_angle_between_vectors(v1, v2):
    a = np.array(v1, dtype=float)
    b = np.array(v2, dtype=float)
    
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    
    if norm_a == 0 or norm_b == 0:
        return 0.0, 0.0 
        
    dot_product = np.dot(a, b)
    cos_theta = dot_product / (norm_a * norm_b)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    angle_rad = np.arccos(cos_theta)
    
    return angle_rad

def closest_point_on_sphere_intersection(c1, r1, c2, r2, p):
    c1 = np.array(c1, dtype=float)
    c2 = np.array(c2, dtype=float)
    p = np.array(p, dtype=float)
    
    d_vec = c2 - c1
    d = np.linalg.norm(d_vec)
    
    if d > r1 + r2 or d < abs(r1 - r2) or (d == 0 and r1 == r2):
        return None
        
    h = (r1**2 - r2**2 + d**2) / (2 * d)
    rc = np.sqrt(max(0.0, r1**2 - h**2))
    n = d_vec / d
    cc = c1 + h * n
    
    if rc == 0:
        return cc
        
    v = p - cc
    v_proj = v - np.dot(v, n) * n
    v_proj_norm = np.linalg.norm(v_proj)
    
    if v_proj_norm > 1e-8:
        direction = v_proj / v_proj_norm
    else:
        if abs(n[0]) > abs(n[1]):
            direction = np.array([-n[2], 0, n[0]])
        else:
            direction = np.array([0, -n[2], n[1]])
        direction = direction / np.linalg.norm(direction)
        
    closest_point = cc + rc * direction
    return closest_point

def ik_calculator(base_link_transmat, leg_targets, self):
    leg_points = [np.array([np.sin(i*math.pi/3), np.cos(i*math.pi/3), 0.0]) for i in range(6)]
    leg_perp_dir = [np.array([-np.cos(i*math.pi/3), np.sin(i*math.pi/3), 0.0]) for i in range(6)]

    angles = defaults.copy()

    def get_local(point):
        return np.matmul(np.linalg.inv(base_link_transmat), np.array([point[0], point[1], point[2], 1]))[0:3]

    for i in range(6):
        target_pos = leg_targets[i]

        target_pos = np.array([target_pos[0], target_pos[1], target_pos[2]])
        target_pos_local = get_local(target_pos)
        p1 = project_point_on_plane(target_pos_local, [0, 0, 0], [0, 0, 1])

        angle_1 = get_angle_between_vectors(leg_perp_dir[i], p1-leg_points[i]*-0.06)-math.pi/2
        x = np.linalg.norm(p1-leg_points[i]*-0.106)
        y = target_pos_local[2]-0.02
        angle_3 = -np.acos(np.clip((x*x+y*y-0.08*0.08-0.12*0.12)/(2*0.08*0.12), -1, 1))
        angle_2 = np.atan2(y, x)-np.atan(0.12*np.sin(angle_3)/(0.08 + 0.12*np.cos(angle_3)))

        angles[self.joint_dictionary[f"revolute_{i+1}"]-1] = angle_1
        if (i==4):
            angles[self.joint_dictionary[f"joint_2"]-1] = -angle_2
            angles[self.joint_dictionary[f"joint_3"]-1] = angle_3
        elif(i==5):
            angles[self.joint_dictionary[f"joint_2_5"]-1] = -angle_2
            angles[self.joint_dictionary[f"joint_3_5"]-1] = angle_3
        else:
            angles[self.joint_dictionary[f"joint_2_{i+1}"]-1] = -angle_2
            angles[self.joint_dictionary[f"joint_3_{i+1}"]-1] = angle_3        

    return angles

class PyBulletBridgeNode(Node):
    def __init__(self):
        super().__init__('pybullet_rviz_bridge')
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.physicsClient = p.connect(p.GUI) 
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, gravity)

        params = p.getPhysicsEngineParameters()
        self.dt = params['fixedTimeStep']

        self.robot_urdf_path = "/home/pradnesh/Desktop/MMR/Hexapod/hexapod/urdf/hexapod_with_collisions.urdf"
        self.robot_id = p.loadURDF(self.robot_urdf_path, [0, 0, 0.2], useFixedBase=False)
            

        p.loadURDF("plane.urdf")
        # terrain_collision_shape = p.createCollisionShape(
        #     shapeType=p.GEOM_MESH,
        #     fileName="terrain.stl",
        #     flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
        #     meshScale=[1.0, 1.0, 1.0]  # Adjust scale [x, y, z] if needed
        # )

        # # 2. Create visual shape (optional, but lets you see it if colors aren't loaded)
        # terrain_visual_shape = p.createVisualShape(
        #     shapeType=p.GEOM_MESH,
        #     fileName="terrain.stl",
        #     meshScale=[1.0, 1.0, 1.0]
        # )

        # # 3. Create a static multi-body (mass = 0 makes it stationary/terrain)
        # terrain_body = p.createMultiBody(
        #     baseMass=0,
        #     baseCollisionShapeIndex=terrain_collision_shape,
        #     baseVisualShapeIndex=terrain_visual_shape,
        #     basePosition=[0, 0, 0],
        #     baseOrientation=[0, 0, 0, 1]
        # )

        # p.changeDynamics(bodyUniqueId=terrain_body, 
        #          linkIndex=-1,  # -1 refers to the base link
        #          lateralFriction=10.0) # Increase from default 0.5 (try 1.0 to 2.0+)

            
        p.changeDynamics(bodyUniqueId=self.robot_id, 
                        linkIndex=-1, 
                        lateralFriction=10.0)

        num_joints = p.getNumJoints(self.robot_id)
        self.revolute_joint_indices = []
        self.revolute_joint_names = []

        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_index = joint_info[0]
            joint_name = joint_info[1].decode("utf-8")
            joint_type = joint_info[2]

            if joint_type == p.JOINT_REVOLUTE:
                self.revolute_joint_indices.append(joint_index)
                self.revolute_joint_names.append(joint_name)

        self.joint_dictionary = get_joint_dictionary(self.robot_id)
        self.link_dictionary = get_link_dictionary(self.robot_id)

        for i in self.joint_dictionary:
            print(i, self.joint_dictionary[i])

        self.get_logger().info(f"Tracking revolute joints: {self.revolute_joint_names}")
        self.sim_time = 0.0
        self.timer = self.create_timer(1.0 / 240.0, self.timer_callback)

        self.total_energy = 0.0
        self.start_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        self.distance_traveled = 0.0

        self.a = 0
        self.b = 0

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()
        self.sim_time += self.dt

        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        rot_matrix_9 = p.getMatrixFromQuaternion(base_orn)
        rot_matrix = np.array(rot_matrix_9).reshape(3, 3)
        base_transform_matrix = np.eye(4)
        base_transform_matrix[:3, :3] = rot_matrix
        base_transform_matrix[:3, 3] = base_pos

        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "world"
        t.child_frame_id = "base_link" 
        t.transform.translation.x = base_pos[0]
        t.transform.translation.y = base_pos[1]
        t.transform.translation.z = base_pos[2]
        t.transform.rotation.x = base_orn[0]
        t.transform.rotation.y = base_orn[1]
        t.transform.rotation.z = base_orn[2]
        t.transform.rotation.w = base_orn[3]
        
        self.tf_broadcaster.sendTransform(t)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time
        joint_state_msg.name = self.revolute_joint_names

        positions = []
        velocities = []
        for joint_index in self.revolute_joint_indices:
            state = p.getJointState(self.robot_id, joint_index)
            positions.append(state[0])
            velocities.append(state[1])

        joint_state_msg.position = positions
        joint_state_msg.velocity = velocities
        self.joint_pub.publish(joint_state_msg)

        keys = p.getKeyboardEvents()
        stride_forward = 0
        turn = None
        if ord('w') in keys or p.B3G_UP_ARROW in keys:
            stride_forward = 0.05
        elif ord('s') in keys or p.B3G_DOWN_ARROW in keys:
            stride_forward = -0.05
        if ord('a') in keys or p.B3G_LEFT_ARROW in keys:
            turn = np.array([0.2, 0])  # Tight left turn
        elif ord('d') in keys or p.B3G_RIGHT_ARROW in keys:
            turn = np.array([-0.2, 0])  # Tight left turn

        self.apply_gait( # tripod # (14.10J/m, 0.092m/s) (39.12, 0.077)
            base_transform_matrix, 
            self.sim_time, 
            duty_factor=0.5, stride_length=stride_forward, phase_offsets=[0.0, 0.5, 0.0, 0.5, 0.0, 0.5], center_of_curv=turn)
        # self.apply_gait(  # tetrapod # (12.24, 0.074) (44.92, 0.050)
        #     base_transform_matrix, 
        #     self.sim_time, 
        #     duty_factor=2/3, stride_length=stride_forward, phase_offsets=[0.0, 1.0/3, 2.0/3, 0.0, 1.0/3, 2.0/3], center_of_curv=turn)
        # self.apply_gait( # wave (13.52, 0.055) (58.45, 0.064)
        #     base_transform_matrix,# 13.92 0.063
        #     self.sim_time, 
        #     duty_factor=5/6, stride_length=stride_forward, phase_offsets=[i/6.0 for i in range(6)], center_of_curv=turn)
        # self.apply_gait( # ripple (13.05, 0.074) (44.74, 0.056)
        #     base_transform_matrix,
        #     self.sim_time, 
        #     duty_factor=2/3, stride_length=stride_forward*0, phase_offsets=[0.0, 6.0/9, 3.0/9, 1.0/9, 7.0/9, 4.0/9], center_of_curv=turn)

        # TESTING CODE
        # linear_vel, _ = p.getBaseVelocity(self.robot_id)
        # current_speed = linear_vel[0] # Forward speed along X

        # # 2. Track Energy Consumption (Power * dt)
        # step_power = 0.0
        # for joint_index in self.revolute_joint_indices:
        #     joint_state = p.getJointState(self.robot_id, joint_index)
        #     joint_velocity = joint_state[1]
        #     applied_torque = joint_state[3] # Motor torque
        #     step_power += abs(applied_torque * joint_velocity)

        # self.total_energy += step_power * self.dt

        # # 3. Track Total Distance
        # current_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        # self.distance_traveled = np.linalg.norm(np.array(current_pos[:2]) - np.array(self.start_pos[:2]))

        # # If distance reaches a target (e.g., 2 meters), calculate final J/m and print it:
        # if self.distance_traveled >= 7.5:
        #     print("Speed =", 7.5/self.sim_time)
        #     energy_per_meter = self.total_energy / self.distance_traveled
        #     print(f"Energy per Distance = {energy_per_meter:.2f} J/m")
        #     quit()


        p.stepSimulation()

    def apply_gait(self, base_transmat, t, duty_factor, phase_offsets, stride_length=0.06, step_height=0.025, center_of_curv=None):
        def get_global(point):
            return np.matmul(base_transmat, np.array([point[0], point[1], point[2], 1]))[0:3]

        points = [np.zeros(3) for i in range(6)]

        if (type(center_of_curv) == type(None)):
            for i in range(6):
                base_pos = np.array([-0.2 * np.sin(i * math.pi / 3), -0.2 * np.cos(i * math.pi / 3), -0.05])
                leg_phase = (t + phase_offsets[i]) % 1.0   
                if leg_phase < duty_factor:
                    # stance
                    progress = leg_phase / duty_factor
                    x_offset = ((stride_length / 2) - (stride_length * progress))*np.sin(math.pi/6)
                    y_offset = -((stride_length / 2) - (stride_length * progress))*np.cos(math.pi/6)
                    points[i] = base_pos + np.array([x_offset, y_offset, 0])
                else:
                    # swing
                    progress = (leg_phase - duty_factor) / (1.0 - duty_factor)
                    x_offset = -((stride_length / 2) - (stride_length * progress))*np.sin(math.pi/6)
                    y_offset = -(-(stride_length / 2) + (stride_length * progress))*np.cos(math.pi/6)
                    z_offset = step_height * math.sin(progress * math.pi)
                    points[i] = base_pos + np.array([x_offset, y_offset, z_offset])
        else:
            for i in range(6):
                base_pos = np.array([-0.2 * np.sin(i * math.pi / 3), -0.2 * np.cos(i * math.pi / 3), -0.05])
                
                vector_to_leg = base_pos[:2] - center_of_curv
                radius_to_leg = np.linalg.norm(vector_to_leg)
                
                tangent_dir = np.array([-vector_to_leg[1], vector_to_leg[0]]) / radius_to_leg
                tangent_dir = np.sign(center_of_curv[0])*rotate_vector_z(tangent_dir, 30)
                
                leg_stride_length = stride_length*(radius_to_leg/np.linalg.norm(np.array([0, 0])-center_of_curv))
                leg_phase = (t + phase_offsets[i]) % 1.0
                if leg_phase < duty_factor:
                    # Stance: move backward along the tangent path
                    progress = leg_phase / duty_factor
                    offset_magnitude = (leg_stride_length / 2) - (leg_stride_length * progress)
                    movement = tangent_dir * offset_magnitude
                    points[i] = base_pos + np.array([movement[0], movement[1], 0])
                else:
                    # Swing: move forward along the tangent path with lift
                    progress = (leg_phase - duty_factor) / (1.0 - duty_factor)
                    offset_magnitude = -(leg_stride_length / 2) + (leg_stride_length * progress)
                    movement = tangent_dir * offset_magnitude
                    z_offset = step_height * math.sin(progress * math.pi)
                    points[i] = base_pos + np.array([movement[0], movement[1], z_offset])

        points = [get_global(pt) for pt in points]
        self.publish_location_marker(points, self.get_clock().now().to_msg())
        
        p.setJointMotorControlArray(
            self.robot_id,
            self.revolute_joint_indices,
            p.POSITION_CONTROL,
            targetPositions=ik_calculator(base_transmat, points, self),
            forces=[strength] * len(self.revolute_joint_indices)
        )

    def publish_location_marker(self, coordinates, timestamp):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = timestamp
        marker.ns = "simulation_marker"
        marker.id = 1
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
    
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        for coord in coordinates:
            pt = Point()
            pt.x = coord[0]
            pt.y = coord[1]
            pt.z = coord[2]
            marker.points.append(pt)

        self.marker_pub.publish(marker)

    def destroy_node(self):
        p.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PyBulletBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()