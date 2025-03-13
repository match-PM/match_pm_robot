import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import sympy as sp
from geometry_msgs.msg import Vector3, Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
import numpy as np
from sensor_msgs.msg import JointState
from typing import Union
from scipy.spatial.transform import Rotation as R
from pm_moveit_interfaces.srv import GetGonioSolution
from scipy.optimize import least_squares

def rotation_matrix_to_quaternion(Rot_mat)-> sp.Matrix:
    r = R.from_matrix(Rot_mat)
    return sp.Matrix(r.as_quat())

def get_transform_for_frame_in_world(frame_name: str, tf_buffer: Buffer, logger = None) -> TransformStamped:
    # this function adapts the tf for parent_frame changes
    #transform:TransformStamped = tf_buffer.lookup_transform(frame_name, 'world',rclpy.time.Time())
    transform = TransformStamped()
    try:
        
        transform:TransformStamped = tf_buffer.lookup_transform('world', frame_name, rclpy.time.Time(),rclpy.duration.Duration(seconds=1.0))
        if logger is not None:
            logger.debug(f"Frame '{frame_name}' found in TF!")
    except Exception as e:

        return TransformStamped()
    return transform

def get_transform_for_frame(frame_name: str, parent_frame:str, tf_buffer: Buffer, logger = None) -> TransformStamped:
    # this function adapts the tf for parent_frame changes
    #transform:TransformStamped = tf_buffer.lookup_transform(frame_name, 'world',rclpy.time.Time())
    try:
        
        transform:TransformStamped = tf_buffer.lookup_transform(parent_frame, frame_name, rclpy.time.Time(),rclpy.duration.Duration(seconds=1.0))
        if logger is not None:
            logger.debug(f"Frame '{frame_name}' found in TF!")
    except Exception as e:
        return TransformStamped()
    return transform


def quaternion_to_rotation_matrix(quaternion:Quaternion)-> sp.Matrix:
    w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z
    r = sp.Matrix([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])
    return r

def get_transform_matrix_from_tf(tf: Union[Pose, TransformStamped])-> sp.Matrix:
    if isinstance(tf, Pose):
        tf: Pose
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.orientation)
        t = sp.Matrix([tf.position.x, tf.position.y, tf.position.z]) 
    elif isinstance(tf, TransformStamped):
        tf: TransformStamped
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.transform.rotation)
        t = sp.Matrix([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])         
    else:
        return None
    transform_matrix = sp.eye(4)

    transform_matrix[:3, :3] = r
    transform_matrix[:3, 3] = t
    return transform_matrix

def get_euler_rotation_matrix(alpha, beta, gamma):
    rotation_z = sp.Matrix([
        [sp.cos(alpha), -sp.sin(alpha), 0],
        [sp.sin(alpha), sp.cos(alpha), 0],
        [0, 0, 1]
    ])

    rotation_y = sp.Matrix([
        [sp.cos(beta), 0, sp.sin(beta)],
        [0, 1, 0],
        [-sp.sin(beta), 0, sp.cos(beta)]
    ])

    rotation_x = sp.Matrix([
        [1, 0, 0],
        [0, sp.cos(gamma), -sp.sin(gamma)],
        [0, sp.sin(gamma), sp.cos(gamma)]
    ])

    rotation_matrix = rotation_z * rotation_y * rotation_x
    return rotation_matrix

def get_rotation_matrix_from_transformation_matrix(matrix: sp.Matrix)-> sp.Matrix:
    return matrix[:3, :3]

def get_rotation_matrix_from_tf(tf: Union[Pose, TransformStamped])-> sp.Matrix:
    if isinstance(tf, Pose):
        tf: Pose
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.orientation)
    elif isinstance(tf, TransformStamped):
        tf: TransformStamped
        r:sp.Matrix = quaternion_to_rotation_matrix(tf.transform.rotation)
    else:
        return None
    return r

class GonioOrientationSolver(Node):

    def __init__(self):
        super().__init__('GonioOrientationSolver')
        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        #self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.callback_group)
        # subscribe to joint_values
        self.joint_values_sub = self.create_subscription(JointState, 'joint_states', self.joint_values_callback, 10, callback_group=self.callback_group)

        self.gonio_right_q2 = 0
        self.gonio_right_q3 = 0
        self.t_axis_q = 0
        self.gonio_left_q2 = 0
        self.gonio_left_q3 = 0

        self.logger = self.get_logger()

        self.gonio_right_solution_srv = self.create_service(GetGonioSolution, self.get_name()+'/get_gonio_right_solution', self.calculate_gonio_right, callback_group=self.callback_group)
        self.gonio_left_solution_srv = self.create_service(GetGonioSolution, self.get_name()+'/get_gonio_left_solution', self.calculate_gonio_left, callback_group=self.callback_group)
        self.logger.info("GonioOrientationSolver Node started!")
    
    # def calculate_gonio_right(self, request:GetGonioSolution.Request, response: GetGonioSolution.Response):
    #     q1, q2, q3 = sp.symbols('q1 q2 q3')

    #     transform_gonio_right = get_transform_for_frame_in_world('Gonio_Right_Stage_1_Bottom', self.tf_buffer, self.get_logger())

    #     gonio_right_endeffector = get_transform_for_frame(request.gonio_endeffector, 
    #                                                 'Gonio_Right_Chuck_Origin' ,
    #                                                 self.tf_buffer, 
    #                                                 self.get_logger())
        
    #     gonio_right_endeffector_matrix = get_rotation_matrix_from_tf(gonio_right_endeffector)
    #     transform_gonio_right_matrix = get_rotation_matrix_from_tf(transform_gonio_right)
        
    #     joint_transformation = get_euler_rotation_matrix(0, -self.gonio_right_q2, -self.gonio_right_q3)
    #     joint_transform_sym = get_euler_rotation_matrix(0, -q2, -q3)

    #     final_gonio_rotations_matrix = transform_gonio_right_matrix * joint_transformation * gonio_right_endeffector_matrix
    #     final_from scipy.optimize import least_squares
    #     #final_gonio_rotations_matrix_quad = rotation_matrix_to_quaternion(final_gonio_rotations_matrix)
    #     # log final rotaion
    #     #self.get_logger().info(f"Final Rotation Matrix Gonio: {str(final_gonio_rotations_matrix_quad)}")

    #     transform_rotation_stage_world = get_transform_for_frame_in_world('Z_Axis', self.tf_buffer, self.get_logger())
    #     transform_rotation_stage_world_matrix = get_rotation_matrix_from_tf(transform_rotation_stage_world)

    #     transform_gripper_endeffector = get_transform_for_frame(request.gripper_endeffector, 
    #                                                 't_axis_toolchanger' ,
    #                                                 self.tf_buffer, 
    #                                                 self.get_logger())
        
    #     transform_gripper_endeffector_matrix = get_rotation_matrix_from_tf(transform_gripper_endeffector)

    #     joint_transformation_gripper = get_euler_rotation_matrix(self.t_axis_q, 0, 0)
    #     joint_transform_gripper = get_euler_rotation_matrix(q1, 0, 0)
        
    #     final_gripper_rotation_matrix = transform_rotation_stage_world_matrix * joint_transformation_gripper * transform_gripper_endeffector_matrix
    #     final_gripper_equation = transform_rotation_stage_world_matrix * joint_transform_gripper * transform_gripper_endeffector_matrix

    #     #final_gripper_rotation_matrix_quad = rotation_matrix_to_quaternion(final_gripper_rotation_matrix)

    #     equations = []
    #     for i in range(3):
    #         for j in range(3):
    #             eq = final_gonio_equation[i, j] - final_gripper_equation[i, j]
    #             equations.append(eq)

    #     initial_guess = (0, 0, 0)
    #     # Solve the equations for the unknowns q1, q2, q3
    #     solution = sp.nsolve(equations, (q1, q2, q3), initial_guess)
        
    #     if solution:
    #         response.joint_values = [float(solution[0]), float(solution[1]), float(solution[2])]
    #         response.joint_names = ['T_Axis_Joint','Gonio_Right_Stage_1_Joint', 'Gonio_Right_Stage_2_Joint']
    #         response.success = True
    #     # Print the solution
    #         self.logger.info(f"Solution T_Axis_Joint: {(solution[0])}")
    #         self.logger.info(f"Solution Gonio_Right_Stage_1_Joint: {(solution[1])}")
    #         self.logger.info(f"Solution Gonio_Right_Stage_2_Joint: {(solution[2])}")

    #     else:
    #         self.logger.error(f"No solution found!")
    #         response.success = False

    #     final_gonio_equation = final_gonio_equation.subs({q1: float(solution[0]), q2: float(solution[1]), q3: float(solution[2])})
    #     self.logger.error(f"Final Rotation Matrix Gonio: {str(final_gonio_equation)}")
    #     final_gonio_quat = rotation_matrix_to_quaternion(final_gonio_equation)  
    #     self.logger.error(f"Final Rotation Matrix Gonio: {str(final_gonio_quat)}")

    #     return response
    
    def calculate_gonio_right(self, request: GetGonioSolution.Request, response: GetGonioSolution.Response):
        q1, q2, q3 = sp.symbols('q1 q2 q3')

        # Get transformations
        transform_gonio_right = get_transform_for_frame_in_world('Gonio_Right_Stage_1_Bottom', self.tf_buffer, self.get_logger())
        gonio_right_endeffector = get_transform_for_frame(request.gonio_endeffector, 'Gonio_Right_Chuck_Origin', self.tf_buffer, self.get_logger())

        gonio_right_endeffector_matrix = get_rotation_matrix_from_tf(gonio_right_endeffector)
        transform_gonio_right_matrix = get_rotation_matrix_from_tf(transform_gonio_right)

        # Joint transformations (Euler angles)
        joint_transform_sym = get_euler_rotation_matrix(0, -q2, -q3)

        final_gonio_equation = transform_gonio_right_matrix * joint_transform_sym * gonio_right_endeffector_matrix

        # Rotation Stage in World
        transform_rotation_stage_world = get_transform_for_frame_in_world('Z_Axis', self.tf_buffer, self.get_logger())
        transform_rotation_stage_world_matrix = get_rotation_matrix_from_tf(transform_rotation_stage_world)

        transform_gripper_endeffector = get_transform_for_frame(request.gripper_endeffector, 't_axis_toolchanger', self.tf_buffer, self.get_logger())
        transform_gripper_endeffector_matrix = get_rotation_matrix_from_tf(transform_gripper_endeffector)

        # Gripper joint transformation
        joint_transform_gripper = get_euler_rotation_matrix(q1, 0, 0)

        final_gripper_equation = transform_rotation_stage_world_matrix * joint_transform_gripper * transform_gripper_endeffector_matrix

        # Function to evaluate the difference between gonio and gripper equations
        def equations(vars):
            q1_val, q2_val, q3_val = vars

            # Evaluate matrices numerically
            eq_gonio = final_gonio_equation.subs({q1: q1_val, q2: q2_val, q3: q3_val}).evalf()
            eq_gripper = final_gripper_equation.subs({q1: q1_val, q2: q2_val, q3: q3_val}).evalf()

            # Compute the difference (should be zero when solved correctly)
            eq_numeric = np.array(eq_gonio - eq_gripper).astype(np.float64)

            # Debugging logs
            self.get_logger().info(f"Equation Matrix Evaluated: {eq_numeric}")

            return eq_numeric.flatten()

        # Initial guess as a NumPy array
        initial_guess = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Solve using least squares (Levenberg-Marquardt method)
        result = least_squares(equations, initial_guess, method='lm')

        if result.success:
            q1_sol, q2_sol, q3_sol = result.x
            response.joint_values = [float(q1_sol), float(q2_sol), float(q3_sol)]
            response.joint_names = ['T_Axis_Joint', 'Gonio_Right_Stage_1_Joint', 'Gonio_Right_Stage_2_Joint']
            response.success = True

            self.get_logger().info(f"Solution T_Axis_Joint: {q1_sol}")
            self.get_logger().info(f"Solution Gonio_Right_Stage_1_Joint: {q2_sol}")
            self.get_logger().info(f"Solution Gonio_Right_Stage_2_Joint: {q3_sol}")
        else:
            self.get_logger().error("No solution found!")
            response.success = False
            return response

        # Substituting solutions to verify rotation matrix
        final_gonio_equation = final_gonio_equation.subs({q1: q1_sol, q2: q2_sol, q3: q3_sol}).evalf()
        self.get_logger().info(f"Final Rotation Matrix Gonio: {str(final_gonio_equation)}")

        # Convert to quaternion
        final_gonio_quat = rotation_matrix_to_quaternion(final_gonio_equation)  
        self.get_logger().info(f"Final Rotation Quaternion Gonio: {str(final_gonio_quat)}")

        return response


    def calculate_gonio_left(self, request: GetGonioSolution.Request, response: GetGonioSolution.Response):
        q1, q2, q3 = sp.symbols('q1 q2 q3')

        # Get transformations
        transform_gonio_left = get_transform_for_frame_in_world('Gonio_Left_Stage_1_Bottom', self.tf_buffer, self.get_logger())
        gonio_left_endeffector = get_transform_for_frame(request.gonio_endeffector, 'Gonio_Left_Chuck_Origin', self.tf_buffer, self.get_logger())

        gonio_left_endeffector_matrix = get_rotation_matrix_from_tf(gonio_left_endeffector)
        transform_gonio_left_matrix = get_rotation_matrix_from_tf(transform_gonio_left)

        # Joint transformations (Euler angles)
        joint_transform_sym = get_euler_rotation_matrix(0, -q3, q2)

        final_gonio_equation = transform_gonio_left_matrix * joint_transform_sym * gonio_left_endeffector_matrix

        # Rotation Stage in World
        transform_rotation_stage_world = get_transform_for_frame_in_world('Z_Axis', self.tf_buffer, self.get_logger())
        transform_rotation_stage_world_matrix = get_rotation_matrix_from_tf(transform_rotation_stage_world)

        transform_gripper_endeffector = get_transform_for_frame(request.gripper_endeffector, 't_axis_toolchanger', self.tf_buffer, self.get_logger())
        transform_gripper_endeffector_matrix = get_rotation_matrix_from_tf(transform_gripper_endeffector)

        # Gripper joint transformation
        joint_transform_gripper = get_euler_rotation_matrix(q1, 0, 0)

        final_gripper_equation = transform_rotation_stage_world_matrix * joint_transform_gripper * transform_gripper_endeffector_matrix

        # Function to evaluate the difference between gonio and gripper equations
        def equations(vars):
            q1_val, q2_val, q3_val = vars

            # Evaluate matrices numerically
            eq_gonio = final_gonio_equation.subs({q1: q1_val, q2: q2_val, q3: q3_val}).evalf()
            eq_gripper = final_gripper_equation.subs({q1: q1_val, q2: q2_val, q3: q3_val}).evalf()

            # Compute the difference (should be zero when solved correctly)
            eq_numeric = np.array(eq_gonio - eq_gripper).astype(np.float64)

            # Debugging logs
            self.get_logger().info(f"Equation Matrix Evaluated: {eq_numeric}")

            return eq_numeric.flatten()

        # Initial guess as a NumPy array
        initial_guess = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # Solve using least squares (Levenberg-Marquardt method)
        result = least_squares(equations, initial_guess, method='lm')

        if result.success:
            q1_sol, q2_sol, q3_sol = result.x
            response.joint_values = [float(q1_sol), float(q2_sol), float(q3_sol)]
            response.joint_names = ['T_Axis_Joint', 'Gonio_Left_Stage_1_Joint', 'Gonio_Left_Stage_2_Joint']
            response.success = True

            self.get_logger().info(f"Solution T_Axis_Joint: {q1_sol}")
            self.get_logger().info(f"Solution Gonio_Left_Stage_1_Joint: {q2_sol}")
            self.get_logger().info(f"Solution Gonio_Left_Stage_2_Joint: {q3_sol}")
        else:
            self.get_logger().error("No solution found!")
            response.success = False
            return response

        # Substituting solutions to verify rotation matrix
        final_gonio_equation = final_gonio_equation.subs({q1: q1_sol, q2: q2_sol, q3: q3_sol}).evalf()
        self.get_logger().info(f"Final Rotation Matrix Gonio: {str(final_gonio_equation)}")

        # Convert to quaternion
        final_gonio_quat = rotation_matrix_to_quaternion(final_gonio_equation)  
        self.get_logger().info(f"Final Rotation Quaternion Gonio: {str(final_gonio_quat)}")

        return response

    # def calculate_gonio_left(self, request:GetGonioSolution.Request, response: GetGonioSolution.Response):
    #     q1, q2, q3 = sp.symbols('q1 q2 q3')

    #     transform_gonio_left = get_transform_for_frame_in_world('Gonio_Left_Stage_1_Bottom', self.tf_buffer, self.get_logger())

    #     gonio_left_endeffector = get_transform_for_frame(request.gonio_endeffector, 
    #                                                 'Gonio_Left_Chuck_Origin' ,
    #                                                 self.tf_buffer, 
    #                                                 self.get_logger())
        
    #     gonio_left_endeffector_matrix = get_rotation_matrix_from_tf(gonio_left_endeffector)
    #     transform_gonio_left_matrix = get_rotation_matrix_from_tf(transform_gonio_left)
        
    #     #joint_transformation = get_euler_rotation_matrix(0, -self.gonio_left_q2, -self.gonio_left_q3)
    #     #joint_transform_sym = get_euler_rotation_matrix(0, q2, -q3)

    #     joint_transformation = get_euler_rotation_matrix(0, -self.gonio_left_q3, -self.gonio_left_q2)
    #     joint_transform_sym = get_euler_rotation_matrix(0,  -q3, q2)
        
    #     final_gonio_rotations_matrix = transform_gonio_left_matrix * joint_transformation * gonio_left_endeffector_matrix
    #     final_gonio_equation = transform_gonio_left_matrix * joint_transform_sym * gonio_left_endeffector_matrix

    #     final_gonio_rotations_matrix_quad = rotation_matrix_to_quaternion(final_gonio_rotations_matrix)
    #     # log final rotaion
    #     #self.get_logger().info(f"Final Rotation Matrix Gonio: {str(final_gonio_rotations_matrix_quad)}")

    #     transform_rotation_stage_world = get_transform_for_frame_in_world('Z_Axis', self.tf_buffer, self.get_logger())
    #     transform_rotation_stage_world_matrix = get_rotation_matrix_from_tf(transform_rotation_stage_world)

    #     transform_gripper_endeffector = get_transform_for_frame(request.gripper_endeffector, 
    #                                                 't_axis_toolchanger' ,
    #                                                 self.tf_buffer, 
    #                                                 self.get_logger())
        
    #     transform_gripper_endeffector_matrix = get_rotation_matrix_from_tf(transform_gripper_endeffector)

    #     joint_transformation_gripper = get_euler_rotation_matrix(self.t_axis_q, 0, 0)
    #     joint_transform_gripper = get_euler_rotation_matrix(q1, 0, 0)
        
    #     final_gripper_rotation_matrix = transform_rotation_stage_world_matrix * joint_transformation_gripper * transform_gripper_endeffector_matrix
    #     final_gripper_equation = transform_rotation_stage_world_matrix * joint_transform_gripper * transform_gripper_endeffector_matrix

    #     final_gripper_rotation_matrix_quad = rotation_matrix_to_quaternion(final_gripper_rotation_matrix)

    #     equations = []
    #     for i in range(3):
    #         for j in range(3):
    #             eq = final_gonio_equation[i, j] - final_gripper_equation[i, j]
    #             equations.append(eq)

    #     initial_guess = (0, 0, 0)
    #     # Solve the equations for the unknowns q1, q2, q3
    #     solution = sp.nsolve(equations, (q1, q2, q3), initial_guess)
        
    #     if solution:
    #         response.joint_values = [float(solution[0]), float(solution[1]), float(solution[2])]
    #         response.joint_names = ['T_Axis_Joint', 'Gonio_Left_Stage_1_Joint', 'Gonio_Left_Stage_2_Joint']

    #         response.success = True
    #     # Print the solution
    #         self.logger.info(f"Solution T_Axis_Joint: {(solution[0])}")
    #         self.logger.info(f"Solution Gonio_Left_Stage_1_Joint: {(solution[1])}")
    #         self.logger.info(f"Solution Gonio_Left_Stage_2_Joint: {(solution[2])}")

    #     else:
    #         self.logger.error(f"No solution found!")
    #         response.success = False

    #     return response
    


    # def timer_callback(self):
    #     #transform_gonio_left = get_transform_for_frame_in_world('gonio_left', self.tf_buffer, self.get_logger())
    #     q1, q2, q3 = sp.symbols('q1 q2 q3')

    #     transform_gonio_right = get_transform_for_frame_in_world('Gonio_Left_Stage_2_Bottom', self.tf_buffer, self.get_logger())

    #     gonio_right_endeffector = get_transform_for_frame('Gonio_Right_Part_1_Origin', 
    #                                                 'Gonio_Right_Chuck_Origin' ,
    #                                                 self.tf_buffer, 
    #                                                 self.get_logger())
        
    #     gonio_right_endeffector_matrix = get_rotation_matrix_from_tf(gonio_right_endeffector)
    #     transform_gonio_right_matrix = get_rotation_matrix_from_tf(transform_gonio_right)
        
    #     joint_transformation = get_euler_rotation_matrix(0, -self.gonio_right_q2, -self.gonio_right_q3)
    #     joint_transform_sym = get_euler_rotation_matrix(0, -q2, -q3)

    #     final_gonio_rotations_matrix = transform_gonio_right_matrix * joint_transformation * gonio_right_endeffector_matrix
    #     final_gonio_equation = transform_gonio_right_matrix * joint_transform_sym * gonio_right_endeffector_matrix

    #     final_gonio_rotations_matrix_quad = rotation_matrix_to_quaternion(final_gonio_rotations_matrix)
    #     # log final rotaion
    #     #self.get_logger().info(f"Final Rotation Matrix Gonio: {str(final_gonio_rotations_matrix_quad)}")

    #     transform_rotation_stage_world = get_transform_for_frame_in_world('Z_Axis', self.tf_buffer, self.get_logger())
    #     transform_rotation_stage_world_matrix = get_rotation_matrix_from_tf(transform_rotation_stage_world)

    #     transform_gripper_endeffector = get_transform_for_frame('test_frame', 
    #                                                 't_axis_toolchanger' ,
    #                                                 self.tf_buffer, 
    #                                                 self.get_logger())
        
    #     transform_gripper_endeffector_matrix = get_rotation_matrix_from_tf(transform_gripper_endeffector)

    #     joint_transformation_gripper = get_euler_rotation_matrix(self.t_axis_q, 0, 0)
    #     joint_transform_gripper = get_euler_rotation_matrix(q1, 0, 0)
        
    #     final_gripper_rotation_matrix = transform_rotation_stage_world_matrix * joint_transformation_gripper * transform_gripper_endeffector_matrix
    #     final_gripper_equation = transform_rotation_stage_world_matrix * joint_transform_gripper * transform_gripper_endeffector_matrix

    #     final_gripper_rotation_matrix_quad = rotation_matrix_to_quaternion(final_gripper_rotation_matrix)

    #     # log final rotaion
    #     #self.get_logger().info(f"Final Rotation Matrix Test: {str(final_gripper_rotation_matrix_quad)}")
    #     #self.get_logger().info(f"Final Rotation Matrix Test: {str(transform_rotation_stage_world.transform.rotation)}")
    #     #self.logger.warn(f"Equations Gonio: {str(final_gonio_equation)}")
    #     #self.logger.warn(f"Equations Test: {str(final_gripper_equation)}")

    #     # solve equation so that final_gonio_equation = final_gripper_equation
    #     # solve for q1, q2, q3

    #     #equations = sp.Eq(final_gonio_equation, final_gripper_equation)
    #     equations = []
    #     for i in range(3):
    #         for j in range(3):
    #             eq = final_gonio_equation[i, j] - final_gripper_equation[i, j]
    #             equations.append(eq)

    #     initial_guess = (0, 0, 0)
    #     # Solve the equations for the unknowns q1, q2, q3
    #     solution = sp.nsolve(equations, (q1, q2, q3), initial_guess)
        
    #     # Print the solution
    #     if solution:
    #         self.logger.error(f"solution q1: {(solution[0])}")
    #         self.logger.error(f"solution q2: {(solution[1])}")
    #         self.logger.error(f"solution q3: {(solution[2])}")

    #     else:
    #         self.logger.error(f"No solution found!")
        

    def joint_values_callback(self, msg: JointState):
        for index, name in enumerate(msg.name):
            if name == 'Gonio_Right_Stage_1_Joint':
                self.gonio_right_q2 = msg.position[index]
            if name == 'Gonio_Right_Stage_2_Joint':
                self.gonio_right_q3 = msg.position[index]
            if name == 'T_Axis_Joint':
                self.t_axis_q = msg.position[index]

            if name == 'Gonio_Left_Stage_1_Joint':
                self.gonio_left_q2 = msg.position[index]
            if name == 'Gonio_Left_Stage_2_Joint':
                self.gonio_left_q3 = msg.position[index]


def main(args=None):
    rclpy.init(args=args)

    SolverNode = GonioOrientationSolver()

    executor = MultiThreadedExecutor(num_threads=6) 

    executor.add_node(SolverNode)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        SolverNode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()