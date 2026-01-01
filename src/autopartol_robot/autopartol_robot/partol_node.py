from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener,Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math




class PartolNode(BasicNavigator):
    def __init__(self, node_name='Partol_Robot'):
        super().__init__(node_name)
        # call the parameter
        self.declare_parameter('initial_point',[0.0, 0.0, 0.0])
        self.declare_parameter('target_points',[0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        
     
        
    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        return the object of PoseStamped
        """
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = "map"
        robot_pose.pose.position.x = x
        robot_pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, yaw)
        robot_pose.pose.orientation.x = quat[0]
        robot_pose.pose.orientation.y = quat[1]
        robot_pose.pose.orientation.z = quat[2]
        robot_pose.pose.orientation.w = quat[3]
        
        return robot_pose
    
        
        
    def init_robot_pose(self):
        """
        initialized the initial pose of the robot
        """
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()
    
        
        
    def get_target_points(self):
        """
        get the list of the target points by declare_parameter
        """
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3 + 1]
            yaw = self.target_points_[index*3 + 2]
            points.append([x, y, yaw])
            self.get_logger().info(f"get the target points{index}-->{x}, {y}, {yaw}")
        
        return points    
            
 
       
    def nav_to_pose(self, target_points):
        """
        navigate to the target
        """
        self.goToPose(target_points)
    
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            self.get_logger().info(f'rest distance:{feedback.distance_remaining}')
        result = self.getResult()
        self.get_logger().info(f'result of nav:{result}')
            
            
            
    def get_current_pose(self):
        """
        get current pose of the robot
        """
        while rclpy.ok():
            try:
                result = self.buffer_.lookup_transform('map', 'base_footprint',
                    rclpy.time.Time(seconds=0.0), rclpy.time.Duration(seconds=1.0))
                transform = result.transform
                self.get_logger().info(f'translation:{transform.translation}')
                # self.get_logger().info(f'orientation: {transform.rotation}')
                # rotation_euler = euler_from_quaternion([
                #         transform.rotation.x,
                #         transform.rotation.y,
                #         transform.rotation.z,
                #         transform.rotation.w
                #         ])
                # self.get_logger().info(f'RPY: {rotation_euler}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'fail to get RPY:{str(e)}')
        




def main():
    rclpy.init()
    partol = PartolNode()
    # rclpy.spin(partol)
    partol.init_robot_pose()
    
    while rclpy.ok():
        points = partol.get_target_points()
        for point in points:
            x, y, yaw = point[0], point[1], point[2]
            target_pose =  partol.get_pose_by_xyyaw(x, y, yaw)
            partol.nav_to_pose(target_pose)
    
    
    rclpy.shutdown()