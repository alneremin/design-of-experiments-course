from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters 
import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient



class NavigationControl:

    def __init__(self, node):
        self.node = node
        self.controller_server_cli = node.create_client(SetParameters, "/artbul/controller_server/set_parameters")
        while not self.controller_server_cli.wait_for_service(timeout_sec=1.0):
            print('Service "/artbul/controller_server/set_parameters" not available, waiting again...')

        self.lifecycle_manager_nav_cli = node.create_client(ManageLifecycleNodes, "/artbul/lifecycle_manager_navigation/manage_nodes")
        while not self.lifecycle_manager_nav_cli.wait_for_service(timeout_sec=1.0):
            print('Service "/artbul/lifecycle_manager_navigation/manage_nodes" not available, waiting again...')

        self.lifecycle_manager_loc_cli = node.create_client(ManageLifecycleNodes, "/artbul/lifecycle_manager_localization/manage_nodes")
        while not self.lifecycle_manager_loc_cli.wait_for_service(timeout_sec=1.0):
            print('Service "/artbul/lifecycle_manager_localization/manage_nodes" not available, waiting again...')
            
        self.init_pose_publisher = node.create_publisher(
            PoseWithCovarianceStamped,
            "/artbul/initialpose",
            10)
        
        self.nav_action_client = ActionClient(node, NavigateToPose, '/artbul/navigate_to_pose')
        self.feedback = None
        self.robot_navigate = False
        self.status = None
        



    def change_nav2_parameter(self, param_name, param_type, param_value):
        param = Parameter()
        param.name = param_name
        param.value.type = param_type
        param.value.double_value = param_value

        set_param_request = SetParameters.Request()
        set_param_request.parameters = [param]

        self.future = self.controller_server_cli.call_async(set_param_request)
        rclpy.spin_until_future_complete(self.node, self.future)
        if self.future.result() is not None:
            print(f'Parameter {param_name} set successfully: {self.future.result().results[0].successful}')
            return True
        else:
            print(f'Service call failed for {param_name}')
            return False


    def reset_navigation_stack(self):
        
        lifecycle_manager_request = ManageLifecycleNodes.Request()
        lifecycle_manager_request.command = 3

        self.future = self.lifecycle_manager_nav_cli.call_async(lifecycle_manager_request)
        rclpy.spin_until_future_complete(self.node, self.future)
        if self.future.result() is not None:
            print(f'Lifecycle of navigation was shutdown successfully, status: {self.future.result().success}')
        else:
            print(f'Service lifecycle_manager_nav call failed')

        self.future = self.lifecycle_manager_loc_cli.call_async(lifecycle_manager_request)
        rclpy.spin_until_future_complete(self.node, self.future)
        if self.future.result() is not None:
            print(f'Lifecycle of localization was shutdown successfully, status: {self.future.result().success}')
        else:
            print(f'Service lifecycle_manager_loc call failed')




    def startup_navigation_stack(self):
        
        lifecycle_manager_request = ManageLifecycleNodes.Request()
        lifecycle_manager_request.command = 0

        self.future = self.lifecycle_manager_loc_cli.call_async(lifecycle_manager_request)
        rclpy.spin_until_future_complete(self.node, self.future)
        if self.future.result() is not None:
            print(f'Lifecycle of localization started successfully, status: {self.future.result().success}')
        else:
            print(f'Service lifecycle_manager_loc call failed')

        self.future = self.lifecycle_manager_nav_cli.call_async(lifecycle_manager_request)
        rclpy.spin_until_future_complete(self.node, self.future)
        if self.future.result() is not None:
            print(f'Lifecycle of navigation started successfully, status: {self.future.result().success}')
        else:
            print(f'Service lifecycle_manager_nav call failed')


    def update_robot_pose(self, pose):

        pose = [float(i) for i in pose]

        amcl_pose = PoseWithCovarianceStamped()
        amcl_pose.header.frame_id = 'map'
        amcl_pose.pose.pose.position.x = pose[0]
        amcl_pose.pose.pose.position.y = pose[1]
        amcl_pose.pose.pose.position.z = pose[2]

        amcl_pose.pose.pose.orientation.x = pose[3]
        amcl_pose.pose.pose.orientation.y = pose[4]
        amcl_pose.pose.pose.orientation.z = pose[5]
        amcl_pose.pose.pose.orientation.w = pose[6]

        self.init_pose_publisher.publish(amcl_pose)
        print("Robot pose is updated.")

    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback
        
    def start_robot_navigation(self, pose):

        pose = [float(i) for i in pose]
        self.nav_action_client.wait_for_server()

        nav_action_client_request = NavigateToPose.Goal()
        nav_action_client_request.pose.header.frame_id = 'map'
        nav_action_client_request.pose.header.stamp = self.node.get_clock().now().to_msg()
        nav_action_client_request.pose.pose.position.x = pose[0]
        nav_action_client_request.pose.pose.position.y = pose[1]
        nav_action_client_request.pose.pose.position.z = pose[2]
        nav_action_client_request.pose.pose.orientation.x = pose[3]
        nav_action_client_request.pose.pose.orientation.y = pose[4]
        nav_action_client_request.pose.pose.orientation.z = pose[5]
        nav_action_client_request.pose.pose.orientation.w = pose[6]

        self.robot_navigate = True
        self.status = "ACTIVE"
        self.future = self.nav_action_client.send_goal_async(nav_action_client_request, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)

        canceled_task = False
        nav_timeout = 60

        navigation_time = 0
        while self.robot_navigate:
            if self.feedback:
                navigation_time = self.feedback.navigation_time.sec + self.feedback.navigation_time.nanosec / (10 ** 9)
                if navigation_time > nav_timeout:
                    canceled_task = True
                    self.cancel_navigation()
            rclpy.spin_once(self.node)

        navigation_time = self.feedback.navigation_time.sec + self.feedback.navigation_time.nanosec / (10 ** 9)
        if canceled_task:
            self.status = "CANCELED"

        return self.status, navigation_time

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            print('Goal rejected.')
            self.status = "REJECTED"
            self.robot_navigate = False
            return

        print('Goal accepted.')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print(f'Result: {result}')
        self.status = "SUCCEEDED" if result.error_code == 0 else "ABORTED"
        self.robot_navigate = False
    
    def cancel_navigation(self):
        if self.goal_handle:
            print('Canceling current navigation goal...')
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future)
            print('Navigation goal cancellation requested.')
            self.goal_handle = None 
        else:
            print('No active navigation goal to cancel.')