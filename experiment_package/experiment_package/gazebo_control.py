import time

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene


class GazeboControl:

    def __init__(self):
        self.ign_node = Node()

    def move_robot(self, model_name, gazebo_pose):
        """ move robot to new pose """

        # создадим объект Pose для отправки запроса в Gazebo
        req = Pose()
        req.name = model_name
        req.position.x = gazebo_pose[0]
        req.position.y = gazebo_pose[1]
        req.position.z = gazebo_pose[2]
        req.orientation.x = gazebo_pose[3]
        req.orientation.y = gazebo_pose[4]
        req.orientation.z = gazebo_pose[5]
        req.orientation.w = gazebo_pose[6]

        result, response = self.ign_node.request("/world/default/set_pose",
                                                 req,
                                                 Pose,
                                                 Boolean,
                                                 3000)

        print(f"move_robot: result: {result} response: {response.data}")
        return result
