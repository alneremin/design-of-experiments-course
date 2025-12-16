from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.empty_pb2 import Empty
import time

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

    def spawn_obstacle(self, model_name, content, gazebo_pose):
        req = EntityFactory()
        req.sdf = content
        req.name = model_name
    
        req.pose.name = model_name
        req.pose.position.x = gazebo_pose[0]
        req.pose.position.y = gazebo_pose[1]
        req.pose.position.z = gazebo_pose[2]
        req.pose.orientation.x = gazebo_pose[3]
        req.pose.orientation.y = gazebo_pose[4]
        req.pose.orientation.z = gazebo_pose[5]
        req.pose.orientation.w = gazebo_pose[6]

        result, response = self.ign_node.request("/world/default/create",
                                                 req,
                                                 EntityFactory,
                                                 Boolean,
                                                 3000)
        print(f"load model '{model_name}': result: {result} response: {response.data}")
        if result:
            while not self.model_is_exists(model_name):
                time.sleep(0.1)
        return response.data


    def remove_obstacle(self, model_name):
        def delete():
            req = Entity()
            req.name = model_name
            req.type = 2 # model type, see https://github.com/gazebosim/gz-msgs/blob/gz-msgs11/proto/gz/msgs/entity.proto
            result, response = self.ign_node.request("/world/default/remove",
                                                    req,
                                                    Entity,
                                                    Boolean,
                                                    3000)
            print(f"delete model'{model_name}': result: {result} response: {response.data}")

            return result, response

        result, response = delete()
        if result:
            while self.model_is_exists(model_name):
                result, response = delete()
                time.sleep(0.5)
    
    
    def spawn_obstacle_by_url(self, model_name, url, gazebo_pose):
        content_file = open(url, 'r', encoding="utf-8")
        content = content_file.read()
        content_file.close()
        return self.spawn_obstacle(model_name, content, gazebo_pose)
    
    def model_is_exists(self, model_name):
        req = Empty()
        result, response = self.ign_node.request("/world/default/scene/info",
                                                 req,
                                                 Empty,
                                                 Scene,
                                                 3000)

        print(f"model '{model_name}' is exists: result: {result} response: {response.name}")

        if result:
            for model in response.model:
                if model.name == model_name:
                    return True

        return False