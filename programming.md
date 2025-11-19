# Программирование эксперимента

## Оглавление
- [Создание проекта](#создание-проекта)
- [Добавление точки входа](#добавление-точки-входа)
- [Разработка модулей](#разработка-модулей)
    - [Модуль управления Gazebo](#модуль-управления-gazebo)
    - [Модуль управления Nav2](#модуль-управления-nav2)
    - [Модуль экспериментов](#модуль-экспериментов)
    - [Добавление моделей препятствий](#добавление-моделей-препятствий)
- [Настройка launch-файла](#настройка-launch-файла)
- [Настройка проекта](#настройка-проекта)
- [Запуск!](#запуск)

## Создание проекта

Создаем в домашней директории (~) рабочую среду (workspace), в которой будем работать
```bash
mkdir experiment_ws
cd experiment_ws
colcon build
```

Создадим пакет для управления экспериментом
```bash
mkdir src
cd src
ros2 pkg create --build-type ament_python experiment_package
```

## Добавление точки входа

Прежде всего создадим наш первый исполняемый файл, из которого будем затем запустить эксперименты. Для этого создадим в папке *experiment_package* файл **main.py** и создадим в нем функцию *main()*, принимающую на вход аргументы:

```python
import sys

def main(argv=sys.argv):
    pass


if __name__ == "__main__":
    main(argv=sys.argv)
```

Для работы с топиками и сервисами ROS создадим узел (Node). Воспользуемся объектом класса **MultiThreadedExecutor** и передадим в отдельный поток вызов функции *spin()* для непрерывной обработки обратных вызовов для входящих сообщений, служб и действий. Выведем также одно сообщение в логи терминала, чтобы убедиться в корректной работе модуля.

```python

import threading
import rclpy
import sys

def main(argv=sys.argv):

    rclpy.init()
    node = rclpy.create_node('experiment')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    node.get_logger().info("Experiment started.")

    rclpy.spin(node)
    executor_thread.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(argv=sys.argv)

```

## Разработка модулей

> К этому разделу можно вернуться после сборки и запуска пакета эксперимента.

### Модуль управления Gazebo

Обратимся к планированию эксперимента. Эксперимент включает запуск Gazebo, спавн и перемещение робота, спавн и удаление препятствия. Перезапускать Gazebo и удалять робота у нас нет необходимости, эти действия будут сделаны единожды при запуске launch-файла.

Создадим модуль, который будет содержать всю логику для работы с Gazebo. Объявим наши функции и обернем их в класс *GazeboControl*.

```python
class GazeboControl:

    def __init__(self):
        pass


    def move_robot(self):
        """ move robot to new pose """
        pass


    def spawn_obstacle(self):
        """ spawn SDF model in Gazebo """
        pass


    def remove_obstacle(self):
        """ remove model from Gazebo """
        pass

```

Прежде всего необходимо понять, каким образом мы будем обращаться к Gazebo. У Gazebo существует [API для Python](https://gazebosim.org/api/transport/13/python.html), который позволяет обращаться к симулятору. Для работы с API необходимо отдельно установить его пакеты.

```bash
sudo apt install python3-gz-transport13
```
> Обратите внимание, что каждой версии симулятора соответствует своя версия API. Порядковый номер можно посмотреть в [списке релизов](https://github.com/gazebosim/gz-transport/releases).

Чтобы работать с Gazebo, в инициализации объекта класса *GazeboControl* создадим узел Node, предварительно импортировав его из *gz.transport13*

```python

from gz.transport13 import Node

class GazeboControl:

    def __init__(self):
        self.ign_node = Node()

    ...

```

Реализуем функцию перемещения робота. Для этого необходимо вызвать сервис Gazebo */world/default/set_pose*. Сервис получает на вход имя модели и позицию, куда необходимо ее переместить. Добавим в аргументы функции *move_robot()* соответствующие параметры: имя робота и позицию.

> Посмотреть типы аргументов можно, использовав команду ```gz service -i -s /world/default/set_pose```. Информацию о типе данных можно получить из ```gz msg -i  gz.msgs.Pose``` 


```python

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

class GazeboControl:

    ...

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
    ...

```

Создадим объект класса *GazeboControl* в нашем *main()* и вызовем функцию *move_robot()*, чтобы убедиться, что все работает. Переместим робота в позицию (2.0, 2.0, 0.0).

> Для проверки необходимо собрать проект, запустить мир с роботом и launch-файл экспериментов.

```python

...
from experiment_package.gazebo_control import GazeboControl

def main(argv=sys.argv):

    rclpy.init()
    node = rclpy.create_node('experiment')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    gz_control = GazeboControl()

    node.get_logger().info("Experiment started.")
    gz_control.move_robot("artbul", [2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    ...

```

Следующим шагом реализуем спавн объекта в **gazebo_control.py**. Для этого обратимся к сервису */world/default/create*. Будем передавать название модели, содержимое SDF и позицию. В коде пропишем:

```python

...
from gz.msgs10.entity_factory_pb2 import EntityFactory

class GazeboControl:

    ...
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

        return response.data
```

Реализуем удавление объекта с помощью сервиса */world/default/remove*. Будем передавать название модели.

```python

...
from gz.msgs10.entity_pb2 import Entity

class GazeboControl:

    ...
    def remove_obstacle(self, model_name):
        req = Entity()
        req.name = model_name
        req.type = 2 # model type, see https://github.com/gazebosim/gz-msgs/blob/gz-msgs11/proto/gz/msgs/entity.proto
        result, response = self.ign_node.request("/world/default/remove",
                                                req,
                                                Entity,
                                                Boolean,
                                                3000)
        print(f"delete model'{model_name}': result: {result} response: {response.data}")

        return result
```

Теперь добавим вспомогательные функции для работы с Gazebo. Добавим функцию *spawn_obstacle_by_url()*, в которой будем указывать путь до SDF-файла вместо всего содержимого файла. Будем читать файл и отправлять его содержимое функции *spawn_obstacle()*.

```python

...
from gz.msgs10.entity_pb2 import Entity

class GazeboControl:

    ...
    def spawn_obstacle_by_url(self, model_name, url, gazebo_pose):
        content_file = open(url, 'r', encoding="utf-8")
        content = content_file.read()
        content_file.close()
        return self.spawn_obstacle(model_name, content, gazebo_pose)
```

Добавим также функцию, которую будет проверять наличие модели. Это важно, поскольку мы должны быть уверены, что одно препятствие действительно удалилось, а другое заспавнилось. Используем для этого информацию из сервиса */world/default/scene/info*. Будем передавать имя модели.

```python

...
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene

class GazeboControl:

    ...
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
```

Добавим логику с проверкой существования модели в функции спавна и удаления модели. Для функции спавна будем ждать, пока создастся модель в случае успешного запроса к Gazebo. Реализуем ожидание через *time.sleep()*.

```python

...
import time

class GazeboControl:
    ...
    
    def spawn_obstacle(self, model_name, content, gazebo_pose):

        ...

        print(f"load model '{model_name}': result: {result} response: {response.data}")

        if result:
            while not self.model_is_exists(model_name):
                time.sleep(0.1)

        return response.data
```

Для функции удаления реализуем логику, при которой будем запрашивать удаление, пока модель существует.

```python

...

class GazeboControl:

    ...
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

            return result

        result, response = delete()
        if result:
            while self.model_is_exists(model_name):
                result, response = delete()
                time.sleep(0.5)
```

### Модуль управления Nav2

Снова обратимся к планированию эксперимента. Запуск навигации будет осуществлен единожды, поэтому это действие программировать не требуется. Таким образом требуется реализация следующих функций: изменение значения горизонта планирования, перезапуск навигации, локализация робота и запуск навигации робота к цели. Добавим к этим функциям также функцию **отмены навигации** (в случае, если робот застрял).

Создадим отдельный модуль **navigation_control.py**, который будет содержать всю логику для работы с Nav2.

```python
class NavigationControl:

    def __init__(self):
        pass


    def change_nav2_parameter(self):
        """ Change paramenter of controller node """
        pass


    def reset_navigation_stack(self):
        """ Turn off navigation stack """
        pass


    def startup_navigation_stack(self):
        """ Start navigation stack """
        pass


    def update_robot_pose(self):
        """ Set robot pose on the map """
        pass


    def start_robot_navigation(self):
        """ Set robot pose for navigation to target """
        pass


    def cancel_navigation(self):
        """ Cancel navigation to target """
        pass
```

Начнем с конструктора класса. Поскольку мы будем работать с топиками и сервисами ROS, нам необходимо создать узел. Поскольку мы уже в *main()* его создали, будем передавать его аргументом.

```python
class NavigationControl:

    def __init__(self, node):
        self.node = node

    ...
```

Определим функцию для изменения параметра локального планировщика. На вход *change_nav2_parameter()* будем подавать название параметра, тип параметра и значение. Для изменения параметра нам нужно обратиться к сервису узла. Поскольку мы меняем параметр узла локального планировщика через сервис, нам необходимо создать клиента, который будет обращаться к серверу. Мы можем найти имя сервера по ключевому слову controller: ```ros2 service list | grep controller```. Нам нужен сервис *set_parameters*.

Добавим в конструктор клиента и будем ожидать появление сервера.


```python

from rcl_interfaces.srv import SetParameters 

class NavigationControl:

    def __init__(self, node):
        self.node = node
        self.controller_server_cli = node.create_client(SetParameters, "/artbul/controller_server/set_parameters")
        while not self.controller_server_cli.wait_for_service(timeout_sec=1.0):
            print('Service "/artbul/controller_server/set_parameters" not available, waiting again...')
    ...
```

В функцию *change_nav2_parameter()* добавим отправку запроса и ожидание ответа.

```python

from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters 
import rclpy

class NavigationControl:
    ...
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
        else:
            print(f'Service call failed for {param_name}')

    ...
```

Проверим работу функции, вызвав ее из точки входа *main()*. Предварительно запустим робота, среду и навигацию.

> Для проверки необходимо собрать проект, запустить мир с роботом и launch-файл экспериментов.

```python

...
from experiment_package.navigation_control import NavigationControl
from rcl_interfaces.msg import ParameterType

def main(argv=sys.argv):

    rclpy.init()
    node = rclpy.create_node('experiment')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    nav_control = NavigationControl(node)

    node.get_logger().info("Experiment started.")
    nav_control.change_nav2_parameter("FollowPath.sim_time", ParameterType.PARAMETER_DOUBLE, 5.0)

    ...

```

Следующая функция - сброс навигации. Для этого мы должны обратиться к сервису управления жизненным циклом lifecycle_manager, посмотреть сервисы можно через ```ros2 service list | grep lifecycle```. В Nav2 в проекте ArtBul реализованы два отдельных жизненных цикла для элементов навигации и локализации, таким образом мы должны сбросить оба эти цикла. Для этого необходимо воспользоваться сервисом *manage_nodes*. Добавим сначала клиентов в конструктор класса:

```python

...
from nav2_msgs.srv import ManageLifecycleNodes

class NavigationControl:

    def __init__(self, node):
        ...

        self.lifecycle_manager_nav_cli = node.create_client(ManageLifecycleNodes, "/artbul/lifecycle_manager_navigation/manage_nodes")
        while not self.lifecycle_manager_nav_cli.wait_for_service(timeout_sec=1.0):
            print('Service "/artbul/lifecycle_manager_navigation/manage_nodes" not available, waiting again...')

        self.lifecycle_manager_loc_cli = node.create_client(ManageLifecycleNodes, "/artbul/lifecycle_manager_localization/manage_nodes")
        while not self.lifecycle_manager_loc_cli.wait_for_service(timeout_sec=1.0):
            print('Service "/artbul/lifecycle_manager_localization/manage_nodes" not available, waiting again...')

    ...
```

В функции сброса будем делать запрос на сервер, указав статус 3 в сообщении (RESET). Статусы можно посмотреть в [спецификации сообщения](https://docs.ros.org/en/iron/p/nav2_msgs/interfaces/srv/ManageLifecycleNodes.html).

```python
...

class NavigationControl:
    ...
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

    ...
```

Аналогично определим функцию *startup_navigation_stack()*, только будем запускать модули в обратном порядке, сначала модуль локализации, затем модуль навигации. Укажем команду 0 (STARTUP).

```python
...

class NavigationControl:
    ...
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

    ...
```

Следующим определим метод *update_robot_pose()* для задания положения робота на карте. Для этого воспользуемся топиком */initialpose* (см. ```ros2 topic list | grep initialpose```). В конструкторе зададим издателя (publisher), указав имя топика и тип данных.

```python

...
from geometry_msgs.msg import PoseWithCovarianceStamped

class NavigationControl:

    def __init__(self, node):
        ...

        self.init_pose_publisher = node.create_publisher(
            PoseWithCovarianceStamped,
            "/artbul/initialpose",
            10)
    ...
```

В функцию будем передавать позицию и публиковать сообщение.

```python
...

class NavigationControl:
    ...
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

    ...
```

Перейдем к навигации робота к цели. Как и в ROS1, в ROS2 необходимо использовать механизм *Actions* для запуска навигации. Название действия можно узнать, отфильтровав текущие действия: ```ros2 action list | grep navigate_to_pose```. Зададим клиента для вызова действия в конструкторе. Также заранее зададим переменные для хранения промежуточных значений навигации (feedback), текущего значения и флага навигации. 

```python

...
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class NavigationControl:

    def __init__(self, node):
        ...

        self.nav_action_client = ActionClient(node, NavigateToPose, '/artbul/navigate_to_pose')
        self.feedback = None
        self.robot_navigate = False
        self.status = None
    ...
```

В функцию *start_robot_navigation()* будем передавать цель. Зададим текущий статус навигации как "ACTIVE" и поднимем флаг навигации robot_navigate. Выполним запрос к серверу навигации и будем ожидать завершения. В запросе укажем колбэк для получения обратной связи от сервера. Ниже также укажем колбэк *goal_response_callback()* для получения статуса запроса. Если запрос был отклонен сервером, меняем статус навигации на завершенный и выходим из цикла.  

```python
...

class NavigationControl:
    ...
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

        navigation_time = 0
        while self.robot_navigate:
            rclpy.spin_once(self.node)

        navigation_time = self.feedback.navigation_time.sec + self.feedback.navigation_time.nanosec / (10 ** 9)

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

    ...
```

Реализуем функцию отмены навигации. Для этого нам необходимо также обратиться к серверу навигации, указав *cancel_goal_async()*.

```python
...

class NavigationControl:
    ...
    def cancel_navigation(self):
        if self.goal_handle:
            print('Canceling current navigation goal...')
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future)
            print('Navigation goal cancellation requested.')
            self.goal_handle = None # Clear the handle after cancellation
        else:
            print('No active navigation goal to cancel.')
    ...
```

Будем вызывать функцию отмены навигации в случае, если робот выполняет навигацию больше 60 секунд. Добавим проверку этого в *start_robot_navigation()* в момент получения обратной связи, содержащей информацию о времени навигации.

```python
...

class NavigationControl:
    ...
    def start_robot_navigation(self, pose):

        ...

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

    ...
```

### Модуль экспериментов

Мы реализовали модули управления симулятором и навигацией, остается только соединить это воедино. Обратимся вновь к планированию, нам необходимо организовать сохранение данных, но кроме того и чтение данных (факторов). 

Создадим отдельный модуль **experiment.py**, который будет содержать всю логику эксперимента. Объявим функции для проведения эксперимента: запуск эксперимента, запуск попытки, сброс навигации и сохранение результатов.

```python
class Experiment:

    def __init__(self):
        pass


    def reset_environment(self):
        """ Restart navigation, localization, move robot and respawn obtacles """
        pass


    def do_trial(self):
        """ Start experimental attempt """
        pass


    def save_results(self):
        """ Save experimental results to file """
        pass


    def do_experiments(self):
        """ Start experiment """
        pass

```

Сразу укажем в конструкторе параметры, которые понадобятся для работы. Это объекты для работы с навигацией, симулятором; таблица экспериментов (комбинаций факторов), путь до файла с результатами, имена столбцов csv-файла, массив с входными факторами текущей попытки.

```python
from experiment_package.gazebo_control import GazeboControl
from experiment_package.navigation_control import NavigationControl

class Experiment:

    def __init__(self, path_to_exp_plan, gz_control, nav_control, control_factors, output_path, columns_names):
        self.exp_plan = np.load(path_to_exp_plan, allow_pickle=True)
        self.gz_control: GazeboControl = gz_control
        self.nav_control: NavigationControl = nav_control
        self.control_factors = control_factors
        self.output_path = output_path
        self.columns_names = columns_names
        self.current_trial = None
    ...
```

Зададим функцию для рестарта всех модулей и сброса значений *reset_environment()*. Будем выполнять действия в следующем порядке:
- отключение навигации
- перемещение робота на стартовую позицию
- удаление старого препятствия
- спавн нового препятствия
- изменение параметра локального планировщика
- запуск навигации
- обновление позиции робота на карте  

```python
...
import time

class Experiment:

    ...
    def reset_environment(self, input_factors):
        self.nav_control.reset_navigation_stack()
        
        self.gz_control.move_robot(self.control_factors["robot_name"], self.control_factors["robot_start_gz_pose"])
        
        old_obstacle_name, new_obstacle_name, new_obstable_url = input_factors["obstacle_type"]
        
        self.gz_control.remove_obstacle(old_obstacle_name)
        self.gz_control.spawn_obstacle_by_url(new_obstacle_name, new_obstable_url, self.control_factors["obstacle_pose"])

        self.nav_control.change_nav2_parameter("FollowPath.sim_time", ParameterType.PARAMETER_DOUBLE, input_factors["planning_horizon"])
        
        self.nav_control.startup_navigation_stack()
        time.sleep(2.0)
        self.nav_control.update_robot_pose(self.control_factors["amcl_start_robot_pose"])
        time.sleep(2.0)
    ...
```

Теперь зададим функцию для выполнения попытки. На вход функции будем передавать входные факторы. Также будем сохранять текущую попытку, чтобы на следующей брать информацию о названии препятствия. Перед каждой попыткой будем сбрасывать все параметры окружения и затем начинать навигацию до цели.

```python
...
import time

class Experiment:

    ...
    def do_trial(self, trial):

        obstacle_path = trial[0]
        planning_horizon = trial[1]
        current_obstacle_type = f"wall_{int(trial[2])}"
        index = trial[3]

        prev_obstacle_type = "" if self.current_trial is None else f"wall_{int(self.current_trial[2])}"

        input_factors = {
            "obstacle_type": (prev_obstacle_type, current_obstacle_type, obstacle_path),
            "planning_horizon": planning_horizon
        }

        self.current_trial = trial

        print(f"Experiment #{index}: In progress..")

        print(f"Experiment #{index}: Reset environment..")
        self.reset_environment(input_factors)

        print(f"Experiment #{index}: Start navigation..")
        nav_status, nav_time = self.nav_control.start_robot_navigation(self.control_factors["amcl_target_robot_pose"])

        print(f"Experiment #{index}: Navigation finished with status: {nav_status}, navigation time: {nav_time}")

        response = {
            "nav_status": nav_status,
            "nav_time": nav_time
        }

        return response
    ...
```

Определим также функцию для сохранения результатов. Для этого предвательно создадим вспомогательный модуль **utils.py** и определим там 2 функции: инициализации файла и добавления строки.

```python
import csv

def save_to_csv(data, columns_names, output_path):
    with open(output_path, "a") as desc:
        writer = csv.DictWriter(desc, fieldnames=columns_names)
        writer.writerows(data)


def init_csv(columns_names, output_path):
    try:
        with open(output_path, "x") as desc:
            writer = csv.DictWriter(desc, fieldnames=columns_names)
            writer.writeheader()
    except FileExistsError:
        print(f"File '{output_path}' already exists.")

```

Будем вызывать функцию *save_to_csv()*, указав названия столбцов и путь до файла.

```python
...
from experiment_package.utils import init_csv, save_to_csv

class Experiment:

    ...
    def save_results(self, data, columns, output_path):
        save_to_csv(data, columns, output_path)
    ...
```

Перейдем к функции экспериментов. Будем создавать CSV-файл до проведения экспериментов, чтобы убедиться в наличии прав на него. Затем будем в цикле выполнять каждый из экспериментов, указанных в *self.exp_plan*. Будем вызывать функцию для проведения опыта и сохранять значения факторов и полученные значения отклика. Итоговая функция будет выглядеть следующим образом:

```python
from experiment_package.gazebo_control import GazeboControl
from experiment_package.navigation_control import NavigationControl

class Experiment:

    ...
    def do_experiments(self):
        
        init_csv(self.columns_names, self.output_path)

        for trial in self.exp_plan:
            
            index = trial[3]
            response = self.do_trial(trial)

            data = { 
                "index": index,
                "planning_horizon": trial[1],
                "obstacle_type": trial[2]
            }
            data.update(response)

            print(f"Experiment #{index}: Save results..")
            self.save_results([data], self.columns_names, self.output_path)
            print(f"Experiment #{index}: Results saved.")
    ...
```

Осталось выполнить запуск экспериментов в точке входа *main()*. Создадим объект эксперимента, менеджера управления Gazebo и Nav2 и передадим необходимые параметры.

> Для вычисления угла поворота препятствия можно использовать [конвертер углов Эйлера в кватернионы](https://www.andre-gaschler.com/rotationconverter/)

```python

...
from experiment_package.gazebo_control import GazeboControl
from experiment_package.navigation_control import NavigationControl
from experiment_package.experiment import Experiment

def main(argv=sys.argv):

    rclpy.init()
    node = rclpy.create_node('experiment')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    nav_control = NavigationControl(node)
    gz_control = GazeboControl()

    control_factors = {
        "robot_start_gz_pose": [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "amcl_start_robot_pose": [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "amcl_target_robot_pose": [7.0, 7.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "obstacle_pose": [4.0, 4.0, 0.5, 0.0, 0.0, 0.3824995, 0.9239557]
    }

    columns_names = ["index", "planning_horizon", "obstacle_type", "nav_status", "nav_time"]
    experiment = Experiment("input.npy", gz_control, nav_control, control_factors, "output.csv", columns_names)
    experiment.do_experiments()
    ...

```

### Добавление моделей препятствий

Каждый из уровней фактора ширины препятствия представляет из себя SDF-файл, описывающий параллелепипед. Вид параллелепипеда нам не важен, поэтому делаем визуальную и коллизионную модели идентичными. Для стены с толщиной 10 см и шириной 10 см получаем:

```xml
<?xml version='1.0' encoding='utf-8'?>
<sdf version="1.6">
  <model name="wall">
    <pose>4 4 0.5 0 0 0.785</pose>
    <static>true</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia> 
          <ixx>0.083</ixx>       
          <ixy>0.0</ixy>        
          <ixz>0.0</ixz>        
          <iyy>0.083</iyy>       
          <iyz>0.0</iyz>         
          <izz>0.083</izz>       
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.1 0.1 1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.1 0.1 1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```
Сохраним файл в проекте в папке *worlds* как **wall10.sdf**. Создадим поочередно аналогичные файлы с шириной препятствия 20, 30, 40 и 50 см. 

## Настройка проекта

Для настройки проекта перейдем в файл **setup.py**. В этом файле вызывается метод *setup()* с параметрами проекта. Укажем в параметре *entry_points* внутри массива исполнительных файлов с ключом *console_scripts* строку для запуска функции *main()* внутри файла **main.py**. 

В общем случае строка имеет вид: *название_исполнительного_файла = название_пакета.название_модуля:название_функции*

У нас пакет называется *experiment_package*, модуль (*py*-файл) называется **main.py** и функция называется также *main()*. Получаем

```
    ...
    entry_points={
        'console_scripts': [
            "start_experiment = experiment_package.main:main" 
        ],
    }
    ...
```

## Настройка launch-файла

Создадим в проекте папку *launch* и в ней файл **experiment.launch.py**. В этом файле указываем общий скелет:

```python
from launch import LaunchDescription

def generate_launch_description():

    ld = LaunchDescription()

	return ld
```

Добавим узел с исполняемым файлом *start_experiment*, который мы указали в файле *setup.py*. В параметрах укажем *use_sim_time* равным *True*, поскольку эксперимент будет проводиться в виртуальной среде и необходимо указать узлу использовать виртуальное время.

```python

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

	experiment = Node(
		package="experiment_package",
		executable="start_experiment",
		arguments=[],
		output='screen',
        parameters=[{'use_sim_time': True}]
	)

    ld = LaunchDescription()
    ld.add_action(experiment)

	return ld
```

В файле *setup.py* добавим launch-файл в release-версию пакета, чтобы ROS мог "увидеть" файл запуска:

```
    ...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", ['launch/experiment.launch.py'])
    ],
    ...
```

## Запуск!

Файл запуска готов. Выполняем сборку проекта: ```colcon build```

Запускаем launch-файл, указав пространство имен для терминала.
```bash
source ~/experiment_ws/install/setup.bash
ros2 launch experiment_package experiment.launch.py 
```