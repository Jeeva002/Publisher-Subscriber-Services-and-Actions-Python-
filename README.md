
# ROS Publisher-Subscriber, Services, and Actions (Python)

This project demonstrates the **Publisher-Subscriber**, **Service**, and **Action** concepts in **ROS** using **Python**. It includes the following nodes:
1. **Publisher Node**: Publishes messages to a topic.
2. **Subscriber Node**: Subscribes to the topic and processes the received messages.
3. **Service Server Node**: Provides a simple service.
4. **Service Client Node**: Requests a service from the service server.
5. **Action Server Node**: Performs long-running tasks with feedback.
6. **Action Client Node**: Sends goals to the action server and receives feedback.

## **Features**
- **Publisher-Subscriber**: Basic communication between nodes via topics.
- **Service**: Synchronous communication between nodes with request-response.
- **Action**: Asynchronous communication with feedback for long-running tasks.

## **Folder Structure**
```
ros_publisher_subscriber_action_service_python/
├── package.xml                  # ROS package configuration file
├── src/
│   ├── publisher.py             # Publisher node implementation
│   ├── subscriber.py            # Subscriber node implementation
│   ├── service_server.py        # Service server node implementation
│   ├── service_client.py        # Service client node implementation
│   ├── action_server.py         # Action server node implementation
│   └── action_client.py         # Action client node implementation
└── README.md                    # This documentation file
```

## **Setup Instructions**

### **1. Install ROS**
Ensure that ROS is installed. You can follow the installation instructions [here](http://wiki.ros.org/ROS/Installation).

### **2. Create a Catkin Workspace**
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_create_pkg ros_publisher_subscriber_action_service_python std_msgs rospy actionlib
cd ~/catkin_ws
catkin_make
```

### **3. Compile the Project**
```bash
cd ~/catkin_ws
catkin_make
```

### **4. Run the Nodes**
Open multiple terminals to run the nodes:

- **Terminal 1 (Publisher)**:
```bash
source devel/setup.bash
rosrun ros_publisher_subscriber_action_service_python publisher.py
```

- **Terminal 2 (Subscriber)**:
```bash
source devel/setup.bash
rosrun ros_publisher_subscriber_action_service_python subscriber.py
```

- **Terminal 3 (Service Server)**:
```bash
source devel/setup.bash
rosrun ros_publisher_subscriber_action_service_python service_server.py
```

- **Terminal 4 (Service Client)**:
```bash
source devel/setup.bash
rosrun ros_publisher_subscriber_action_service_python service_client.py
```

- **Terminal 5 (Action Server)**:
```bash
source devel/setup.bash
rosrun ros_publisher_subscriber_action_service_python action_server.py
```

- **Terminal 6 (Action Client)**:
```bash
source devel/setup.bash
rosrun ros_publisher_subscriber_action_service_python action_client.py
```

## **Dependencies**
- ROS Noetic or Melodic
- Python 2.7 or Python 3.x
- **rospy** (ROS Python client library)
- **actionlib** (for Actions)
- **std_msgs** (for messages)

