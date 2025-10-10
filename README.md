# send_to_dock

Package waiting for request and sending a robot to a dock.

## Launch File

* `send_to_dock.launch.py`: Launches a node `send_to_dock_node` which sends a robot to a docking station.

## Configuration File

* `send_to_dock.yaml`: Defines parameters for `send_to_dock_node`.

## ROS Nodes

### send_to_dock_node

Node `send_to_dock_node` creates service server SetBool `send_robot_to_dock` and based on its value it either sends a robot to a dock or stops the docking action.

**Service server**

* `send_robot_to_dock` [*std_srvs/SetBool*]: Receives requests for docking or stopping the docking action.
  * If *std_srvs/SetBool: true* then the node sends request of docking with specified parameters to the action client.
  * If *std_srvs/SetBool: false* then the node sends request of cancelling current docking action.

**Action client**

* `dock_robot` [*nav2_msgs/DockRobot*]: Docks robot or stops docking depending on the request sent by the service server `send_robot_to_dock`.

**Parameters**

* `dock_type` [*string*, default: "charging_dock"]: Type of the dock the robot navigates to.
* `navigate_to_staging_pose` [*bool*, default: true]: Whether the robot has to use navigation stack to dock or not.
* `dock_id` [*string*, default: "main"]: Name of the docking station.
