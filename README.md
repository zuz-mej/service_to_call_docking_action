# send_to_dock

Package waiting for request and sending robot to dock.

## Launch File

* `send_to_dock.launch.py`: Launches a node that ...

## Configuration File

* `send_to_dock.yaml`: Defines the following parameters connected to docking:
  * `dock_type` (e.g. `dock_type: charging_dock`)
  * `navigate_to_staging_pose` (e.g. `navigate_to_staging_pose: true`)
  * `dock_id` (e.g. `dock_id: {{DOCK_NAME}}`)

## ROS Nodes

### send_to_dock_node

...
