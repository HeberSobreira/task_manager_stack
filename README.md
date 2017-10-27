# Task Manager

The **Task Manager** is responsible for receiving mission assignments and for the issuing of skills that allow the fulfillment of a certain mission.

## Table of Contents

* [Installation](#installation)
* [Usage](#usage)

## <a name="installation"></a>Installation

1. Follow the Installation steps for the [robis_custom_msgs_stack](https://gitlab.inesctec.pt/heber.m.sobreira/robis_custom_msgs_stack) repository.

2. Download the [task_manager_stack]() repository to the src folder of your catkin workspace.

    ```bash
    git clone https://gitlab.inesctec.pt/CRIIS/task_manager_stack
    ```

3. Install [pytest-mock](https://github.com/pytest-dev/pytest-mock), used for testing:

    ```bash
    pip install pytest-mock
    ```

4. Build your code. Assuming your catkin workspace is located in ~/catkin_ws:

    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

## <a name="usage"></a> Usage

To initiate the Task Manager, perform:

```bash
roslaunch task_manager_server run.launch
```

In order to test it, a Skill simulation system can be deployed:

```bash
roslaunch skill_simulation_server run.launch
```

To assign a mission, perform:

```bash
roslaunch mission_assigner run.launch
```
