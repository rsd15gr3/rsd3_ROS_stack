This node supports HTML UI reachable at: http://www.evee.cz/sdu/rsd

For manual control of Frobit (publishing /fmCommand/cmd_vel, /fmPlan/automode, /fmSafe/deadman), you need to run:

This node:
$ roslaunch hmi_support_frobit hmi_support_frobit.launch

ROSBRIDGE:
$ roslaunch rosbridge_server rosbridge_websocket.launch


