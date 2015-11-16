This node enables the HMI to control Mobile Robot (Frobit) and its Tipper

HMI also reachable at: http://www.evee.cz/sdu/rsd

subscribing:
- ui/str_control_frobit

publishing:
- /fmPlan/automode      (on change, Frobit's automode state, IntStamped)
- /fmCommand/cmd_vel    (20Hz if automode=0, Frobit's manual control, TwistStamped)
- /ui/tipper_automode   (on change, Tipper's automode state, BoolStamped)
- /ui/tipper_position   (on change, Tipper's manul control, FloatStamped : 0.0 .. bottom, 1.0 .. top)

Run:
$ roslaunch hmi_support_frobit hmi_support_frobit.launch

ROSBRIDGE:
$ roslaunch rosbridge_server rosbridge_websocket.launch


