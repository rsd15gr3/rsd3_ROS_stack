# rsd3_ROS_stack
Folder containing all the nodes from the RSD group 3.

#/Mission-code
The mission/decition making part of the frobomind implementation.

#/perception_manualcontrol
A part of the HMI interface. Not really a perception, but it is implemented in the same way.

#/perception_pub_example
This is a example of how to pulish perception.
Each part (eks. line follow) must publish a bool, if its able verify where it is/what to do (eks. able to see the line).

#/action_sub_example
This is a example of how to subscripbe to mission/decition making for a action.
Subscribes to an Int32. This Int32 is compared to a actions defined value. The actions value's is defined in actions_states.h.

#/defines
Folder contating defines used by multiple packages.
