//Action state definitions /published to mission/action_state
//Control actions(CTR = Control) perception/manualcontrol_verify
#define CTR_TEST    	0
#define CTR_IDLE        1
#define CTR_BACKTRACK   2
#define CTR_MANUAL	3
//Box area perception/box_verify
#define BOX_CHARGE      4
#define BOX_BRICK	5
#define BOX_DOOR 	6
//GPS area perception/gps_verify
#define GPS_DOOR	7
#define GPS_LINE	8
//#define GPS_BRIDGE
//Line area perception/line_verify
#define LINE_GPS	9
#define LINE_TIPPER	10
//Tipper area perception/tipper_verify
#define TIPPER_UP      	11
#define TIPPER_DOWN    	12
//Relative movements inside robot cell
#define TURN90LEFT	13
#define TURN90RIGHT	14
#define TURN180	15