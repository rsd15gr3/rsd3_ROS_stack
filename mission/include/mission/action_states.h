//Action state definitions /published to mission/action_state
//Control actions(CTR = Control) perception/manualcontrol_verify
#define CTR_TEST    	0
#define CTR_IDLE        1
#define CTR_BACKTRACK   2
#define CTR_MANUAL	3

#define CHARGE      	4
#define BRICK    	5
#define DELIVERY      	6
#define TRANSITION    	7
#define CELL_1      	8
#define CELL_2    	9
#define CELL_3      	10

//Box area perception/box_verify
#define BOX_CHARGE      40
#define BOX_BRICK	41
#define BOX_DOOR 	42
//GPS area perception/gps_verify
#define GPS_DOOR	43
#define GPS_LINE	44
//#define GPS_BRIDGE
//Line area perception/line_verify
#define LINE_GPS	45
#define LINE_TIPPER	46
//Tipper area perception/tipper_verify
#define TIPPER_UP      	47
#define TIPPER_DOWN    	48


