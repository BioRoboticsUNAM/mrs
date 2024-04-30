typedef struct parameters_
{
	float robot_x;
	float robot_y;
	float robot_theta;
	float robot_radio;
	float robot_max_advance;
	float robot_turn_angle;
	int   laser_num_sensors;
	float laser_origin;
	float laser_range;
	float laser_value;
	char  world_name[20];
	bool    noise;
	bool    run;
	float light_x;
	float light_y;
	int behavior;
	int steps;
	bool useRealRobot;
	bool useLidar;
	bool useSArray;
} parameters;

typedef struct next_position_
{
	float robot_x;
	float robot_y;
	float robot_theta;
} next_position;

typedef struct movement_
{
	float twist;
	float advance;
} movement;

typedef struct step_
{
	int node;
	float x;
	float y;	
} step;


typedef struct actions_
{
	char action_plan[50][100][300];
	int status[50][100];
	int num[50];
} Actions;



typedef struct rooms_
{
        char name[100][300];
        float x[100];
        float y[100];
        float z[100];
} Rooms;


typedef struct furniture_
{
        char name[100][300];
        char room[100][300];
        float x[100];
        float y[100];
        float z[100];
} Furniture;


typedef struct objects_
{
        char name[100][300];
        char room[100][300];
        float x[100];
        float y[100];
        float z[100];
} Objects;



