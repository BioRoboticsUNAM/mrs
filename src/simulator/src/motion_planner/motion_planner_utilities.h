#include "ros/ros.h"
#include "simulator/Parameters.h"
#include "simulator/Laser_values.h"
#include "sensor_msgs/LaserScan.h"
#include "simulator/simulator_set_light_position.h"
#include "simulator/simulator_stop.h"
#include "simulator/simulator_robot_step.h"
#include "simulator/simulator_parameters.h"
#include "simulator/simulator_robot_laser_values.h"
#include "simulator/simulator_base.h"
#include "simulator/simulator_laser.h"
#include "simulator/simulator_light.h"
#include "simulator/simulator_algorithm_result.h"
#include "simulator/simulator_MoveRealRobot.h"
#include "simulator/simulator_object_interaction.h"
#include "simulator/ActionPlanner.h"
#include <random>


#include <string.h>

#define GRASP 1
#define RELEASE 0

#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4

const double mean = 0.0;
double stddev_x = 0.03;
double stddev_y = 0.03;
std::default_random_engine generator;
std::normal_distribution<double> noise_x(mean, stddev_x);
std::normal_distribution<double> noise_y(mean, stddev_y);


next_position next;
parameters params;
int new_simulation = 1;
float lasers[512];
int action_data=0;
char Action_Planner[300]="test";


float get_angle(float ang,float c,float d,float X,float Y){
	double PI=3.1415926535f;
        float x,y;
        x=c-X;
        y=d-Y;
        if((x == 0) && (y == 0)) return(0);
        if(fabs(x)<0.0001)      return((float) ((y<0.0f)? 3*PI/2  : PI/2) - ang );
        else{
                if(x>=0.0f&&y>=0.0f) return( (float)(atan(y/x)-ang) );
                else if(x< 0.0f&&y>=0.0f) return( (float)(atan(y/x)+PI-ang) );
                else if(x< 0.0f&&y<0.0f) return( (float)(atan(y/x)+PI-ang) );
                else return( (float)(atan(y/x)+2*PI-ang));
        }
}



void get_distance_theta(float x,float y,float angle,float x1,float y1,float *distance,float *theta){

 // it calculates the distance
 *distance=(float)sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
 printf("Distance: %f\n",*distance);

 // it calculates the rotation angle
 *theta=get_angle(angle,x,y,x1,y1);
 printf("rotation angle: %f\n",*theta);

}

void read_initial_state(Rooms *rooms, Objects *objects, Furniture *furniture, int *r, int *o, int *f) {

 FILE *fpr;
 char file_obj[350];
 char tmp[300];
 char name[300];
 char room[300];
 float x,y,z;
 int flg;
 int jr=1;
 int jo=1;
 int jf=1;



 strcpy(file_obj,"src/simulator/src/action_planner/ViRBot_Cubes_ROS/virbot_initial_state.txt");
 printf("\nObjects file: %s\n",file_obj);
 if((fpr=fopen(file_obj,"r")) == NULL){
	printf("File %s can not be open\n",file_obj);
 }
 else while(fscanf(fpr,"%s",tmp)==1){
	if(strcmp(tmp,"room")==0){
		flg=fscanf(fpr,"%s%f%f%f",name,&x,&y,&z);
		strcpy(rooms->name[jr],name);
		rooms->x[jr]=x;
		rooms->y[jr]=y;
		rooms->z[jr]=z;
		printf("Room %s x %f y %f z %f\n",rooms->name[jr],rooms->x[jr],rooms->y[jr],rooms->z[jr]);
		jr++;
	}
	else if(strcmp(tmp,"object")==0){
		flg=fscanf(fpr,"%s%s%f%f%f",name,room,&x,&y,&z);
		strcpy(objects->name[jo],name);
		strcpy(objects->room[jo],room);
                objects->x[jo]=x;
                objects->y[jo]=y;
                objects->z[jo]=z;
                printf("Object %s room %s x %f y %f z %f\n",objects->name[jo],objects->room[jo],objects->x[jo],objects->y[jo],objects->z[jo]);
		jo++;
	}
	else if(strcmp(tmp,"furniture")==0){
		flg=fscanf(fpr,"%s%s%f%f%f",name,room,&x,&y,&z);
		strcpy(furniture->name[jf],name);
		strcpy(furniture->room[jf],room);
                furniture->x[jf]=x;
                furniture->y[jf]=y;
                furniture->z[jf]=z;
                printf("Furniture %s room %s x %f y %f z %f\n",furniture->name[jf],furniture->room[jf],furniture->x[jf],furniture->y[jf],furniture->z[jf]);
		jf++;
	}
	else printf("Item not listed %s\n",tmp);


 }

 strcpy(rooms->name[jr],"NIL"); 
 strcpy(objects->name[jo],"NIL");
 strcpy(furniture->name[jf],"NIL");

 *r=jr;
 *o=jo;
 *f=jf;

 fclose(fpr);

}


void get_coordinates_rooms(Rooms rooms, int num, char *room, char * zone, float *x, float *y){

 int j;
 float tmpx, tmpy;

 for(j=1;j<num;j++){

	printf("room %d %s\n",j,rooms.name[j]);
	if(strcmp(rooms.name[j],room) == 0){
		*x=rooms.x[j];	
		*y=rooms.y[j];	
		break;
	}

 }

}


void modify_coordinates_objects(Objects *objects, int num, char *object, float x, float y){

 int j;
 float tmpx, tmpy;

 for(j=1;j<num;j++){

        printf("object %d %s\n",j,objects->name[j]);
        if(strcmp(objects->name[j],object) == 0){
                objects->x[j]=x;
                objects->y[j]=y;
                break;
        }

 }

}


void get_coordinates_objects(Objects objects, int num, char *object, float *x, float *y){

 int j;
 float tmpx, tmpy;

 for(j=1;j<num;j++){

        printf("object %d %s\n",j,objects.name[j]);
        if(strcmp(objects.name[j],object) == 0){
                *x=objects.x[j];
                *y=objects.y[j];
                break;
        }

 }

}


int exe_action(char *action, char *complete, float px, float py, float theta, movement *movements){

 int flg = 0;
 static int flg_initial = 1;
 float angle = 3.1416/4;
 float distance = 0.1;
 char exe_action[300];
 char room[300];
 char zone[300];
 char object[300];
 char previous_object[300];
 float x,y,z=0.0;
 char ROS_System[300];
 static Rooms rooms;
 static Objects objects;
 static Furniture furniture;
 static int r,o,f;
 char plan[300];
 int j;
 int plan_num,id;


 if(flg_initial == 1){
	read_initial_state(&rooms,&objects,&furniture,&r,&o,&f);
	flg_initial=0;
 }
 printf("Complete message %s\n",complete);

 if(strcmp(action,"goto")==0){

		//ACT-PLN plan 1 1 goto corridor deposit
                sscanf(complete,"%s %s %d %d %s %s %s",ROS_System,plan,&plan_num,&id,exe_action,room,zone);
                printf("Room %s zone %s\n",room,zone);
		get_coordinates_rooms(rooms,r,room,zone,&x,&y);
                printf("Coordinates room %s x %f y %f\n",room,x,y);

                //get_distance_theta(x,y,theta,px,py,&distance,&angle);
                //printf("COMMAND ACTION-PLANNER goto angle %f distance %f\n",angle,distance);

		// It returns the coordinates of the destination
                movements->advance = x;
                movements->twist = y;

		flg = 1; 
		//movements->twist = 3.1416/4;
                //movements->advance = 0.1;
}
else if(strcmp(action,"mv")==0){

		// ACT-PLN plan 10 3 mv shampoo 
                sscanf(complete,"%s %s %d %d %s %s",ROS_System,plan,&plan_num,&id,exe_action,object);
                printf("%s object %s\n",exe_action,object);
		get_coordinates_objects(objects,o,object,&x,&y);
                printf("Coordinates object %s x %f y %f\n",object,x,y);

                //get_distance_theta(x,y,theta,px,py,&distance,&angle);
                //printf("mv angle %f distance %f\n",angle,distance);

                movements->advance = x;
                movements->twist = y;

		flg = 1; 
		strcpy(previous_object,object);
              	printf("Previous object %s\n",previous_object);

 }
 else if(strcmp(action,"go")==0){

		// ACT-PLN plan 9 6 go freespace
                sscanf(complete,"%s %s %d %d %s %s",ROS_System,plan,&plan_num,&id,exe_action,object);
                printf("%s object %s\n",exe_action,object);
		if(strcmp(object,"freespace")==0){
			get_coordinates_objects(objects,o,previous_object,&x,&y);
			//x = x + 0.2;
			//y = y + 0.2;	
			x = x + noise_x(generator);
			y = y + noise_y(generator);	
			//modify_coordinates_objects(objects,o,previous_object,x,y);
                	//printf("New coordinates object %s x %f y %f\n",previous_object,x,y);
			//x = px + 0.0001;
			//y = py + 0.0001;	
		}
		else get_coordinates_objects(objects,o,object,&x,&y);
                printf("Coordinates object %s x %f y %f\n",object,x,y);

                //get_distance_theta(x,y,theta,px,py,&distance,&angle);
                //printf("mv angle %f distance %f\n",angle,distance);

                movements->advance = x;
                movements->twist = y;

                flg = 1;
 }
 else if(strcmp(action,"dropped")==0){

                //ACT-PLN plan 10 8 drop shampoo 
                sscanf(complete,"%s %s %d %d %s %s %f %f %f",ROS_System,plan,&plan_num,&id,exe_action,object,&x,&y,&theta);
                printf("dropped %s object %s %f %f %f\n",exe_action,object,x,y,theta);
                modify_coordinates_objects(&objects,o,object,x,y);
                printf("New coordinates object %s x %f y %f\n",object,x,y);
                get_coordinates_objects(objects,o,object,&x,&y);
                printf("Coordinates object %s x %f y %f\n",object,x,y);

                movements->advance = 0;
                movements->twist = 0;

                flg = 0;
 }
 else{
	printf("This action %s do not exist\n",action);
 }


 return flg;

}




movement generate_output(int out ,float advance ,float twist)
{

    movement output;

    switch(out) {

    case 0: // Stop
        output.advance = 0.0f;
        output.twist = 0.0f;
        //printf("STOP\n");
        break;

    case 1: // Forward
        output.advance = advance;
        output.twist = 0.0f;
        //printf("FORWARD\n");
        break;

    case 2: // backward
        output.advance = -advance;
        output.twist = 0.0f;
        //printf("BACKWARD\n");
        break;

    case 3:// Turn left
        output.advance = 0.0f;
        output.twist = twist;
        //printf("LEFT\n");
        break;

    case 4: // Turn right
        output.advance = 0.0f;
        output.twist = -twist;
        printf("RIGHT %f\n",output.twist);
        break;

    default:
        printf("Output %d not defined used ", out);
        output.advance = 0.0f;
        output.twist = 0.0f;
        break;
    }

    return(output);

}


void parametersCallback(const simulator::Parameters::ConstPtr& paramss)
{
    params.robot_x             = paramss->robot_x   ;
    params.robot_y             = paramss->robot_y   ;
    params.robot_theta         = paramss->robot_theta   ;
    params.robot_radio         = paramss->robot_radio   ;
    params.robot_max_advance   = paramss->robot_max_advance   ;
    params.robot_turn_angle    = paramss->robot_turn_angle   ;
    params.laser_num_sensors   = paramss->laser_num_sensors   ;
    params.laser_origin        = paramss->laser_origin         ;
    params.laser_range         = paramss->laser_range   ;
    params.laser_value         = paramss->laser_value   ;
    strcpy(params.world_name ,paramss -> world_name.c_str());
    params.noise               = paramss->noise   ;
    params.run                 = paramss->run   ;
    params.light_x             = paramss->light_x;
    params.light_y             = paramss->light_y;
    params.behavior            = paramss->behavior;
    params.steps               = paramss->steps;
    params.useRealRobot        = paramss->useRealRobot;
    params.useLidar            = paramss->useLidar;
    params.useSArray           = paramss->useSArray;

}


/// GUI interaction


int stop()
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_stop srv;
    client = n.serviceClient<simulator::simulator_stop>("simulator_stop");
    srv.request.stop = true;

    if ( !client.call(srv) )
        ROS_ERROR("Failed to call service simulator_stop");

    return 1;
}

bool object_interaction(int action, char name[50])
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_object_interaction srv;
    client = n.serviceClient<simulator::simulator_object_interaction>("simulator_object_interaction");
    std::string s;
    s=name;
    srv.request.name = s;
    srv.request.grasp = action;

    if( !client.call(srv) )
    {
        ROS_ERROR("Failed to call service simulator_object_interaction");
    }
    printf("%d\n",srv.response.done );
    return srv.response.done;
}


int set_light_position(float x, float y)
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_set_light_position srv;
    client = n.serviceClient<simulator::simulator_set_light_position>("simulator_set_light_position");
    srv.request.light_x = x;
    srv.request.light_y = y;

    if( !client.call(srv) )
    {
        ROS_ERROR("Failed to call service simulator_set_light_position");
    }
    return 1;
}


int print_algorithm_graph (step *steps )
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_algorithm_result srv;
    client = n.serviceClient<simulator::simulator_algorithm_result>("simulator_print_graph"); //create the client

    for(int i = 0; i < 200; i++)
        srv.request.nodes_algorithm[i] = steps[i].node;

    if( !client.call(srv) )
        ROS_ERROR("Failed to call service simulator_print_graph");

    return 1;
}

//// Light Functions

int get_light_values(float *intensity, float *values)
{
    int sensor;
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_light srv;
    client = n.serviceClient<simulator::simulator_light>("simulator_light"); //create the client
    srv.request.req = 1;

    if ( client.call(srv) )
    {
        for(int i = 0; i < 8; i++)
            values[i] = srv.response.values[i];

        sensor = 0;

        for(int i = 1; i < 8; i++)
        {
            if( values[i] > values[sensor])
                sensor = i;
        }
        *intensity = values[sensor];
    }
    else
    {
        ROS_ERROR("Failed to call service  simulator_light");
    }
}

int get_light_values_RealRobot(float *intensity, float *values)
{
    int sensor;
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_light srv;
    client = n.serviceClient<simulator::simulator_light>("simulator_light_RealRobot"); //create the client
    srv.request.req = 1;

    if ( client.call(srv) )
    {
        for(int i = 0; i < 8; i++)
            values[i] = srv.response.values[i];

        sensor = 0;

        for(int i = 1; i < 8; i++)
        {
            if( values[i] > values[sensor])
                sensor = i;
        }
        *intensity = values[sensor];
    }
    else
    {
        ROS_ERROR("Failed to call service  simulator_light_RealRobot");
    }
}

int quantize_light(float *light_values)
{
    int sensor = 0;

    for(int i = 1; i < 8; i+=2 )
    {
        if( light_values[i] > light_values[sensor] )
            sensor = i;
    }
    //printf("biggest value sensor %d %f\n",sensor,light_values[sensor]);
    if(sensor == 0)
        return 2;
    else if(sensor == 1)
        return 3;
    else if(sensor == 3)
        return 1;
    else if(sensor == 5)
        return 0;
    else if(sensor == 7)
        return 2;
    else
        return 0;
}


//////LASER Functions

int quantize_laser_noise(float *observations, int size, float laser_value  )
{
    /*
      It quantizes the inputs
    */
    int a,b,cta;
    int iz,de,salida;
    int j;

    printf(" noise laser value %f\n",laser_value);
    iz = de = salida = 0;
    if( size % 2 != 0)
    {
        a = ( size - 1 ) / 2;
        b = a + 1;
    }
    else
    {
        a = b = size / 2;
    }

    cta = 0;
    for (int i = b; i < size ; i++ ) //izquierda
    {
        if( observations[i] < laser_value  )
            cta++;
        if( cta >=  size*.4  )
        {
            iz = 2;
	    printf(" observations[%d] %f laser value %f\n",i,observations[i],laser_value);
            break;
        }
    }

    cta = 0;
    for (int i = 0; i < a ; i++ ) //derecha
    {
        if( observations[i] < laser_value  )
            cta++;
        if( cta >=  size*.4  )
        {
            de = 1;
	    printf(" observations[%d] %f laser value %f\n",i,observations[i],laser_value);
            break;
        }
    }

    return iz + de ;
}

int quantize_laser(float *observations, int size, float laser_value  )
{
    /*
      It quantizes the inputs
    */
    int a,b;
    int iz,de,salida;
    int j;



    printf(" noise laser value %f\n",laser_value);
    iz = de = salida = 0;
    if( size % 2 != 0)
    {
        a = ( size - 1 ) / 2;
        b = a + 1;
    }
    else
    {
        a = b = size / 2;
    }

    for (int i = b; i < size ; i++ ) //izquierda
    {
        if( observations[i] < laser_value  )
        {
            iz = 2;
	    printf(" observations[%d] %f laser value %f\n",i,observations[i],laser_value);
            break;
        }
    }

    for (int i = 0; i < a ; i++ ) //derecha
    {
        if( observations[i] < laser_value  )
        {
            de = 1;
	    printf(" observations[%d] %f laser value %f\n",i,observations[i],laser_value);
            break;
        }
    }

    return iz + de ;
}


// Message callback function. This is a function is called when a topic
// message named 'action_planner_msg' is received. As an input message,
void msgCallbackActionPlanner(const simulator::ActionPlanner::ConstPtr& msg)
{

 char str1[300];
 char str2[300];
 char ROS_System[300];
 char action[300];
 movement movements;
 int action_goto;
 int plan_num,id;


 ROS_INFO("received msg action planner = %d", msg->stamp.sec);	// Shows the 'stamp.sec' message
 ROS_INFO("received msg action planner = %d", msg->stamp.nsec);	// Shows the 'stamp.nsec' message
 ROS_INFO("received msg action planner = %d", msg->data); 	// Shows the 'data' message
 ROS_INFO("recieve msg action planner = %s", msg->action.c_str()); 	// Shows the 'action' message
 printf("received msg action planner %s\n",msg->action.c_str());     // Shows the 'action' message
 //action_data = msg->data;
 action_data = 1;
 strcpy(Action_Planner,msg->action.c_str());
 printf("CallBack %s\n",Action_Planner);

 //ACT-PLN plan 1 goto bedroom deposit
 sscanf(Action_Planner,"%s %s %d %d %s",ROS_System,str1,&plan_num,&id,action);

 printf("Movement action plan_num %d id %d action %s\n",plan_num,id,action);

 ros::spinOnce();
}




// Message callback function. This is a function is called when a topic
// message named 'action_planner_manipulator_msg' is received. As an input message,
void msgCallbackActionPlannerManipulator(const simulator::ActionPlanner::ConstPtr& msg)
{

 char str1[300];
 char str2[300];
 char ROS_System[300];
 char action[300];
 movement movements;
 int action_goto;


 ROS_INFO("received msg action planner manipulator = %d", msg->stamp.sec);  // Shows the 'stamp.sec' message
 ROS_INFO("received msg action planner manipulator = %d", msg->stamp.nsec); // Shows the 'stamp.nsec' message
 ROS_INFO("received msg action planner manipulator = %d", msg->data);       // Shows the 'data' message
 ROS_INFO("recieve msg action planner manipulator = %s", msg->action.c_str());      // Shows the 'action' message
 printf("received msg action planner manipulator %s\n",msg->action.c_str());     // Shows the 'action' message
 //action_data = msg->data;
 action_data = 1;
 strcpy(Action_Planner,msg->action.c_str());
 printf("CallBack %s\n",Action_Planner);

 //ACT-PLN plan 1 goto bedroom deposit
 sscanf(Action_Planner,"%s %s %s %s",ROS_System,str1,str2,action);

 if(strcmp(action,"dropped")==0){
        printf("Manipulator action %s\n",Action_Planner);
        action_goto=exe_action(action,Action_Planner,params.robot_x,params.robot_y,params.robot_theta,&movements);
 } 

 ros::spinOnce();
}




void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    /*
       This functions returns the lidars value acoording to gui settings
    */
    double PI;
    double K1;
    double theta;
    double ranges;
    double inc_angle;
    double init_angle;
    double sensors[512];
    double complete_range;
    int index;
    int num_points;
    int range_laser;
    int num_sensors;

    if(params.useLidar)
    {
        num_points = 512;
        PI = 3.1415926535;
        range_laser = 360;//240;
        complete_range = range_laser * PI / 180;
        K1 = complete_range / num_points;
        num_sensors = params.laser_num_sensors;
        ranges = params.laser_range;
        init_angle = params.laser_origin;
        inc_angle = ranges / num_sensors;
        theta = init_angle;

        for(int j = 0, k = 1 ; j < num_sensors; j++, k++)
        {
            index = int ( ( theta * 256 ) / 1.5707 ) + 256;
            lasers[j] = float( msg->ranges[index] );
            theta = k * inc_angle + init_angle;
        }
    }else
    {
        for(int j = 0 ; j < params.laser_num_sensors ; j++)
        {
            lasers[j] = float( msg->ranges[j] );
        }
    }
    


}


int get_lidar_values(float *lasers, float robot_x ,float robot_y, float robot_theta, bool useRealRobot)
{
    ros::NodeHandle n;
    ros::ServiceClient client;

    simulator::simulator_laser srv;
    client = n.serviceClient<simulator::simulator_laser>("simulator_laser_serv"); //create the client

    srv.request.robot_x = robot_x;
    srv.request.robot_y = robot_y;
    srv.request.robot_theta = robot_theta;

    if (client.call(srv))
    {
        for(int i = 0; i < 512; i++)
            lasers[i] = srv.response.sensors[i];
    }
    else
    {
        ROS_ERROR("Failed to call service simulator_robot_laser_values");
    }

    return 1;
}


/// BASE

int move_gui(float angle ,float distance ,next_position *next,float lidar_readings[512] )
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_robot_step srv;
    client = n.serviceClient<simulator::simulator_robot_step>("simulator_robot_step"); //create the client

    srv.request.theta = angle;
    srv.request.distance = distance;

    for(int i = 0; i < 512; i++ )
        srv.request.sensors[i] = lidar_readings[i];

    if (client.call(srv))
    {
        next->robot_x = srv.response.robot_x;
        next->robot_y = srv.response.robot_y;
        next->robot_theta =srv.response.theta;
    }
    else
    {
        ROS_ERROR("Failed to call service simulator_robot_step");

    }

    return 1;
}


int move_RealRobot(float theta,float distance)
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_MoveRealRobot srv;
    client = n.serviceClient<simulator::simulator_MoveRealRobot>("simulator_move_RealRobot");
    srv.request.theta = theta;
    srv.request.distance = distance;

    if (client.call(srv))
    {
        if(srv.response.done)
            printf("Robot move done \n");
        else
            printf("Robot move fail \n");
    }
    else
    {
        ROS_ERROR("Failed to call service simulator_move_RealRobot");

    }

    return 1;
}



void check_collision(float theta ,float distance ,int new_simulation,float *final_theta,float *final_distance )
{
    ros::NodeHandle n;
    ros::ServiceClient client;
    simulator::simulator_base srv;
    client = n.serviceClient<simulator::simulator_base>("simulator_base"); //create the client

    srv.request.x1 = next.robot_x;
    srv.request.y1 = next.robot_y;
    srv.request.orientation = next.robot_theta;
    srv.request.theta = theta;
    srv.request.distance = distance;
    srv.request.new_simulation =new_simulation;

    if (client.call(srv))
    {
        *final_distance = srv.response.distance;
        *final_theta = srv.response.theta;

        //printf("TTTTTdistance: %f   , req  %f \n",srv.response.distance ,distance );
    }
    else
    {
        *final_distance = 0;
        *final_theta = 0;
        ROS_ERROR("Failed to call service simulator_base");
    }

  
}


int move_robot(float theta,float advance,float lidar_readings[512] )
{
    float final_distance,final_theta;
    check_collision(theta ,advance ,new_simulation,&final_theta,&final_distance);

    move_gui(final_theta ,final_distance ,&next,lidar_readings);
    if(params.useRealRobot)
        move_RealRobot(theta,advance);
    ros::spinOnce();
    return 1;
}
