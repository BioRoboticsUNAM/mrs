#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "ros/ros.h"
#include <ros/package.h>
#include "simulator/Parameters.h"
#include "../utilities/simulator_structures.h"
#include "simulator/simulator_find_obj.h"
#include "../simrep/SimuladorRepresentation.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
//#include "simulator/ActionPlanner.h"
#include "simulator/ActionPlannerFindObject.h"
#include "simulator/FindObject.h"
#include "random_simulation.h"
#include "prb_failure_actions.h"



ros::Publisher find_object_msg;

/*bool find_object(simulator::simulator_find_obj::Request  &req ,simulator::simulator_find_obj::Response &res)
{

    char str[300];
    sprintf(str,"(assert (answer %s command %s %s %f %f %f %s 1))",req.ROS_System.c_str(),req.action.c_str(),req.object.c_str(),req.x,req.y,req.z,req.arm.c_str());
    printf("\nSend fact %s\n",str);
    res.answer = str;

    return true;
}
*/




parameters params_act;
bool find_object(char *Action_Planner){

 int action_data=0;
 static int cta_steps = 0;
 simulator::FindObject obj_msgs;
 static int flg_init = 1;

 //cta_steps++;
 //cta_steps= cta_steps % 4;

 if( params_act.run ){
 
 	cta_steps=success(PRB_ERROR_FIND_OBJECT);
 	//sleep(0.5);
 	sleep(EXE_TIME_FIND_OBJECT);
 	printf("Finding object %s execution time %f success action %d\n",Action_Planner,EXE_TIME_FIND_OBJECT,cta_steps);

 	if(cta_steps == 1) 
	 	printf("object found %s\n",Action_Planner);
 	else
	 	printf("object not found %s\n",Action_Planner);

 	obj_msgs.stamp = ros::Time::now();                   // Save current time in the stamp of 'msg'
 	obj_msgs.data = action_data;                        // Save the the 'count' value in the data of 'msg'
 	obj_msgs.data = cta_steps;                        // Save the the 'count' value in the data of 'msg'
 	obj_msgs.find=Action_Planner;
 	//ROS_INFO("FO send msg sec = %d", obj_msgs.stamp.sec);       // Print the 'stamp.sec' message
 	//ROS_INFO("FO send msg nsec = %d", obj_msgs.stamp.nsec);      // Print the 'stamp.nsec' message
 	//ROS_INFO("FO send msg cta_steps = %d", obj_msgs.data);            // Print the 'data' message
 	find_object_msg.publish(obj_msgs);                // Publishes motion_planner message
 	//ROS_INFO("send msg find-object = %s", obj_msgs.find.c_str());           // Print the 'data' message
 	ros::spinOnce();

 }

}


// Message callback function. This is a function is called when a topic
// message named 'action_planner_find_object_msg' is received. As an input message,
void msgCallbackActionPlanner(const simulator::ActionPlannerFindObject::ConstPtr& msg)
{

 char str1[300];
 char str2[300];
 char ROS_System[300];
 char action[300];
 char object[300];
 movement movements;
 int action_goto;
 char Action_Planner[300]="test";
 int plan_num,id;



 ROS_INFO("received msg action planner = %d", msg->stamp.sec);  // Shows the 'stamp.sec' message
 ROS_INFO("received msg action planner = %d", msg->stamp.nsec); // Shows the 'stamp.nsec' message
 ROS_INFO("received msg action planner = %d", msg->data);       // Shows the 'data' message
 ROS_INFO("recieved msg action planner = %s", msg->action.c_str());      // Shows the 'action' message
 printf("\nReceived msg action planner %s\n",msg->action.c_str());     // Shows the 'action' message
 //action_data = msg->data;
 //action_data = 1;
 strcpy(Action_Planner,msg->action.c_str());
 //printf("CallBack %s\n",Action_Planner);

 //ACT-PLN plan 11 2 find-object perfume 
 sscanf(Action_Planner,"%s %s %d %d %s %s",ROS_System,str1,&plan_num,&id,action,object);
 printf("num_plan %d id %d action %s object %s",plan_num,id,action,object);


 if(strcmp(action,"find-object")==0){
        printf("\nFIND-OBJECT %s\n",Action_Planner);
	find_object(Action_Planner);
 }

 ros::spinOnce();
}
       
void paramCallback(const simulator::Parameters::ConstPtr& paramss)
{
    params_act.robot_x             = paramss->robot_x   ;
    params_act.robot_y             = paramss->robot_y   ;
    params_act.robot_theta         = paramss->robot_theta   ;
    params_act.robot_radio         = paramss->robot_radio   ;
    params_act.robot_max_advance   = paramss->robot_max_advance   ;
    params_act.robot_turn_angle    = paramss->robot_turn_angle   ;
    params_act.laser_num_sensors   = paramss->laser_num_sensors   ;
    params_act.laser_origin        = paramss->laser_origin         ;
    params_act.laser_range         = paramss->laser_range   ;
    params_act.laser_value         = paramss->laser_value   ;
    strcpy(params_act.world_name ,paramss -> world_name.c_str());
    params_act.noise               = paramss->noise   ;
    params_act.run                 = paramss->run   ;
    params_act.light_x             = paramss->light_x;
    params_act.light_y             = paramss->light_y;
    params_act.behavior            = paramss->behavior;
    params_act.steps               = paramss->steps;
    params_act.useRealRobot        = paramss->useRealRobot;
    params_act.useLidar            = paramss->useLidar;
    params_act.useSArray           = paramss->useSArray;

}




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simulator_find_obj");
    ros::NodeHandle n;
    SimuladorRepresentation::setNodeHandle(&n);
    // Declare publisher, create publisher find_object_msg 
    //ros::ServiceServer find_objects_msg = n.advertiseService("simulator_find_obj", find_object);
    //ros::ServiceServer service = n.advertiseService("simulator_find_obj", find_object);

    //ros::Publisher find_object_msg  = n.advertise<simulator::FindObject>("find_object_msg", 100);
    find_object_msg  = n.advertise<simulator::FindObject>("find_object_msg", 100);
    simulator::FindObject msg_obj;

    // It subscribes to the action planner messages
    //ros::Subscriber params_act = n.subscribe("action_planner_msg", 100, msgCallbackActionPlanner);
    ros::Subscriber params = n.subscribe("action_planner_find_object_msg", 100, msgCallbackActionPlanner);

    // It subscribes to the GUI messages
    ros::Subscriber params_act = n.subscribe("simulator_parameters_pub",100, paramCallback);


    ros::spin();
    return 0;
}

