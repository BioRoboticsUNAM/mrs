/***********************************************
*                                              *
*      speech_generation_node.cpp              *
*                                              *
*      Jesus Savage                            *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 12-28-2022                *
*                                              *
*                                              *
************************************************/

#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <ctime>
#include <map>
#include <sstream>
#include <string>
#include "simulator/SpeechGeneration.h"



#define GENERATE_SPEECH 1


int action_data=0;
char Action_Planner[300]="test";


void speak_message(char *message){

 char str[500];
 int flg;
 static char previous_message[300] ="";

 if(strcmp(message,previous_message) != 0){

 	sprintf(str,"wine say.exe -pre '[:nk :ra200]' %s",message);
 	printf("\nSAY %s\n",str);
 	if(GENERATE_SPEECH ==1) flg=system(str);
	strcpy(previous_message,message);
 }

}




// Message callback function. This is a function is called when a topic
// message named 'action_planner_msg' is received. As an input message,
//void msgCallbackActionPlanner(const simulator::ActionPlanner::ConstPtr& msg)
void msgCallbackSpeechGeneration(const simulator::SpeechGeneration::ConstPtr& msg)
{

 char str1[300];
 char str2[300];
 char ROS_System[300];
 char action[300];
 char item[300]=" ";
 int action_goto;
 int plan_num,id;
 char message[300];


 ROS_INFO("received msg speech generation = %d", msg->stamp.sec);  // Shows the 'stamp.sec' message
 ROS_INFO("received msg speech generation = %d", msg->stamp.nsec); // Shows the 'stamp.nsec' message
 ROS_INFO("received msg speech generation = %d", msg->data);       // Shows the 'data' message
 ROS_INFO("recieve msg speech generation = %s", msg->action.c_str());      // Shows the 'action' message
 printf("received msg speech generation %s\n",msg->action.c_str());     // Shows the 'action' message
 //action_data = msg->data;
 action_data = 1;
 strcpy(Action_Planner,msg->action.c_str());
 printf("CallBack %s\n",Action_Planner);

 //ACT-PLN plan 1 goto bedroom deposit
 sscanf(Action_Planner,"%s %s",action,item);
 printf("Speech generation %s  %s\n",action,item);
 sprintf(message,"%s %s",action,item);
// speak_message(message);
 speak_message(Action_Planner);

 ros::spinOnce();

}



int main(int argc ,char **argv)
{
 int dummy;
 char message[300];
 char str[500];
 std::string result_str;
 int i=0;
 int flg;


 // ROS directives
 // Initializes Node Name
 ros::init(argc ,argv ,"simulator_speech_generation_node");
 // Node handle declaration for communication with ROS system
 ros::NodeHandle n;

 // It subscribes to the speech generation messages
 ros::Subscriber params_act = n.subscribe("speech_generation_msg", 100, msgCallbackSpeechGeneration);

 ros::Rate r(20);


 printf("\n ************* SPEECH GENERATION NODE ***************************\n");
 flg=system("wine say.exe -pre '[:nk :ra200]' This is the voice of a robot. My name is Pak Eto.");
 
 while(1){

 	 ros::spinOnce();
	 sleep(0.01);

 }



}
