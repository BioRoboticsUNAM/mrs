/***********************************************
*                                              *
*      natural_language_node.cpp               *
*                                              *
*      Jesus Savage                            *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 12-12-2022                *
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
#include "../simrep/SimuladorRepresentation.h"
#include "simulator/NaturalLanguage.h"


// It starts the communication with the CLIPS node
int start_clips(){

 bool init_kdb = false;
 std::string file;
 std::string result;

 std::cout << "Starting CLIPS" << std::endl;


 //This functions loads initial facts and rules from a file
 // The first parameter is a file that contains the names of CLIPS files *.clp
 // The second parameter indicates with false do not start executing the loaded files
 // The third parameter is a timeout
 //file = "/src/expert_system/oracle.dat";
// file = "/src/action_planner/ViRBot_Cubes_ROS/objects_deftemplates.dat";
 file = "/src/action_planner/ViRBot_Cubes_ROS/ROS_virbot.dat";
 init_kdb = SimuladorRepresentation::initKDB(file, false, 2000);
 if(!init_kdb){
                std::cout << "CLIPS error file not found: " << file  << std::endl;
                return 0;
 }


 //Function to RESET CLIPS
 SimuladorRepresentation::resetCLIPS(true);

 //Function to print facts 
 SimuladorRepresentation::factCLIPS(true);

 //Function to print the loaded rules' names
 SimuladorRepresentation::ruleCLIPS(true);

 //Function to start running Clips
 SimuladorRepresentation::runCLIPS(true);

 //Function to asserting a fact to the clips node to check if Clips is alive
 SimuladorRepresentation::strQueryKDB("(assert (alive clips))", result, 10000);

 std::cout << "CLIPS answer: " << result << std::endl;


}



int main(int argc ,char **argv)
{
 int dummy;
 char message[300];
 char str[500];
 std::string result_str;
 int i=1;
 

 // ROS directives
 // Initializes Node Name
 ros::init(argc ,argv ,"simulator_natural_language_node");
 // Node handle declaration for communication with ROS system
 ros::NodeHandle n;

 SimuladorRepresentation::setNodeHandle(&n);
 ros::Rate r(20);


 // It starts the communication with the Clips node
 start_clips();


 printf("\n ************* Natural Language Node ***************************\n");
 printf("The commands are send to the Action Planner CLIPS node\n");
 printf("Examples: \natrans (actor robot)(obj book)(to father))\n");
 printf("ptrans (actor robot)(obj robot)(to service))\n");
 printf("ptrans (actor robot)(obj robot)(to kitchen))\n");
 printf("ptrans (actor robot)(obj book)(to bedroom))\n");
 printf("ptrans (actor robot)(obj milk)(to kitchen))\n");
 printf("ptrans (actor robot)(obj shampoo)(to service))\n");
 printf("ptrans (actor robot)(obj robot)(to mother))\n");
 printf("\nptrans (actor robot)(obj robot)(to studio))\n");
 printf("atrans (actor robot)(obj book)(to mother))\n");
 printf("attend (actor robot)(obj mother)(from studio))\n");
 printf("num-sentences 3\n");
 printf("\nptrans (actor robot)(obj robot)(to deposit))\n");
 printf("atrans (actor robot)(obj hammer)(to father))\n");
 printf("attend (actor robot)(obj father)(from kitchen))\n");
 printf("num-sentences 3\n");
 printf("\nptrans (actor robot)(obj orange)(to kitchen))\n");
 printf("state (attribute location)(obj orange)(value bedroom))\n");
 printf("\nqtrans (obj mother)(question where))\n");
 printf("qtrans (obj shampoo)(question where))\n");
 printf("qtrans (obj book)(question who))\n");
 printf("num-sentences 1\n");
 printf("\natrans (actor robot)(obj book))\n");
 printf("num-sentences 1\n");
 printf("state (attribute recipient)(value father))\n");
 printf("num-sentences 1\n");
 printf("\natrans (actor robot)(to father))\n");
 printf("num-sentences 1\n");
 printf("state (attribute object)(value book))\n");
 printf("num-sentences 1\n");

 while(1){

	 printf("-> ");
   	 dummy=scanf("%[^\n]%*c",message);
	 sprintf(str,"(assert (%s))",message);
	 i++;
	 printf("\nSend NL representation %d %s\n",i,str);
         SimuladorRepresentation::strQueryKDB(str, result_str, 10000);
         //printf("\nCLIPS answer: %d %s\n",i,result_str.c_str());

 }



}
