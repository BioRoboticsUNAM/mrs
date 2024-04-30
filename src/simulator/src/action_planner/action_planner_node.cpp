/***********************************************
*                                              *
*      action_planner_node.cpp                 *
*                                              *
*      Jesus Savage                            *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 2022                      *
*                                              *
*                                              *
************************************************/



#include "ros/ros.h"
#include "../utilities/simulator_structures.h"
#include "../simrep/SimuladorRepresentation.h"
#include "../motion_planner/motion_planner_utilities.h"
#include "action_planner.h"
#include "simulator/ActionPlannerFindObject.h"
#include "simulator/ActionPlannerManipulator.h"
#include "simulator/SpeechGeneration.h"




int main(int argc ,char **argv)
{
    int i,j=0,k,num_actions=0;
    int test = 1;
    movement movements;
    char path[200];
    int flg_once=1;
    int count = 1;
    Actions plan;
    std::string ss;
    char exe_action[300];
    char string1[300];
    char string2[500];
    char ROS_System[300];
    char object[300];
    char answer[500];
    int plan_no_executed[300];
    int plan_no_executed_id[300];
    int num_no_exe=0;
    int result=0;
    char command[100],object_res[100];
    int plan_num,id;
    std::string result_str;
    char str[300];
    char place[300];
    int success=1;
    int num_plan=1;
    char attribute[300];
    char attribute1[300];
    char attribute2[300];



    // ROS directives
    // Initializes Node Name
    ros::init(argc ,argv ,"simulator_action_planner_node");
    // Node handle declaration for communication with ROS system
    ros::NodeHandle n;
    //ros::Rate loop_rate(10);

    // It subscribes to the following topics: GUI, laser sensor, motion planner and manipulator
    // It subscribes to the GUI messages
    ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub",100, paramCallback);
    // It subscribes to the laser simulator messages
    ros::Subscriber sub = n.subscribe("/scan", 10, laserCallback);
    // It subscribes to the motion_planner messages
    ros::Subscriber params_mot = n.subscribe("motion_planner_msg", 100, msgCallbackMotionPlanner);
    // It subscribes to the find_object messages
    ros::Subscriber params_obj = n.subscribe("find_object_msg", 100, msgCallbackFindObject);
    // It subscribes to the manipulator messages
    //ros::Subscriber params_manipulator = n.subscribe("manipulator_msg", 100, msgCallbackManipulator);


    // Declare publisher, create publisher action_planner_msg 
    ros::Publisher action_planner_msg  = n.advertise<simulator::ActionPlanner>("action_planner_msg", 100);
    simulator::ActionPlanner msg;
    simulator::ActionPlanner msg1;

    ros::Publisher action_planner_find_object_msg  = n.advertise<simulator::ActionPlannerFindObject>("action_planner_find_object_msg", 100);
    simulator::ActionPlannerFindObject msg_action_planner_find_object;

    ros::Publisher action_planner_manipulator_msg  = n.advertise<simulator::ActionPlannerManipulator>("action_planner_manipulator_msg", 100);
    simulator::ActionPlannerManipulator msg_action_planner_manipulator;

    ros::Publisher speech_generation_msg  = n.advertise<simulator::SpeechGeneration>("speech_generation_msg", 100);
    simulator::SpeechGeneration msg_speech_generation;


    SimuladorRepresentation::setNodeHandle(&n);
    ros::Rate r(20);



   // it sets the environment's path
    strcpy(path,"./src/simulator/src/data/");
  



    printf("\n\n             PLANS EXECUTER %d \n________________________________\n",j++);

    while( ros::ok()  )
    {

     	ros::spinOnce();
        //printf("\n params run %d",params_act.run);
        //printf(" params behavior %d\n",params_act.behavior);

     while( params_act.run )
     {


     	ros::spinOnce();

	switch ( params_act.behavior) {

		 case 4:
			if(j > 1000){
                    		printf(" ******* SELECTION 4 %d *******\n",j++);
				j=0;
			}

			break;

		 case 10:

     			ros::spinOnce();

			while(flg_once){
     			
			    ros::spinOnce();
			    num_plan=num_plan+num_actions;
               		    printf(" ******* Waiting for a new plan to be executed %d *******\n",num_plan);
                       	    num_actions=action_planner(params_act.robot_x, params_act.robot_y,params_act.robot_theta,&plan,num_plan);
			    //flg_once=0;

			    printf("Num. accions %d\n",num_actions);
			    for(i=num_plan; i< num_plan + num_actions;i++){
					printf("\nPLAN to be executed: %d\n",i);
					for(k=1; k<= plan.num[i];k++){
                        			printf("	Subplan: %d %d %s\n",i,k,plan.action_plan[i][k]);
                			}
			    }


		  	    for(i=num_plan; i< num_plan + num_actions;i++){

				num_no_exe=0;
				printf("\nExecuting action: %d\n",i);
				for(k=1; k<= plan.num[i];k++){
		                        printf("\nExecuting: %d %s\n",k,plan.action_plan[i][k]);

					msg.stamp = ros::Time::now();	 		// Save current time in the stamp of 'msg'
					msg.data = k;					// Save the the 'count' value in the data of 'msg'
					ss=plan.action_plan[i][k];	
					//ss << "action planner " << k;
					msg.action=ss;
					flg_msg = 0;

					sscanf(plan.action_plan[i][k],"%s %s %d %d %s",ROS_System,string1,&plan_num,&id,exe_action);
					printf("exe_action %d %d %s\n",plan_num,id,exe_action);
					success=1;


					// GO_TO action
					if(strcmp(exe_action,"goto")==0 ) {
						count=0;
						action_planner_msg.publish(msg);	 	// Publishes 'msg' message
						sscanf(plan.action_plan[i][k],"%s %s %d %d %s %s",ROS_System,string1,&plan_num,&id,exe_action,place);
						sprintf(string2,"%s %s",exe_action,place);
						msg1.action=string2;
						speech_generation_msg.publish(msg1);	 	// Publishes 'msg' message
                                                sleep(2.00);
                                               	ros::spinOnce();
						flg_msg_navigation = 0;

                                        	while(flg_msg_navigation == 0){

					  	    if( params_act.run ){			// Check if the simulation continues

							printf("Waiting for an answer from motion planner go_to command %d, flg_message_navigation %d\n", 
										count,flg_msg_navigation);
                                                	sleep(2.00);

                                                        msg.data = count;       // Save the the 'count' value in the data of 'msg'
                                                        count++;
                                                        if(count > 9){
                                                                printf("ACTION %s could not be executed\n",exe_action);
                                                                num_no_exe++;
                                                                plan_no_executed[num_no_exe]=i;
                                                                plan_no_executed_id[num_no_exe]=k;
								flg_msg_navigation=1;
								success=0;
                                                        }
                                                        else{
							       printf("Sending message to go_to %d\n",count);
                                                               if(flg_msg_navigation == 0) action_planner_msg.publish(msg); 
                                               		       ros::spinOnce();
							       printf("After sending message to go_to %d\n",count);
                                                        }
						   }

                                                   ros::spinOnce();
					           //printf("waiting for the simulation to continue for go_to %d\n",count);
                                              }

					      sprintf(string2,"arrived %s",place);
					      msg1.action=string2;
					      speech_generation_msg.publish(msg1);	 	// Publishes 'msg' message
                                              sleep(2.00);
					      /*
					      if(success == 1){
							printf("\ngoto executed correctly\n");
					      }
					      else{
							 printf("\ngoto not executed\n");
					      }
					      sprintf(str,"(assert (plans %d %d %d %s))",i,k,success,exe_action);
    					      printf("\nSend plan result %s\n",str);
    					      SimuladorRepresentation::strQueryKDB(str, result_str, 10000);
    					      printf("\nCLIPS answer: %d %s\n",i,result_str.c_str());
					      */

					}

					// MV action, the robot gets close to an object or person
					else if(strcmp(exe_action,"mv")==0){

						flg_msg_navigation = 0;
						count=0;
						action_planner_msg.publish(msg);	 	// Publishes 'msg' message
                                                ros::spinOnce();

                                                while(flg_msg_navigation == 0){

						   if( params_act.run ){                 // Check if the simulation continues
							printf("Waiting for an answer from motion planner mv command, flg_message_navigation %d\n", 
											flg_msg_navigation);
                                                        sleep(2.00);

							msg.data = count;       // Save the the 'count' value in the data of 'msg'
                                                        count++;
                                                        if(count > 9){
                                                                printf("ACTION %s could not be executed\n",exe_action);
                                                                num_no_exe++;
                                                                plan_no_executed[num_no_exe]=i;
                                                                plan_no_executed_id[num_no_exe]=k;
                                                                flg_msg_navigation=1;
								success=0;
                                                        }
                                                        else{
							       printf("Sending message to mv %d\n",count);
                                                               if(flg_msg_navigation == 0) action_planner_msg.publish(msg);
							       ros::spinOnce();
							       printf("After sending message to mv %d\n",count);
                                                        }
                                                    }

                                                    ros::spinOnce();
					            //printf("waiting for the simulation to continue for mv %d\n",count);
                                          	}
                                        }

					// GO action, the robot gets close to a region
					else if(strcmp(exe_action,"go")==0){

						flg_msg_navigation = 0;
						count=0;
                                                action_planner_msg.publish(msg);                // Publishes 'msg' message
						sscanf(plan.action_plan[i][k],"%s %s %d %d %s %s",ROS_System,string1,&plan_num,&id,exe_action,place);
						sprintf(string2,"%s %s",exe_action,place);
						msg1.action=string2;
						speech_generation_msg.publish(msg1);	 	// Publishes 'msg1' message to be spoken
                                                sleep(1.00);
						ros::spinOnce();

                                                while(flg_msg_navigation == 0){

						   if( params_act.run ){                 // Check if the simulation continues

                                                	printf("Waiting for an answer from motion planner go command, flg_message_navigation %d\n", 
                                                                                        flg_msg_navigation);
                                                        sleep(2.00);
							msg.data = count;       // Save the the 'count' value in the data of 'msg'
                                                        count++; 

							if(count > 9){
                                                                flg_msg_navigation = 1;
                                                                printf("ACTION %s could not be executed\n",exe_action);
                                                                num_no_exe++;
                                                                plan_no_executed[num_no_exe]=i;
                                                                plan_no_executed_id[num_no_exe]=k;
                                                                flg_msg_navigation=1;
								success=0;
                                                        }
                                                        else{
                                                               printf("Sending message to go %d\n",count);
                                                               if(flg_msg_navigation == 0) action_planner_msg.publish(msg);
                                                               ros::spinOnce();
                                                               printf("After sending message to go %d\n",count);
                                                        }
                                                    }
                                                    
                                                    ros::spinOnce();
					      	    sprintf(string2,"arrived %s",place);
					      	    msg1.action=string2;
					      	    speech_generation_msg.publish(msg1);	 	// Publishes 'msg' message
                                              	    sleep(2.00);
                                                    //printf("waiting for the simulation to continue for go %d\n",count);
                                                }
                                        }



 					else if(strcmp(exe_action,"grab")==0){
						sscanf(plan.action_plan[i][k],"%s %s %d %d %s %s",ROS_System,string1,&plan_num,&id,exe_action,object);
						sprintf(string2,"%s %s",exe_action,object);
						msg1.action=string2;
						speech_generation_msg.publish(msg1);	 	// Publishes 'msg' message
                                                sleep(2.00);
						result=send_grab_drop(exe_action,object,answer);
						printf("\nAnswer MANIPULATOR %d %s\n",result,answer);
						sscanf(answer,"%s %s %d",command,object_res,&result);
						printf("command %s object %s result %d\n",command,object_res,result);
						if(result == 0){
							printf("object not grasped\n");
							result=wait_answer_service(&num_no_exe,plan_no_executed,i,plan_no_executed_id,k,exe_action,object);
							success=0;
						}
						flg_msg = result;
                                                flg_msg = 1;
                                                ros::spinOnce();
						sprintf(string2,"%s %s %d %s",ROS_System,string1,j,answer);
						msg.action=string2;
                                        	//action_planner_manipulator_msg.publish(msg);   // Publishes the grab command
						sprintf(string2,"%sed %s",exe_action,object);
						msg1.action=string2;
						speech_generation_msg.publish(msg1);	 	// Publishes 'msg' message
                                                sleep(2.00);
					}	
					else if(strcmp(exe_action,"drop")==0){
                                                sscanf(plan.action_plan[i][k],"%s %s %d %d %s %s",ROS_System,string1,&plan_num,&id,exe_action,object);
						sprintf(string2,"%s %s",exe_action,object);
						msg1.action=string2;
						speech_generation_msg.publish(msg1);	 	// Publishes 'msg' message
                                                sleep(1.00);
                                                send_grab_drop(exe_action,object,answer);
                                                printf("\nAnswer MANIPULATOR %s\n",answer);
						ROS_INFO("send msg = %d", msg.stamp.sec);       // Print the 'stamp.sec' message
                                        	ROS_INFO("send msg = %d", msg.stamp.nsec);      // Print the 'stamp.nsec' message
                                        	ROS_INFO("send msg = %d", msg.data);            // Print the 'data' message
						sprintf(string2,"%s %s %d %s",ROS_System,string1,j,answer);
						msg.action=string2;
                                        	//ROS_INFO("send DROP position = %s", msg.action.c_str());           // Print the 'data' message
						//sleep(0.5);
                                        	//action_planner_manipulator_msg.publish(msg);   // Publishes to motion planner to update the dropped position
                                                flg_msg = 1;
						sprintf(string2,"%sed %s",exe_action,object);
						msg1.action=string2;
						speech_generation_msg.publish(msg1);	 	// Publishes 'msg' message
                                                sleep(2.00);
                                        }

					// FIND-OBJECT action
					else if(strcmp(exe_action,"find-object")==0){
                                                flg_msg = 0;
						count=0;

						printf("Sending message to find-object %d\n",count);
                                                action_planner_find_object_msg.publish(msg);                // Publishes 'msg' message

                                                sscanf(plan.action_plan[i][k],"%s %s %d %d %s %s",ROS_System,string1,&plan_num,&id,exe_action,object);
						action_planner_manipulator_msg.publish(msg); 
						sprintf(string2,"%s %s",exe_action,object);
						msg1.action=string2;
						speech_generation_msg.publish(msg1);	 	// Publishes 'msg' message
                                                sleep(2.00);
                                               	ros::spinOnce();

						while(flg_msg == 0){

						   if( params_act.run ){                       // Check if the simulation continues
                                                        printf("Waiting for an answer from find_object %d, flg_message_navigation %d\n",
                                                                                count,flg_msg);

							msg.data = count;				// Save the the 'count' value in the data of 'msg'
                                                	sleep(LIMIT_EXE_TIME_FIND_OBJECT);
							count++;
							if(count > INTENTS_FIND_OBJECT+1){
							        flg_msg = 1;	
								printf("ACTION %s could not be executed\n",exe_action);
    								num_no_exe++;
                                                                plan_no_executed[num_no_exe]=i;
                                                                plan_no_executed_id[num_no_exe]=k;
								success=0;
							}
							else{
								printf("Sending message to find-object %d\n",count);
                                                		if(flg_msg == 0) action_planner_find_object_msg.publish(msg); // Publishes 'msg' message
								ros::spinOnce();
                                                                printf("After sending message to find-object %d\n",count);

							}

                                        	}


						ros::spinOnce();
                                                //printf("waiting for the simulation to continue for find-object %d\n",count);
					   }
					   sprintf(string2,"found %s",object);
					   msg1.action=string2;
					   speech_generation_msg.publish(msg1);	 	// Publishes 'msg' message
                                           sleep(2.00);

                                        }

					// ASK action
                                        else if(strcmp(exe_action,"ask")==0){
                                                flg_msg = 0;
                                                count=0;


                                                sscanf(plan.action_plan[i][k],"%s %s %d %d %s %s %s",ROS_System,string1,&plan_num,&id,exe_action,object,attribute);
                                                //sprintf(string2,"Please tell me, in which %s is the %s",attribute,object);
                                                sprintf(string2,"Please tell me, in which %s is the %s",attribute,object);
                                                msg1.action=string2;
                                                printf("Sending message to ask a question %s\n",string2);
                                                speech_generation_msg.publish(msg1);            // Publishes 'msg' message
                                                sleep(2.00);
                                                ros::spinOnce();

                                        }

					// ANSWER action
					else if(strcmp(exe_action,"answer")==0){

                                                flg_msg = 0;
                                                count=0;


                                                sscanf(plan.action_plan[i][k],"%s %s %d %d %s %s %s %s",ROS_System,string1,&plan_num,&id,exe_action,object,attribute1,attribute2);
                                                //sprintf(string2,"Please tell me, in which %s is the %s",attribute,object);
                                                sprintf(string2,"The %s is in the %s in the %s",object,attribute1,attribute2);
                                                msg1.action=string2;
                                                printf("Sending message to ask a question %s\n",string2);
                                                speech_generation_msg.publish(msg1);            // Publishes 'msg' message
                                                sleep(2.00);
                                                ros::spinOnce();

                                        }


					else flg_msg = 1;

					//printf("Waiting for an answer flg_message %d\n", flg_msg);
					//while(flg_msg == 0){
						//ros::spinOnce();
						//sleep(0.01);
					//}
					// It notifies to the action planner in CLIPS that action plan number was executed	
					//send_clips(k);

					 if(success == 1){
                                                        printf("\nAction %s executed correctly\n",exe_action);
                                         }
                                         else{
                                                         printf("\nAction %s not executed\n",exe_action);
                                         }
                                         sprintf(str,"(assert (plans %d %d %d %s))",i,k,success,exe_action);
                                         printf("\nSend plan result %s\n",str);
                                         SimuladorRepresentation::strQueryKDB(str, result_str, 10000);
                                         printf("\nCLIPS answer: %d %s\n",i,result_str.c_str());


                		}
			   }


			   ros::spinOnce();
			   sprintf(string2,"Finishedplan %d",num_plan);
                           msg1.action=string2;
                           //speech_generation_msg.publish(msg1);              // Publishes 'msg' message
                           sleep(2.00);

     			   if(num_actions > 0){
        			printf("\n ******* Complete plan fullfilled  *******\n");

        			if(num_no_exe == 0){
                			printf("There were not actions no executed\n");
					success=1;

        			}
        			else{
                			for(k=1;k <= num_no_exe;k++){
                        			plan_num=plan_no_executed[k];
                        			id=plan_no_executed_id[k];
                        			printf("Action %d %s not executed\n",k,plan.action_plan[plan_num][id]);
                			}
					success=0;
        			}

        			//num_actions = 0;
				sprintf(str,"(assert (attempts %d %d))",i-1,success);
                                printf("\nSend attempt result %s\n",str);
                                SimuladorRepresentation::strQueryKDB(str, result_str, 10000);
                                printf("\nCLIPS answer: %d %s\n",i,result_str.c_str());

     			   }

			}

                	break;

		default:
                    printf(" ******* SELECTION NO DEFINED %d *******\n",j++);
                    break;
	}


     }



   }


}
