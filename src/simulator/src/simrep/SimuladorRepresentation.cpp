/***********************************************
*                                              *
*      SimuladorRepresentation.cpp             *
*                                              *
*      Julio Cruz                              *
*      Jesus Savage                            *
*                                              *
*              Bio-Robotics Laboratory         *
*              UNAM, 2019                      *
*                                              *
*                                              *
************************************************/


#include "SimuladorRepresentation.h"

ros::Publisher* SimuladorRepresentation::command_runCLIPS(NULL);
ros::Publisher* SimuladorRepresentation::command_clearCLIPS(NULL);
ros::Publisher* SimuladorRepresentation::command_resetCLIPS(NULL);
ros::Publisher* SimuladorRepresentation::command_factCLIPS(NULL);
ros::Publisher* SimuladorRepresentation::command_ruleCLIPS(NULL);
ros::Publisher* SimuladorRepresentation::command_agendaCLIPS(NULL);
ros::Publisher* SimuladorRepresentation::command_sendCLIPS(NULL);
ros::Publisher* SimuladorRepresentation::command_loadCLIPS(NULL);
ros::Publisher* SimuladorRepresentation::command_sendAndRunCLIPS(NULL);
ros::Publisher* SimuladorRepresentation::command_response(NULL);
ros::Subscriber* SimuladorRepresentation::subClipsToRos(NULL);
ros::ServiceClient* SimuladorRepresentation::cliSpechInterpretation(NULL);
ros::ServiceClient* SimuladorRepresentation::cliStringInterpretation(NULL);
ros::ServiceClient* SimuladorRepresentation::cliQueryKDB(NULL);
ros::ServiceClient* SimuladorRepresentation::cliInitKDB(NULL);
ros::ServiceClient* SimuladorRepresentation::cliClearKDB(NULL);
ros::ServiceClient* SimuladorRepresentation::cliResetKDB(NULL);

bool SimuladorRepresentation::busy_clips = false;
movements SimuladorRepresentation::output;
using namespace boost::algorithm;

SimuladorRepresentation::~SimuladorRepresentation(){
    delete command_runCLIPS;
    delete command_clearCLIPS;
    delete command_resetCLIPS;
    delete command_factCLIPS;
    delete command_ruleCLIPS;
    delete command_agendaCLIPS;
    delete command_sendCLIPS;
    delete command_loadCLIPS;
    delete command_sendAndRunCLIPS;
    delete cliSpechInterpretation;
    delete cliStringInterpretation;
    delete cliQueryKDB;
    delete cliInitKDB;
    delete cliClearKDB;
    delete cliResetKDB;
    delete command_response;
    delete subClipsToRos;
}

void SimuladorRepresentation::setNodeHandle(ros::NodeHandle * nh) {
    command_runCLIPS = new ros::Publisher(nh->advertise<std_msgs::Bool>("/planning_rm/command_runCLIPS", 1));
    command_clearCLIPS = new ros::Publisher(nh->advertise<std_msgs::Bool>("/planning_rm/command_clearCLIPS", 1));
    command_resetCLIPS = new ros::Publisher(nh->advertise<std_msgs::Bool>("/planning_rm/command_resetCLIPS", 1));
    command_factCLIPS = new ros::Publisher(nh->advertise<std_msgs::Bool>("/planning_rm/command_factCLIPS", 1));
    command_ruleCLIPS = new ros::Publisher(nh->advertise<std_msgs::Bool>("/planning_rm/command_ruleCLIPS", 1));
    command_agendaCLIPS = new ros::Publisher(nh->advertise<std_msgs::Bool>("/planning_rm/command_agendaCLIPS", 1));
    command_sendCLIPS = new ros::Publisher(nh->advertise<std_msgs::String>("/planning_rm/command_sendCLIPS", 1));
    command_loadCLIPS = new ros::Publisher(nh->advertise<std_msgs::String>("/planning_rm/command_loadCLIPS", 1));
    command_sendAndRunCLIPS = new ros::Publisher(nh->advertise<std_msgs::String>("/planning_rm/command_sendAndRunCLIPS", 1));
    cliSpechInterpretation = new ros::ServiceClient(nh->serviceClient<simulator::planning_cmd>("/planning_rm/spr_interpreter"));
    cliStringInterpretation = new ros::ServiceClient(nh->serviceClient<simulator::planning_cmd>("/planning_rm/str_interpreter"));

    cliQueryKDB = new ros::ServiceClient(nh->serviceClient<simulator::QueryKDB>("/planning_rm/queryKDB"));
    cliInitKDB = new ros::ServiceClient(nh->serviceClient<simulator::InitKDB>("/planning_rm/initKDB"));
    cliClearKDB = new ros::ServiceClient(nh->serviceClient<simulator::ClearKDB>("/planning_rm/clearKDB"));
    cliResetKDB = new ros::ServiceClient(nh->serviceClient<simulator::ResetKDB>("/planning_rm/resetKDB"));
    command_response = new ros::Publisher(nh->advertise<simulator::PlanningCmdClips>("/planning_rm/command_response", 1));
   
    subClipsToRos = new ros::Subscriber(nh->subscribe("/planning_rm/clips_to_ros", 1, callbackClipsToRos));
}

void SimuladorRepresentation::runCLIPS(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    command_runCLIPS->publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
}

void SimuladorRepresentation::clearCLIPS(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    command_clearCLIPS->publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
}

void SimuladorRepresentation::resetCLIPS(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    command_resetCLIPS->publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
}

void SimuladorRepresentation::factCLIPS(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    command_factCLIPS->publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
}

void SimuladorRepresentation::ruleCLIPS(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    command_ruleCLIPS->publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
}

void SimuladorRepresentation::agendaCLIPS(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    command_agendaCLIPS->publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
}

void SimuladorRepresentation::sendCLIPS(std::string command){
    std_msgs::String msg;
    msg.data = command;
    command_sendCLIPS->publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
}

void SimuladorRepresentation::loadCLIPS(std::string file)
{
    std_msgs::String msg;
    msg.data = file;
    command_loadCLIPS->publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
}

void SimuladorRepresentation::sendAndRunCLIPS(std::string command){
    std_msgs::String msg;
    msg.data = command;
    command_sendAndRunCLIPS->publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
}

bool SimuladorRepresentation::strQueryKDB(std::string query, std::string &result, int timeout){
    simulator::QueryKDB srv;
    bool success = true;
    if(timeout > 0)
        success = ros::service::waitForService("/planning_rm/queryKDB", timeout);
    if (success) {
        srv.request.query = query;
        if (cliQueryKDB->call(srv)) {
            std::string queryResult = srv.response.result;
            std::cout << "SimuladorRepresentation.->Query Result:" << queryResult << std::endl;
            if(queryResult.compare("None") == 0){
                std::cout << "SimuladorRepresentation.->Query failed." << std::endl;
                result = "";
                return false;
            }
            result = queryResult;
            return true;
        }
    }
    std::cout << "SimuladorRepresentation.->Failed to call service of queryKDB" << std::endl;
    return false;
}


bool SimuladorRepresentation::clearKDB(int timeout){
    bool success = true;
    if(timeout > 0)
        success = ros::service::waitForService("/planning_rm/clearKDB", timeout);
    if (success) {
        simulator::ClearKDB srv;
        srv.request.clear = true;
        if (cliClearKDB->call(srv)) {
            std::cout << "SimuladorRepresentation.->Clear KDB" << std::endl;
            return true;
        }
    }
    std::cout << "SimuladorRepresentation.->Failed to call service of clearKDB" << std::endl;
    return false;
}


bool SimuladorRepresentation::resetKDB(int timeout){
    bool success = true;
    if(timeout > 0)
        success = ros::service::waitForService("/planning_rm/resetKDB", timeout);
    if (success) {
        simulator::ResetKDB srv;
        if (cliResetKDB->call(srv)) {
            std::cout << "SimuladorRepresentation.->reset KDB" << std::endl;
            return true;
        }
    }
    std::cout << "SimuladorRepresentation.->Failed to call service of resetKDB" << std::endl;
    return false;
}

bool SimuladorRepresentation::initKDB(std::string filePath, bool run, float timeout){
    bool success = true;
    if(timeout > 0)
        success = ros::service::waitForService("/planning_rm/initKDB", timeout);
    if (success) {
        simulator::InitKDB srv;
        srv.request.filePath = filePath;
        srv.request.run = run;
        if (cliInitKDB->call(srv)) {
            std::cout << "SimuladorRepresentation.->Init KDB" << std::endl;
            return true;
        }
    }
    std::cout << "SimuladorRepresentation.->Failed to call service of initKDB" << std::endl;
    return false;
}

bool SimuladorRepresentation::insertKDB(std::string nameRule, std::vector<std::string> params, int timeout){
    std::stringstream ss;
    std::string result;
    ss << "(assert( " << nameRule << " ";
    for(int i = 0; i < params.size(); i++){
        ss << params[i] << " ";
    }
    ss << "1))";
    bool success = SimuladorRepresentation::strQueryKDB(ss.str(), result, timeout);
    if(success)
        return true;
    return false;
}



void SimuladorRepresentation::callbackClipsToRos(const simulator::PlanningCmdClips::ConstPtr& msg){
    std::cout << "--------- Clips to Ros-----" << std::endl;
    std::cout << "name: " << msg->name << std::endl;
    std::cout << "params: " << msg->params << std::endl;
    
    int num;
    float status;
    float advance, rotation;
    char action[30];

 
    simulator::PlanningCmdClips responseMsg;
    responseMsg.name = msg->name;
    responseMsg.params = msg->params;
    responseMsg.id = msg->id;

    std::stringstream ss;
    std::vector<std::string> tokens;
    std::string str = responseMsg.params;
    split(tokens, str, is_any_of(" "));

    ss << tokens[0];
    for(int i=1 ; i<tokens.size(); i++)
        ss << " "<< tokens[i];

    std::cout << "CLIPS Message: " << ss.str() << std::endl;

    //Here you need to put your code for make an action, then send a response to CLIPS if succeeded or not
    sscanf(responseMsg.params.c_str(),"%s%d%f%f%f",action,&num,&rotation,&advance,&status);
    output.advance=advance;
    output.twist=rotation;

  
    //response to CLIPS
    responseMsg.params = "ROS_RESPONSE"; 
    responseMsg.successful = 1;
    command_response->publish(responseMsg);
    busy_clips = false;
	
	
}

bool SimuladorRepresentation::get_movement(float& advance, float& twist){
	if (busy_clips)
		return false;

	advance = output.advance;
	twist = output.twist;	

	return true;
}

void SimuladorRepresentation::set_busy_clips(bool flag){
	busy_clips = flag;
}
