#ifndef TOOLS_SIMULADOR_TOOLS_SRC_SIMULADORREPRESENTATION_H_
#define TOOLS_SIMULADOR_TOOLS_SRC_SIMULADORREPRESENTATION_H_

#include "ros/ros.h"

#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"

#include "simulator/PlanningCmdClips.h"
#include "simulator/planning_cmd.h"
#include "simulator/QueryKDB.h"
#include "simulator/InitKDB.h"
#include "simulator/ClearKDB.h"
#include "simulator/ResetKDB.h"

#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
typedef struct movements_
{
	float twist;
	float advance;
}movements;

class SimuladorRepresentation {
    private:
        ros::NodeHandle * nh;

	static bool busy_clips;
	static movements output;
	
        static ros::Publisher * command_runCLIPS;
        static ros::Publisher * command_clearCLIPS;
        static ros::Publisher * command_resetCLIPS;
        static ros::Publisher * command_factCLIPS;
        static ros::Publisher * command_ruleCLIPS;
        static ros::Publisher * command_agendaCLIPS;
        static ros::Publisher * command_sendCLIPS;
        static ros::Publisher * command_loadCLIPS;
        static ros::Publisher * command_sendAndRunCLIPS;
        static ros::Publisher * command_response;
    	static ros::Subscriber * subClipsToRos;
        static ros::ServiceClient * cliSpechInterpretation;
        static ros::ServiceClient * cliStringInterpretation;
        static ros::ServiceClient * cliQueryKDB;
        static ros::ServiceClient * cliInitKDB;
        static ros::ServiceClient * cliClearKDB;
        static ros::ServiceClient * cliResetKDB;


    public:

        ~SimuladorRepresentation();

        static void setNodeHandle(ros::NodeHandle * nh);
        static void runCLIPS(bool enable);
	   static void clearCLIPS(bool enable);
        static void resetCLIPS(bool enable);
        static void factCLIPS(bool enable);
        static void ruleCLIPS(bool enable);
        static void agendaCLIPS(bool enable);
        static void sendCLIPS(std::string command);
        static void loadCLIPS(std::string file);
        static void sendAndRunCLIPS(std::string command);
        static bool initKDB(std::string filePath, bool run, float timeout = 1000);
        static bool insertKDB(std::string nameRule, std::vector<std::string> params, int timeout = 1000);
        static bool clearKDB(int timeout = 1000);
        static bool resetKDB(int timeout = 1000);
        static bool strQueryKDB(std::string query, std::string &result, int timeout = 1000);
	static void callbackClipsToRos(const simulator::PlanningCmdClips::ConstPtr& msg);
	static bool get_movement(float& advance, float&twist);
	static void set_busy_clips(bool flag);

};

#endif /* TOOLS_SIMULADOR_TOOLS_SRC_SIMULADORREPRESENTATION_H_ */
