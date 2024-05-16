#include "clips_bridge.h"
#include "bridge.h"

#include <cstdio>
#include <fstream>
#include <sstream>

#include "utils.h"
#include "clipswrapper.h"



/* ** ********************************************************
* Macros
* *** *******************************************************/
#define contains(s1,s2) s1.find(s2) != std::string::npos


/* ** ********************************************************
* Local helpers
* *** *******************************************************/
static inline
bool ends_with(const std::string& s, const std::string& end){
	if (end.size() > s.size()) return false;
	return std::equal(end.rbegin(), end.rend(), s.rbegin());
}

static inline
void split_path(const std::string& fpath, std::string& dir, std::string& fname){
	size_t slashp = fpath.rfind("/");
	if(slashp == std::string::npos){
		dir = std::string();
		fname = fpath;
		return;
	}
	dir = fpath.substr(0, slashp);
	fname = fpath.substr(slashp+1);
}

static inline
std::string get_current_path(){
	char buff[FILENAME_MAX];
	getcwd(buff, sizeof(buff));
	return std::string(buff);
}



/* ** ********************************************************
* Constructor
* *** *******************************************************/
Bridge::Bridge() : ClipsBridge(),
	qr(QueryRouter::getInstance()){
		qr.setLogicalNames(
		clips::LogicalName::stdout
		// | clips::LogicalName::wdisplay
		// | clips::LogicalName::wtrace
	);
}



/* ** ********************************************************
*
* Class methods
* Initialization
*
* *** *******************************************************/

void Bridge::initServices(ros::NodeHandle& nh){
	ClipsBridge::initServices(nh);

	ros::ServiceServer
	srv = nh.advertiseService("/planning_rm/str_query_KDB",
		&Bridge::srvQueryKDB, this);
	srvServers["/planning_rm/str_query_KDB"] = srv;

	srv = nh.advertiseService("/planning_rm/init_kdb",
		&Bridge::srvInitKDB, this);
	srvServers["/planning_rm/init_KDB"] = srv;

	srv = nh.advertiseService("/planning_rm/clear_kdb",
		&Bridge::srvClearKDB, this);
	srvServers["/planning_rm/clear_KDB"] = srv;
}


void Bridge::initPublishers(ros::NodeHandle& nh){
	ClipsBridge::initPublishers(nh);
}


void Bridge::initSubscribers(ros::NodeHandle& nh){
	ClipsBridge::initSubscribers(nh);

	ros::Subscriber
	// sub = nh.subscribe<>("/planning_rm/command_response", 10,
	// 		boost::bind(&Bridge::cmdResponseCallback, this, _1));
	// subscribers["/planning_rm/command_response"] = sub;

	// sub = nh.subscribe<>("/planning_rm/command_send_command", 10,
	// 		boost::bind(&Bridge::cmdSendCommandCallback, this, _1));
	// subscribers["/planning_rm/command_send_command"] = sub;

	sub = nh.subscribe<std_msgs::Bool>("/planning_rm/command_runCLIPS", 10,
			boost::bind(&Bridge::cmdRunCallback, this, _1));
	subscribers["/planning_rm/command_runCLIPS"] = sub;

    sub = nh.subscribe<std_msgs::Bool>("/planning_rm/command_clearCLIPS", 10,
    		boost::bind(&Bridge::cmdClearCallback, this, _1));
	subscribers["/planning_rm/command_clearCLIPS"] = sub;

    sub = nh.subscribe<std_msgs::Bool>("/planning_rm/command_resetCLIPS", 10,
    		boost::bind(&Bridge::cmdResetCallback, this, _1));
	subscribers["/planning_rm/command_resetCLIPS"] = sub;

    sub = nh.subscribe<std_msgs::Bool>("/planning_rm/command_factCLIPS", 10,
    		boost::bind(&Bridge::cmdFactCallback, this, _1));
	subscribers["/planning_rm/command_factCLIPS"] = sub;

    sub = nh.subscribe<std_msgs::Bool>("/planning_rm/command_ruleCLIPS", 10,
    		boost::bind(&Bridge::cmdRuleCallback, this, _1));
	subscribers["/planning_rm/command_ruleCLIPS"] = sub;

    sub = nh.subscribe<std_msgs::Bool>("/planning_rm/command_agendaCLIPS", 10,
    		boost::bind(&Bridge::cmdAgendaCallback, this, _1));
	subscribers["/planning_rm/command_agendaCLIPS"] = sub;

    sub = nh.subscribe<std_msgs::String>("/planning_rm/command_sendCLIPS", 10,
    		boost::bind(&Bridge::cmdSendCLIPSCallback, this, _1));
	subscribers["/planning_rm/command_sendCLIPS"] = sub;

    sub = nh.subscribe<std_msgs::String>("/planning_rm/command_loadCLIPS", 10,
    		boost::bind(&Bridge::cmdLoadCallback, this, _1));
	subscribers["/planning_rm/command_loadCLIPS"] = sub;

    sub = nh.subscribe<std_msgs::String>("/planning_rm/command_sendAndRunCLIPS", 10,
    		boost::bind(&Bridge::cmdSendAndRunCallback, this, _1));
	subscribers["/planning_rm/command_sendAndRunCLIPS"] = sub;

}



/* ** ********************************************************
*
* Class methods: Callbacks
*
* *** *******************************************************/
// void Bridge::cmdResponseCallback( const& msg){
//	// clipsFunctions.Assert('(BB_received "{0}" {1} {2} "{3}")'.format(data.name, data.id, data.successful, data.params))
//	// clipsFunctions.PrintOutput()
//	// clipsFunctions.Run(BBFunctions.gui.getRunTimes())
//	// clipsFunctions.PrintOutput()
// }


// void Bridge::cmdSendCommandCallback( const& msg){
// 	// clips.SendCommand(data.command, True)
// 	// clipsFunctions.PrintOutput()
// }


void Bridge::cmdRunCallback(std_msgs::Bool::ConstPtr const& msg){
	ROS_INFO("Command: Run");
	clips::run();
}


void Bridge::cmdClearCallback(std_msgs::Bool::ConstPtr const& msg){
	ROS_INFO("Command: Clear");
	clearCLIPS();
}


void Bridge::cmdResetCallback(std_msgs::Bool::ConstPtr const& msg){
	ROS_INFO("Command: Reset");
	resetCLIPS();
}


void Bridge::cmdFactCallback(std_msgs::Bool::ConstPtr const& msg){
	ROS_INFO("Command: Fact");
	clips::printFacts();
}


void Bridge::cmdRuleCallback(std_msgs::Bool::ConstPtr const& msg){
	ROS_INFO("Command: Rule");
	clips::printRules();
}


void Bridge::cmdAgendaCallback(std_msgs::Bool::ConstPtr const& msg){
	ROS_INFO("Command: Agenda");
	clips::printAgenda();
}


void Bridge::cmdSendCLIPSCallback(std_msgs::String::ConstPtr const& msg){
	ROS_INFO("Command: Send");
	// clips.SendCommand(data.data, True) ->
	// 	_c.sendCommand(command, verbose)
	// clipsFunctions.PrintOutput()
	clips::sendCommand(msg->data);
}


void Bridge::cmdSendAndRunCallback(std_msgs::String::ConstPtr const& msg){
	ROS_INFO("Command: SendAndRun");
	// clips.SendCommand(data.data, True) ->
	// 	_c.sendCommand(command, verbose)
	// clipsFunctions.PrintOutput()
	// clipsFunctions.Run('')
	// clipsFunctions.PrintOutput()
	clips::sendCommand(msg->data);
	clips::run();
}


void Bridge::cmdLoadCallback(std_msgs::String::ConstPtr const& msg){
	ROS_INFO("Command: Load");
	if(!ends_with(msg->data, ".dat") && !ends_with(msg->data, ".clp")){
		ROS_INFO("Load file %s: FAIL. Unsupported file format.", msg->data.c_str());
		return;
	}
	bool result = loadFile(msg->data);
	ROS_INFO("Load file %s: %s", msg->data.c_str(), result ? "SUCCESS" : "FAIL");
}


//string ← f(string query)
bool Bridge::srvQueryKDB(simulator::StrQueryKDB::Request& req, simulator::StrQueryKDB::Response& res){
	qr.enable();
	clips::sendCommand(req.query, true);
	clips::run();
	res.result = qr.read();
	qr.disable();
	return true;
}


// void ← f(string filePath, bool run)
bool Bridge::srvInitKDB(simulator::InitKDB::Request& req, simulator::InitKDB::Response& res){
	ROS_INFO("Service: InitKDB");
	if(!ends_with(req.filePath, ".dat") && !ends_with(req.filePath, ".clp")){
		ROS_INFO("Load file %s: FAIL. Unsupported file format.", req.filePath.c_str());
		return true;
	}
	if(!loadFile(req.filePath)) return true;
	if(req.run) clips::run();
	return true;
}


// bool ← f(bool clear)
bool Bridge::srvClearKDB(simulator::clearKDB::Request& req, simulator::clearKDB::Response& res){
	ROS_INFO("Service: ClearKDB");
	if(!req.clear) return true;
	clearCLIPS();
	return res.cleared = true;
}
