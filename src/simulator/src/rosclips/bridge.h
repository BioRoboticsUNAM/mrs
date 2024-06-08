#ifndef __BRIDGE_H__
#define __BRIDGE_H__

#pragma once

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include "clips_bridge.h"
#include "queryrouter.h"

#include "simulator/QueryKDB.h"
#include "simulator/InitKDB.h"
#include "simulator/ClearKDB.h"
#include "simulator/ResetKDB.h"

class ClipsBridge;

class Bridge : public ClipsBridge{
public:
	Bridge();
	void test();

private:
	Bridge(Bridge const& obj) = delete;
	Bridge& operator=(Bridge const&) = delete;

protected:
	// virtual void initCLIPS(int argc, char **argv);
	virtual void initPublishers(ros::NodeHandle& nh);
	virtual void initServices(ros::NodeHandle& nh);
	virtual void initSubscribers(ros::NodeHandle& nh);

private:
	// void cmdResponseCallback(std_msgs::Bool::ConstPtr const& msg);
	// void cmdSendCommandCallback(std_msgs::Bool::ConstPtr const& msg);
	void cmdRunCallback(std_msgs::Bool::ConstPtr const& msg);
	void cmdClearCallback(std_msgs::Bool::ConstPtr const& msg);
	void cmdResetCallback(std_msgs::Bool::ConstPtr const& msg);
	void cmdFactCallback(std_msgs::Bool::ConstPtr const& msg);
	void cmdRuleCallback(std_msgs::Bool::ConstPtr const& msg);
	void cmdAgendaCallback(std_msgs::Bool::ConstPtr const& msg);
	void cmdSendCLIPSCallback(std_msgs::String::ConstPtr const& msg);
	void cmdLoadCallback(std_msgs::String::ConstPtr const& msg);
	void cmdSendAndRunCallback(std_msgs::String::ConstPtr const& msg);

	bool srvQueryKDB(simulator::QueryKDB::Request& req, simulator::QueryKDB::Response& res);
	bool srvInitKDB(simulator::InitKDB::Request& req, simulator::InitKDB::Response& res);
	bool srvClearKDB(simulator::ClearKDB::Request& req, simulator::ClearKDB::Response& res);
	bool srvResetKDB(simulator::ResetKDB::Request& req, simulator::ResetKDB::Response& res);

};

#endif // __BRIDGE_H__

