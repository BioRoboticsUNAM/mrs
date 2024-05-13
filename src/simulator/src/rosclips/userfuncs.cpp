#include <set>
#include <map>
#include <chrono>
#include <string>
#include <thread>
#include <utility>
#include <functional>

#include "userfuncs.h"
#include "clipswrapper.h"

static inline
void timedAssert(int delayms, const std::string& fact);


int CLIPS_pycall_wrapper(){
	std::set<std::string> deprecated{"SendCommand", "SendResponse", "SendResponse", "CreateSharedVar",
									 "WriteSharedVar", "SubscribeToSharedVar" };
	std::map<std::string, std::pair<int,std::function<int()>>> supported{
		{"setTimer",    {2, pyc_setTimer} },
		{"setCmdTimer", {3, pyc_setCmdTimer} }
	};

	if(clips::argCountCheck("python-call", clips::ArgCountRestriction::AtLeast, 1) == -1)
		return -1;
	// 1. Fetch function name
	std::string pyFuncName = clips::returnLexeme(1);
	// 2. If deprecated, return true
	if (deprecated.find(pyFuncName) != deprecated.end()) return 1;
	// 3. If not implemented or wrong arg count, return false
	if (supported.find(pyFuncName) == supported.end()) return 0;
	int argc = 1 + supported[pyFuncName].first;
	if(clips::argCountCheck("python-call", clips::ArgCountRestriction::Exactly, argc) == -1) return -1;

	std::function<int()> f = supported[pyFuncName].second;
	return f();
}


int pyc_setTimer(){
	int delayms = clips::returnInt(2);
	std::string cmd = clips::returnLexeme(3);

	std::string fact("(BB_timer \"" + cmd + "\")");
	timedAssert(delayms, fact);

	return 1;
}


int pyc_setCmdTimer(){
	int delayms = clips::returnInt(2);
	std::string cmd = clips::returnLexeme(3);
	int cmdId = clips::returnInt(4);

	std::string fact("(BB_timer \"" + cmd + "\"" + std::to_string(cmdId) + ")");
	timedAssert(delayms, fact);

	return 1;
}


int CLIPS_setCmdTimer_wrapper(){
	if(clips::argCountCheck("setCmdTimer", clips::ArgCountRestriction::Exactly, 3) == -1)
		return -1;

	int delayms = clips::returnInt(1);
	std::string cmd = clips::returnLexeme(2);
	int cmdId = clips::returnInt(3);

	std::string fact("(BB_timer \"" + cmd + "\" " + std::to_string(cmdId) + ")");
	timedAssert(delayms, fact);

	return 1;
}


static inline
void timedAssert(int delayms, const std::string& fact){
	std::thread t([delayms, fact]{
		std::this_thread::sleep_for(std::chrono::milliseconds(delayms));
		clips::assertString(fact);
		clips::run();
	});
	t.detach();
}

