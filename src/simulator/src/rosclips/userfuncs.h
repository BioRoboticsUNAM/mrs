#ifndef __PYCALLFUNCS_H__
#define __PYCALLFUNCS_H__
#pragma once

#include "clipswrapper.h"

// Deprecated (Registered but always return 1)
// (python-call SendCommand ?cmd ?args)
// (python-call SendResponse ?cmd ?id ?result ?params)
// (python-call CreateSharedVar ?type ?name))
// (python-call WriteSharedVar ?type ?name $?data)
// (python-call SubscribeToSharedVar ?name $?options)

// Implemented (Always return 1)
// (setCmdTimer ?time ?cmd ?id)
// (python-call setTimer ?time ?sym)
// (python-call setCmdTimer ?time ?cmd ?id)

int CLIPS_pycall_wrapper();
int pyc_setTimer();
int pyc_setCmdTimer();

int CLIPS_setCmdTimer_wrapper();
int CLIPS_logMessage_wrapper();

#endif // __PYFUNCS_H__