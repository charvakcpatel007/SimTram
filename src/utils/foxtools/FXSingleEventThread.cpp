/****************************************************************************/
/// @file    FXSingleEventThread.cpp
/// @author  unknown_author
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @author  Jakob Erdmann
/// @date    2004-03-19
/// @version $Id: FXSingleEventThread.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
//
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2004-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/

/* =========================================================================
 * included modules
 * ======================================================================= */
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/common/StdDefs.h>
#include "MFXInterThreadEventClient.h"
#include "FXSingleEventThread.h"
#include "fxexdefs.h"
#ifndef WIN32
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#else
#include <process.h>
#endif

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif

#ifndef WIN32
# define PIPE_READ 0
# define PIPE_WRITE 1
#endif

using namespace FXEX;

// Message map
FXDEFMAP(FXSingleEventThread) FXSingleEventThreadMap[] = {
    FXMAPFUNC(SEL_IO_READ, FXSingleEventThread::ID_THREAD_EVENT, FXSingleEventThread::onThreadSignal),
    FXMAPFUNC(SEL_THREAD, 0, FXSingleEventThread::onThreadEvent),
};
FXIMPLEMENT(FXSingleEventThread, FXObject, FXSingleEventThreadMap, ARRAYNUMBER(FXSingleEventThreadMap))



FXSingleEventThread::FXSingleEventThread(FXApp* a, MFXInterThreadEventClient* client)
    : FXObject(), myClient(client) {
    myApp = (a);
#ifndef WIN32
    FXMALLOC(&event, FXThreadEventHandle, 2);
    FXint res = pipe(event);
    FXASSERT(res == 0);
    UNUSED_PARAMETER(res); // only used for assertion
    myApp->addInput(event[PIPE_READ], INPUT_READ, this, ID_THREAD_EVENT);
#else
    event = CreateEvent(NULL, FALSE, FALSE, NULL);
    FXASSERT(event != NULL);
    myApp->addInput(event, INPUT_READ, this, ID_THREAD_EVENT);
#endif
}


FXSingleEventThread::~FXSingleEventThread() {
#ifndef WIN32
    myApp->removeInput(event[PIPE_READ], INPUT_READ);
    ::close(event[PIPE_READ]);
    ::close(event[PIPE_WRITE]);
    FXFREE(&event);
#else
    myApp->removeInput(event, INPUT_READ);
    ::CloseHandle(event);
#endif
}


void
FXSingleEventThread::signal() {
#ifndef WIN32
    FXuint seltype = SEL_THREAD;
    ::write(event[PIPE_WRITE], &seltype, sizeof(seltype));
#else
    ::SetEvent(event);
#endif
}


void
FXSingleEventThread::signal(FXuint seltype) {
    UNUSED_PARAMETER(seltype);
#ifndef WIN32
    ::write(event[PIPE_WRITE], &seltype, sizeof(seltype));
#else
    ::SetEvent(event);
#endif
}


long
FXSingleEventThread::onThreadSignal(FXObject*, FXSelector, void*) {
#ifndef WIN32
    FXuint seltype = SEL_THREAD;
    ::read(event[PIPE_READ], &seltype, sizeof(seltype));
#else
    //FIXME need win32 support
#endif
    FXSelector sel = FXSEL(SEL_THREAD, 0);
    handle(this, sel, 0);
    return 0;
}


long
FXSingleEventThread::onThreadEvent(FXObject*, FXSelector , void*) {
    myClient->eventOccured();
    /*
    FXuint seltype1 = FXSELTYPE(SEL_THREAD);
    if(myTarget && myTarget->handle(this,FXSEL(seltype1,mySelector),NULL)) {
    }
    FXuint seltype = FXSELTYPE(sel);
    return myTarget && myTarget->handle(this,FXSEL(seltype,mySelector),NULL);
    */
    return 1;
}


void
FXSingleEventThread::sleep(long ms) {
#ifdef WIN32
    Sleep(ms);
#else
    long long us = ms * 1000;
    usleep(us);
#endif
}



