/****************************************************************************/
/// @file    FXWorkerThreadTest.cpp
/// @author  Michael Behrisch
/// @date    Oct 2010
/// @version $Id: FXWorkerThreadTest.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// Tests the class FXWorkerThread
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2001-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <gtest/gtest.h>
#include <utils/common/StdDefs.h>
#include <utils/foxtools/FXWorkerThread.h>

class TestTask : public FXWorkerThread::Task {
public:
    void run(FXWorkerThread* /* context */) {
    }
};

// ===========================================================================
// test definitions
// ===========================================================================
/* Test the initialization.*/
TEST(FXWorkerThread, test_init) {
    FXWorkerThread::Pool g(4);
}

/* Test retrieving all tasks.*/
TEST(FXWorkerThread, test_get_all) {
    FXWorkerThread::Pool g(4);
    FXWorkerThread::Task* task1 = new TestTask();
    FXWorkerThread::Task* task2 = new TestTask();
    FXWorkerThread::Task* task3 = new TestTask();
    FXWorkerThread::Task* task4 = new TestTask();
    g.add(task1);
    g.add(task2);
    g.add(task3);
    g.add(task4);
    g.waitAll();
}

