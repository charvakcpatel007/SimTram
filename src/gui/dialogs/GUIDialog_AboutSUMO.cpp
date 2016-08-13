/****************************************************************************/
/// @file    GUIDialog_AboutSUMO.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Mon, 08.03.2004
/// @version $Id: GUIDialog_AboutSUMO.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// The application's "About" - dialog
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

#ifdef HAVE_VERSION_H
#include <version.h>
#endif

#include "GUIDialog_AboutSUMO.h"
#include <utils/foxtools/FXLinkLabel.h>
#include <utils/common/StdDefs.h>
#include <utils/gui/images/GUIIconSubSys.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// bitmap definitions
// ===========================================================================
static const char* dlr_icon[] = {
    "40 42 3 1",
    "# c #000000",
    "a c #333333",
    ". c #ffffff",
    "........................................",
    "........................................",
    "...........................#............",
    "..........................##............",
    ".........................###............",
    "........................####............",
    ".......................#####............",
    "......................###.##............",
    ".....................###..##............",
    "....................###...##............",
    "...................###....##............",
    "..................###.....##............",
    ".........##############################.",
    "........##############################..",
    ".......###.....###.......###......###...",
    "......###.....###.......###......###....",
    ".....###.....###.......###......###.....",
    "....###.....###.......###......###......",
    "...###.....###.......###......###.......",
    "..##############################........",
    ".##############################.........",
    "...........##.....###...................",
    "...........##....###....................",
    "...........##...###.....................",
    "...........##..###......................",
    "...........##.###.......................",
    "...........#####........................",
    "...........####.........................",
    "...........###..........................",
    "...........##...........................",
    "...........#...#####....##.....#####....",
    "...............######...##.....######...",
    "...............##..###..##.....##..##...",
    "...............##...##..##.....##..##...",
    "...............##...##..##.....#####....",
    "...............##...##..##.....#####....",
    "...............##..###..##.....##.###...",
    "...............######...#####..##..##...",
    "...............#####....#####..##..###..",
    "........................................",
    "........................................",
    "........................................"
};


// ===========================================================================
// method definitions
// ===========================================================================
GUIDialog_AboutSUMO::GUIDialog_AboutSUMO(FXWindow* parent,  const char* name, int x, int y)
    : FXDialogBox(parent, name, DECOR_CLOSE | DECOR_TITLE, x, y, 0, 0) {
    FXVerticalFrame* f1 = new FXVerticalFrame(this, LAYOUT_TOP | FRAME_NONE | LAYOUT_FILL_X, 0, 0, 0, 0, 0, 0, 1, 1);
    // build icons
    FXHorizontalFrame* f2 = new FXHorizontalFrame(f1, LAYOUT_TOP | LAYOUT_CENTER_X | FRAME_NONE, 0, 0, 0, 0, 0, 0, 1, 1);
    myDLRIcon = new FXXPMIcon(getApp(), dlr_icon);
    new FXButton(f2, "\tDLR\t.", myDLRIcon, 0, 0, LAYOUT_CENTER_Y | TEXT_OVER_ICON, 5, 0, 40 + 5, 0,  0, 0, 0, 0);
    // "SUMO <VERSION>"
    FXVerticalFrame* f4 = new FXVerticalFrame(f2, FRAME_NONE, 0, 0, 0, 0,   20, 0, 0, 0);
    myHeadlineFont = new FXFont(getApp(), "Arial", 18, FXFont::Bold);
    FXLabel* l = new FXLabel(f4, "SUMO " VERSION_STRING, 0, LAYOUT_CENTER_Y | LAYOUT_CENTER_X | JUSTIFY_CENTER_X | LABEL_NORMAL, 0, 0, 0, 0, 0, 0, 0, 0);
    l->setFont(myHeadlineFont);
    new FXLabel(f4, "Simulation of Urban MObility", 0, LAYOUT_CENTER_Y | LAYOUT_CENTER_X | JUSTIFY_CENTER_X | LABEL_NORMAL, 0, 0, 0, 0, 0, 0, 0, 0);
    new FXLabel(f4, HAVE_ENABLED, 0, LAYOUT_CENTER_Y | LAYOUT_CENTER_X | JUSTIFY_CENTER_X | LABEL_NORMAL, 0, 0, 0, 0, 0, 0, 0, 0);
    //
    // additional infos
    FXVerticalFrame* f3 = new FXVerticalFrame(f1, FRAME_NONE, 0, 0, 0, 0, 0, 0, 0, 0);
    // copyright notice
    new FXLabel(f3, "A microscopic, multi-modal, open source", 0, LABEL_NORMAL, 0, 0, 0, 0, 0, 0, 0, 0);
    new FXLabel(f3, "road traffic simulation.", 0, LABEL_NORMAL, 0, 0, 0, 0, 0, 0, 0, 0);
    new FXLabel(f3, "Copyright (C) 2001-2016 DLR / Institute of Transportation Systems", 0, LAYOUT_CENTER_X | JUSTIFY_CENTER_X | LABEL_NORMAL, 0, 0, 0, 0, 0, 0, 0, 0);

    FXLinkLabel* link = new FXLinkLabel(f3, "http://sumo.dlr.de", 0, LAYOUT_CENTER_X | JUSTIFY_CENTER_X | LABEL_NORMAL, 0, 0, 0, 0, 5, 5, 5, 5);
    link->setTipText("http://sumo.dlr.de");
    // ok-button
    new FXButton(f1, "OK\t\t", 0, this, ID_ACCEPT, LAYOUT_FIX_WIDTH | LAYOUT_CENTER_X | JUSTIFY_CENTER_X | FRAME_THICK | FRAME_RAISED, 0, 0, 50, 30);
    setIcon(GUIIconSubSys::getIcon(ICON_APP));
}


void
GUIDialog_AboutSUMO::create() {
    FXDialogBox::create();
    myDLRIcon->create();
}


GUIDialog_AboutSUMO::~GUIDialog_AboutSUMO() {
    delete myDLRIcon;
    delete myHeadlineFont;
}



/****************************************************************************/

