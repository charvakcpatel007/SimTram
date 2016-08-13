/****************************************************************************/
/// @file    GNEInspectorFrame.h
/// @author  Jakob Erdmann
/// @date    Mar 2011
/// @version $Id: GNEInspectorFrame.h 21131 2016-07-08 07:59:22Z behrisch $
///
// The Widget for modifying network-element attributes (i.e. lane speed)
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
#ifndef GNEInspectorFrame_h
#define GNEInspectorFrame_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "GNEFrame.h"

// ===========================================================================
// class declarations
// ===========================================================================
class GNENet;
class GNEEdge;
class GNEAttributeCarrier;
class GNEAdditional;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNEInspectorFrame
 * The Widget for modifying network-element attributes (i.e. lane speed)
 */
class GNEInspectorFrame : public GNEFrame {
    /// @brief FOX-declaration
    FXDECLARE(GNEInspectorFrame)

public:

    // ===========================================================================
    // class AttrInput
    // ===========================================================================

    class AttrInput : public FXMatrix  {
        /// @brief FOX-declaration
        FXDECLARE(GNEInspectorFrame::AttrInput)

    public:
        /// @brief constructor
        AttrInput(FXComposite* parent, GNEInspectorFrame* inspectorFrameParent);

        /// @brief show attribute
        void showAttribute(SumoXMLTag tag, SumoXMLAttr attr, const std::string& value);

        /// @brief show attribute
        void hiddeAttribute();

        /// @brief get current tag
        SumoXMLTag getTag() const;

        /// @brief get current Attr
        SumoXMLAttr getAttr() const;

        /// @brief try to set new attribute value
        long onCmdSetAttribute(FXObject*, FXSelector, void*);

        /// @brief open model dialog for more comfortable attribute editing
        long onCmdOpenAttributeEditor(FXObject*, FXSelector, void*);

    protected:
        /// @brief FOX needs this
        AttrInput() {}

    private:
        /// @brief pointer to GNEInspectorFrame parent
        GNEInspectorFrame* myInspectorFrameParent;

        /// @brief current tag
        SumoXMLTag myTag;

        /// @brief current Attr
        SumoXMLAttr myAttr;

        /// @brief pointer to label
        FXLabel* myLabel;

        /// @brief textField to modify the value of int values
        FXTextField* myTextFieldInt;

        /// @brief textField to modify the value of real values
        FXTextField* myTextFieldReal;

        /// @brief textField to modify the value of strings values
        FXTextField* myTextFieldStrings;

        /// @brief pointer to combo box choices
        FXComboBox* myChoicesCombo;

        /// @brief pointer to checkBox
        FXCheckButton* myCheckBox;

        /// @brief pointer to buttonCombinableChoices
        FXButton* myButtonCombinableChoices;

        /// @brief set show as private function
        void show();

        /// @brief set hide as private function
        void hide();
    };

    // ===========================================================================
    // class AttrEditor
    // ===========================================================================

    class AttrEditor : public FXDialogBox {
        /// @brief FOX-declaration
        FXDECLARE(GNEInspectorFrame::AttrEditor)

    public:
        /// @brief constructor
        AttrEditor(AttrInput* attrInputParent, FXTextField* textFieldAttr);

        /// @brief destructor
        ~AttrEditor();

        /// @brief call when user press button reset
        long onCmdReset(FXObject*, FXSelector, void*);

    protected:
        /// @brief FOX needs this
        AttrEditor() {}

    private:
        /// @brief Pointer to AttrInput parent
        AttrInput* myAttrInputParent;

        // @brief Pointer to TexField in which write attribute
        FXTextField* myTextFieldAttr;

        // @brief Matrix in that CheckBoxs will be inserted
        FXMatrix* myCheckBoxMatrix;

        /// @brief vector of CheckBoxs
        std::vector<FXCheckButton*> myVectorOfCheckBox;

        /// @brief frame for the buttons
        FXHorizontalFrame* frameButtons;

        /// @brief Button Accept
        FXButton* myAcceptButton;

        /// @brief Button Cancel
        FXButton* myCancelButton;

        /// @brief Button Reset
        FXButton* myResetButton;
    };

public:
    /**@brief Constructor
     * @brief parent FXFrame in which this GNEFrame is placed
     * @brief viewNet viewNet that uses this GNEFrame
     */
    GNEInspectorFrame(FXComposite* parent, GNEViewNet* viewNet);

    /// @brief Destructor
    ~GNEInspectorFrame();

    /// @brief show Frame
    void show();

    /// @brief hide Frame
    void hide();

    /// @brief Inspect the given multi-selection
    void inspect(const std::vector<GNEAttributeCarrier*>& ACs);

    /// @brief get current list of ACs
    const std::vector<GNEAttributeCarrier*>& getACs() const;

    /// @brief get the template edge (to copy attributes from)
    GNEEdge* getEdgeTemplate() const;

    /// @brief seh the template edge (we assume shared responsibility via reference counting)
    void setEdgeTemplate(GNEEdge* tpl);

    /// @brief copy edge attributes from edge template
    long onCmdCopyTemplate(FXObject*, FXSelector, void*);

    /// @brief set current edge as new template
    long onCmdSetTemplate(FXObject*, FXSelector, void*);

    /// @brief update the copy button with the name of the template
    long onUpdCopyTemplate(FXObject*, FXSelector, void*);

    /// @brief called when user toogle the blocking button
    long onCmdSetBlocking(FXObject*, FXSelector, void*);

protected:
    /// @brief FOX needs this
    GNEInspectorFrame() {}

private:
    /// @brief groupBox for attributes
    FXGroupBox* myGroupBoxForAttributes;

    /// @brief list of Attribute inputs
    std::vector<GNEInspectorFrame::AttrInput*> vectorOfAttrInput;

    /// @brief groupBox for templates
    FXGroupBox* myGroupBoxForTemplates;

    /// @brief copy template button
    FXButton* myCopyTemplateButton;

    /// @brief set template button
    FXButton* mySetTemplateButton;

    /// @brief the edge template
    GNEEdge* myEdgeTemplate;

    /// @brief GropuBox for editor attributes
    FXGroupBox* myGroupBoxForEditor;

    /// @brief pointer to check button block
    FXCheckButton* myCheckBlocked;

    /// @brief pointer to additional element
    GNEAdditional* myAdditional;

    /// @brief the multi-selection currently being inspected
    std::vector<GNEAttributeCarrier*> myACs;
};


#endif

/****************************************************************************/

