/****************************************************************************/
/// @file    GNEAttributeCarrier.h
/// @author  Jakob Erdmann
/// @date    Mar 2011
/// @version $Id: GNEAttributeCarrier.h 21131 2016-07-08 07:59:22Z behrisch $
///
// Abstract Base class for gui objects which carry attributes
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
#ifndef GNEAttributeCarrier_h
#define GNEAttributeCarrier_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <vector>
#include <map>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/common/ToString.h>
#include <utils/common/TplConvert.h>
#include "GNEReferenceCounter.h"


// ===========================================================================
// class declarations
// ===========================================================================
class GNENet;
class GNEUndoList;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNEAttributeCarrier
 *
 * Abstract Base class for gui objects which carry attributes
 * inherits from GNEReferenceCounter for convenience
 */
class GNEAttributeCarrier : public GNEReferenceCounter {
    /// @brief declare friend class
    friend class GNEChange_Attribute;

public:
    /**@brief Constructor
     * @param[in] tag SUMO Tag assigned to this type of object
     */
    GNEAttributeCarrier(SumoXMLTag tag);

    /// @brief Destructor
    virtual ~GNEAttributeCarrier() {};

    /* @brief method for getting the Attribute of an XML key
     * @param[in] key The attribute key
     * @return string with the value associated to key
     */
    virtual std::string getAttribute(SumoXMLAttr key) const = 0;

    /* @brief method for setting the attribute and letting the object perform additional changes
     * @param[in] key The attribute key
     * @param[in] value The new value
     * @param[in] undoList The undoList on which to register changes
     * @param[in] net optionally the GNENet to inform about gui updates
     */
    virtual void setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) = 0;

    /* @brief method for checking if the key and their conrrespond attribute are valids
     * @param[in] key The attribute key
     * @param[in] value The value asociated to key key
     * @return true if the value is valid, false in other case
     */
    virtual bool isValid(SumoXMLAttr key, const std::string& value);

    /// @brief how should this attribute carrier be called
    virtual std::string getDescription();

    /// @brief get Tag assigned to this object
    SumoXMLTag getTag() const;

    /// @brief get vector of attributes
    std::vector<SumoXMLAttr> getAttrs() const;

    /// @brief function to support debugging
    const std::string getID() const;

    /// @brief get parent's tag of a certain additional element
    static SumoXMLTag getParentType(SumoXMLTag tag);

    /// @brief get all editable attributes for tag and their default values.
    static const std::vector<std::pair <SumoXMLAttr, std::string> >& allowedAttributes(SumoXMLTag tag);

    /// @brief get all editable for tag.
    static const std::vector<SumoXMLTag>& allowedTags();

    /// @brief get all editable tags for netElements
    static const std::vector<SumoXMLTag>& allowedNetElementTags();

    /// @brief get all editable tags for additionals
    static const std::vector<SumoXMLTag>& allowedAdditionalTags();

    /// @brief whether an attribute is numerical (int or float)
    static bool isNumerical(SumoXMLAttr attr);

    /// @brief whether an attribute is numerical or type int
    static bool isInt(SumoXMLAttr attr);

    /// @brief whether an attribute is numerical of type float
    static bool isFloat(SumoXMLAttr attr);

    /// @brief whether an attribute is of type bool
    static bool isBool(SumoXMLAttr attr);

    /// @brief whether an attribute is of type string
    static bool isString(SumoXMLAttr attr);

    /// @brief whether an attribute is of type bool
    static bool isList(SumoXMLAttr attr);

    /// @brief whether an attribute is unique (may not be edited for a multi-selection)
    /// @note unique attributes don't have a default value
    static bool isUnique(SumoXMLAttr attr);

    /// @brief whether an attribute is Discrete
    static bool isDiscrete(SumoXMLTag tag, SumoXMLAttr attr);

    /// @brief check if a element with certain tag has another additional element as parent
    static bool hasParent(SumoXMLTag tag);

    /// @brief check if a element with certain tag has a certain attribute
    static bool hasAttribute(SumoXMLTag tag, SumoXMLAttr attr);

    /// @brief return a list of discrete choices for this attribute or an empty vector
    static const std::vector<std::string>& discreteChoices(SumoXMLTag tag, SumoXMLAttr attr);

    /// @brief return whether the given attribute allows for a combination of discrete values
    static bool discreteCombinableChoices(SumoXMLTag tag, SumoXMLAttr attr);

    /// @brief return definition of a certain SumoXMLAttr
    static std::string getDefinition(SumoXMLTag tag, SumoXMLAttr attr);

    /// @brief return the number of attributes of the tag with the most highter number of attributes
    static int getHigherNumberOfAttributes();

    /// @brief return the default value of the attribute of an element
    /// @note It's advisable to check before with function hasDefaultValue if  exits a default value
    template<typename T>
    static T getDefaultValue(SumoXMLTag tag, SumoXMLAttr attr);

    /// @brief true if a number of type T can be parsed from string
    template<typename T>
    static bool canParse(const std::string& string) {
        try {
            parse<T>(string);
        } catch (NumberFormatException&) {
            return false;
        } catch (EmptyData&) {
            return false;
        } catch (BoolFormatException&) {
            return false;
        }
        return true;
    }

    /// @brief parses a number of type T from string
    template<typename T>
    static T parse(const std::string& string);

    /// @brief true if a positive number of type T can be parsed from string
    template<typename T>
    static bool isPositive(const std::string& string) {
        return canParse<T>(string) && parse<T>(string) > 0;
    }

    /// @brief true if value is a valid sumo ID
    static bool isValidID(const std::string& value);

    /// @brief true if value is a valid file value
    static bool isValidFileValue(const std::string& value);

    /// @brief true if value is a valid string vector
    static bool isValidStringVector(const std::string& value);

    /// @brief feature is still unchanged after being loaded (implies approval)
    static const std::string LOADED;

    /// @brief feature has been reguessed (may still be unchanged be we can't tell (yet)
    static const std::string GUESSED;

    /// @brief feature has been manually modified (implies approval)
    static const std::string MODIFIED;

    /// @brief feature has been approved but not changed (i.e. after being reguessed)
    static const std::string APPROVED;

private:
    /// @brief method for setting the attribute and nothing else (used in GNEChange_Attribute)
    virtual void setAttribute(SumoXMLAttr key, const std::string& value) = 0;

    /// @brief the xml tag to which this carrier corresponds
    const SumoXMLTag myTag;

    /// @brief map with the allowed attributes
    static std::map<SumoXMLTag, std::vector<std::pair <SumoXMLAttr, std::string> > > _allowedAttributes;

    /// @brief vector with the allowed tags
    static std::vector<SumoXMLTag> myAllowedTags;

    /// @brief vector with the allowed tags of netElements
    static std::vector<SumoXMLTag> myAllowedNetElementTags;

    /// @brief vector with the allowed tags of additionals
    static std::vector<SumoXMLTag> myAllowedAdditionalTags;

    /// @brief set with the numerical attributes of type Int
    static std::set<SumoXMLAttr> myNumericalIntAttrs;

    /// @brief set with the numerical attributes of type Float
    static std::set<SumoXMLAttr> myNumericalFloatAttrs;

    /// @brief set with the attributes of type list
    static std::set<SumoXMLAttr> myListAttrs;

    /// @brief set with the unique attributes (i.e. attributes without default values)
    static std::set<SumoXMLAttr> myUniqueAttrs;

    /// @brief map with the allowed tags of additionals with parent and their parent
    static std::map<SumoXMLTag, SumoXMLTag> myAllowedAdditionalWithParentTags;

    /// @brief map with the values of discrete choices
    static std::map<SumoXMLTag, std::map<SumoXMLAttr, std::vector<std::string> > > myDiscreteChoices;

    /// @brief map with the definition of attributes
    static std::map<SumoXMLTag, std::map<SumoXMLAttr, std::string> > myAttrDefinitions;

private:
    /// @brief Invalidated assignment operator
    GNEAttributeCarrier& operator=(const GNEAttributeCarrier& src);
};

#endif

/****************************************************************************/

