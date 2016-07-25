/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.1 (2011-09-09)
*/

/** \file Timestamp.cpp
  * \brief This file contains the definitions of the \ref TRTK::Timestamp
  *        "Timestamp" class and some corresponding helper classes.
  */

#include <cassert>
#include <cstddef>
#include <queue>

#include "TRTK/Timestamp.hpp"


using std::queue;
using std::map;
using std::set;
using std::string;


namespace TRTK
{


//////////////////////////////////////////////////////////////////////////////
//                                Timestamp                                 //
//////////////////////////////////////////////////////////////////////////////

/** Constructs an Timestamp object. */

Timestamp::Timestamp()
{
};


/** Destroys an Timestamp object. */

Timestamp::~Timestamp()
{
};


/** \brief Element access.
  *
  * \param [in] object_name name of the object to be processed
  *
  * This method gives access to the object properties. You might update the
  * timestamp or you can add or check for dependencies etc.
  *
  * If \p object_name does not exist, it is created.
  *
  * \see ObjectInfo, ObjectInfo::update(), ObjectInfo::addDependency(), etc.
  */

Timestamp::ObjectInfo & Timestamp::operator[](const string & object_name)
{
    if (objects.count(object_name))
    {
        return objects[object_name];
    }
    else
    {
        // First initialize the ObjectInfo object, store it in the
        // map and then return its reference. Otherwise it would
        // not know its parent collection, that is /this/.

        objects.insert(std::pair<string, ObjectInfo>(object_name, ObjectInfo(*this)));
        return objects[object_name];
    }
};


//////////////////////////////////////////////////////////////////////////////
//                                ObjectInfo                                //
//////////////////////////////////////////////////////////////////////////////

/** Constructs an invalid ObjectInfo object.
  *
  * This constructor must be defined to allow the use of ObjectInfo
  * objects in certain associative containers of the STL. Nevertheless
  * add only otherwise initialized objects...
  */

Timestamp::ObjectInfo::ObjectInfo() :
    parent_collection(*reinterpret_cast<Timestamp *>(NULL)),
    timestamp(0)
{
    const bool DO_NO_USE_THIS_CONSTRUCTOR = false;
    assert(DO_NO_USE_THIS_CONSTRUCTOR);
}


/** Constructs an ObjectInfo object. */

Timestamp::ObjectInfo::ObjectInfo(Timestamp & parent_collection) :
    parent_collection(parent_collection),
    timestamp(0)
{
}


/** Destroys an ObjectInfo object. */

Timestamp::ObjectInfo::~ObjectInfo()
{
}


/** \brief Adds dependencies to the given object.
  *
  * \param [in] object_name name of the object to be processed
  *
  * This allows you to check if some required objects were already set up or
  * up-to-date (providing you use the timestamp mechanism consistently).
  *
  * \see removeDependency()
  */

void Timestamp::ObjectInfo::addDependency(const std::string & object_name)
{
    dependencies.insert(object_name);
}


/** \brief Check timestamp of all dependencies.
  *
  * Check if the timestamps of all dependencies are further back in time than
  * the timestamp of this object.
  *
  * \return true if one or more dependencies were not set up so far or if they
  *         are outdated
  *
  * \see dependencyHasChanged()
  */

bool Timestamp::ObjectInfo::dependenciesHaveChanged() const
{
    // Check if the timestamps of all dependencies are further back in
    // time than the timestamp of this object.

    std::set<std::string>::const_iterator it;

    for (it = dependencies.begin(); it != dependencies.end(); ++it)
    {
        if (!(parent_collection[*it].dependenciesUpToDate(timestamp)))
        {
            return true;
        }
    }

    return false;
}


/** \brief Check timestamp of a single dependency.
  *
  * Check if the timestamp of a single dependency is further back in time than
  * the timestamp of this object (checking is done recursively, though).
  *
  * \return true if the dependency was not set up so far or if it is outdated
  *
  * \see dependenciesHaveChanged()
  */

bool Timestamp::ObjectInfo::dependencyHasChanged(const std::string & other_object) const
{
    return !parent_collection[other_object].dependenciesUpToDate(timestamp);
}


/** Checks (recursively) if the timestamps of the dependencies are further back
  * in time than the given timestamp. If this is the case, \c true is returned.
  *
  * \param [in] timestamp to check against
  * \return boolean
  */

bool Timestamp::ObjectInfo::dependenciesUpToDate(const clock_t & timestamp) const
{
    // Are there any dependencies?

    if (dependencies.size() == 0)
    {
        // No, there aren't any dependencies.

        if (this->timestamp < timestamp)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        // Yes, there are dependencies. Thus, check, if all dependencies
        // have a timestamp that is further back in time than the given
        // one. In this case return true, else return false.

        std::set<std::string>::const_iterator it;

        for (it = dependencies.begin(); it != dependencies.end(); ++it)
        {
            if (!(parent_collection[*it].dependenciesUpToDate(timestamp)))
            {
                return false;
            }
        }

        return true;
    }
}


/** Checks if the timestamp of the given object is further back in time than
  * the one of \c *this.
  *
  * If \p dependencies is set to \c true, dependencies of \p object are
  * regarded as well, i.e. if any dependency is more recent than \c *this
  * then \c false is returned.
  *
  * \param [in] other_object    object to check against
  * \param [in] dependencies    specifies if dependencies shall be checked as well
  * \return boolean
  */

bool Timestamp::ObjectInfo::isMoreRecentThan(const std::string & object, bool dependencies) const
{
    if (!dependencies)
    {
        return timestamp > parent_collection[object].timestamp;
    }
    else
    {
        // Check if 'object' is back in time compared to *this. If this
        // is the case also check its dependencies. Do this by inserting
        // direct dependencies into a queue and subsequently comparing
        // these entries in the same manner as above.

        queue<string> dependencies;
        dependencies.push(object);

        while (!dependencies.empty())
        {
            string object = dependencies.front();
            dependencies.pop();

            if (timestamp < parent_collection[object].timestamp)
            {
                return false;
            }

            set<string>::const_iterator it = parent_collection[object].dependencies.begin();

            while (it != parent_collection[object].dependencies.end())
            {
                dependencies.push(*it++);
            }
        }

        return true;
    }
}


/** \brief Removes dependencies from a given object.
  *
  * \param [in] object_name name of the object to be processed
  *
  * \see addDependency()
  */

void Timestamp::ObjectInfo::removeDependency(const std::string & object_name)
{
    dependencies.erase(object_name);
}


/** \brief Updates the timestamp of the object.
  *
  * \param [in] object_name name of the object to be processed
  */

void Timestamp::ObjectInfo::update()
{
    timestamp = std::clock();
}


} // namespace TRTK
