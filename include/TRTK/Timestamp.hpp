/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.3.0 (2014-07-01)
*/

/** \file Timestamp.hpp
  * \brief This file contains the declarations of the \ref TRTK::Timestamp
  *        "Timestamp" class and some corresponding helper classes.
  */

#ifndef TIMESTAMP_HPP_4765312314
#define TIMESTAMP_HPP_4765312314

#include <ctime>
#include <map>
#include <set>
#include <string>


namespace TRTK
{


/** \brief A class for managing timestamps of various objects by
  *        using names. Dependencies between objects can be specified
  *        and checked for being up-to-date.
  *
  * This class provides means to assign timestamps to arbitrary objects
  * such as instances of classes, functions etc. Dependencies between
  * objects can also be specified. The purpose is to check whether one
  * or more dependencies were already set up or are up-to-date and to
  * skip them where appropriate. The aim is to avoid potentially
  * expensive recomputations.
  *
  * \warning The timestamp mechanism is though for computationally
  *          expensive tasks. If objects do not take up enough time
  *          (at least one clock tick, which in general is less than
  *          a millisecond) the timestamp between two distinctive
  *          objects does not differ (even if they were executed
  *          correctly and consecutively).
  *
  * Here is a more elaborate example showing the programming idiom for
  * the timestamp class (you might interprete the wait function as some
  * time consuming code):
  *
  * \code
  *
  * #include <ctime>
  * #include <iostream>
  *
  * #include <TRTK/Timestamp.hpp>
  *
  *
  * void wait(const double seconds)
  * {
  *     clock_t time = std::clock();
  *
  *     while (double(std::clock() - time) / CLOCKS_PER_SEC < seconds)
  *     {
  *         // do nothing
  *     }
  * }
  *
  *
  * class Demo
  * {
  * public:
  *
  *     Demo()
  *     {
  *         // Initialize the object dependencies.
  *
  *         timestamp["B"].addDependency("A");
  *
  *         timestamp["D"].addDependency("B");
  *         timestamp["D"].addDependency("C");
  *     }
  *
  *     void A()
  *     {
  *         std::cout << "    A is called." << std::endl;
  *
  *         wait(0.5);
  *
  *         timestamp["A"].update();
  *     }
  *
  *     void B()
  *     {
  *         std::cout << "    B is called." << std::endl;
  *
  *         if (timestamp["B"].dependenciesHaveChanged())
  *         {
  *             if (timestamp["B"].dependencyHasChanged("A")) A();
  *
  *             wait(0.5);
  *
  *             timestamp["B"].update();
  *         }
  *     }
  *
  *     void C()
  *     {
  *         std::cout << "    C is called." << std::endl;
  *
  *         wait(0.5);
  *
  *         timestamp["C"].update();
  *     }
  *
  *     void D()
  *     {
  *         std::cout << "    D is called." << std::endl;
  *
  *         if (timestamp["D"].dependenciesHaveChanged())
  *         {
  *             if (timestamp["D"].dependencyHasChanged("B")) B();
  *             if (timestamp["D"].dependencyHasChanged("C")) C();
  *
  *             wait(0.5);
  *
  *             timestamp["D"].update();
  *         }
  *     }
  *
  * private:
  *
  *     TRTK::Timestamp timestamp;
  * };
  *
  *
  * int main()
  * {
  *     Demo demo;
  *
  *     std::cout << "Call D:" << std::endl;
  *     demo.D();
  *
  *     std::cout << "Call D:" << std::endl;
  *     demo.D();
  *
  *     std::cout << "Call A:" << std::endl;
  *     demo.A();
  *
  *     std::cout << "Call D:" << std::endl;
  *     demo.D();
  *
  *     std::cout << "Call D:" << std::endl;
  *     demo.D();
  *
  *     return 0;
  * }
  *
  * \endcode
  *
  * The output is
  *
  * \code
  *
  * Call D:
  *     D is called.
  *     B is called.
  *     A is called.
  *     C is called.
  * Call D:
  *     D is called.
  * Call A:
  *     A is called.
  * Call D:
  *     D is called.
  *     B is called.
  *     A is called.
  * Call D:
  *     D is called.
  *
  * \endcode
  *
  * \see ObjectInfo
  *
  * \author  Christoph Haenisch
  * \version 0.3.0
  * \date    last changed on 2014-07-01
  */

class Timestamp
{
public:
    Timestamp();
    virtual ~Timestamp();

    class ObjectInfo;
    ObjectInfo & operator[](const std::string & object);

private:
    std::map<std::string, ObjectInfo> objects;
};


/** \brief This class is a helper class for the Timestamp class and
  *        holds various information of an object like the timestamp,
  *        its dependencies etc.
  *
  * \author  Christoph Haenisch
  */

class Timestamp::ObjectInfo
{
public:
    ObjectInfo(); //!< Do not use it; it will cause an assertion to fire!
    ObjectInfo(Timestamp & parent_collection);
    virtual ~ObjectInfo();

    void addDependency(const std::string & object_name);
    void removeDependency(const std::string & object_name);

    bool dependenciesHaveChanged() const;
    bool dependencyHasChanged(const std::string & other_object) const;
    bool isMoreRecentThan(const std::string & object, bool dependencies = true) const;
    void update();

private:
    bool dependenciesUpToDate(const clock_t &) const;

    Timestamp & parent_collection; // contains all InfoObjects (that is the other dependencies)
    clock_t timestamp;
    std::set<std::string> dependencies;
};


} // namespace TRTK


#endif // TIMESTAMP_HPP_4765312314
