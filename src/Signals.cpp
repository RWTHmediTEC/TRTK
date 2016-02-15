/*
    This class provides basic signals and slots support.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 1.2.2 (2014-07-03)
*/

/** \file Signals.cpp
  * \brief This file contains function definitions related to the
  *        \ref TRTK::Signals::Signal "signal and slot" concept.
  */


#include "TRTK/Signals.hpp"


using namespace std;


namespace TRTK
{


namespace Signals
{


////////////////////////////////////////////////////////////////////////////////
//                             SlotBase and Slot                              //
////////////////////////////////////////////////////////////////////////////////


SlotBase::SlotBase()
{
    for (unsigned i = 0; i < sizeof(function_address); ++i)
    {
        function_address[i] = 0;
    }
}


bool operator==(const SlotBase & x, const SlotBase & y)
{
    if (x.getObject() == NULL && y.getObject() == NULL)
    {
        // global function

        const char * function_address_x = x.getGlobalFunction();
        const char * function_address_y = y.getGlobalFunction();

        bool is_equal = true;

        for (int i = 0; i < SlotBase::MAX_FUNCTION_POINTER_SIZE; ++i)
        {
            is_equal = is_equal && (function_address_x[i] == function_address_y[i]);
        }

        return is_equal;
    }
    else if (x.getObject() == y.getObject()) // We checked 'x.getObject() != NULL' above.
    {
        // class function

        const char * function_address_x = x.getClassFunction();
        const char * function_address_y = y.getClassFunction();

        bool is_equal = true;

        for (int i = 0; i < SlotBase::MAX_FUNCTION_POINTER_SIZE; ++i)
        {
            is_equal = is_equal && (function_address_x[i] == function_address_y[i]);
        }

        return is_equal;
    }
    else
    {
        return false;
    }
}


////////////////////////////////////////////////////////////////////////////////
//                                   Receiver                                 //
////////////////////////////////////////////////////////////////////////////////


Receiver::Receiver()
{
};


// Informs all signals, which are connected to a class derived from Receiver,
// from its destruction.

Receiver::~Receiver()
{
    // Automatically disconnect all signals.

    list<pair<const SignalBase *, SlotAdapterBase *> >::iterator it;

    for (it = m_connected_signals.begin(); it != m_connected_signals.end(); ++it)
    {
        const SignalBase * signal = (*it).first;
        SlotAdapterBase * slot = (*it).second;
        signal->disconnectReceiver(slot);
    }
};


// If a signal is connected to a class derived from Receiver, it calls this function.

void Receiver::notifyConnection(const SignalBase * signal, SlotAdapterBase * slot) const
{
    m_connected_signals.push_back(make_pair(signal, slot));
}


// If a signal connected to a class derived from Receiver is destroyed or entirely
// disconnected, it calls this function.

void Receiver::notifyDisconnection(const SignalBase * signal) const
{
    // Delete all entries that refer to 'signal'.

    list<pair<const SignalBase *, SlotAdapterBase *> >::iterator it = m_connected_signals.begin();

    while (it != m_connected_signals.end())
    {
        if (signal == (*it).first)
        {
            it = m_connected_signals.erase(it);
        }
        else
        {
            ++it;
        }
    }
}


} // namespace Signals


} // namespace TRTK
