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

/** \file Signals.hpp
  * \brief This file contains all classes and function related to the
  *        \ref TRTK::Signals::Signal "signal and slot" concept.
  */

#ifndef SIGNALS_HPP_6243120120
#define SIGNALS_HPP_6243120120


#include <cstddef>
#include <cstring>
#include <list>
#include <set>
#include <stdexcept>
#include <utility>

#include "Tools.hpp"


/*

How does the implementation of the signal/slot concecpt work?

The signal/slot concecpt works according to the publish/subscribe pattern. A
signal with certain properties is defined and subscribers with the same method
signature are able to connect to this signal. If the signal is called (or emitted
or send), the subscribers are informed (or called) as well. Everything is done
transparently.

Example:

    class A
    {
    public:
        // ...

        void run()
        {
            // ...
            ++value;
            hasChanged.send(value);
        }

        Signal<int> hasChanged;

    private:
        int value;
    };

    class B
    {
    public:
        void function(int) {};
    };

    A a;
    B b;

    a.hasChanged.connect(&b, &B::function);
    a.run(); // calls also b.function()

B::function must have the same signature (int as first argument) as the signal
hasChanged.

So, at the beginning, a signal must be defined which fixes the signature -- in our
case the number and type of the input arguments. This can be easily done with
templates. But, since the number of input arguments may vary, we need to specialize
them. This is all done with(in) the Signal class.

With this information, appropriate connect functions are created. These are
overloaded such that connecting to different slot types is possible. Slots may
be global functions as well as static, non-static, constant, and non-constant
member functions. We need to abstract these different types to be able to call
them over the same interface. This is done with functors (GlobalFunctionSlot,
MemberFunctionSlot, etc.) which are basically functions with an internal state.
These are all derived from the same interface (SlotBase) whereupon we have to take
into account the various numbers of input parameters (hence again the specialized
Slot classes).

Managing the connections is done in the SignalBase class. Since all slots are
derived from SlotBase, we can easily store pointers of this class. This way, we
do not have to cope with different signatures. Calling a slot is done in the
derived class, so that (after back casting) an appropriate call is easily possible.

SignalBase interacts with the Receiver class. If a class is derived from Receiver,
disconnecting the slots is done fully automatically (this covers the destruction of
signals or objects derived from the Receiver class as well as ordinary
disconnections).

This is, basically, the overall logic. Everything around is more or less glue code.

One thing shall be noted, yet: Casting arbitrary pointers to void * is not possible
with all compilers (in fact, it is not standard C++), since their implementation size
might vary. For instance, a member function pointer might save the address to its
member function plus an additional object address. That is why conversions between
ordinary function pointers and member function pointers is not allowed. A way out of
this is to use a field of characters and to copy the function addresses by using
memcpy().

Addendum:

All methods of the Signal class as well as of the Receiver class were changed to
be constant. This allows constant member functions of classes that hold various
instances of Signal to emit theses signals. To be able to do this, our class
members must be declared mutable. In addition, it is now possible to connect slots
to signals of constant classes.

Update:

A translation layer was added to be able to connect to signatures whose elements
are compatible to that of the signal. That is, now, you can connect integers to
doubles and vice versa; an implicit conversion is performed.

So the class hierarchy is as follows:

    Slot classes abstract different function types and provide a unified call interface.

        SlotBase
            Slot<>
                GlobalFunctionSlot<>
                MemberFunctionSlot<>
                ConstMemberFunction<>

    Adapter classes allow for calling slots with a slightly different signature.
    The adapters store the needed type casts internally. This allows for performing
    calls if not all information is available (e.g. when casting base class pointers
    while only the caller-side's signature [i.e. the signal signature] is known).

        SlotAdapterBase
            SlotAdapter<>
                ConcreteSlotAdapter<>

    Signals store and manage slot connections and they provide call interfaces to
    the internally stored slots. In conjunction with the Receiver class an automatic
    disconnection mechanism is provieded.

        SignalBase
            Signal<>

        Receiver

Now, instead of directly dealing with Slots (i.e. storing SlotBase pointers) the
signal classes deal with slot adapters (pointers to the SlotAdapterBase class).

*/


namespace TRTK
{


/** \brief This namespace contains various classes and functions related to
  *        the signals and slots implementation.
  */

namespace Signals
{


using TRTK::Tools::isDerivedFrom;


class SignalBase;
class SlotAdapterBase;


////////////////////////////////////////////////////////////////////////////////
//                                   Receiver                                 //
////////////////////////////////////////////////////////////////////////////////


// Classes that are derived from Receiver automatically inform signals, when
// the classes are destroyed. This avoids dangling function pointers, if a
// connection has not been disconnected manually (well, and it is way more
// comfortable...).

class Receiver
{
public:

    Receiver();

    // Informs all signals, which are connected to a class derived from Receiver,
    // from its destruction.

    virtual ~Receiver();

    // If a signal is connected to a class derived from Receiver, it calls this function.

    void notifyConnection(const SignalBase * signal, SlotAdapterBase * slot) const;

    // If a signal connected to a class derived from Receiver is destroyed, it calls this function.

    void notifyDisconnection(const SignalBase * signal) const;

    // This function (and its overloaded counterpart) extracts the pointer to the receiver
    // class if 'pointer' is derived from it. In all other cases NULL is returned.

    static const Receiver * getAddress(const Receiver * pointer)
    {
        return pointer;
    }

    static const Receiver * getAddress(const void *)
    {
        return NULL;
    }

private:

    mutable std::list<std::pair<const SignalBase *, SlotAdapterBase *> > m_connected_signals;
};


////////////////////////////////////////////////////////////////////////////////
//                             SlotBase and Slot                              //
////////////////////////////////////////////////////////////////////////////////


// WARNING: The size of a member function pointer is compiler specific and can
// even differ between function pointers of classes that use or do not use multiple
// inheritance. Thus, to be able to compare different function pointers, we use
// a big enough character array to store them. Still, the question is, what is the
// right size? It turned out, that using the size of a member function pointer of
// a class with multiple inheritance works with most/all compilers, since these
// pointers store most information and thus are the biggest ones. If this does not
// work, just increase the size of MAX_MEMBER_FUNCTION_SIZE.


class Base1
{
public:
    Base1() {};
    virtual ~Base1() {};
};


class Base2
{
public:
    Base2() {};
    virtual ~Base2() {};
};


class FunctionPointerTraits : public Base1, virtual public Base2
{
    FunctionPointerTraits() {}
    virtual ~FunctionPointerTraits() {}

    typedef void (FunctionPointerTraits::*mem_func_ptr)();

public:
    enum {MAX_FUNCTION_POINTER_SIZE = sizeof(mem_func_ptr)};
};


class SlotBase
{
public:

    SlotBase();
    virtual ~SlotBase() {};

    // Shall return the function pointer to a member function or NULL in the
    // case of global functions.

    virtual const char * getClassFunction() const {return NULL;}

    // Shall return the function pointer to a global function or NULL in the
    // case of member functions.

    virtual const char * getGlobalFunction() const {return NULL;}

    // Shall return the object address or NULL in the case of global functions.

    virtual const void * getObject() const {return NULL;}

    // If the slot points to a member function and the object is derived from
    // Receiver, this function shall return true.

    virtual bool isReceiver() const {return false;}

    // If an object associated with a slot is derived from Receiver this function
    // shall return the receiver address and otherwise NULL.

    virtual const Receiver * getReceiver() const {return NULL;}

    enum {MAX_FUNCTION_POINTER_SIZE = FunctionPointerTraits::MAX_FUNCTION_POINTER_SIZE};

protected:

    char function_address[MAX_FUNCTION_POINTER_SIZE];
};


bool operator==(const SlotBase & x, const SlotBase & y);



// four arguments specified

template <typename arg1_type = void, typename arg2_type = void, typename arg3_type = void, typename arg4_type = void>
class Slot : public SlotBase
{
public:

    // Use this function to call the appropriate slot.

    virtual void callFunction(arg1_type, arg2_type, arg3_type, arg4_type) = 0;
};


// three arguments specified

template <typename arg1_type, typename arg2_type, typename arg3_type>
class Slot<arg1_type, arg2_type, arg3_type, void> : public SlotBase
{
public:

    // Use this function to call the appropriate slot.

    virtual void callFunction(arg1_type, arg2_type, arg3_type) = 0;
};


// two arguments specified

template <typename arg1_type, typename arg2_type>
class Slot<arg1_type, arg2_type, void, void> : public SlotBase
{
public:

    // Use this function to call the appropriate slot.

    virtual void callFunction(arg1_type, arg2_type) = 0;
};


// one argument specified

template <typename arg1_type>
class Slot<arg1_type, void, void, void> : public SlotBase
{
public:

    // Use this function to call the appropriate slot.

    virtual void callFunction(arg1_type) = 0;
};


// no argument specified

template <>
class Slot<void, void, void, void> : public SlotBase
{
public:

    // Use this function to call the appropriate slot.

    virtual void callFunction(void) = 0;
};


////////////////////////////////////////////////////////////////////////////////
//                             GlobalFunctionSlot                             //
////////////////////////////////////////////////////////////////////////////////


// four arguments specified

template <typename return_type, typename arg1_type = void, typename arg2_type = void, typename arg3_type = void, typename arg4_type = void>
class GlobalFunctionSlot : public Slot<arg1_type, arg2_type, arg3_type, arg4_type>
{
public:
    typedef return_type (*function_ptr)(arg1_type, arg2_type, arg3_type, arg4_type);

    GlobalFunctionSlot(function_ptr function)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    }

    void callFunction(arg1_type arg1, arg2_type arg2, arg3_type arg3, arg4_type arg4)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (*function)(arg1, arg2, arg3, arg4);
    }

    const char * getGlobalFunction() const
    {
        return SlotBase::function_address;
    }
};


// three arguments specified

template <typename return_type, typename arg1_type, typename arg2_type, typename arg3_type>
class GlobalFunctionSlot<return_type, arg1_type, arg2_type, arg3_type, void> : public Slot<arg1_type, arg2_type, arg3_type>
{
public:
    typedef return_type (*function_ptr)(arg1_type, arg2_type, arg3_type);

    GlobalFunctionSlot(function_ptr function)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    }

    void callFunction(arg1_type arg1, arg2_type arg2, arg3_type arg3)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (*function)(arg1, arg2, arg3);
    }

    const char * getGlobalFunction() const
    {
        return SlotBase::function_address;
    }
};


// two arguments specified

template <typename return_type, typename arg1_type, typename arg2_type>
class GlobalFunctionSlot<return_type, arg1_type, arg2_type, void, void> : public Slot<arg1_type, arg2_type>
{
public:
    typedef return_type (*function_ptr)(arg1_type, arg2_type);

    GlobalFunctionSlot(function_ptr function)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    }

    void callFunction(arg1_type arg1, arg2_type arg2)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (*function)(arg1, arg2);
    }

    const char * getGlobalFunction() const
    {
        return SlotBase::function_address;
    }
};


// one argument specified

template <typename return_type, typename arg1_type>
class GlobalFunctionSlot<return_type, arg1_type, void, void, void> : public Slot<arg1_type>
{
public:
    typedef return_type (*function_ptr)(arg1_type);

    GlobalFunctionSlot(function_ptr function)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    }

    void callFunction(arg1_type arg1)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (*function)(arg1);
    }

    const char * getGlobalFunction() const
    {
        return SlotBase::function_address;
    }
};


// no argument specified

template <typename return_type>
class GlobalFunctionSlot<return_type, void> : public Slot<void>
{
public:
    typedef return_type (*function_ptr)();

    GlobalFunctionSlot(function_ptr function)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    }

    void callFunction()
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (*function)();
    }

    const char * getGlobalFunction() const
    {
        return SlotBase::function_address;
    }
};


////////////////////////////////////////////////////////////////////////////////
//                             MemberFunctionSlot                             //
////////////////////////////////////////////////////////////////////////////////


// four arguments specified

template <typename class_type, typename return_type, typename arg1_type = void, typename arg2_type = void, typename arg3_type = void, typename arg4_type = void>
class MemberFunctionSlot : public Slot<arg1_type, arg2_type, arg3_type, arg4_type>
{
public:
    typedef return_type (class_type::*function_ptr)(arg1_type, arg2_type, arg3_type, arg4_type);

    MemberFunctionSlot(class_type * object, function_ptr function) : object(object)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    };

    void callFunction(arg1_type arg1, arg2_type arg2, arg3_type arg3, arg4_type arg4)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (object->*function)(arg1, arg2, arg3, arg4);
    }

    const char * getClassFunction() const
    {
        return SlotBase::function_address;
    }

    const void * getObject() const
    {
        return object;
    }

    const Receiver * getReceiver() const
    {
        return Receiver::getAddress(object);
    }

private:
    class_type * object;
};


// three arguments specified

template <typename class_type, typename return_type, typename arg1_type, typename arg2_type, typename arg3_type>
class MemberFunctionSlot<class_type, return_type, arg1_type, arg2_type, arg3_type, void> : public Slot<arg1_type, arg2_type, arg3_type>
{
public:
    typedef return_type (class_type::*function_ptr)(arg1_type, arg2_type, arg3_type);

    MemberFunctionSlot(class_type * object, function_ptr function) : object(object)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    };

    void callFunction(arg1_type arg1, arg2_type arg2, arg3_type arg3)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (object->*function)(arg1, arg2, arg3);
    }

    const char * getClassFunction() const
    {
        return SlotBase::function_address;
    }

    const void * getObject() const
    {
        return object;
    }

    const Receiver * getReceiver() const
    {
        return Receiver::getAddress(object);
    }

private:
    class_type * object;
};


// two arguments specified

template <typename class_type, typename return_type, typename arg1_type, typename arg2_type>
class MemberFunctionSlot<class_type, return_type, arg1_type, arg2_type, void, void> : public Slot<arg1_type, arg2_type>
{
public:
    typedef return_type (class_type::*function_ptr)(arg1_type, arg2_type);

    MemberFunctionSlot(class_type * object, function_ptr function) : object(object)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    };

    void callFunction(arg1_type arg1, arg2_type arg2)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (object->*function)(arg1, arg2);
    }

    const char * getClassFunction() const
    {
        return SlotBase::function_address;
    }

    const void * getObject() const
    {
        return object;
    }

    const Receiver * getReceiver() const
    {
        return Receiver::getAddress(object);
    }


private:
    class_type * object;
};


// one argument specified

template <typename class_type, typename return_type, typename arg1_type>
class MemberFunctionSlot<class_type, return_type, arg1_type, void, void, void> : public Slot<arg1_type>
{
public:
    typedef return_type (class_type::*function_ptr)(arg1_type);

    MemberFunctionSlot(class_type * object, function_ptr function) : object(object)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    };

    void callFunction(arg1_type arg1)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (object->*function)(arg1);
    }

    const char * getClassFunction() const
    {
        return SlotBase::function_address;
    }

    const void * getObject() const
    {
        return object;
    }

    const Receiver * getReceiver() const
    {
        return Receiver::getAddress(object);
    }

private:
    class_type * object;
};


// no argument specified

template <typename class_type, typename return_type>
class MemberFunctionSlot<class_type, return_type, void, void, void, void> : public Slot<void>
{
public:
    typedef return_type (class_type::*function_ptr)();

    MemberFunctionSlot(class_type * object, function_ptr function) : object(object)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    };

    void callFunction()
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (object->*function)();
    }

    const char * getClassFunction() const
    {
        return SlotBase::function_address;
    }

    const void * getObject() const
    {
        return object;
    }

    const Receiver * getReceiver() const
    {
        return Receiver::getAddress(object);
    }

private:
    class_type * object;
};


////////////////////////////////////////////////////////////////////////////////
//                          ConstMemberFunctionSlot                           //
////////////////////////////////////////////////////////////////////////////////


// four arguments specified

template <typename class_type, typename return_type, typename arg1_type = void, typename arg2_type = void, typename arg3_type = void, typename arg4_type = void>
class ConstMemberFunctionSlot : public Slot<arg1_type, arg2_type, arg3_type, arg4_type>
{
public:
    typedef return_type (class_type::*function_ptr)(arg1_type, arg2_type, arg3_type, arg4_type) const;

    ConstMemberFunctionSlot(const class_type * object, function_ptr function) : object(object)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    };

    void callFunction(arg1_type arg1, arg2_type arg2, arg3_type arg3, arg4_type arg4)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (object->*function)(arg1, arg2, arg3, arg4);
    }

    const char * getClassFunction() const
    {
        return SlotBase::function_address;
    }

    const void * getObject() const
    {
        return object;
    }

    const Receiver * getReceiver() const
    {
        return Receiver::getAddress(object);
    }

private:
    const class_type * object;
};


// three arguments specified

template <typename class_type, typename return_type, typename arg1_type, typename arg2_type, typename arg3_type>
class ConstMemberFunctionSlot<class_type, return_type, arg1_type, arg2_type, arg3_type, void> : public Slot<arg1_type, arg2_type, arg3_type>
{
public:
    typedef return_type (class_type::*function_ptr)(arg1_type, arg2_type, arg3_type) const;

    ConstMemberFunctionSlot(const class_type * object, function_ptr function) : object(object)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    };

    void callFunction(arg1_type arg1, arg2_type arg2, arg3_type arg3)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (object->*function)(arg1, arg2, arg3);
    }

    const char * getClassFunction() const
    {
        return SlotBase::function_address;
    }

    const void * getObject() const
    {
        return object;
    }

    const Receiver * getReceiver() const
    {
        return Receiver::getAddress(object);
    }

private:
    const class_type * object;
};


// two arguments specified

template <typename class_type, typename return_type, typename arg1_type, typename arg2_type>
class ConstMemberFunctionSlot<class_type, return_type, arg1_type, arg2_type, void, void> : public Slot<arg1_type, arg2_type>
{
public:
    typedef return_type (class_type::*function_ptr)(arg1_type, arg2_type) const;

    ConstMemberFunctionSlot(const class_type * object, function_ptr function) : object(object)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    };

    void callFunction(arg1_type arg1, arg2_type arg2)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (object->*function)(arg1, arg2);
    }

    const char * getClassFunction() const
    {
        return SlotBase::function_address;
    }

    const void * getObject() const
    {
        return object;
    }

    const Receiver * getReceiver() const
    {
        return Receiver::getAddress(object);
    }


private:
    const class_type * object;
};


// one argument specified

template <typename class_type, typename return_type, typename arg1_type>
class ConstMemberFunctionSlot<class_type, return_type, arg1_type, void, void, void> : public Slot<arg1_type>
{
public:
    typedef return_type (class_type::*function_ptr)(arg1_type) const;

    ConstMemberFunctionSlot(const class_type * object, function_ptr function) : object(object)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    };

    void callFunction(arg1_type arg1)
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (object->*function)(arg1);
    }

    const char * getClassFunction() const
    {
        return SlotBase::function_address;
    }

    const void * getObject() const
    {
        return object;
    }

    const Receiver * getReceiver() const
    {
        return Receiver::getAddress(object);
    }

private:
    const class_type * object;
};


// no argument specified

template <typename class_type, typename return_type>
class ConstMemberFunctionSlot<class_type, return_type, void, void, void, void> : public Slot<void>
{
public:
    typedef return_type (class_type::*function_ptr)() const;

    ConstMemberFunctionSlot(const class_type * object, function_ptr function) : object(object)
    {
        // store function pointer
        memcpy(SlotBase::function_address, &function, sizeof(function));
    };

    void callFunction()
    {
        // restore function pointer
        function_ptr function;
        memcpy(&function, SlotBase::function_address, sizeof(function));

        (object->*function)();
    }

    const char * getClassFunction() const
    {
        return SlotBase::function_address;
    }

    const void * getObject() const
    {
        return object;
    }

    const Receiver * getReceiver() const
    {
        return Receiver::getAddress(object);
    }

private:
    const class_type * object;
};


////////////////////////////////////////////////////////////////////////////////
//                                Slot Adapters                               //
////////////////////////////////////////////////////////////////////////////////


class SlotAdapterBase
{
public:
    SlotAdapterBase() : slot(NULL)
    {
    }

    virtual ~SlotAdapterBase()
    {
    }

    const char * getClassFunction() const
    {
        assert(slot != NULL);
        return slot->getClassFunction();
    }

    const char * getGlobalFunction() const
    {
        assert(slot != NULL);
        return slot->getGlobalFunction();
    }

    virtual unsigned getNumberOfArguments() const = 0;

    const void * getObject() const
    {
        assert(slot != NULL);
        return slot->getObject();
    }

    const Receiver * getReceiver() const
    {
        assert(slot != NULL);
        return slot->getReceiver();
    }

    bool operator==(const SlotAdapterBase & other)
    {
        assert(slot != NULL);
        return *slot == *other.slot;
    }

protected:
    SlotBase * slot;
};


// four arguments specified

template <typename arg1_type = void, typename arg2_type = void, typename arg3_type = void, typename arg4_type = void>
class SlotAdapter : public SlotAdapterBase
{
public:
    virtual void callFunction(arg1_type, arg2_type, arg3_type, arg4_type) = 0;
};


// three arguments specified

template <typename arg1_type, typename arg2_type, typename arg3_type>
class SlotAdapter<arg1_type, arg2_type, arg3_type, void> : public SlotAdapterBase
{
public:
    virtual void callFunction(arg1_type, arg2_type, arg3_type) = 0;
};


// two arguments specified

template <typename arg1_type, typename arg2_type>
class SlotAdapter<arg1_type, arg2_type, void, void> : public SlotAdapterBase
{
public:
    virtual void callFunction(arg1_type, arg2_type) = 0;
};


// one argument specified

template <typename arg1_type>
class SlotAdapter<arg1_type, void, void, void> : public SlotAdapterBase
{
public:
    virtual void callFunction(arg1_type) = 0;
};


// zero arguments specified

template <>
class SlotAdapter<void, void, void, void> : public SlotAdapterBase
{
public:
    virtual void callFunction() = 0;
};


// four arguments specified

template <typename arg1_type = void, typename arg2_type = void, typename arg3_type = void, typename arg4_type = void,
          typename slot_arg1_type = void, typename slot_arg2_type = void, typename slot_arg3_type = void, typename slot_arg4_type = void>
class ConcreteSlotAdapter : public SlotAdapter<arg1_type, arg2_type, arg3_type, arg4_type>
{
public:
    typedef Slot<slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type> * slot_ptr;

    // ConcreteSlotAdapter takes the ownership of 'slot'.

    ConcreteSlotAdapter(slot_ptr slot)
    {
        SlotAdapterBase::slot = slot;
    }

    ~ConcreteSlotAdapter()
    {
        delete SlotAdapterBase::slot;
    }

    void callFunction(arg1_type arg1, arg2_type arg2, arg3_type arg3, arg4_type arg4)
    {
        static_cast<slot_ptr>(SlotAdapterBase::slot)->callFunction(static_cast<slot_arg1_type>(arg1),
                                                                   static_cast<slot_arg2_type>(arg2),
                                                                   static_cast<slot_arg3_type>(arg3),
                                                                   static_cast<slot_arg4_type>(arg4));
    }

    unsigned getNumberOfArguments() const
    {
        return 4;
    }
};


// three arguments specified

template <typename arg1_type, typename arg2_type, typename arg3_type,
          typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
class ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void>
    : public SlotAdapter<arg1_type, arg2_type, arg3_type>
{
public:
    typedef Slot<slot_arg1_type, slot_arg2_type, slot_arg3_type> * slot_ptr;

    // ConcreteSlotAdapter takes the ownership of 'slot'.

    ConcreteSlotAdapter(slot_ptr slot)
    {
        SlotAdapterBase::slot = slot;
    }

    ~ConcreteSlotAdapter()
    {
        delete SlotAdapterBase::slot;
    }

    void callFunction(arg1_type arg1, arg2_type arg2, arg3_type arg3)
    {
        static_cast<slot_ptr>(SlotAdapterBase::slot)->callFunction(static_cast<slot_arg1_type>(arg1),
                                                                   static_cast<slot_arg2_type>(arg2),
                                                                   static_cast<slot_arg3_type>(arg3));
    }

    unsigned getNumberOfArguments() const
    {
        return 3;
    }
};


// two arguments specified

template <typename arg1_type, typename arg2_type, typename slot_arg1_type, typename slot_arg2_type>
class ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void>
    : public SlotAdapter<arg1_type, arg2_type>
{
public:
    typedef Slot<slot_arg1_type, slot_arg2_type> * slot_ptr;

    // ConcreteSlotAdapter takes the ownership of 'slot'.

    ConcreteSlotAdapter(slot_ptr slot)
    {
        SlotAdapterBase::slot = slot;
    }

    ~ConcreteSlotAdapter()
    {
        delete SlotAdapterBase::slot;
    }

    void callFunction(arg1_type arg1, arg2_type arg2)
    {
        static_cast<slot_ptr>(SlotAdapterBase::slot)->callFunction(static_cast<slot_arg1_type>(arg1), static_cast<slot_arg2_type>(arg2));
    }

    unsigned getNumberOfArguments() const
    {
        return 2;
    }
};


// one argument specified

template <typename arg1_type, typename slot_arg1_type>
class ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void>
    : public SlotAdapter<arg1_type>
{
public:
    typedef Slot<slot_arg1_type> * slot_ptr;

    // ConcreteSlotAdapter takes the ownership of 'slot'.

    ConcreteSlotAdapter(slot_ptr slot)
    {
        SlotAdapterBase::slot = slot;
    }

    ~ConcreteSlotAdapter()
    {
        delete SlotAdapterBase::slot;
    }

    void callFunction(arg1_type arg1)
    {
        static_cast<slot_ptr>(SlotAdapterBase::slot)->callFunction(static_cast<slot_arg1_type>(arg1));
    }

    unsigned getNumberOfArguments() const
    {
        return 1;
    }
};


// zero arguments specified

template <>
class ConcreteSlotAdapter<void, void, void, void, void, void, void, void>
    : public SlotAdapter<>
{
public:
    typedef Slot<> * slot_ptr;

    // ConcreteSlotAdapter takes the ownership of 'slot'.

    ConcreteSlotAdapter(slot_ptr slot)
    {
        SlotAdapterBase::slot = slot;
    }

    ~ConcreteSlotAdapter()
    {
        delete SlotAdapterBase::slot;
    }

    void callFunction()
    {
        static_cast<slot_ptr>(SlotAdapterBase::slot)->callFunction();
    }

    unsigned getNumberOfArguments() const
    {
        return 0;
    }
};


////////////////////////////////////////////////////////////////////////////////
//                               ConnectionType                               //
////////////////////////////////////////////////////////////////////////////////


/** \relates Signal
  *
  * \brief Slot connection type.
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2011-08-25
  */

enum ConnectionType
{
    MULTIPLE_CONNECTIONS,   ///< Allows multiple connections of the same slot to a certain signal.
    SINGLE_CONNECTION       ///< Allows only a single connection of a certain slot to a certain signal. If multiple connections were established before, the number of connections remains the same.
};


////////////////////////////////////////////////////////////////////////////////
//                                 SignalBase                                 //
////////////////////////////////////////////////////////////////////////////////


class SignalBase
{
public:

    SignalBase() {}

    virtual ~SignalBase()
    {
        // Inform classes that are derived from 'Receiver' that the signal has been destroyed.

        std::set<const Receiver *>::iterator it;

        for (it = m_receivers.begin(); it != m_receivers.end(); ++it)
        {
            (*it)->notifyDisconnection(this);
        }

        // Delete all stored slots.

        std::list<SlotAdapterBase *>::iterator slot_it = m_slots.begin();

        while (slot_it != m_slots.end())
        {
            delete *slot_it;
            m_slots.erase(slot_it++);
        }
    }

    // Save/manage the connection to a signal.
    // Note: The deletion of the input argument slot is managed by SignalBase.

    void connect(SlotAdapterBase * slot, const ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        switch (connection_type)
        {
            case MULTIPLE_CONNECTIONS:
                // Fall through...
                break;

            case SINGLE_CONNECTION:
                if (isConnected(slot))
                {
                    return;
                }
                else
                {
                    // Fall through...
                }
                break;

            default:
                throw std::logic_error("SignalBase::connect(): Unknown connection type.");
                break;
        }

        m_slots.push_back(slot);

        // Notify 'Receiver' objects about the connection.

        if (slot->getReceiver())
        {
            const Receiver * receiver = slot->getReceiver();
            m_receivers.insert(receiver);
            receiver->notifyConnection(this, slot);
        }
    }

    // Deletes the last added entry in the case of SINGLE_CONNECTION.
    // Deletes all entries in the case of MULTIPLE_CONNECTIONS.

    void disconnect(SlotAdapterBase * slot, const ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        // Delete slot(s).

        std::list<SlotAdapterBase *>::reverse_iterator rit = m_slots.rbegin();

        while (rit != m_slots.rend())
        {
            if (**rit == *slot)
            {
                delete *rit;
                rit.base() = m_slots.erase(--rit.base());

                if (connection_type == SINGLE_CONNECTION)
                {
                    break;
                }
            }
            else
            {
                ++rit;
            }
        }

        // Update the set of 'Receiver' objects.

        // There may be connections to other slots of the receiver object, hence
        // check how many are left and erase the receiver from the internal set
        // if necessary. Also notify the receiver object if applicable.

        const Receiver * receiver = slot->getReceiver();

        if (receiver)
        {
            bool any_connections = false; // are there any connections left to the receiver object

            std::list<SlotAdapterBase *>::iterator it;

            for (it = m_slots.begin(); it != m_slots.end(); ++it)
            {
                if ((*it)->getReceiver() == receiver)
                {
                    any_connections = true;
                    break;
                }
            }

            if (!any_connections)
            {
                m_receivers.erase(receiver);
                receiver->notifyDisconnection(this);
            }
        }
    }

    // This function is called by 'Receiver' objects during their destruction
    // (automatic slot removal).

    void disconnectReceiver(const SlotAdapterBase * slot) const
    {
        std::list<SlotAdapterBase *>::iterator it = m_slots.begin();

        while (it != m_slots.end())
        {
            if (*it == slot)
            {
                it = m_slots.erase(it);
                break;
            }
            else
            {
                ++it;
            }
        }

        m_receivers.erase(slot->getReceiver());
    }

    // Checks, whether a certain function/slot is already connected to this signal.

    bool isConnected(const SlotAdapterBase * slot) const
    {
        std::list<SlotAdapterBase *>::const_iterator it;

        for (it = m_slots.begin(); it != m_slots.end(); ++it)
        {
            if (**it == *slot)
            {
                return true;
            }
        }

        return false;
    }

protected:

    mutable std::list<SlotAdapterBase *> m_slots;
    mutable std::set<const Receiver *> m_receivers;
};


////////////////////////////////////////////////////////////////////////////////
//                                    Signal                                  //
////////////////////////////////////////////////////////////////////////////////


class Receiver;

/** \brief Basic signals and slots support.
  *
  * Signals and slots are callbacks with multiple targets. They work according to
  * the publish/subscribe pattern which facilitates decoupling and modularization.
  * A signal with certain properties is defined and subscribers with the same method
  * signature are able to connect to this signal (or in other words: if a signal is
  * instanciated, only a defined set of functions is able to be connected to the
  * signal). If the signal is called (or emitted or send), the subscribers are
  * informed (or called) as well. A publisher does not care about its
  * subscribers---it just publishes and the subscribers are informed. All this is
  * done fully transparently. (See below for some examples.)
  *
  * This signals and slots implementation is compatible with Qt, which means that it
  * does not interfere with Qt's own signals and slots concept as well as its defined
  * keywords \e emit, \e signals and \e slots.
  *
  * The interface is designed to be flexible and easy to use. Connections are even
  * managed fully automatically in the case that objects are (virtually) derived from
  * the Receiver class.
  *
  * \code
  * class Object : virtual public Receiver
  * {
  *     // ...
  * };
  * \endcode
  *
  * A signal is defined by instanciating the Signal template class. For instance:
  *
  * \code
  * Signal<int> signal;
  * \endcode
  *
  * All functions that take an integer as its first argument can be connected to
  * the above defined signal. This implies global functions as well as all kinds of
  * member functions where the type of the return value can be arbitrary.
  *
  * \note If a function's signature differs from the signal's signature, a connection
  *       is still possible if the argument types can be converted to each other.
  *       For instance, the following function can still be connected to the above
  *       signal: \code void function(double); \endcode
  *
  * </p>
  *
  * \note Since version 1.2.0 it is also possible to <b> connect to functions with
  *       less parameters </b> than those given in the signature of the signal. For
  *       instance, it is also possible to connect to the above signal a slot like
  *       \code void function(); \endcode
  *
  * Up to four parameters can be specified:
  *
  * \code
  * Signal<arg1_type, arg2_type, arg3_type, arg4_type> signal;
  * \endcode
  *
  * Now, a connection can be established as follows:
  *
  * \code
  * signal.connect(&function);              // global function slot
  * signal.connect(&object, &function);     // member function slot
  * \endcode
  *
  * If a signal is emitted (or send) the connected functions are called in the
  * order they were connected to a signal. Connections can either be unique or
  * multiple-connected. The former is the default behaviour. To change this, you
  * can specify the connection type, for instance:
  *
  * \code
  * signal.connect(&function, TRTK::Signals::MULTIPLE_CONNECTIONS);
  * \endcode
  *
  * A slot is disconnected as follows:
  *
  * \code
  * signal.disconnect(&function);           // global function slot
  * signal.disconnect(&object, &function);  // member function slot
  * \endcode
  *
  * Always, the last found/connected slot is disconnected (or all slots if
  * <tt>MULTIPLE_CONNECTIONS</tt> is passed as an additional argument). It
  * follows, that the former connection order is preserved. If \c object in the
  * former example is derived from the Receiver class, all connections are
  * disconnected automatically during the destruction of this object. This
  * avoids hanging references within the signal.
  *
  * Now, we will present an example to show, how signals and slots can be used.
  *
  * Example:
  *
  * \code
  *
  * #include <iostream>
  * #include <TRTK/Clock.hpp>
  * #include <TRTK/Signals.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  *
  *
  * class DisplayClass // This could be a status bar, for instance.
  * {
  * public:
  *     void print(unsigned int value)
  *     {
  *         cout << value << endl;
  *     }
  * };
  *
  *
  * class CountDown
  * {
  * public:
  *     CountDown(unsigned int counter) : m_counter(counter) {}
  *
  *     void start()
  *     {
  *         for (int i = m_counter; i >= 0; --i)
  *         {
  *             Clock().wait_seconds(1);
  *             state.send(i);
  *         }
  *
  *         finished.send();
  *     }
  *
  *     Signal<unsigned int> state;
  *     Signal<void> finished;
  *
  * private:
  *     unsigned int m_counter;
  * };
  *
  *
  * void print()
  * {
  *     cout << "print() was called." << endl;
  * }
  *
  *
  * int main()
  * {
  *     CountDown countDown(10);
  *
  *     DisplayClass displayClass;
  *
  *     countDown.state.connect(&displayClass, &DisplayClass::print);
  *
  *     countDown.finished.connect(&print);
  *     countDown.finished.connect(&print, MULTIPLE_CONNECTIONS); // print is called twice
  *
  *     countDown.start();
  *
  *     // The next line could be ommited, if DisplayClass was derived from Receiver.
  *     countDown.state.disconnect(&displayClass, &DisplayClass::print);
  *     countDown.finished.connect(&print, MULTIPLE_CONNECTIONS);
  *
  *     return 0;
  * }
  *
  * \endcode
  *
  * Output:
  *
  * \code
  *
  * 10
  * 9
  * 8
  * 7
  * 6
  * 5
  * 4
  * 3
  * 2
  * 1
  * 0
  * print() was called.
  * print() was called.
  *
  * \endcode
  *
  * As we can see, now, the CountDown class is fully decoupled from its
  * graphical representation.
  *
  * \note
  *   - For convenience reasons, the Signals and the Receiver class as well as
  *     the connection types are also directly available in the TRTK namespace.
  *   - Signals can always be modified, even if they are declared \c const.
  *
  * \author Christoph Haenisch
  * \version 1.2.2
  * \date last changed on 2014-07-03
  */

// four arguments specified


template <typename arg1_type = void, typename arg2_type = void, typename arg3_type = void, typename arg4_type = void>
class Signal : public SignalBase
{
public:

    Signal() {}
    virtual ~Signal() {}

    /** \brief Connects to a non-static non-constant member function with four parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type, typename slot_arg4_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, arg4_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type>(object, function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects to a non-static non-constant member function with three parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(object, function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects to a non-static non-constant member function with two parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects to a non-static non-constant member function with one parameter.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects to a non-static non-constant member function with zero parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects to a non-static constant member function with four parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type, typename slot_arg4_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, arg4_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type>(object, function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects to a non-static constant member function with three parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(object, function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects to a non-static constant member function with two parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects to a non-static constant member function with one parameter.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects to a non-static constant member function with zero parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)() const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects a global or a static member function with four parameters.
      *
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&function);
      * \endcode
      */

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type, typename slot_arg4_type>
    void connect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, arg4_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type>(function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects a global or a static member function with three parameters.
      *
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&function);
      * \endcode
      */

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void connect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects a global or a static member function with two parameters.
      *
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&function);
      * \endcode
      */

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void connect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type>(function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects a global or a static member function with one parameter.
      *
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&function);
      * \endcode
      */

    template <typename slot_return_type, typename slot_arg1_type>
    void connect(slot_return_type (*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type>(function));
        super::connect(slot, connection_type);
    }

    /** \brief Connects a global or a static member function with zero parameters.
      *
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * Example:
      *
      * \code
      * signal.connect(&function);
      * \endcode
      */

    template <typename slot_return_type>
    void connect(slot_return_type (*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type>(function));
        super::connect(slot, connection_type);
    }

    /** \brief Disconnects a non-static non-constant member function with four parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type, typename slot_arg4_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, arg4_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a non-static non-constant member function with three parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a non-static non-constant member function with two parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a non-static non-constant member function with one parameter.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a non-static non-constant member function with zero parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a non-static constant member function with four parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type, typename slot_arg4_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, arg4_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a non-static constant member function with three parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a non-static constant member function with two parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a non-static constant member function with one parameter.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a non-static constant member function with zero parameters.
      *
      * \param [in] object              Instance whose function shall be called.
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&object, &class::function);
      * \endcode
      */

    template <typename slot_class_type, typename slot_return_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)() const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a global or a static member function with four parameters.
      *
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&function);
      * \endcode
      */

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type, typename slot_arg4_type>
    void disconnect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, arg4_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type, slot_arg4_type>(function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a global or a static member function with three parameters.
      *
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&function);
      * \endcode
      */

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void disconnect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a global or a static member function with two parameters.
      *
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&function);
      * \endcode
      */

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void disconnect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type>(function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a global or a static member function with one parameter.
      *
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&function);
      * \endcode
      */

    template <typename slot_return_type, typename slot_arg1_type>
    void disconnect(slot_return_type (*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type>(function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Disconnects a global or a static member function with zero parameters.
      *
      * \param [in] function            Function to be called.
      * \param [in] connection_type     Type of connection.
      *
      * If connection_type is SINGLE_CONNECTION, only the last found connection
      * is deleted. \n If connection_type is MULTIPLE_CONNECTIONS, all connections
      * found are deleted.
      *
      * Example:
      *
      * \code
      * signal.disconnect(&function);
      * \endcode
      */

    template <typename slot_return_type>
    void disconnect(slot_return_type (*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type>(function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    /** \brief Emits a signal.
      *
      * \param [in] arg1    First input argument of the called slot.
      * \param [in] arg2    Second input argument of the called slot.
      * \param [in] arg3    Third input argument of the called slot.
      * \param [in] arg4    Fourth input argument of the called slot.
      *
      * Currently, only up to four arguments are supported. An argument may only
      * (and then must) be given, if the signal and the corresponding slots provide
      * the particular signature.
      *
      * Example:
      *
      * \code
      * Signal<void> signal1;
      * Signal<string> signal2;
      *
      * signal1.send();
      * signal2.send("Some text.");
      * \endcode
      */

    void send(arg1_type arg1, arg2_type arg2, arg3_type arg3, arg4_type arg4) const
    {
        std::list<SlotAdapterBase *>::iterator it;

        for (it = super::m_slots.begin(); it != super::m_slots.end(); ++it)
        {
            int num_args = (*it)->getNumberOfArguments();

            switch (num_args)
            {
                case 4:
                {
                    typedef SlotAdapter<arg1_type, arg2_type, arg3_type, arg4_type> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction(arg1, arg2, arg3, arg4);
                    break;
                }

                case 3:
                {
                    typedef SlotAdapter<arg1_type, arg2_type, arg3_type> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction(arg1, arg2, arg3);
                    break;
                }

                case 2:
                {
                    typedef SlotAdapter<arg1_type, arg2_type> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction(arg1, arg2);
                    break;
                }

                case 1:
                {
                    typedef SlotAdapter<arg1_type> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction(arg1);
                    break;
                }

                case 0:
                {
                    typedef SlotAdapter<> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction();
                    break;
                }
            }
        }
    }

private:

    typedef SignalBase super;
};


// three arguments specified


template <typename arg1_type, typename arg2_type, typename arg3_type>
class Signal<arg1_type, arg2_type, arg3_type, void> : public SignalBase
{
public:

    Signal() {}
    virtual ~Signal() {}

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)() const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void connect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(function));
        super::connect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void connect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type>(function));
        super::connect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type>
    void connect(slot_return_type (*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type>(function));
        super::connect(slot, connection_type);
    }

    template <typename slot_return_type>
    void connect(slot_return_type (*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type>(function));
        super::connect(slot, connection_type);
    }





    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)() const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type, typename slot_arg3_type>
    void disconnect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type, slot_arg3_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, arg3_type, void, slot_arg1_type, slot_arg2_type, slot_arg3_type, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type, slot_arg3_type>(function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void disconnect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type>(function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type>
    void disconnect(slot_return_type (*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type>(function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_return_type>
    void disconnect(slot_return_type (*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type>(function));
        super::disconnect(slot, connection_type);
    }

    void send(arg1_type arg1, arg2_type arg2, arg3_type arg3) const
    {
        std::list<SlotAdapterBase *>::iterator it;

        for (it = super::m_slots.begin(); it != super::m_slots.end(); ++it)
        {
            int num_args = (*it)->getNumberOfArguments();

            switch (num_args)
            {
                case 3:
                {
                    typedef SlotAdapter<arg1_type, arg2_type, arg3_type> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction(arg1, arg2, arg3);
                    break;
                }

                case 2:
                {
                    typedef SlotAdapter<arg1_type, arg2_type> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction(arg1, arg2);
                    break;
                }

                case 1:
                {
                    typedef SlotAdapter<arg1_type> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction(arg1);
                    break;
                }

                case 0:
                {
                    typedef SlotAdapter<> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction();
                    break;
                }
            }
        }
    }

private:

    typedef SignalBase super;
};


// two arguments specified


template <typename arg1_type, typename arg2_type>
class Signal<arg1_type, arg2_type, void, void> : public SignalBase
{
public:

    Signal() {}
    virtual ~Signal() {}

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)() const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void connect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type>(function));
        super::connect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type>
    void connect(slot_return_type (*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type>(function));
        super::connect(slot, connection_type);
    }

    template <typename slot_return_type>
    void connect(slot_return_type (*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type>(function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type, slot_arg2_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type, slot_arg2_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)() const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type, typename slot_arg2_type>
    void disconnect(slot_return_type (*function)(slot_arg1_type, slot_arg2_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, arg2_type, void, void, slot_arg1_type, slot_arg2_type, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type, slot_arg2_type>(function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type>
    void disconnect(slot_return_type (*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type>(function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_return_type>
    void disconnect(slot_return_type (*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type>(function));
        super::disconnect(slot, connection_type);
    }

    void send(arg1_type arg1, arg2_type arg2) const
    {
        std::list<SlotAdapterBase *>::iterator it;

        for (it = super::m_slots.begin(); it != super::m_slots.end(); ++it)
        {
            int num_args = (*it)->getNumberOfArguments();

            switch (num_args)
            {
                case 2:
                {
                    typedef SlotAdapter<arg1_type, arg2_type> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction(arg1, arg2);
                    break;
                }

                case 1:
                {
                    typedef SlotAdapter<arg1_type> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction(arg1);
                    break;
                }

                case 0:
                {
                    typedef SlotAdapter<> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction();
                    break;
                }
            }
        }
    }

private:

    typedef SignalBase super;
};


// one argument specified


template <typename arg1_type>
class Signal<arg1_type, void, void, void> : public SignalBase
{
public:

    Signal() {}
    virtual ~Signal() {}

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)() const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type>
    void connect(slot_return_type (*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type>(function));
        super::connect(slot, connection_type);
    }

    template <typename slot_return_type>
    void connect(slot_return_type (*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type>(function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type, typename slot_arg1_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)(slot_arg1_type) const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type, slot_arg1_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)() const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_return_type, typename slot_arg1_type>
    void disconnect(slot_return_type (*function)(slot_arg1_type), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<arg1_type, void, void, void, slot_arg1_type, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type, slot_arg1_type>(function));
        super::disconnect(slot, connection_type);
    }

    template <typename slot_return_type>
    void disconnect(slot_return_type (*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<void, void, void, void, void, void, void, void> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type>(function));
        super::disconnect(slot, connection_type);
    }

    void send(arg1_type arg1) const
    {
        std::list<SlotAdapterBase *>::iterator it;

        for (it = super::m_slots.begin(); it != super::m_slots.end(); ++it)
        {
            int num_args = (*it)->getNumberOfArguments();

            switch (num_args)
            {
                case 1:
                {
                    typedef SlotAdapter<arg1_type> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction(arg1);
                    break;
                }

                case 0:
                {
                    typedef SlotAdapter<> * slot_ptr;
                    static_cast<slot_ptr>(*it)->callFunction();
                    break;
                }
            }
        }
    }

private:

    typedef SignalBase super;
};


// no arguments specified


template<>
class Signal<void, void, void, void> : public SignalBase
{
public:

    Signal() {}
    virtual ~Signal() {}

    template <typename slot_class_type, typename slot_return_type>
    void connect(slot_class_type * object, slot_return_type (slot_class_type::*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void connect(const slot_class_type * object, slot_return_type (slot_class_type::*function)() const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::connect(slot, connection_type);
    }

    template <typename slot_return_type>
    void connect(slot_return_type (*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type>(function));
        super::connect(slot, connection_type);
    }

    template <typename slot_class_type, typename slot_return_type>
    void disconnect(slot_class_type * object, slot_return_type (slot_class_type::*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new MemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    template <typename slot_class_type, typename slot_return_type>
    void disconnect(const slot_class_type * object, slot_return_type (slot_class_type::*function)() const, ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new ConstMemberFunctionSlot<slot_class_type, slot_return_type>(object, function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    template <typename slot_return_type>
    void disconnect(slot_return_type (*function)(), ConnectionType connection_type = SINGLE_CONNECTION) const
    {
        typedef ConcreteSlotAdapter<> slot_adapter_type;
        SlotAdapterBase * slot = new slot_adapter_type(new GlobalFunctionSlot<slot_return_type>(function));
        super::disconnect(slot, connection_type);
        delete slot;
    }

    void send() const
    {
        std::list<SlotAdapterBase *>::iterator it;

        for (it = super::m_slots.begin(); it != super::m_slots.end(); ++it)
        {
            typedef SlotAdapter<> * slot_ptr;
            static_cast<slot_ptr>(*it)->callFunction();
        }
    }

private:

    typedef SignalBase super;
};


} // namespace Signals


// For convenience reasons, populate the TRTK namespace with some entities
// from the Signals namespace.

using Signals::Signal;
using Signals::Receiver;
using Signals::ConnectionType;
using Signals::MULTIPLE_CONNECTIONS;
using Signals::SINGLE_CONNECTION;


} // namespace TRTK


#endif // SIGNALS_HPP_6243120120
