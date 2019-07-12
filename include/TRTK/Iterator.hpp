/*
    Copyright (C) 2010 - 2019 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.4.0 (2019-06-24)
*/

/** \file Iterator.hpp
  * \brief This file contains the \ref TRTK::Iterator "Iterator" and
  *        \ref TRTK::IteratorAdapter "IteratorAdapter"classes.
  */

#ifndef ITERATOR_HPP_3437312701
#define ITERATOR_HPP_3437312701


#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <typeinfo>


namespace TRTK
{


////////////////////////////////////////////////////////////////////////
//                              Iterator                              //
////////////////////////////////////////////////////////////////////////


/* Forward declarations */

template <class T>
class Iterator;

template <class T>
unsigned long distance(Iterator<T> first, Iterator<T> last);


/** \tparam T       Value type of the dereferenced iterator (note: this is not necessarily the type of the container element)
  *
  * \brief Common base class for input iterator adapter classes.
  *
  * In C++ templated virtual member functions are not allowed. Thus,
  * classes offering a generic iterator interface are not directly
  * feasible (the STL does not offer a common base class). However,
  * an adapter can be used to delegate the required operations to the
  * particular iterator class.
  *
  * This class is the common base class for the derived templated
  * input iterator adapter classes (see \ref IteratorAdapter).
  *
  * \b Example:
  *
  * \code
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Iterator.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  *
  * class Interface
  * {
  * public:
  *     virtual ~Interface() {}
  *     virtual void doSomething(Iterator<double> begin, Iterator<double> end) = 0;
  * };
  *
  *
  * class Implementation
  * {
  * public:
  *     void doSomething(Iterator<double> begin, Iterator<double> end)
  *     {
  *         // Print the container's content.
  *
  *         Iterator<double> it = begin;
  *
  *         while (it != end)
  *         {
  *             cout << *it;
  *             ++it;
  *         }
  *     }
  * };
  *
  * int main()
  * {
  *     vector<double> vec;
  *     vec.push_back(1);
  *     vec.push_back(2);
  *
  *     Implementation impl;
  *
  *     Iterator<double> itBegin = make_iterator(vec.begin());
  *     Iterator<double> itEnd = make_iterator(vec.end());
  *
  *     impl.doSomething(itBegin, itEnd);
  * }
  * \endcode
  *
  * \b Output:
  *
  * \code
  * 12
  * \endcode
  *
  * \note
  *
  * You might have noticed the use of \ref make_iterator. This avoids
  * the need of explicitly stating the template arguments:
  *
  * \code
  * Iterator<double> it = make_iterator(vec.begin());
  * \endcode
  *
  * Instead of:
  *
  * \code
  * Iterator<double> it = IteratorAdapter<double, vector<double>::iterator>(vec.begin());
  * \endcode
  *
  * However, the advantage of the latter form is that it enables
  * you to mix different container types with different value
  * types (provided, these are convertible to each other):
  *
  * \code
  * Iterator<int> it = IteratorAdapter<int, vector<double>::iterator>(vec.begin());
  * \endcode
  *
  * \see IteratorAdapter, make_iterator, Range
  *
  * \author Christoph Haenisch
  * \version 0.4.0
  * \date last changed on 2019-06-24
  */

template <class T>
class Iterator : public std::iterator<std::input_iterator_tag, T>
{
protected:
    typedef void (Iterator<T>::*bool_type)() const; ///< Used in the safe bool idiom implementation
    void safe_bool_idiom_helper_function() const; ///< Used in the safe bool idiom implementation

public:
    Iterator();
    Iterator(const Iterator<T> & other);
    virtual ~Iterator();

    virtual Iterator & operator=(const Iterator<T> & other);
    virtual T operator*() const;
    virtual Iterator & operator++();
    virtual Iterator operator++(int);
    virtual bool operator==(const Iterator<T> & other) const;
    virtual bool operator!=(const Iterator<T> & other) const;
    virtual operator bool_type() const; // safe bool idiom

    virtual bool isValid() const;
    const Iterator & getIterator() const;

    friend unsigned long distance<T>(Iterator<T> first, Iterator<T> last);

protected:
    virtual Iterator * clone() const;
    virtual unsigned long distance(const Iterator<T> & other) const;

private:
    Iterator<T> * iterator; // is always a clone of a derived iterator (or NULL)
    const char * error_message;
};


/** \brief Default constructor. */

template <class T>
Iterator<T>::Iterator() : iterator(NULL), error_message("Iterator<T>: Operation on uninitialized instance.")
{
}


/** \brief Copy constructor. */

template <class T>
Iterator<T>::Iterator(const Iterator<T> & other) : error_message("Iterator<T>: Operation on uninitialized instance.")
{
    iterator = other.clone();
}


/** \brief Destructor. */

template <class T>
Iterator<T>::~Iterator()
{
    if (iterator) delete iterator;
}


/** \brief Assignment operator. */

template <class T>
Iterator<T> & Iterator<T>::operator=(const Iterator<T> & other)
{
    if (&other == this) return *this; // case it = it

    if (iterator) delete iterator;
    iterator = other.clone();

    return *this;
}


/** \brief Dereference operator. */

template <class T>
T Iterator<T>::operator*() const
{
    if (iterator)
    {
        return iterator->operator*();
    }
    else
    {
        throw std::runtime_error(error_message);
    }
}


/** \brief Pre-increment operator. */

template <class T>
Iterator<T> & Iterator<T>::operator++()
{
    if (iterator)
    {
        return iterator->operator++();
    }
    else
    {
        throw std::runtime_error(error_message);
    }
}


/** \brief Post-increment operator. */

template <class T>
Iterator<T> Iterator<T>::operator++(int)
{
    if (iterator)
    {
        return iterator->operator++(0);
    }
    else
    {
        throw std::runtime_error(error_message);
    }
}


/** \brief Comparison operator.
  *
  * \note   The internally stored iterators must belong to the same sequence
  *         and must be of the same type, otherwise the behavior is undefined.
  *         \par
  *         Example:
  *         \code
  *         vector<double> vec;
  *         vec.push_back(1);
  *         vector<double> vec2 = vec;
  *         vec.begin() == vec2.begin(); // undefined behavior
  *         \endcode
  */

template <class T>
bool Iterator<T>::operator==(const Iterator<T> & other) const
{
    if (iterator)
    {
        return iterator->operator==(other);
    }
    else
    {
        throw std::runtime_error(error_message);
    }
}


/** \brief Comparison operator. */

template <class T>
bool Iterator<T>::operator!=(const Iterator<T> & other) const
{
    if (iterator)
    {
        return iterator->operator!=(other);
    }
    else
    {
        throw std::runtime_error(error_message);
    }
}


/** \brief Returns true if the instance references an iterator.
  *
  * The default instantiation does not reference an iterator and
  * thus returns false.
  *
  * \code
  * Iterator<double> it = Iterator<double>();
  * if (it) doSomething(it); // instead of using it.isValid()
  * \endcode
  */

template <class T>
Iterator<T>::operator typename Iterator<T>::bool_type() const // safe bool idiom
{
    return isValid() ? &Iterator<T>::safe_bool_idiom_helper_function : NULL;
}


template <class T>
void Iterator<T>::safe_bool_idiom_helper_function() const
{
}


/** \brief Clones the current object (i.e. \c *this).
  *
  * \return Returns a clone of the wrapped iterator created on the heap.
  */

template <class T>
Iterator<T> * Iterator<T>::clone() const
{
    if (iterator == NULL)
    {
        return NULL;
    }
    else
    {
        return iterator->clone();
    }
}


/** \brief Returns a reference to the internally stored IteratorAdapter. */

template <class T>
const Iterator<T> & Iterator<T>::getIterator() const
{
    return *iterator;
}


/** \brief Returns true if the instance references an iterator.
  *
  * The default instantiation does not reference an iterator and
  * thus returns false.
  *
  * \code
  * Iterator<double> it = Iterator<double>();
  * it.isValid(); // returns false
  * \endcode
  *
  * \note For convenience Iterator<T> also provides a boolean conversion
  *       operator that allows you to write code like this:
  *       \code
  *       Iterator<double> it = make_iterator(vec.begin());
  *       if (it) doSomething(it); // instead of using it.isValid()
  *       \endcode
  */

template <class T>
bool Iterator<T>::isValid() const
{
    return iterator != NULL;
}


/** \brief Returns the distance to the given iterator.
  *
  * The given iterator must be from the same container otherwise the
  * behaviour is undefined.
  */

template <class T>
unsigned long Iterator<T>::distance(const Iterator<T> & other) const
{
    if (iterator)
    {
        return iterator->distance(other);
    }
    else
    {
        throw std::runtime_error(error_message);
    }
}


/** \relates Iterator
  *
  * \brief Returns the distance between to iterators.
  *
  * This function call is delegated to the distance function specifically
  * defined for the internally stored iterator type.
  */

template <class T>
unsigned long distance(const Iterator<T> first, Iterator<T> last) // friend function
{
    return first.distance(last);
}


////////////////////////////////////////////////////////////////////////
//                          IteratorAdaptor                           //
////////////////////////////////////////////////////////////////////////


/* Forward declarations */

template <class T, class InputIterator>
class IteratorAdapter;

template <class T, class InputIterator>
unsigned long distance(IteratorAdapter<T, InputIterator> first, IteratorAdapter<T, InputIterator> last);


/** \tparam T               Value type of the dereferenced iterator (can differ from the container element type)
  * \tparam InputIterator   Type of the wrapped input iterator
  *
  * \brief Adapter class for arbitrary input iterator classes.
  *
  * In C++ templated virtual member functions are not allowed. Thus,
  * classes offering a generic iterator interface are not directly
  * feasible (the STL does not offer a common base class, for instance).
  * However, an adapter can be used to delegate the required operations
  * to the particular iterator class.
  *
  * This class adapts (wraps) an input iterator such that it has a
  * uniformcan interface (\ref Iterator).
  *
  * The first template parameter \p T denotes the type of the return value
  * of the dereference operator of this class. It can differ from that of
  * the wrapped input iterator. The only requirement is that the latter
  * type can be converted to the former one. This adapter can be assigned
  * to instances of \ref Iterator<T>.
  *
  * The second template parameter denotes the type of the input iterator
  * to be wrapped.
  *
  * \note
  *
  * \ref make_iterator can simplify the use of this class. It automatically
  * determines the types \p T and \p InputIterator from its argument,
  * creates an instance of IteratorAdapter, and returns an instance of
  * Iterator.
  *
  * See \ref Iterator for an example.
  *
  * \see Iterator, make_iterator
  *
  * \author Christoph Haenisch
  * \version 0.4.0
  * \date last changed on 2019-06-24
  */

template <class T, class InputIterator>
class IteratorAdapter : public Iterator<T>
{
protected:
    typedef Iterator<T> super;

public:
    IteratorAdapter();
    IteratorAdapter(InputIterator iterator);
    IteratorAdapter(const IteratorAdapter<T, InputIterator> & other);
    ~IteratorAdapter();

    IteratorAdapter & operator=(const Iterator<T> & other);
    IteratorAdapter & operator=(const IteratorAdapter<T, InputIterator> & other);
    T operator*() const;
    Iterator<T> & operator++();
    Iterator<T> operator++(int);
    bool operator==(const Iterator<T> & other) const;
    bool operator!=(const Iterator<T> & other) const;
    operator typename Iterator<T>::bool_type() const;

    bool isValid() const;

    friend unsigned long TRTK::distance<T, InputIterator>(IteratorAdapter<T, InputIterator> first, IteratorAdapter<T, InputIterator> last);

protected:
    Iterator<T> * clone() const;
    unsigned long distance(const Iterator<T> & other) const;

private:
    InputIterator iterator;
    bool valid;
};


/** \brief Default constructor. */

template <class T, class InputIterator>
IteratorAdapter<T, InputIterator>::IteratorAdapter() : iterator(InputIterator()), valid(false)
{
}


/** \brief Constructor. */

template <class T, class InputIterator>
IteratorAdapter<T, InputIterator>::IteratorAdapter(InputIterator iterator) : iterator(iterator), valid(true)
{
}


/** \brief Copy constructor. */

template <class T, class InputIterator>
IteratorAdapter<T, InputIterator>::IteratorAdapter(const IteratorAdapter<T, InputIterator> & other) : iterator(other.iterator), valid(other.valid)
{
}


/** \brief Destructor. */

template <class T, class InputIterator>
IteratorAdapter<T, InputIterator>::~IteratorAdapter()
{
}


/** \brief Assignment operator. */

template <class T, class InputIterator>
IteratorAdapter<T, InputIterator> & IteratorAdapter<T, InputIterator>::operator=(const Iterator<T> & other)
{
    // The case it = it is handled in the overloaded function below.

    try
    {
        // Is 'other.iterator' of type 'IteratorAdapter'?
        const IteratorAdapter<T, InputIterator> & iteratorAdapter = dynamic_cast<const IteratorAdapter<T, InputIterator> &>(other.getIterator());
        iterator = iteratorAdapter.iterator;
        valid = iteratorAdapter.valid;
    }
    catch (std::bad_cast &)
    {
        throw std::runtime_error("IteratorAdapter: Iterator types do not match");
    }

    return *this;
}


/** \brief Assignment operator. */

template <class T, class InputIterator>
IteratorAdapter<T, InputIterator> & IteratorAdapter<T, InputIterator>::operator=(const IteratorAdapter<T, InputIterator> & other)
{
    if (&other == this) return *this; // case it = it

    iterator = other.iterator;
    valid = other.valid;
    return *this;
}


/** \brief Dereference operator. */

template <class T, class InputIterator>
T IteratorAdapter<T, InputIterator>::operator*() const
{
    return *iterator;
}


/** \brief Pre-Increment operator. */

template <class T, class InputIterator>
Iterator<T> & IteratorAdapter<T, InputIterator>::operator++()
{
    ++iterator;
    return *this;
}


/** \brief Post-Increment operator. */

template <class T, class InputIterator>
Iterator<T> IteratorAdapter<T, InputIterator>::operator++(int)
{
    Iterator<T> temporary_iterator = *this;
    ++iterator;
    return temporary_iterator;
}


/** \brief Comparison operator. */

template <class T, class InputIterator>
bool IteratorAdapter<T, InputIterator>::operator==(const Iterator<T> & other) const
{
    try
    {
        // Is 'other' of type 'IteratorAdapter'?
        const IteratorAdapter<T, InputIterator> & iteratorAdapter = dynamic_cast<const IteratorAdapter<T, InputIterator> &>(other);
        return this->iterator == iteratorAdapter.iterator;
    }
    catch (std::bad_cast &)
    {
        // 'other' might be of type 'Iterator<T>'. So is 'other.iterator' of type 'IteratorAdapter'?
        try
        {
            const IteratorAdapter<T, InputIterator> & iteratorAdapter = dynamic_cast<const IteratorAdapter<T, InputIterator> &>(other.getIterator());
            return this->iterator == iteratorAdapter.iterator;
        }
        catch (std::bad_cast &)
        {
            throw std::runtime_error("IteratorAdapter: Iterator types do not match");
        }
    }
}


/** \brief Comparison operator. */

template <class T, class InputIterator>
bool IteratorAdapter<T, InputIterator>::operator!=(const Iterator<T> & other) const
{
    return !this->operator==(other);
}


/** \brief Returns true if the instance was initialized with an iterator.
  *
  * The default instantiation does not reference a valid iterator and
  * thus returns false.
  *
  * \b Example:
  *
  * \code
  * IteratorAdapter<double, vector<double>::iterator> it;
  * if (it) doSomething(it); // instead of using it.isValid()
  * \endcode
  */

template <class T, class InputIterator>
IteratorAdapter<T, InputIterator>::operator typename Iterator<T>::bool_type() const // safe bool idiom
{
    return isValid() ? &IteratorAdapter<T, InputIterator>::safe_bool_idiom_helper_function : NULL;
}


template <class T, class InputIterator>
Iterator<T> * IteratorAdapter<T, InputIterator>::clone() const
{
    return new IteratorAdapter<T, InputIterator>(iterator);
}


/** \brief Returns the distance to the given iterator.
  *
  * The given iterator must be from the same container otherwise the
  * behaviour is undefined.
  */

template <class T, class InputIterator>
unsigned long IteratorAdapter<T, InputIterator>::distance(const Iterator<T> & other) const
{
    using std::distance;

    try
    {
        // Is 'other' of type 'IteratorAdapter'?
        const IteratorAdapter<T, InputIterator> & iteratorAdapter = dynamic_cast<const IteratorAdapter<T, InputIterator> &>(other);
        return (unsigned long ) distance(iterator, iteratorAdapter.iterator);
    }
    catch (std::bad_cast &)
    {
        // 'other' might be of type 'Iterator<T>'. So is 'other.iterator' of type 'IteratorAdapter'?
        try
        {
            const IteratorAdapter<T, InputIterator> & iteratorAdapter = dynamic_cast<const IteratorAdapter<T, InputIterator> &>(other.getIterator());
            return (unsigned long) distance(iterator, iteratorAdapter.iterator);
        }
        catch (std::bad_cast &)
        {
            throw std::runtime_error("IteratorAdapter: Iterator types do not match");
        }
    }
}


/** \brief  Returns true if the instance was initialized with an iterator.
  *
  * The default instantiation does not reference an iterator and
  * thus returns false.
  *
  * \code
  * IteratorAdapter<double, vector<double>::iterator> it;
  * it.isValid(); // returns false
  * \endcode
  *
  * \note For convenience IteratorAdapter also provides a boolean
  *       conversion operator that allows you to write code like this:
  *       \code
  *       IteratorAdapter<double, vector<double>::iterator> it = vec.begin();
  *       if (it) doSomething(it); // instead of using it.isValid()
  *       \endcode
  */

template <class T, class InputIterator>
bool IteratorAdapter<T, InputIterator>::isValid() const
{
    return valid;
}


/** \relates IteratorAdapter
  *
  * \brief Returns the distance between to iterators.
  *
  * This function call is delegated to the distance function specifically
  * defined for the internally stored iterator type.
  */

template <class T, class InputIterator>
unsigned long distance(IteratorAdapter<T, InputIterator> first, IteratorAdapter<T, InputIterator> last) // friend function
{
    using std::distance;
    return distance(first.iterator, last.iterator);
}


/** \relates IteratorAdapter
  *
  * \brief Creates an iterator adapter for arbitrary input iterator types.
  *
  * This is a convenience function which avoids the need of explicitly
  * stating the template arguments. The input iterator must provide its
  * value type via \c InputIterator::value_type. See \ref Iterator for
  * an example.
  *
  * \returns Returns an instance of the Iterator<T> base class.
  *
  * \see Iterator, IteratorAdapter
  */

template <class InputIterator>
Iterator<typename InputIterator::value_type> make_iterator(InputIterator iterator)
{
    return IteratorAdapter<typename InputIterator::value_type, InputIterator>(iterator);
}


} // namespace TRTK


#endif // ITERATOR_HPP_3437312701
