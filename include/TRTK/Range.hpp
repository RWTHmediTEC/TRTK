/*
    Copyright (C) 2010 - 2014, 2019 Christoph Hänisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.2 (2019-05-21)
*/

/** \file Range.hpp
  * \brief This file contains the \ref TRTK::Range "Range" class
  *        as well as some helper functions.
  */

#ifndef RANGE_HPP_7631082974
#define RANGE_HPP_7631082974


#include "Iterator.hpp"


namespace TRTK
{


////////////////////////////////////////////////////////////////////////
//                               Range                                //
////////////////////////////////////////////////////////////////////////


/** \tparam T       Value type of the dereferenced iterators (note: this is not necessarily the type of the container element)
  *
  * \brief Aggregate of two input iterator adapters.
  *
  * Many algorithms (like sorting) can be formulated such that they do
  * not depend on a certain data structure. Access to the data might be
  * abstracted via data traversal. This can be realized via iterators,
  * for instance, as done in the STL; also many algorithms in TRTK are
  * implemented in this manner. However, this generally involves passing
  * two arguments; one for the begin of a data sequence and one for its
  * end. Using ranges, this verbosity can be significantly reduced.
  *
  * The range of a sequence is the half-open interval <tt>[first, last)</tt>
  * which means that the first element is included but but not the last.
  *
  * A range can be traversed using \ref next(). The actual element is
  * obtained via \ref currentItem(). To check whether a traversal is done
  * use \ref isDone(). The position can be reseted via \ref first().
  *
  * \b Example:
  *
  * \code
  * vector<double> vec;
  * vec.push_back(1);
  * vec.push_back(2);
  * vec.push_back(3);
  *
  * // Iterator<double> first = make_iterator(vec.begin());
  * // Iterator<double> last = make_iterator(vec.end());
  * // Range<double> range(first, last);
  * //
  * // or simply
  *
  * Range<double> range = make_range(vec);
  *
  * while(!range.isDone())
  * {
  *     echo << range.currentItem() << " ";
  *     range.next();
  * }
  * \endcode
  *
  * \b Output:
  *
  * \code
  * 1 2 3
  * \endcode
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date last changed on 2014-08-31
  */

template <class T>
class Range
{
private:

    typedef void (Range<T>::*bool_type)() const; ///< Used in the safe bool idiom implementation
    void safe_bool_idiom_helper_function() const; ///< Used in the safe bool idiom implementation

public:

    Range();
    Range(const Iterator<T> & begin, const Iterator<T> & end);
    Range(const Range<T> & other);
    virtual ~Range();

    Range<T> & operator=(const Range<T> & other);
    bool operator==(const Range<T> & other) const;
    bool operator!=(const Range<T> & other) const;
    operator bool_type() const; // safe bool idiom

    Iterator<T> begin() const;
    Iterator<T> end() const;

    bool isValid() const;
    bool isEmpty() const;

    void first() const;
    bool isDone() const;
    void next() const;
    T currentItem() const;

    size_t size() const;

    typedef Iterator<T> const_iterator;

private:

    Iterator<T> m_begin;
    Iterator<T> m_end;
    mutable Iterator<T> m_current;
};


/** \brief Default constructor. */

template <class T>
Range<T>::Range()
{
}


/** \brief Copy constructor. */

template <class T>
Range<T>::Range(const Range<T> & other) : m_begin(other.m_begin), m_end(other.m_end), m_current(other.m_begin)
{
}


/** \brief Constructor.
  *
  * \param [in] begin   First element of a sequence.
  * \param [in] end     Last element of a sequence (as done in the STL).
  *
  * The range is a half-open intervall [begin, last).
  */

template <class T>
Range<T>::Range(const Iterator<T> & begin, const Iterator<T> & end) : m_begin(begin), m_end(end), m_current(begin)
{
}


/** \brief Destructor. */

template <class T>
Range<T>::~Range()
{
}


/** \brief Assignment operator. */

template <class T>
Range<T> & Range<T>::operator=(const Range<T> & other)
{
    if (this == &other) return *this;

    m_begin = other.m_begin;
    m_end = other.m_end;
    m_current = other.m_begin;

    return *this;
}


/** \brief Comparison operator.
  *
  * \note   The internally stored iterators must belong to the same sequence
  *         and must be of the same type, otherwise the behavior is undefined.
  *         However, comparing subranges is well defined.
  *         \par
  *         Example:
  *         \code
  *         vector<double> vec;
  *         vec.push_back(1);
  *         vector<double> vec2 = vec;
  *         vec.begin() == vec2.begin(); // undefined behavior
  *         make_range(vec) == make_range(vec2); // undefined behavior
  *         \endcode
  *
  * \returns If both ranges are valid, \c true is returned in the case
  *          of mutually identic iterators \c begin() and \c end(). If
  *          both ranges are uninitialized, \c true is returned as well.
  *          In all other cases \c false is returned.
  */

template <class T>
bool Range<T>::operator==(const Range<T> & other) const
{
    if (isValid() && other.isValid())
    {
        return (m_begin == other.m_begin) && (m_end == other.m_end);
    }
    else if (!m_begin.isValid() && !other.begin().isValid() && !m_end.isValid() && !other.end().isValid())
    {
        return true;
    }
    else
    {
        return false;
    }
}


/** \brief Comparison operator. */

template <class T>
bool Range<T>::operator!=(const Range<T> & other) const
{
    return !(*this == other);
}


/** \brief Returns true if \c begin() and \c end() reference iterators.
  *
  * The default instantiation does not reference an iterators and
  * thus returns false.
  *
  * \code
  * Range<double> range = Range<double>();
  * if (range) doSomething(); // instead of using range.isValid()
  * \endcode
  */

template <class T>
Range<T>::operator typename Range<T>::bool_type() const // safe bool idiom
{
    return isValid() ? &Range<T>::safe_bool_idiom_helper_function : NULL;
}


template <class T>
void Range<T>::safe_bool_idiom_helper_function() const
{
}


/** \brief Returns the first element of the range. */

template <class T>
Iterator<T> Range<T>::begin() const
{
    return m_begin;
}


/** \brief Returns the current element of the range traversal. */

template <class T>
T Range<T>::currentItem() const
{
    if (m_begin && m_end)
    {
        if (m_current != m_end)
        {
            return *m_current;
        }
        else
        {
            throw std::range_error("Range<T>::next(): The last element was already reached.");
        }
    }
    else
    {
        throw std::range_error("Range<T>::next(): This instance does not reference any iterators.");
    }
}


/** \brief Returns the last element of the range (as done in the STL). */

template <class T>
Iterator<T> Range<T>::end() const
{
    return m_end;
}


/** \brief Sets the internal iterator to the begin of the range. */

template <class T>
void Range<T>::first() const
{
    m_current = m_begin;
}


/** \brief Returns true if the traversal is done or if a sequence is empty or uninitialized. */

template <class T>
bool Range<T>::isDone() const
{
    if (m_begin && m_end)
    {
        return m_current == m_end;
    }
    else
    {
        return true;
    }
}


/** \brief Returns true if the sequence does not contain any elements.
  *
  * If the range is uninitialized \c true is returned.
  */

template <class T>
bool Range<T>::isEmpty() const
{
    if (m_begin && m_end)
    {
        return m_begin == m_end;
    }
    else
    {
        return true;
    }
}


/** \brief Returns true if \c begin() and \c end() reference iterators.
  *
  * The default instantiation does not reference an iterators and
  * thus returns false.
  *
  * \code
  * Range<double> range = Range<double>();
  * range.isValid(); // returns false
  * \endcode
  *
  * \note For convenience Range<T> also provides a boolean conversion
  *       operator that allows you to write code like this:
  *       \code
  *       vector<double> vec;
  *       Range<double> range = make_range(vec);
  *       if (range) doSomething(); // instead of using it.isValid()
  *       \endcode
  */

template <class T>
bool Range<T>::isValid() const
{
    return m_begin.isValid() && m_end.isValid();
}


/** \brief Selects the next element in the range. */

template <class T>
void Range<T>::next() const
{
    if (m_begin && m_end)
    {
        if (m_current != m_end)
        {
            ++m_current;
        }
        else
        {
            throw std::range_error("Range<T>::next(): The last element was already reached.");
        }
    }
    else
    {
        throw std::range_error("Range<T>::next(): This instance does not reference any iterators.");
    }
}


/** \brief Returns the size of the sequence.
  *
  * Coputational complexity: O(n) where n is the number of elements in the range.
  */

template <class T>
size_t Range<T>::size() const
{
    if (m_begin && m_end)
    {
        return distance(m_begin, m_end);
    }
    else
    {
        return 0;
    }
}


/** \relates Range
  *
  * \brief Convenience function for creating a range of an arbitrary sequence (i.e. an STL container).
  *
  * The sequence must provide its value type via \c Sequence::value_type.
  * It must also provide two input iterators via the begin() and end()
  * methods. See \ref Range for an example.
  *
  * \returns Returns an instance of the Range<T> class.
  *
  * \see Range, Iterator, IteratorAdapter
  */

template <class Sequence>
Range<typename Sequence::value_type> make_range(const Sequence & sequence)
{
    typedef typename Sequence::value_type T; // TODO: in C++11 second template parameter class T = Sequence::value_type
    const Iterator<T> & begin = IteratorAdapter<T, typename Sequence::const_iterator>(sequence.begin());
    const Iterator<T> & end = IteratorAdapter<T, typename Sequence::const_iterator>(sequence.end());
    return Range<T>(begin, end);
}


} // namespace TRTK


#endif // RANGE_HPP_7631082974
