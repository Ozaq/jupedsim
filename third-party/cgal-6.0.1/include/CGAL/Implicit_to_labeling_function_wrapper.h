// Copyright (c) 2009 INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Mesh_3/include/CGAL/Implicit_to_labeling_function_wrapper.h $
// $Id: include/CGAL/Implicit_to_labeling_function_wrapper.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Stéphane Tayeb, Aymeric PELLE
//
//******************************************************************************
// File Description :
// Implicit_to_labeling_function_wrapper and
// Implicit_vector_to_labeling_function_wrapper class declaration
// and implementation.
//
// See classes description to have more information.
//******************************************************************************

#ifndef CGAL_IMPLICIT_TO_LABELING_FUNCTION_WRAPPER_H
#define CGAL_IMPLICIT_TO_LABELING_FUNCTION_WRAPPER_H

#include <CGAL/license/Mesh_3.h>

#include <CGAL/disable_warnings.h>

#if defined(BOOST_MSVC)
#  pragma warning(push)
#  pragma warning(disable:4180) // qualifier applied to function type has no meaning; ignored
#endif

#include <boost/dynamic_bitset.hpp>
#include <boost/type_traits/is_function.hpp>

#include <CGAL/config.h>
#include <CGAL/assertions.h>

namespace CGAL {

/**
 * @class Implicit_to_labeling_function_wrapper
 *
 * This class is designed to wrap an implicit function which describes a domain
 * by [p is inside if f(p)<0] to a function which takes its values into {0,1}.
 * f(p)=0 means that p is outside the domain.
 */
template<class Function_, class BGT>
class Implicit_to_labeling_function_wrapper
{
public:
  // Types
  typedef int                     return_type;
  typedef typename BGT::Point_3   Point_3;

  /// Constructor
  Implicit_to_labeling_function_wrapper(Function_ f) : f_(f) {}

  // Default copy constructor, assignment operator, and destructor are ok

  /// Operator ()
  return_type operator()(const Point_3& p) const
  {
    return ( (f_(p)<0) ? 1 : 0 );
  }

private:
  typedef std::conditional_t<std::is_function_v<Function_>,
                             Function_*,
                             Function_> Stored_function;
  /// Function to wrap
  Stored_function f_;

};  // end class Implicit_to_labeling_function_wrapper

template <typename BGT, typename Function>
Implicit_to_labeling_function_wrapper<Function, BGT>
make_implicit_to_labeling_function_wrapper(Function f)
{
  return Implicit_to_labeling_function_wrapper<Function, BGT>(f);
}

/**
 * \deprecated
 *
 * @class Implicit_vector_to_labeling_function_wrapper
 *
 * Wraps a set of implicit function [f1,f2,...] to one function F which
 * takes its values into N.
 *
 * Let p be a point.
 * F(p) = 0b000000(f2(p)<0)(f1(p)<0)
 *
 * It can handle at most 8 functions.
 */
template<class Function_, class BGT>
class Implicit_vector_to_labeling_function_wrapper
{
public:
  // Types
  typedef int                       return_type;
  typedef std::vector<Function_*>   Function_vector;
  typedef typename BGT::Point_3     Point_3;

  /// Constructor
  Implicit_vector_to_labeling_function_wrapper(const std::vector<Function_*>& v)
    : function_vector_(v)
  {
    if ( v.size() > 8 )
    {
      CGAL_error_msg("We support at most 8 functions !");
    }
  }

  // Default copy constructor and assignment operator are ok

  /// Destructor
  ~Implicit_vector_to_labeling_function_wrapper() {}

  /// Operator ()
  return_type operator()(const Point_3& p) const
  {
    const int nb_func = static_cast<int>(function_vector_.size());
    char bits = 0;
    for ( int i = 0 ; i < nb_func ; ++i )
    {
      // Insert value into bits : we compute fi(p) and insert result at
      // bit i of bits
      bits = char(bits | ( ((*function_vector_[i])(p) < 0) << i ));
    }

    return ( static_cast<return_type>(bits) );
  }

private:
  /// Functions to wrap
  const Function_vector function_vector_;

};  // end class Implicit_to_labeling_function_wrapper


  /*!
\ingroup PkgMesh3Domains

The class `Implicit_multi_domain_to_labeling_function_wrapper` is a helping class to get a function with integer values
labeling the components of a multidomain. The multidomain is described through a set of functions {fi(p), i=1, ...n}.
Each component corresponds to a sign vector [s1, s2, ..., sn] where si is the sign of the function fi(p) at a point p of the component.
This wrapper class can be passed to `Labeled_mesh_domain_3` as first template parameter.

\par Example
For example, the multidomain described by the three functions [f1,f2,f3] and the two sign vectors [-,-,+] and [+,-,+]
 includes two components.<br />
The first one matches the locus of points satisfying f1(p)<0 and f2(p)<0 and f3(p)>0.<br />
The second one matches the locus of points satisfying f1(p)>0 and f2(p)<0 and f3(p)>0.<br />

\tparam Function provides the definition of the function.
This parameter stands for a model of the concept `ImplicitFunction` described in the surface mesh generation package.
The number types `Function::FT` and `BGT::FT` are required to match.

\sa `CGAL::Labeled_mesh_domain_3`.
*/
#ifdef DOXYGEN_RUNNING
template <class Function>
#else
template <class ImplicitFunction>
#endif
class Implicit_multi_domain_to_labeling_function_wrapper
{
  template <class T_>
  class Implicit_function_traits
  {
  public:
    typedef typename T_::Point Point;
  };

  template <class RT_, class Point_>
  class Implicit_function_traits<RT_ (*)(Point_)>
  {
  public:
    typedef std::remove_reference_t<
            std::remove_cv_t< Point_ >> Point;
  };

public:
  /// \name Types
  /// @{

#ifdef DOXYGEN_RUNNING
  typedef typename Function::Point                             Point_3;
#else
  typedef ImplicitFunction                                     Function;
  typedef typename Implicit_function_traits<Function>::Point   Point_3;
  typedef int                                                  return_type;
#endif

  typedef std::vector<Function>                                Function_vector;

  /// @}

private:
  std::vector<Function> funcs;
  typedef boost::dynamic_bitset<std::size_t> Bmask;
  std::vector<Bmask> bmasks;

public:
  /// \name Creation
  /// @{

  /*!
   * \brief Construction from a vector of implicit functions and a vector of vector of signs.
   *
   * \param implicit_functions the vector of implicit functions.
   * \param position_vectors the vector of vector of signs. Each vector of positions describes a component.
   *
   * \sa `Sign`
   */
  Implicit_multi_domain_to_labeling_function_wrapper (const Function_vector& implicit_functions, const std::vector<std::vector<Sign> >& position_vectors)
  : funcs(implicit_functions), bmasks(position_vectors.size(), Bmask(funcs.size() * 2, false))
  {
    CGAL_assertion(funcs.size() != 0);

    std::size_t mask_index = 0;
    for (std::vector<std::vector<Sign> >::const_iterator mask_iter = position_vectors.begin(), mask_end_iter = position_vectors.end();
         mask_iter != mask_end_iter;
         ++mask_iter)
    {
      const std::vector<Sign>& mask = *mask_iter;
      CGAL_assertion(funcs.size() == mask.size());
      Bmask& bmask = bmasks[mask_index++];

      typename Bmask::size_type bit_index = 0;
      for (std::vector<Sign>::const_iterator iter = mask.begin(), endIter = mask.end(); iter != endIter; ++iter)
      {
        Sign character = *iter;
        CGAL_assertion(character == POSITIVE || character == NEGATIVE);

        bmask[bit_index] = (character == POSITIVE);
        ++bit_index;
        bmask[bit_index] = (character == NEGATIVE);
        ++bit_index;
      }
    }
    std::sort(bmasks.begin(), bmasks.end());
  }
  /*!
   * \brief Construction from a vector of implicit functions.

   * \param implicit_functions the vector of implicit functions.
   *
   * Position vectors are built automatically so that the union of components equals the union of the functions.
   */
  Implicit_multi_domain_to_labeling_function_wrapper (const Function_vector& implicit_functions)
  : funcs(implicit_functions)
  {
    CGAL_assertion(funcs.size() != 0);

    bmasks.reserve((1 << funcs.size()) - 1);
    bmasks.push_back(Bmask(std::string("10")));
    bmasks.push_back(Bmask(std::string("01")));

    for (std::size_t i = 0; i < funcs.size()-1; ++i)
    {
      std::size_t c_size = bmasks.size();
      for (std::size_t index = 0; index < c_size; ++index)
      {
        Bmask aux = bmasks[index];
        aux.push_back(true);
        aux.push_back(false);
        bmasks.push_back(aux);
        bmasks[index].push_back(false);
        bmasks[index].push_back(true);
      }
    }
    bmasks.pop_back();
    std::sort(bmasks.begin(), bmasks.end());
  }

  /*!
   * \brief Construction from a vector of implicit functions and a vector of strings.
   *
   * \param implicit_functions the vector of implicit functions.
   * \param position_strings the vector of strings. The strings contained in this vector must contain '+' or '-' only. Each string (vector of positions) describes a component.
   */
  Implicit_multi_domain_to_labeling_function_wrapper (const Function_vector& implicit_functions, const std::vector<std::string>& position_strings)
  : funcs(implicit_functions), bmasks(position_strings.size(), Bmask(funcs.size() * 2, false))
  {
    CGAL_assertion(funcs.size() != 0);

    std::size_t mask_index = 0;
    for (std::vector<std::string>::const_iterator mask_iter = position_strings.begin(), mask_end_iter = position_strings.end();
         mask_iter != mask_end_iter;
         ++mask_iter)
    {
      const std::string& mask_str = *mask_iter;
      CGAL_assertion(funcs.size() == mask_str.length());
      Bmask& bmask = bmasks[mask_index++];

      typename Bmask::size_type bit_index = 0;
      for (std::string::const_iterator iter = mask_str.begin(), endIter = mask_str.end(); iter != endIter; ++iter)
      {
        std::string::value_type character = *iter;
        CGAL_assertion(character == '+' || character == '-');

        bmask[bit_index] = (character == '+');
        ++bit_index;
        bmask[bit_index] = (character == '-');
        ++bit_index;
      }
    }
    std::sort(bmasks.begin(), bmasks.end());
  }

  /// @}

  return_type operator() (const Point_3& p) const
  {
    Bmask bmask(funcs.size() * 2, false);

    std::size_t i = 0;
    for (typename std::vector<Function>::const_iterator iter = funcs.begin(), endIter = funcs.end();
         iter != endIter;
         ++iter)
    {
      const Function& function = *iter;

      double fres = function(p);
      bmask[i] = fres > 0;
      ++i;
      bmask[i] = fres < 0;
      ++i;
    }

    typename std::vector<Bmask>::const_iterator iter = std::lower_bound(bmasks.begin(), bmasks.end(), bmask);
    if (iter != bmasks.end() && *iter == bmask)
      return static_cast<return_type>(1 + (iter - bmasks.begin()));
    return 0;
  }
};

} // end namespace CGAL

#if defined(BOOST_MSVC)
#  pragma warning(pop)
#endif

#include <CGAL/enable_warnings.h>

#endif // CGAL_IMPLICIT_TO_LABELING_FUNCTION_WRAPPER_H
