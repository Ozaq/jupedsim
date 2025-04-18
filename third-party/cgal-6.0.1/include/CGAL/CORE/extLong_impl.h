/****************************************************************************
 * Core Library Version 1.7, August 2004
 * Copyright (c) 1995-2004 Exact Computation Project
 * All rights reserved.
 *
 * This file is part of CGAL (www.cgal.org).
 *
 * File: extLong.cpp
 * Synopsis:
 *      The class extLong is basically a wrapper around the machine
 *      type long.  It is an important class to provide several
 *      additional facilities to detect overflows and undefined values.
 *      Future development includes extensions to level arithmetic
 *      (i.e., if a number overflows level i, we will go to level i+1).
 *      Level i representation of a number n is just i iterations
 *      of log_2 applied to n.
 *
 * Written by
 *       Chee Yap <yap@cs.nyu.edu>
 *       Chen Li <chenli@cs.nyu.edu>
 *       Zilin Du <zilin@cs.nyu.edu>
 *       Sylvain Pion <pion@cs.nyu.edu>
 *
 * WWW URL: https://cs.nyu.edu/exact/
 * Email: exact@cs.nyu.edu
 *
 * $URL: https://github.com/CGAL/cgal/blob/v6.0.1/CGAL_Core/include/CGAL/CORE/extLong_impl.h $
 * $Id: include/CGAL/CORE/extLong_impl.h 50cfbde3b84 $
 * SPDX-License-Identifier: LGPL-3.0-or-later
 ***************************************************************************/

#ifdef CGAL_HEADER_ONLY
#define CGAL_INLINE_FUNCTION inline
#else
#define CGAL_INLINE_FUNCTION
#endif

#include <CGAL/CORE/extLong.h>

namespace CORE {

CGAL_INLINE_FUNCTION
const extLong& extLong::getNaNLong() {
  static const extLong NaNLong(true);
  return NaNLong;
}

CGAL_INLINE_FUNCTION
const extLong& extLong::getPosInfty() {
  static const extLong posInfty(EXTLONG_MAX);
  return posInfty;
}

CGAL_INLINE_FUNCTION
const extLong& extLong::getNegInfty() {
  static const extLong negInfty(EXTLONG_MIN);
  return negInfty;
}

CGAL_INLINE_FUNCTION
void extLong::add(extLong& z, long x, long y) {
  if (x > 0 && y > 0 && x >= EXTLONG_MAX - y) {
    z.val = EXTLONG_MAX;
    z.flag = 1;
  } else if (x < 0 && y < 0 && x <= EXTLONG_MIN - y) {
    z.val = EXTLONG_MIN;
    z.flag = -1;
  } else {
    z.val = x + y;
    z.flag = 0;
  }
}

//  arithmetic and assignment operators
CGAL_INLINE_FUNCTION
extLong& extLong::operator+= (const extLong& y) {
  if (flag == 2 || y.flag == 2 || (flag * y.flag < 0)) {
#ifdef CORE_DEBUG
    if (flag * y.flag < 0) //want a message at the first creation of NaN
      CGAL_CORE_warning_msg(false, "extLong NaN Error in addition.");
#endif

    *this = CORE_NaNLong;
  } else if (flag == 1 || y.flag == 1) { // one of them is +Inf
    *this = CORE_posInfty;
  } else if (flag == -1 || y.flag == -1) { // one of them is -Inf
    *this = CORE_negInfty;
  } else { // x and y are normal now
    add(*this, val, y.val);
  }
  return *this;
}

CGAL_INLINE_FUNCTION
extLong& extLong::operator-= (const extLong& y) {
  if (flag == 2 || y.flag == 2 || (flag * y.flag > 0)) {
#ifdef CORE_DEBUG
    if (flag * y.flag > 0) //want a message at the first creation of NaN
      CGAL_CORE_warning_msg(false, "extLong NaN Error in subtraction.");
#endif

    *this = CORE_NaNLong;
  } else if (flag == 1 || y.flag == -1) {
    *this = CORE_posInfty;
  } else if (flag == -1 || y.flag == 1) {
    *this = CORE_negInfty;
  } else {
    add(*this, val, -y.val);
  }
  return *this;
}

CGAL_INLINE_FUNCTION
extLong& extLong::operator*= (const extLong& y) {
  if (flag == 2 || y.flag == 2) {
    *this = CORE_NaNLong;
  } else if ((flag != 0) || (y.flag != 0)) {
    if (sign() * y.sign() > 0)
      *this = CORE_posInfty;
    else
      *this = CORE_negInfty;
  } else { // flag == 0 and y.flag == 0
    double d = double(val) * double(y.val);
    long   p = val * y.val;
    if (std::fabs(d - p) <= std::fabs(d) * relEps) {
      val = p;
      flag = 0;
    } else if (d > static_cast<double>(EXTLONG_MAX)) {
      *this = CORE_posInfty;
    } else if (d < static_cast<double>(EXTLONG_MIN)) {
      *this = CORE_negInfty;
    } else {
#ifdef CORE_DEBUG
      CGAL_CORE_warning_msg(false, "extLong NaN Error in multiplication.");
#endif
      *this = CORE_NaNLong;
    }
  }
  return *this;
}

CGAL_INLINE_FUNCTION
extLong& extLong::operator/= (const extLong& y) {
  if (flag==2 || y.flag==2 || ((flag != 0) && (y.flag != 0)) || (y.val == 0)) {
#ifdef CORE_DEBUG
    if (y.val == 0)
      CGAL_CORE_warning_msg(false, "extLong NaN Error, Divide by Zero.");
    else if ((flag !=0) && (y.flag !=0))
      CGAL_CORE_warning_msg(false, "extLong NaN Error, +/-Inf/Inf.");
#endif

    *this = CORE_NaNLong;
  } else if ((flag != 0) || (y.flag != 0)) { // y.flag == 0 now and y != 0
    if (sign() * y.sign() > 0)
      *this = CORE_posInfty;
    else
      *this = CORE_negInfty;
  } else { // flag == 0 and y.flag == 0
    val /= y.val; // no overflow in divisions
    flag = 0;
  }
  return *this;
}

//  unary minus
CGAL_INLINE_FUNCTION
extLong extLong::operator- () const {
  if (flag == 0)
    return extLong(-val);
  else if (flag == 1)
    return CORE_negInfty;
  else if (flag == -1)
    return CORE_posInfty;
  else // NaN
    return CORE_NaNLong;
}

// sign
//    You should check "flag" before calling this, otherwise
//    you cannot interpret the returned value!
CGAL_INLINE_FUNCTION
int extLong::sign() const {
  CGAL_CORE_warning_msg(flag != 2, "NaN Sign can not be determined!");
  return ((val == 0) ? 0 : ((val > 0) ? 1 : -1));
}

//  stream operators
CGAL_INLINE_FUNCTION
std::ostream& operator<< (std::ostream& o, const extLong& x) {
  if (x.flag == 1)
    o << " infty ";
  else if (x.flag == - 1)
    o << " tiny ";
  else if (x.flag == 2)
    o << " NaN ";
  else
    o << x.val;
  return o;
}

} //namespace CORE
