// Copyright (c) 1997-2001
// Utrecht University (The Netherlands),
// ETH Zurich (Switzerland),
// INRIA Sophia-Antipolis (France),
// Max-Planck-Institute Saarbruecken (Germany),
// and Tel-Aviv University (Israel).  All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Optimisation_basic/include/CGAL/Optimisation/Access_dimension_d.h $
// $Id: include/CGAL/Optimisation/Access_dimension_d.h 50cfbde3b84 $
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Sven Schoenherr <sven@inf.ethz.ch>

#ifndef CGAL_OPTIMISATION_ACCESS_DIMENSION_D_H
#define CGAL_OPTIMISATION_ACCESS_DIMENSION_D_H

namespace CGAL {

// Class declaration
// =================
template < class R_ >
class Access_dimension_d;

// Class interface
// ===============
template < class R_ >
class Access_dimension_d {
  public:
    // self
    typedef  R_                         R;
    typedef  Access_dimension_d<R>      Self;

    // types
    typedef  typename R::Point_d        Point;

    // unary function class types
    typedef  int                        result_type;
    typedef  Point                      argument_type;

    // creation
    Access_dimension_d( ) { }

    // operations
    int  operator() ( const Point& p) const { return p.dimension(); }
};

} //namespace CGAL

#endif // CGAL_OPTIMISATION_ACCESS_DIMENSION_D_H

// ===== EOF ==================================================================
