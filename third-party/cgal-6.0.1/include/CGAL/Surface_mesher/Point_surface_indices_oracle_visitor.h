// Copyright (c) 2005  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Surface_mesher/include/CGAL/Surface_mesher/Point_surface_indices_oracle_visitor.h $
// $Id: include/CGAL/Surface_mesher/Point_surface_indices_oracle_visitor.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Laurent Rineau


#ifndef CGAL_SURFACE_MESHER_POINT_SURFACE_INDICES_VISITOR_H
#define CGAL_SURFACE_MESHER_POINT_SURFACE_INDICES_VISITOR_H

#include <CGAL/license/Surface_mesher.h>

#define CGAL_DEPRECATED_HEADER "<CGAL/Surface_mesher/Point_surface_indices_oracle_visitor.h>"
#define CGAL_DEPRECATED_MESSAGE_DETAILS \
  "The 3D Mesh Generation package (see https://doc.cgal.org/latest/Mesh_3/) should be used instead."
#include <CGAL/Installation/internal/deprecation_warning.h>

namespace CGAL {

  namespace Surface_mesher {

  /** Model of the OracleVisitor concept.
      This model of OracleVisitor sets the point "surface_index" to a
      constant \c int.
   */
  struct Point_surface_indices_visitor
  {
    int i;

    Point_surface_indices_visitor(const int index) : i(index)
    {
    }

    template <class P>
    void new_point(P& p) const
    {
      p.set_surface_index(i);
    }
  }; // end class Point_surface_indices_visitor

  }  // namespace Surface_mesher

} // namespace CGAL


#endif  // CGAL_SURFACE_MESHER_POINT_SURFACE_INDICES_VISITOR_H
