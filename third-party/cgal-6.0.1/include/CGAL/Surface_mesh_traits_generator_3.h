// Copyright (c) 2006-2007  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Surface_mesher/include/CGAL/Surface_mesh_traits_generator_3.h $
// $Id: include/CGAL/Surface_mesh_traits_generator_3.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Laurent Rineau

#ifndef CGAL_SURFACE_MESH_TRAITS_GENERATOR_3_H
#define CGAL_SURFACE_MESH_TRAITS_GENERATOR_3_H

#include <CGAL/license/Surface_mesher.h>

#define CGAL_DEPRECATED_HEADER "<CGAL/Surface_mesh_traits_generator_3.h>"
#define CGAL_DEPRECATED_MESSAGE_DETAILS \
  "The 3D Mesh Generation package (see https://doc.cgal.org/latest/Mesh_3/) should be used instead."
#include <CGAL/Installation/internal/deprecation_warning.h>

#include <CGAL/Surface_mesher/Sphere_oracle_3.h>

namespace CGAL {

template <class K>
class Sphere_3;

/** Default traits class.
 *  Partial specialization will be in other headers
*/
template <typename Surface>
struct Surface_mesh_traits_generator_3
{
  typedef typename Surface::Surface_mesher_traits_3 Type;
  typedef Type type; // for Boost compatibility (meta-programming)
};

  // specialization for Kernel::Sphere_3
template <typename Kernel>
struct Surface_mesh_traits_generator_3<CGAL::Sphere_3<Kernel> >
{
  typedef Surface_mesher::Sphere_oracle_3<Kernel> Type;
  typedef Type type; // for Boost compatibility (meta-programming)
};

} // end namespace CGAL

#endif // CGAL_SURFACE_MESH_TRAITS_GENERATOR_3_H
