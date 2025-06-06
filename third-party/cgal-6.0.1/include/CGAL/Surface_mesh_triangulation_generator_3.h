// Copyright (c) 2006-2007  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Surface_mesher/include/CGAL/Surface_mesh_triangulation_generator_3.h $
// $Id: include/CGAL/Surface_mesh_triangulation_generator_3.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Laurent Rineau

#ifndef CGAL_SURFACE_MESH_TRIANGULATION_GENERATOR_3_H
#define CGAL_SURFACE_MESH_TRIANGULATION_GENERATOR_3_H

#include <CGAL/license/Surface_mesher.h>

#define CGAL_DEPRECATED_HEADER "<CGAL/Surface_mesh_triangulation_generator_3.h>"
#define CGAL_DEPRECATED_MESSAGE_DETAILS \
  "The 3D Mesh Generation package (see https://doc.cgal.org/latest/Mesh_3/) should be used instead."
#include <CGAL/Installation/internal/deprecation_warning.h>

#include <CGAL/Triangulation_data_structure_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Surface_mesh_vertex_base_3.h>
#include <CGAL/Surface_mesh_cell_base_3.h>

namespace CGAL {

template <class Kernel>
class Surface_mesh_triangulation_generator_3
{
  typedef CGAL::Surface_mesh_vertex_base_3<Kernel> Vb;
  typedef CGAL::Surface_mesh_cell_base_3<Kernel> Cb;
  typedef CGAL::Triangulation_data_structure_3<Vb, Cb> Tds;
public:
  typedef CGAL::Delaunay_triangulation_3<Kernel, Tds> Type;
  typedef Type type; // Boost meta-programming compatibility
};

} // end namespace CGAL

#endif // CGAL_SURFACE_MESH_TRIANGULATION_GENERATOR_3_H
