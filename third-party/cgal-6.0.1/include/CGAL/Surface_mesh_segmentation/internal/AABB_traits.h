// Copyright (c) 2014  GeometryFactory Sarl (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Surface_mesh_segmentation/include/CGAL/Surface_mesh_segmentation/internal/AABB_traits.h $
// $Id: include/CGAL/Surface_mesh_segmentation/internal/AABB_traits.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Ilker O. Yaz


#ifndef CGAL_INTERNAL_SURFACE_MESH_SEGMENTATION_AABB_TRAITS_H
#define CGAL_INTERNAL_SURFACE_MESH_SEGMENTATION_AABB_TRAITS_H

#include <CGAL/license/Surface_mesh_segmentation.h>


#include <CGAL/AABB_traits_3.h>
#include <boost/type_traits.hpp>

#include <type_traits>

namespace CGAL
{

/// @cond CGAL_DOCUMENT_INTERNAL
template<typename GeomTraits, typename AABB_primitive, bool fast_bbox_intersection>
class AABB_traits_SDF :
  public AABB_traits_3<GeomTraits, AABB_primitive>
{
public:
  typedef AABB_traits_3<GeomTraits, AABB_primitive> Base_traits;
  typedef typename Base_traits::Bounding_box Bounding_box;
  typedef typename Base_traits::Point Point_3;

  class Do_intersect
    : public Base_traits::Do_intersect
  {
  public:
    Do_intersect(const AABB_traits_3<GeomTraits,AABB_primitive>& traits)
      :Base_traits::Do_intersect(traits) {}

    // not sure is it safe on templated funcs ? may be do not inherit and repeat functions...
    using Base_traits::Do_intersect::operator ();

    // activate functions below if K::FT is floating point and fast_bbox_intersection = true
    template <class K>
    std::enable_if_t<
      std::is_floating_point<typename K::FT>::value && fast_bbox_intersection,
          bool >
    operator()(const CGAL::Segment_3<K>& segment, const Bounding_box& bbox) const {
      const Point_3& p = segment.source();
      const Point_3& q = segment.target();

      return Intersections::internal::do_intersect_bbox_segment_aux
             <double,
             true, // bounded at t=0
             true, // bounded at t=1
             false> // do not use static filters
             (p.x(), p.y(), p.z(),
              q.x(), q.y(), q.z(),
              bbox);
    }

    template <class K>
    std::enable_if_t<
      std::is_floating_point<typename K::FT>::value && fast_bbox_intersection,
          bool >
    operator()(const CGAL::Ray_3<K>& ray, const Bounding_box& bbox) const {
      const Point_3& p = ray.source();
      const Point_3& q = ray.second_point();

      return Intersections::internal::do_intersect_bbox_segment_aux
             <double,
             true, // bounded at t=0
             false,// not bounded at t=1
             false> // do not use static filters
             (p.x(), p.y(), p.z(),
              q.x(), q.y(), q.z(),
              bbox);
    }

  };

  Do_intersect do_intersect_object() const {
    return Do_intersect(*this);
  }
};
/// @endcond

} //namespace CGAL
#endif //CGAL_INTERNAL_SURFACE_MESH_SEGMENTATION_AABB_TRAITS_H
