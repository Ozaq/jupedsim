// Copyright (c) 1999
// Utrecht University (The Netherlands),
// ETH Zurich (Switzerland),
// INRIA Sophia-Antipolis (France),
// Max-Planck-Institute Saarbruecken (Germany),
// and Tel-Aviv University (Israel).  All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Kernel_23/include/CGAL/Segment_3.h $
// $Id: include/CGAL/Segment_3.h 50cfbde3b84 $
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Andreas Fabri, Stefan Schirra

#ifndef CGAL_SEGMENT_3_H
#define CGAL_SEGMENT_3_H

#include <CGAL/assertions.h>
#include <CGAL/Kernel/Return_base_tag.h>
#include <CGAL/kernel_assertions.h>
#include <CGAL/kernel_config.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/Dimension.h>

namespace CGAL {

template <class R_>
class Segment_3 : public R_::Kernel_base::Segment_3
{
  typedef typename R_::RT                    RT;
  typedef typename R_::FT                    FT;
  typedef typename R_::Point_3               Point_3;
  typedef typename R_::Vector_3              Vector_3;
  typedef typename R_::Direction_3           Direction_3;
  typedef typename R_::Line_3                Line_3;
  typedef typename R_::Aff_transformation_3  Aff_transformation_3;

  typedef Segment_3                          Self;
  static_assert(std::is_same<Self, typename R_::Segment_3>::value);

public:

  typedef Dimension_tag<3>  Ambient_dimension;
  typedef Dimension_tag<1>  Feature_dimension;

  typedef typename R_::Kernel_base::Segment_3  Rep;

  const Rep& rep() const
  {
    return *this;
  }

  Rep& rep()
  {
    return *this;
  }

  typedef          R_                       R;

  Segment_3() {}

  Segment_3(const Rep& s)
      : Rep(s) {}

  Segment_3(Rep&& s)
      : Rep(std::move(s)) {}

  Segment_3(const Point_3& sp, const Point_3& ep)
    : Rep(typename R::Construct_segment_3()(Return_base_tag(), sp, ep)) {}

  decltype(auto)
  source() const
  {
    return R_().construct_source_3_object()(*this);
  }

  decltype(auto)
  target() const
  {
    return R_().construct_target_3_object()(*this);
  }

  decltype(auto)
  start() const
  {
    return source();
  }

  decltype(auto)
  end() const
  {
    return target();
  }

  decltype(auto)
  min BOOST_PREVENT_MACRO_SUBSTITUTION() const {
    typename R_::Less_xyz_3 less_xyz;
    return less_xyz(source(), target()) ? source() : target();
  }

  decltype(auto)
  max BOOST_PREVENT_MACRO_SUBSTITUTION() const {
    typename R_::Less_xyz_3 less_xyz;
    return less_xyz(source(), target()) ? target() : source();
  }

  decltype(auto)
  vertex(int i) const
  { return (i%2 == 0) ? source() : target(); }

  decltype(auto)
  point(int i) const
  { return vertex(i); }

  decltype(auto)
  operator[](int i) const
  { return vertex(i); }


  Segment_3 transform(const Aff_transformation_3 &t) const
  {
    return Segment_3(t.transform(this->source()), t.transform(this->target()));
  }

  FT squared_length() const
  {
    return squared_distance(this->target(), this->source());
  }

  Vector_3 to_vector() const
  {
    return R().construct_vector_3_object()(*this);
  }

  bool has_on(const Point_3 &p) const
  {
    return R().has_on_3_object()(*this, p);
  }

  Segment_3 opposite() const
  {
    return R().construct_opposite_segment_3_object()(*this);
  }

  Direction_3 direction() const
  {
    return R().construct_direction_3_object()(*this);
  }

  bool is_degenerate() const
  {
    return R().is_degenerate_3_object()(*this);
  }

  Bbox_3 bbox() const
  {
    return R().construct_bbox_3_object()(*this);
  }

  Line_3
  supporting_line() const
  {
    return R().construct_line_3_object()(*this);
  }

};


template < class R >
std::ostream &
operator<<(std::ostream &os, const Segment_3<R> &s)
{
    switch(IO::get_mode(os)) {
    case IO::ASCII :
        return os << s.source() << ' ' << s.target();
    case IO::BINARY :
        return os << s.source() << s.target();
    default:
        return os << "Segment_3(" << s.source() <<  ", " << s.target() << ")";
    }
}

template < class R >
std::istream &
operator>>(std::istream &is, Segment_3<R> &s)
{
    typename R::Point_3 p, q;

    is >> p >> q;

    if (is)
        s = Segment_3<R>(p, q);
    return is;
}

} //namespace CGAL

#endif // CGAL_SEGMENT_3_H
