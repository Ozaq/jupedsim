// Copyright (c) 2005,2006,2007,2009,2010,2011 Tel-Aviv University (Israel).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Arrangement_on_surface_2/include/CGAL/Arr_non_caching_segment_basic_traits_2.h $
// $Id: include/CGAL/Arr_non_caching_segment_basic_traits_2.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s): Efi Fogel <efif@post.tau.ac.il>
//            Ron Wein  <wein@post.tau.ac.il>
//            (based on old version by: Iddo Hanniel,
//                                      Eyal Flato,
//                                      Oren Nechushtan,
//                                      Ester Ezra,
//                                      Shai Hirsch,
//                                      and Eugene Lipovetsky)

#ifndef CGAL_ARR_NON_CACHING_SEGMENT_BASIC_TRAITS_2_H
#define CGAL_ARR_NON_CACHING_SEGMENT_BASIC_TRAITS_2_H

#include <CGAL/license/Arrangement_on_surface_2.h>

#include <CGAL/disable_warnings.h>

/*! \file The basic non-caching segment traits-class for the arrangement
 * package. This traits class handles x-monotone non-intersecting segments.
 * It is a model of the ArrangementBasicTraits_2 concept. The class is
 * templated by a kernel and inherits from it all the types and many of the
 * functors required by the concept it models.
 */

#include <CGAL/Cartesian.h>
#include <CGAL/Algebraic_structure_traits.h>
#include <CGAL/number_utils.h>
#include <CGAL/tags.h>
#include <CGAL/Arr_tags.h>
#include <CGAL/assertions.h>
#include <CGAL/Arr_geometry_traits/Segment_assertions.h>

namespace CGAL {

/*! \class
 * A model of the ArrangementBasicTraits_2 concept that handles x-monotone
 * non-intersecting line segments.
 */
template <class T_Kernel>
class Arr_non_caching_segment_basic_traits_2 : public T_Kernel
{
public:

  typedef T_Kernel                              Kernel;

  typedef typename Kernel::FT                   FT;

private:
  typedef Algebraic_structure_traits<FT> AST;
  typedef typename AST::Is_exact FT_is_exact;
public:

  typedef Boolean_tag<FT_is_exact::value> Has_exact_division;

  typedef
  CGAL::Segment_assertions<Arr_non_caching_segment_basic_traits_2<Kernel> >
                                                Segment_assertions;

  // Categories:
  typedef Tag_true                              Has_left_category;
  typedef Tag_false                             Has_do_intersect_category;

  typedef Arr_oblivious_side_tag                Left_side_category;
  typedef Arr_oblivious_side_tag                Bottom_side_category;
  typedef Arr_oblivious_side_tag                Top_side_category;
  typedef Arr_oblivious_side_tag                Right_side_category;

  /*! Default Constructor */
  Arr_non_caching_segment_basic_traits_2()
  {}

  /// \name Types and functor inherited from the kernel
  //@{

  // Traits types:
  typedef typename Kernel::Point_2                 Point_2;
  typedef typename Kernel::Segment_2               X_monotone_curve_2;
  typedef unsigned int                             Multiplicity;

  /*! Compare the x-coordinates of two points */
  typedef typename Kernel::Compare_x_2             Compare_x_2;

  /*! Compare two points lexigoraphically; by x, then by y */
  typedef typename Kernel::Compare_xy_2            Compare_xy_2;

  /*! Obtain the left endpoint of a given segment */
  typedef typename Kernel::Construct_min_vertex_2  Construct_min_vertex_2;

  /*! Obtain the right endpoint of a given segment */
  typedef typename Kernel::Construct_max_vertex_2  Construct_max_vertex_2;

  /*! Check whether a given segment is vertical */
  typedef typename Kernel::Is_vertical_2           Is_vertical_2;

  /*! Return the location of a given point with respect to an input segment */
  typedef typename Kernel::Compare_y_at_x_2        Compare_y_at_x_2;

  /*! Check if two segments or if two points are identical */
  typedef typename Kernel::Equal_2                 Equal_2;

  //@}

  /// \name Functor introduced here (based on the kernel)
  //@{

  /*! \class
   * A functor for comparing two segments to the left of a point
   */
  class Compare_y_at_x_left_2 {
  public:

    /*
     * Compare the y value of two segments immediately to the left of their
     * intersection point.
     * \param cv1 The first segment.
     * \param cv2 The second segment.
     * \param p The intersection point.
     * \pre The point p lies on both segments, and both of them must be also be
     *      defined (lexicographically) to its left.
     * \return The relative position of cv1 with respect to cv2 immdiately to
     *         the left of p: SMALLER, LARGER or EQUAL.
     */
    Comparison_result operator()(const X_monotone_curve_2& cv1,
                                 const X_monotone_curve_2& cv2,
                                 const Point_2& CGAL_precondition_code(p)) const
    {
      Kernel kernel;

      // The two segments must be defined at q and also to its left.
      CGAL_precondition
        (Segment_assertions::_assert_is_point_on(p, cv1, Has_exact_division())&&
         Segment_assertions::_assert_is_point_on(p, cv2, Has_exact_division()));

      CGAL_precondition_code(
        Compare_xy_2 compare_xy = kernel.compare_xy_2_object();
        typename Kernel::Construct_vertex_2 construct_vertex =
          kernel.construct_vertex_2_object();
        const Point_2 & source1 = construct_vertex(cv1, 0);
        const Point_2 & target1 = construct_vertex(cv1, 1);
        const Point_2 & left1 =
          (kernel.less_xy_2_object()(source1, target1)) ? source1 : target1;
        const Point_2 & source2 = construct_vertex(cv2, 0);
        const Point_2 & target2 = construct_vertex(cv2, 1);
        const Point_2 & left2 =
          (kernel.less_xy_2_object()(source2, target2)) ? source2 : target2;
        );

      CGAL_precondition(compare_xy(left1, p) == SMALLER &&
                        compare_xy(left2, p) == SMALLER);

      // Compare the slopes of the two segments to determine thir relative
      // position immediately to the left of q.
      // Notice that we swap the order of the curves in order to obtain the
      // correct result to the left of p.
      return kernel.compare_slope_2_object()(cv2, cv1);
    }
  };

  /*! Obtain a Compare_y_at_x_left_2 functor object. */
  Compare_y_at_x_left_2 compare_y_at_x_left_2_object() const
  {
    return Compare_y_at_x_left_2();
  }

  /*! \class
   * A functor for comparing two segments to the right of a point.
   */
  class Compare_y_at_x_right_2 {
  public:

    /*!
     * Compare the y value of two segments immediately to the right of their
     * intersection point.
     * \param cv1 The first segment.
     * \param cv2 The second segment.
     * \param p The intersection point.
     * \pre The point p lies on both segments, and both of them must be also be
     *      defined (lexicographically) to its right.
     * \return The relative position of cv1 with respect to cv2 immdiately to
     *         the right of p: SMALLER, LARGER or EQUAL.
     */

    Comparison_result operator()(const X_monotone_curve_2 & cv1,
                                 const X_monotone_curve_2 & cv2,
                                 const Point_2 & CGAL_precondition_code(p)) const
    {
      Kernel kernel;

      // The two segments must be defined at q and also to its right.
      CGAL_precondition
        (Segment_assertions::_assert_is_point_on(p, cv1, Has_exact_division())&&
         Segment_assertions::_assert_is_point_on(p, cv2, Has_exact_division()));

      CGAL_precondition_code(
        Compare_xy_2 compare_xy = kernel.compare_xy_2_object();
        typename Kernel::Construct_vertex_2 construct_vertex =
          kernel.construct_vertex_2_object();
        const Point_2 & source1 = construct_vertex(cv1, 0);
        const Point_2 & target1 = construct_vertex(cv1, 1);
        const Point_2 & right1 =
          (kernel.less_xy_2_object()(source1, target1)) ? target1 : source1;
        const Point_2 & source2 = construct_vertex(cv2, 0);
        const Point_2 & target2 = construct_vertex(cv2, 1);
        const Point_2 & right2 =
          (kernel.less_xy_2_object()(source2, target2)) ? target2 : source2;
        );

      CGAL_precondition(compare_xy(right1, p) == LARGER &&
                        compare_xy(right2, p) == LARGER);

      // Compare the slopes of the two segments to determine thir relative
      // position immediately to the left of q.
       return kernel.compare_slope_2_object()(cv1, cv2);
    }
  };

  /*! Obtain a Compare_y_at_x_right_2 functor object. */
  Compare_y_at_x_right_2 compare_y_at_x_right_2_object() const
  {
    return Compare_y_at_x_right_2();
  }
  //@}

  /// \name Functor definitions for the landmarks point-location strategy.
  //@{
  typedef double                                        Approximate_number_type;
  typedef CGAL::Cartesian<Approximate_number_type>      Approximate_kernel;
  typedef Approximate_kernel::Point_2                   Approximate_point_2;

  class Approximate_2 {
  protected:
    using Traits = Arr_non_caching_segment_basic_traits_2<Kernel>;

    /*! The traits (in case it has state) */
    const Traits& m_traits;

    /*! Constructor
     * \param traits the traits.
     */
    Approximate_2(const Traits& traits) : m_traits(traits) {}

    friend class Arr_non_caching_segment_basic_traits_2<Kernel>;

  public:
    /*! Return an approximation of a point coordinate.
     * \param p The exact point.
     * \param i The coordinate index (either 0 or 1).
     * \pre i is either 0 or 1.
     * \return An approximation of p's x-coordinate (if i == 0), or an
     *         approximation of p's y-coordinate (if i == 1).
     */
    Approximate_number_type operator() (const Point_2& p, int i) const {
      CGAL_precondition (i == 0 || i == 1);
      return (i == 0) ? (CGAL::to_double(p.x())) : (CGAL::to_double(p.y()));
    }

    /*! Obtain an approximation of a point.
     */
    Approximate_point_2 operator()(const Point_2& p) const
    { return Approximate_point_2(operator()(p, 0), operator()(p, 1)); }

    /*! Obtain an approximation of an \f$x\f$-monotone curve.
     */
    template <typename OutputIterator>
    OutputIterator operator()(const X_monotone_curve_2& xcv, double /* error */,
                              OutputIterator oi, bool l2r = true) const {
      auto min_vertex = m_traits.construct_min_vertex_2_object();
      auto max_vertex = m_traits.construct_max_vertex_2_object();
      const auto& src = (l2r) ? min_vertex(xcv) : max_vertex(xcv);
      const auto& trg = (l2r) ? max_vertex(xcv) : min_vertex(xcv);
      auto xs = CGAL::to_double(src.x());
      auto ys = CGAL::to_double(src.y());
      auto xt = CGAL::to_double(trg.x());
      auto yt = CGAL::to_double(trg.y());
      *oi++ = Approximate_point_2(xs, ys);
      *oi++ = Approximate_point_2(xt, yt);
      return oi;
    }
  };

  /*! Get an Approximate_2 functor object. */
  Approximate_2 approximate_2_object () const { return Approximate_2(*this); }

  typedef typename Kernel::Construct_segment_2    Construct_x_monotone_curve_2;

  /*! Get a Construct_x_monotone_curve_2 functor object. */
  Construct_x_monotone_curve_2 construct_x_monotone_curve_2_object () const
  {
    return (this->construct_segment_2_object());
  }
  //@}
};

} //namespace CGAL

#include <CGAL/enable_warnings.h>

#endif
