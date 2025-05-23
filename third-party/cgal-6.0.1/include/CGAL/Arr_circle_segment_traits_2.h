// Copyright (c) 2005,2006,2007,2009,2010,2011 Tel-Aviv University (Israel).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Arrangement_on_surface_2/include/CGAL/Arr_circle_segment_traits_2.h $
// $Id: include/CGAL/Arr_circle_segment_traits_2.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s): Ron Wein          <wein@post.tau.ac.il>
//            Baruch Zukerman   <baruchzu@post.tau.ac.il>
//            Waqar Khan        <wkhan@mpi-inf.mpg.de>
//            Efi Fogel         <efifogel@gmail.com>

#ifndef CGAL_ARR_CIRCLE_SEGMENT_TRAITS_2_H
#define CGAL_ARR_CIRCLE_SEGMENT_TRAITS_2_H

#include <CGAL/license/Arrangement_on_surface_2.h>

#include <CGAL/disable_warnings.h>

/*! \file
 * The header file for the Arr_circle_segment_traits_2<Kernel> class.
 */

// Keep the following 2 lines first.
#include <cmath>
#include <fstream>
#include <atomic>

#include <CGAL/tags.h>
#include <CGAL/Arr_tags.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Arr_geometry_traits/Circle_segment_2.h>

namespace CGAL {

/*! \class
 * A traits class for maintaining an arrangement of circles.
 */
template <typename Kernel_, bool Filter = true>
class Arr_circle_segment_traits_2 {
public:
  typedef Kernel_                                        Kernel;
  typedef typename Kernel::FT                            NT;
  typedef typename Kernel::Point_2                       Rational_point_2;
  typedef typename Kernel::Segment_2                     Rational_segment_2;
  typedef typename Kernel::Circle_2                      Rational_circle_2;
  typedef _One_root_point_2<NT, Filter>                  Point_2;
  typedef typename Point_2::CoordNT                      CoordNT;
  typedef _Circle_segment_2<Kernel, Filter>              Curve_2;
  typedef _X_monotone_circle_segment_2<Kernel, Filter>   X_monotone_curve_2;
  typedef unsigned int                                   Multiplicity;
  typedef Arr_circle_segment_traits_2<Kernel, Filter>    Self;

  // Category tags:
  typedef Tag_true                                   Has_left_category;
  typedef Tag_true                                   Has_merge_category;
  typedef Tag_false                                  Has_do_intersect_category;

  typedef Arr_oblivious_side_tag                     Left_side_category;
  typedef Arr_oblivious_side_tag                     Bottom_side_category;
  typedef Arr_oblivious_side_tag                     Top_side_category;
  typedef Arr_oblivious_side_tag                     Right_side_category;

protected:
  // Type definition for the intersection points mapping.
  typedef typename X_monotone_curve_2::Intersection_map   Intersection_map;

  mutable Intersection_map inter_map;   // Mapping pairs of curve IDs to their
                                        // intersection points.
  bool m_use_cache;

public:
  /*! Default constructor. */
  Arr_circle_segment_traits_2 (bool use_intersection_caching = false) :
    m_use_cache(use_intersection_caching)
  {}

  /*! Get the next curve index. */
  static unsigned int get_index ()
  {
#ifdef CGAL_NO_ATOMIC
    static unsigned int index;
#else
    static std::atomic<unsigned int> index;
#endif
    return (++index);
  }

  /// \name Basic functor definitions.
  //@{

  class Compare_x_2
  {
  public:
    /*!
     * Compare the x-coordinates of two points.
     * \param p1 The first point.
     * \param p2 The second point.
     * \return LARGER if x(p1) > x(p2);
     *         SMALLER if x(p1) < x(p2);
     *         EQUAL if x(p1) = x(p2).
     */
    Comparison_result operator() (const Point_2& p1, const Point_2& p2) const
    {
      if (p1.identical (p2))
        return (EQUAL);

      return (CGAL::compare (p1.x(), p2.x()));
    }
  };

  /*! Get a Compare_x_2 functor object. */
  Compare_x_2 compare_x_2_object () const
  {
    return Compare_x_2();
  }

  class Compare_xy_2
  {
  public:
    /*!
     * Compares two points lexigoraphically: by x, then by y.
     * \param p1 The first point.
     * \param p2 The second point.
     * \return LARGER if x(p1) > x(p2), or if x(p1) = x(p2) and y(p1) > y(p2);
     *         SMALLER if x(p1) < x(p2), or if x(p1) = x(p2) and y(p1) < y(p2);
     *         EQUAL if the two points are equal.
     */
    Comparison_result operator() (const Point_2& p1, const Point_2& p2) const
    {
      if (p1.identical (p2))
        return (EQUAL);

      Comparison_result  res = CGAL::compare (p1.x(), p2.x());

      if (res != EQUAL)
        return (res);

      return (CGAL::compare (p1.y(), p2.y()));
    }
  };

  /*! Get a Compare_xy_2 functor object. */
  Compare_xy_2 compare_xy_2_object () const
  {
    return Compare_xy_2();
  }

  class Construct_min_vertex_2
  {
  public:
    /*!
     * Get the left endpoint of the x-monotone curve (segment).
     * \param cv The curve.
     * \return The left endpoint.
     */
    const Point_2& operator() (const X_monotone_curve_2 & cv) const
    {
      return (cv.left());
    }
  };

  /*! Get a Construct_min_vertex_2 functor object. */
  Construct_min_vertex_2 construct_min_vertex_2_object () const
  {
    return Construct_min_vertex_2();
  }

  class Construct_max_vertex_2
  {
  public:
    /*!
     * Get the right endpoint of the x-monotone curve (segment).
     * \param cv The curve.
     * \return The right endpoint.
     */
    const Point_2& operator() (const X_monotone_curve_2 & cv) const
    {
      return (cv.right());
    }
  };

  /*! Get a Construct_max_vertex_2 functor object. */
  Construct_max_vertex_2 construct_max_vertex_2_object () const
  {
    return Construct_max_vertex_2();
  }

  class Is_vertical_2
  {
  public:
    /*!
     * Check whether the given x-monotone curve is a vertical segment.
     * \param cv The curve.
     * \return (true) if the curve is a vertical segment; (false) otherwise.
     */
    bool operator() (const X_monotone_curve_2& cv) const
    {
      return (cv.is_vertical());
    }
  };

  /*! Get an Is_vertical_2 functor object. */
  Is_vertical_2 is_vertical_2_object () const
  {
    return Is_vertical_2();
  }

  class Compare_y_at_x_2
  {
  public:
    /*!
     * Return the location of the given point with respect to the input curve.
     * \param cv The curve.
     * \param p The point.
     * \pre p is in the x-range of cv.
     * \return SMALLER if y(p) < cv(x(p)), i.e. the point is below the curve;
     *         LARGER if y(p) > cv(x(p)), i.e. the point is above the curve;
     *         EQUAL if p lies on the curve.
     */
    Comparison_result operator() (const Point_2& p,
                                  const X_monotone_curve_2& cv) const
    {
      CGAL_precondition (cv.is_in_x_range (p));

      return (cv.point_position (p));
    }
  };

  /*! Get a Compare_y_at_x_2 functor object. */
  Compare_y_at_x_2 compare_y_at_x_2_object () const
  {
    return Compare_y_at_x_2();
  }

  class Compare_y_at_x_right_2
  {
  public:
    /*!
     * Compares the y value of two x-monotone curves immediately to the right
     * of their intersection point.
     * \param cv1 The first curve.
     * \param cv2 The second curve.
     * \param p The intersection point.
     * \pre The point p lies on both curves, and both of them must be also be
     *      defined (lexicographically) to its right.
     * \return The relative position of cv1 with respect to cv2 immdiately to
     *         the right of p: SMALLER, LARGER or EQUAL.
     */
    Comparison_result operator() (const X_monotone_curve_2& cv1,
                                  const X_monotone_curve_2& cv2,
                                  const Point_2& p) const
    {
      // Make sure that p lies on both curves, and that both are defined to its
      // right (so their right endpoint is lexicographically larger than p).
      CGAL_precondition (cv1.point_position (p) == EQUAL &&
                         cv2.point_position (p) == EQUAL);

      if ((CGAL::compare (cv1.left().x(),cv1.right().x()) == EQUAL) &&
          (CGAL::compare (cv2.left().x(),cv2.right().x()) == EQUAL))
      { //both cv1 and cv2 are vertical
        CGAL_precondition (!(cv1.right()).equals(p) && !(cv2.right()).equals(p));
      }
      else if ((CGAL::compare (cv1.left().x(),cv1.right().x()) != EQUAL) &&
               (CGAL::compare (cv2.left().x(),cv2.right().x()) == EQUAL))
      { //only cv1 is vertical
        CGAL_precondition (!(cv1.right()).equals(p));
      }
      else if ((CGAL::compare (cv1.left().x(),cv1.right().x()) == EQUAL) &&
               (CGAL::compare (cv2.left().x(),cv2.right().x()) != EQUAL))
      { //only cv2 is vertical
        CGAL_precondition (!(cv2.right()).equals(p));
      }
      else
      { //both cv1 and cv2 are non vertical
        CGAL_precondition (CGAL::compare (cv1.right().x(),p.x()) == LARGER &&
                           CGAL::compare (cv2.right().x(),p.x()) == LARGER);
      }
      // Compare the two curves immediately to the right of p:
      return (cv1.compare_to_right (cv2, p));
    }
  };

  /*! Get a Compare_y_at_x_right_2 functor object. */
  Compare_y_at_x_right_2 compare_y_at_x_right_2_object () const
  {
    return Compare_y_at_x_right_2();
  }

  class Compare_y_at_x_left_2
  {
  public:
    /*!
     * Compares the y value of two x-monotone curves immediately to the left
     * of their intersection point.
     * \param cv1 The first curve.
     * \param cv2 The second curve.
     * \param p The intersection point.
     * \pre The point p lies on both curves, and both of them must be also be
     *      defined (lexicographically) to its left.
     * \return The relative position of cv1 with respect to cv2 immdiately to
     *         the left of p: SMALLER, LARGER or EQUAL.
     */
    Comparison_result operator() (const X_monotone_curve_2& cv1,
                                  const X_monotone_curve_2& cv2,
                                  const Point_2& p) const
    {
      // Make sure that p lies on both curves, and that both are defined to its
      // left (so their left endpoint is lexicographically smaller than p).

      CGAL_precondition (cv1.point_position (p) == EQUAL &&
                         cv2.point_position (p) == EQUAL);

      if ((CGAL::compare (cv1.left().x(),cv1.right().x()) == EQUAL) &&
          (CGAL::compare (cv2.left().x(),cv2.right().x()) == EQUAL))
          { //both cv1 and cv2 are vertical
         CGAL_precondition (!(cv1.left()).equals(p) && !(cv2.left()).equals(p));
          }
          else if ((CGAL::compare (cv1.left().x(),cv1.right().x()) != EQUAL) &&
                   (CGAL::compare (cv2.left().x(),cv2.right().x()) == EQUAL))
          { //only cv1 is vertical
         CGAL_precondition (!(cv1.left()).equals(p));
          }
          else if ((CGAL::compare (cv1.left().x(),cv1.right().x()) == EQUAL) &&
                   (CGAL::compare (cv2.left().x(),cv2.right().x()) != EQUAL))
          { //only cv2 is vertical
         CGAL_precondition (!(cv2.left()).equals(p));
          }
          else
          { //both cv1 and cv2 are non vertical
        CGAL_precondition (CGAL::compare (cv1.left().x(),p.x()) == SMALLER &&
                           CGAL::compare (cv2.left().x(),p.x()) == SMALLER);
          }
      // Compare the two curves immediately to the left of p:
      return (cv1.compare_to_left (cv2, p));
    }
  };

  /*! Get a Compare_y_at_x_left_2 functor object. */
  Compare_y_at_x_left_2 compare_y_at_x_left_2_object () const
  {
    return Compare_y_at_x_left_2();
  }

  class Equal_2
  {
  public:
    /*!
     * Check if the two x-monotone curves are the same (have the same graph).
     * \param cv1 The first curve.
     * \param cv2 The second curve.
     * \return (true) if the two curves are the same; (false) otherwise.
     */
    bool operator() (const X_monotone_curve_2& cv1,
                     const X_monotone_curve_2& cv2) const
    {
      if (&cv1 == &cv2)
        return (true);

      return (cv1.equals (cv2));
    }

    /*!
     * Check if the two points are the same.
     * \param p1 The first point.
     * \param p2 The second point.
     * \return (true) if the two point are the same; (false) otherwise.
     */
    bool operator() (const Point_2& p1, const Point_2& p2) const
    {
      return (p1.equals (p2));
    }
  };

  /*! Get an Equal_2 functor object. */
  Equal_2 equal_2_object () const
  {
    return Equal_2();
  }
  //@}

  /// \name Functor definitions for approximations. Used by the landmarks
  // point-location strategy and the drawing procedure.
  //@{
  typedef double                                        Approximate_number_type;
  typedef CGAL::Cartesian<Approximate_number_type>      Approximate_kernel;
  typedef Approximate_kernel::Point_2                   Approximate_point_2;

  class Approximate_2 {
  protected:
    using Traits = Arr_circle_segment_traits_2<Kernel, Filter>;

    /*! The traits (in case it has state) */
    const Traits& m_traits;

    /*! Constructor
     * \param traits the traits.
     */
    Approximate_2(const Traits& traits) : m_traits(traits) {}

    friend class Arr_circle_segment_traits_2<Kernel, Filter>;

  public:
    /*! Obtain an approximation of a point coordinate.
     * \param p the exact point.
     * \param i the coordinate index (either 0 or 1).
     * \pre i is either 0 or 1.
     * \return An approximation of p's x-coordinate (if i == 0), or an
     *         approximation of p's y-coordinate (if i == 1).
     */
    Approximate_number_type operator()(const Point_2& p, int i) const {
      CGAL_precondition((i == 0) || (i == 1));
      return (i == 0) ? (CGAL::to_double(p.x())) : (CGAL::to_double(p.y()));
    }

    /*! Obtain an approximation of a point.
     */
    Approximate_point_2 operator()(const Point_2& p) const
    { return Approximate_point_2(operator()(p, 0), operator()(p, 1)); }

    /*! Obtain an approximation of an \f$x\f$-monotone curve.
     */
    template <typename OutputIterator>
    OutputIterator operator()(const X_monotone_curve_2& xcv, double error,
                              OutputIterator oi, bool l2r = true) const {
      if (xcv.is_linear()) return approximate_segment(xcv, oi, l2r);
      return approximate_arc(xcv, error, oi, l2r);;
    }

  private:
    /*! Handle segments.
     */
    template <typename OutputIterator>
    OutputIterator approximate_segment(const X_monotone_curve_2& xcv,
                                       OutputIterator oi,
                                       bool l2r = true) const {
      // std::cout << "SEGMENT\n";
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

    template <typename OutputIterator, typename Op, typename Transform>
    OutputIterator add_points(double x1, double y1, double t1,
                              double x2, double y2, double t2,
                              double error, OutputIterator oi,
                              Op op, Transform transform) const {
      auto tm = (t1 + t2)*0.5;

      // Compute the canocal point where the error is maximal.
      double xm, ym;
      op(tm, xm, ym);

      auto dx = x2 - x1;
      auto dy = y2 - y1;

      // Compute the error; abort if it is below the threshold
      auto l = std::sqrt(dx*dx + dy*dy);
      auto e = std::abs((xm*dy - ym*dx + x2*y1 - x1*y2) / l);
      if (e < error) return oi;

      double x, y;
      transform(xm, ym, x, y);
      add_points(x1, y1, t1, xm, ym, tm, error, oi, op, transform);
      *oi++ = Approximate_point_2(x, y);
      add_points(xm, ym, tm, x2, y2, t2, error, oi, op, transform);
      return oi;
    }

    /*! Compute the circular point given the parameter t and the transform
     * data, that is, the center (translation) and the sin and cos of the
     * rotation angle.
     */
    void circular_point(double r, double t, double& x, double& y) const {
      x = r * std::cos(t);
      y = r * std::sin(t);
    }

    /*! Transform a point. In particular, rotate the canonical point
     * (`xc`,`yc`) by an angle, the sine and cosine of which are `sint` and
     * `cost`, respectively, and translate by (`cx`,`cy`).
     */
    void transform_point(double xc, double yc, double cx, double cy,
                         double& x, double& y) const {
      x = xc + cx;
      y = yc + cy;
    }

    /*! Handle circular arcs.
     */
    template <typename OutputIterator>
    OutputIterator approximate_arc(const X_monotone_curve_2& xcv,
                                   double error, OutputIterator oi,
                                   bool l2r = true) const {
      auto min_vertex = m_traits.construct_min_vertex_2_object();
      auto max_vertex = m_traits.construct_max_vertex_2_object();
      const auto& src = (l2r) ? min_vertex(xcv) : max_vertex(xcv);
      const auto& trg = (l2r) ? max_vertex(xcv) : min_vertex(xcv);
      auto xs = CGAL::to_double(src.x());
      auto ys = CGAL::to_double(src.y());
      auto xt = CGAL::to_double(trg.x());
      auto yt = CGAL::to_double(trg.y());

      const typename Kernel::Circle_2& circ = xcv.supporting_circle();
      auto r_sqr = circ.squared_radius();
      auto r = std::sqrt(CGAL::to_double(r_sqr));

      // Obtain the center:
      auto cx = CGAL::to_double(circ.center().x());
      auto cy = CGAL::to_double(circ.center().y());

      // Inverse transform the source and target
      auto xs_t = xs - cx;
      auto ys_t = ys - cy;
      auto xt_t = xt - cx;
      auto yt_t = yt - cy;

      // Compute the parameters ts and tt such that
      // source == (x(ts),y(ts)), and
      // target == (x(tt),y(tt))
      auto ts = std::atan2(r*ys_t, r*xs_t);
      if (ts < 0) ts += 2*CGAL_PI;
      auto tt = std::atan2(r*yt_t, r*xt_t);
      if (tt < 0) tt += 2*CGAL_PI;
      auto orient(xcv.orientation());
      if (xcv.source() != src) orient = CGAL::opposite(orient);
      if (orient == COUNTERCLOCKWISE) {
        if (tt < ts) tt += 2*CGAL_PI;
      }
      else {
        if (ts < tt) ts += 2*CGAL_PI;
      }

      *oi++ = Approximate_point_2(xs, ys);
      add_points(xs_t, ys_t, ts, xt_t, yt_t, tt, error, oi,
                 [&](double tm, double& xm, double& ym) {
                   circular_point(r, tm, xm, ym);
                 },
                 [&](double xc, double& yc, double& x, double& y) {
                   transform_point(xc, yc, cx, cy, x, y);
                 });
      *oi++ = Approximate_point_2(xt, yt);
      return oi;
    }
  };

  /*! Obtain an Approximate_2 functor object. */
  Approximate_2 approximate_2_object() const { return Approximate_2(*this); }
  //@}

  /// \name Intersections, subdivisions, and mergings
  //@{

  /*! \class
   * A functor for subdividing a curve into x-monotone curves.
   */
  class Make_x_monotone_2 {
  private:
    typedef Arr_circle_segment_traits_2<Kernel_, Filter> Self;

    bool m_use_cache;

  public:
    Make_x_monotone_2(bool use_cache = false) : m_use_cache(use_cache) {}

    /*! Subdivide a given circular arc or line segment into x-monotone subcurves
     * and insert them to a given output iterator.
     * \param cv the curve.
     * \param oi the output iterator for the result. Its dereference type is a
     *           variant that wraps a \c Point_2 or an \c X_monotone_curve_2
     *           objects.
     * \return the past-the-end iterator.
     */
    template <typename OutputIterator>
    OutputIterator operator()(const Curve_2& cv, OutputIterator oi) const
    {
      // Increment the serial number of the curve cv, which will serve as its
      // unique identifier.
      unsigned int index = 0;
      if (m_use_cache) index = Self::get_index();

      if (cv.orientation() == COLLINEAR) {
        // The curve is a line segment.
        *oi++ = X_monotone_curve_2(cv.supporting_line(),
                                   cv.source(),
                                   cv.target(),
                                   index);
        return oi;
      }

      // Check the case of a degenerate circle (a point).
      const typename Kernel::Circle_2&  circ = cv.supporting_circle();
      CGAL::Sign   sign_rad = CGAL::sign (circ.squared_radius());
      CGAL_precondition (sign_rad != NEGATIVE);

      if (sign_rad == ZERO) {
        // Create an isolated point.
        *oi++ = Point_2(circ.center().x(),
                        circ.center().y());
        return oi;
      }

      // The curve is circular: compute the to vertical tangency points
      // of the supporting circle.
      Point_2         vpts[2];
      unsigned int    n_vpts = cv.vertical_tangency_points (vpts);

      if (cv.is_full()) {
        CGAL_assertion (n_vpts == 2);

        // Subdivide the circle into two arcs (an upper and a lower half).
        *oi++ = X_monotone_curve_2(circ,
                                   vpts[0], vpts[1],
                                   cv.orientation(),
                                   index);

        *oi++ = X_monotone_curve_2(circ,
                                   vpts[1], vpts[0],
                                   cv.orientation(),
                                   index);
      }
      else {
        // Act according to the number of vertical tangency points.
        if (n_vpts == 2) {
          // Subdivide the circular arc into three x-monotone arcs.
          *oi++ = X_monotone_curve_2(circ,
                                     cv.source(), vpts[0],
                                     cv.orientation(),
                                     index);

          *oi++ = X_monotone_curve_2(circ,
                                     vpts[0], vpts[1],
                                     cv.orientation(),
                                     index);

          *oi++ = X_monotone_curve_2(circ,
                                     vpts[1],
                                     cv.target(),
                                     cv.orientation(),
                                     index);
        }
        else if (n_vpts == 1) {
          // Subdivide the circular arc into two x-monotone arcs.
          *oi++ = X_monotone_curve_2(circ,
                                     cv.source(),
                                     vpts[0],
                                     cv.orientation(),
                                     index);

          *oi++ = X_monotone_curve_2(circ,
                                     vpts[0],
                                     cv.target(),
                                     cv.orientation(),
                                     index);
        }
        else {
          CGAL_assertion(n_vpts == 0);

          // The arc is already x-monotone:
          *oi++ = X_monotone_curve_2(circ,
                                     cv.source(),
                                     cv.target(),
                                     cv.orientation(),
                                     index);
        }
      }

      return oi;
    }
  };

  /*! Get a Make_x_monotone_2 functor object. */
  Make_x_monotone_2 make_x_monotone_2_object() const
  { return Make_x_monotone_2(m_use_cache); }

  class Split_2
  {
  public:

    /*!
     * Split a given x-monotone curve at a given point into two sub-curves.
     * \param cv The curve to split
     * \param p The split point.
     * \param c1 Output: The left resulting subcurve (p is its right endpoint).
     * \param c2 Output: The right resulting subcurve (p is its left endpoint).
     * \pre p lies on cv but is not one of its end-points.
     */
    void operator() (const X_monotone_curve_2& cv, const Point_2& p,
                     X_monotone_curve_2& c1, X_monotone_curve_2& c2) const
    {
      CGAL_precondition (cv.point_position(p)==EQUAL &&
      ! p.equals (cv.source()) &&
      ! p.equals (cv.target()));

      cv.split (p, c1, c2);
      return;
    }
  };

  /*! Get a Split_2 functor object. */
  Split_2 split_2_object () const
  {
    return Split_2();
  }

  class Intersect_2 {
  private:
    Intersection_map& _inter_map;       // The map of intersection points.

  public:
    /*! Constructor. */
    Intersect_2(Intersection_map& map) : _inter_map(map) {}

    /*! Find the intersections of the two given curves and insert them to the
     * given output iterator. As two segments may itersect only once, only a
     * single will be contained in the iterator.
     * \param cv1 The first curve.
     * \param cv2 The second curve.
     * \param oi The output iterator.
     * \return The past-the-end iterator.
     */
    template <typename OutputIterator>
    OutputIterator operator()(const X_monotone_curve_2& cv1,
                              const X_monotone_curve_2& cv2,
                              OutputIterator oi) const
    { return (cv1.intersect(cv2, oi, &_inter_map)); }
  };

  /*! Get an Intersect_2 functor object. */
  Intersect_2 intersect_2_object() const { return (Intersect_2(inter_map)); }

  class Are_mergeable_2
  {
  public:
    /*!
     * Check whether it is possible to merge two given x-monotone curves.
     * \param cv1 The first curve.
     * \param cv2 The second curve.
     * \return (true) if the two curves are mergeable - if they are supported
     *         by the same line and share a common endpoint; (false) otherwise.
     */
    bool operator() (const X_monotone_curve_2& cv1,
                     const X_monotone_curve_2& cv2) const
    {
      return (cv1.can_merge_with (cv2));
    }
  };

  /*! Get an Are_mergeable_2 functor object. */
  Are_mergeable_2 are_mergeable_2_object () const
  {
    return Are_mergeable_2();
  }

  /*! \class Merge_2
   * A functor that merges two x-monotone arcs into one.
   */
  class Merge_2
  {
  protected:
    typedef Arr_circle_segment_traits_2<Kernel, Filter> Traits;

    /*! The traits (in case it has state) */
    const Traits* m_traits;

    /*! Constructor
     * \param traits the traits (in case it has state)
     */
    Merge_2(const Traits* traits) : m_traits(traits) {}

    friend class Arr_circle_segment_traits_2<Kernel, Filter>;

  public:
    /*!
     * Merge two given x-monotone curves into a single curve.
     * \param cv1 The first curve.
     * \param cv2 The second curve.
     * \param c Output: The merged curve.
     * \pre The two curves are mergeable.
     */
    void operator() (const X_monotone_curve_2& cv1,
                     const X_monotone_curve_2& cv2,
                     X_monotone_curve_2& c) const
    {
      CGAL_precondition(m_traits->are_mergeable_2_object()(cv2, cv1));

      c = cv1;
      c.merge (cv2);
    }
  };

  /*! Get a Merge_2 functor object. */
  Merge_2 merge_2_object () const
  {
    return Merge_2(this);
  }

  class Compare_endpoints_xy_2
  {
  public:
    /*!
     * compare lexicogrphic the endpoints of a x-monotone curve.
     * \param cv the curve
     * \return SMALLER if the curve is directed right, else return SMALLER
     */
    Comparison_result operator()(const X_monotone_curve_2& cv) const
    {
      if(cv.is_directed_right())
        return(SMALLER);
      return (LARGER);
    }
  };

  /*! Get a Compare_endpoints_xy_2 functor object. */
  Compare_endpoints_xy_2 compare_endpoints_xy_2_object() const
  {
    return Compare_endpoints_xy_2();
  }

  class Construct_opposite_2
  {
  public:
    /*!
     * construct an opposite x-monotone curve.
     * \param cv the curve
     * \return an opposite x-monotone curve.
     */
    X_monotone_curve_2 operator()(const X_monotone_curve_2& cv) const
    {
      return cv.construct_opposite();
    }
  };

  /*! Get a Construct_opposite_2 functor object. */
  Construct_opposite_2 construct_opposite_2_object() const
  {
    return Construct_opposite_2();
  }

  class Trim_2 {
  protected:
    typedef Arr_circle_segment_traits_2<Kernel, Filter> Traits;

    /*! The traits (in case it has state) */
    const Traits& m_traits;

    /*! Constructor
     * \param traits the traits (in case it has state)
     */
    Trim_2(const Traits& traits) : m_traits(traits) {}

    friend class Arr_circle_segment_traits_2<Kernel, Filter>;

  public:
    /*! Obtain a trimmed version of an arc
     * \param xcv The arc
     * \param src the new first endpoint
     * \param tgt the new second endpoint
     * \return The trimmed arc
     * \pre src != tgt
     * \pre both points must be interior and must lie on \c cv
     */
    X_monotone_curve_2 operator()(const X_monotone_curve_2& xcv,
                                  const Point_2& src,
                                  const Point_2& tgt)const
    {
      // make functor objects
      CGAL_precondition_code(Compare_y_at_x_2 compare_y_at_x_2 =
                             m_traits.compare_y_at_x_2_object());
      CGAL_precondition_code(Equal_2 equal_2 = m_traits.equal_2_object());
      Compare_x_2 compare_x_2 = m_traits.compare_x_2_object();
      // Check whether source and target are two distinct points and they lie
      // on the line.
      CGAL_precondition(compare_y_at_x_2(src, xcv) == EQUAL);
      CGAL_precondition(compare_y_at_x_2(tgt, xcv) == EQUAL);
      CGAL_precondition(! equal_2(src, tgt));

      //check if the orientation conforms to the src and tgt.
      if( (xcv.is_directed_right() && compare_x_2(src, tgt) == LARGER) ||
          (! xcv.is_directed_right() && compare_x_2(src, tgt) == SMALLER) )
        return (xcv.trim(tgt, src) );
      else return (xcv.trim(src, tgt));
    }
  };

  /*! Obtain a Trim_2 functor object. */
  Trim_2 trim_2_object() const { return Trim_2(*this); }

  // @}

};

} //namespace CGAL

#include <CGAL/enable_warnings.h>

#endif
