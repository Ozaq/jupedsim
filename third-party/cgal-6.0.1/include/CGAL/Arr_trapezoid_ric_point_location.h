// Copyright (c) 2005,2006,2007,2009,2010,2011 Tel-Aviv University (Israel).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Arrangement_on_surface_2/include/CGAL/Arr_trapezoid_ric_point_location.h $
// $Id: include/CGAL/Arr_trapezoid_ric_point_location.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s): Idit Haran   <haranidi@post.tau.ac.il>
//            (based on old version by Oren Nechushtan and Iddo Hanniel)

#ifndef CGAL_ARR_TRAPEZOID_RIC_POINT_LOCATION_H
#define CGAL_ARR_TRAPEZOID_RIC_POINT_LOCATION_H

#include <CGAL/license/Arrangement_on_surface_2.h>

#include <CGAL/disable_warnings.h>

/*! \file
 * Definition of the Arr_trapezoid_ric_point_location<Arrangement> template.
 */

#include <CGAL/Arr_point_location_result.h>
#include <CGAL/Arrangement_2/Arr_traits_adaptor_2.h>
#include <CGAL/Arr_point_location/Trapezoidal_decomposition_2.h>
#include <CGAL/Arr_point_location/Td_traits.h>

namespace CGAL {

/*! \class
 * A class that answers point-location and queries
 * on a planar arrangement using the trapezoid_ric algorithm.
 * The Arrangement parameter corresponds to an arrangement instantiation.
 */
template <typename Arrangement_>
class Arr_trapezoid_ric_point_location : public Arrangement_::Observer {
public:
  //type of arrangement on surface
  using Arrangement_on_surface_2 = Arrangement_;
  using Base_aos = typename Arrangement_on_surface_2::Base_aos;

  //type of geometry traits
  using Geometry_traits_2 = typename Base_aos::Geometry_traits_2;

  //type of traits adaptor
  using Traits_adaptor_2 = typename Base_aos::Traits_adaptor_2;

  //type of vertex handle
  using Vertex_handle = typename Base_aos::Vertex_handle;

  //type of vertex const handle
  using Vertex_const_handle = typename Base_aos::Vertex_const_handle;

  //type of halfedge handle
  using Halfedge_handle = typename Base_aos::Halfedge_handle;

  //type of halfedge const handle
  using Halfedge_const_handle = typename Base_aos::Halfedge_const_handle;

  //type of face const handle
  using Face_const_handle = typename Base_aos::Face_const_handle;

  //type of edge const iterator
  using Edge_const_iterator = typename Base_aos::Edge_const_iterator;

  //type of isolated vertex const iterator
  using Isolated_vertex_const_iterator =
    typename Base_aos::Isolated_vertex_const_iterator;

  //type of point
  using Point_2 = typename Geometry_traits_2::Point_2;

  //type of x-monotone curve
  using X_monotone_curve_2 = typename Geometry_traits_2::X_monotone_curve_2;


  //type of trapezoidal decomposition traits class
  using Td_traits = CGAL::Td_traits<Traits_adaptor_2, Base_aos>;

  //type of trapezoidal decomposition class
  using Trapezoidal_decomposition = Trapezoidal_decomposition_2<Td_traits>;

  //!types of Td_map_item-s
  using Td_map_item = typename Trapezoidal_decomposition::Td_map_item;
  using Td_active_vertex = typename Trapezoidal_decomposition::Td_active_vertex;
  using Td_active_fictitious_vertex =
    typename Trapezoidal_decomposition::Td_active_fictitious_vertex;
  using Td_active_edge = typename Trapezoidal_decomposition::Td_active_edge;
  using Td_active_trapezoid =
    typename Trapezoidal_decomposition::Td_active_trapezoid;

  //!type of side tags
  using Left_side_category = typename Traits_adaptor_2::Left_side_category;
  using Bottom_side_category = typename Traits_adaptor_2::Bottom_side_category;
  using Top_side_category = typename Traits_adaptor_2::Top_side_category;
  using Right_side_category = typename Traits_adaptor_2::Right_side_category;

protected:
  using Result = Arr_point_location_result<Base_aos>;
  using Result_type = typename Result::Type;

public:
  // Support cpp11::result_of
  using result_type = Result_type;

protected:
  //type of trapezoidal decomposition class
  using TD = Trapezoidal_decomposition;

  using All_sides_oblivious_category=
    typename Arr_all_sides_oblivious_category<Left_side_category,
                                              Bottom_side_category,
                                              Top_side_category,
                                              Right_side_category>::result;

  // Data members:
  const Traits_adaptor_2* m_traits; // Its associated traits object.
  TD td;                            // instance of trapezoidal decomposition
  bool m_with_guarantees;
  //for the notification functions
  X_monotone_curve_2 m_cv_before_split;
  Halfedge_handle m_he_after_merge;
  //X_monotone_curve_2 m_cv_before_merge1;
  //X_monotone_curve_2 m_cv_before_merge2;

  template <typename T>
  Result_type make_result(T t) const { return Result::make_result(t); }
  inline Result_type default_result() const { return Result::default_result(); }

public:
  /*! Default constructor. */
  Arr_trapezoid_ric_point_location
  (bool with_guarantees = true,
   double depth_thrs = CGAL_TD_DEFAULT_DEPTH_THRESHOLD,
   double size_thrs = CGAL_TD_DEFAULT_SIZE_THRESHOLD) :
    m_traits(nullptr), m_with_guarantees(with_guarantees) {
    td.set_with_guarantees(with_guarantees);
    td.depth_threshold(depth_thrs);
    td.size_threshold(size_thrs);
  }

  /*! Constructor given an arrangement. */
  Arr_trapezoid_ric_point_location
  (const Base_aos& arr,
   bool with_guarantees = true,
   double depth_thrs = CGAL_TD_DEFAULT_DEPTH_THRESHOLD,
   double size_thrs = CGAL_TD_DEFAULT_SIZE_THRESHOLD) :
    Base_aos::Observer(const_cast<Base_aos&>(arr)),
    m_with_guarantees(with_guarantees) {
    m_traits = static_cast<const Traits_adaptor_2*>(arr.geometry_traits());
    td.set_with_guarantees(with_guarantees);
    td.init_arrangement_and_traits(&arr);
    td.depth_threshold(depth_thrs);
    td.size_threshold(size_thrs);
    _construct_td();
  }

  /*! Destructor. */
  ~Arr_trapezoid_ric_point_location() { }

  /*! defines whether the underlying search structure guarantees logarithmic
   *   query time and linear size */
  void with_guarantees(bool with_guarantees) {
    //if with_guarantees was changed from false to true - reconstruct
    //  the search structure with guarantees
    td.set_with_guarantees(with_guarantees);
    if (with_guarantees && !m_with_guarantees) {
      td.clear();
      _construct_td();
    }
    m_with_guarantees = with_guarantees;
  }

  /*! returns the depth of the underlying search structure
   *    (the longest path in the DAG)
   */
  unsigned long depth() //longest_dag_path()
  { return td.largest_leaf_depth() + 1; }

  /*! returns the longest query path in the underlying search structure */
  unsigned long longest_query_path_length()
  { return td.longest_query_path_length(); }

#ifdef CGAL_TD_DEBUG
  //void  locate_and_print (std::ostream& out, const Point_2& p) const
  //{  td.locate_and_print(out, p); }

  void print_dag(std::ostream& out) const { td.print_dag(out); }
#endif

  /*! Locate the arrangement feature containing the given point.
   * \param p The query point.
   * \return An object representing the arrangement feature containing the
   *         query point. This object is either a Face_const_handle or a
   *         Halfedge_const_handle or a Vertex_const_handle.
   */
  result_type locate(const Point_2& p) const;

  /*! Locate the arrangement feature which a upward vertical ray emanating from
   * the given point hits.
   * \param p The query point.
   * \return An object representing the arrangement feature the ray hits.
   *         This object is either an empty object or a
   *         Halfedge_const_handle or a Vertex_const_handle.
   */
  result_type ray_shoot_up(const Point_2& p) const
  { return (_vertical_ray_shoot(p, true)); }

  /*! Locate the arrangement feature which a downward vertical ray emanating
   * from the given point hits.
   * \param p The query point.
   * \return An object representing the arrangement feature the ray hits.
   *         This object is either an empty object or a
   *         Halfedge_const_handle or a Vertex_const_handle.
   */
  result_type ray_shoot_down(const Point_2& p) const
  { return (_vertical_ray_shoot(p, false)); }

  /// \name Notification functions, inherited and overloaded from the
  //        base observer.
  //@{

  /*! Notification before the arrangement is assigned with the content of
   * another arrangement.
   * \param arr The other arrangement. Notice that the arrangement type is the
   *            type used to instantiate the observer, which is conveniently
   *            defined as `Arrangement_2::Base_aos`.
   */
  virtual void before_assign(const Base_aos& arr) override {
    td.clear();
    m_traits = static_cast<const Traits_adaptor_2*> (arr.geometry_traits());
    td.init_arrangement_and_traits(&arr, false);
  }

  virtual void after_assign() override { _construct_td(); }

  virtual void before_clear() override { td.clear(); }

  virtual void after_clear() override { _construct_td(); }

  /*! Notification before the observer is attached to an arrangement.
   * \param arr The arrangement that is about to attach the observer. Notice
   *        that the arrangement type is the type used to instantiate the
   *        observer, which is conveniently defined as
   *        `Arrangement_2::Base_aos`.
   */
  virtual void before_attach(const Base_aos& arr) override {
    td.clear();
    m_traits = static_cast<const Traits_adaptor_2*> (arr.geometry_traits());
    td.init_arrangement_and_traits(&arr);
  }

  virtual void after_attach() override { _construct_td(); }

  virtual void before_detach() override { td.clear(); }

  virtual void after_create_edge(Halfedge_handle e) override { td.insert(e); }

  //TODO IDIT OREN: what can be done in order to avoid the need
  //to save the original curve is to find the common endpoint of the
  //two new halfedges, locate it in the trapezoid in order to find the
  //curve it lies on, which is the curve that was split, and then remove
  //this curve.
  virtual void before_split_edge(Halfedge_handle e,
                                 Vertex_handle /* v */,
                                 const X_monotone_curve_2& /* cv1 */,
                                 const X_monotone_curve_2& /* cv2 */) override {

    ////MICHAL: commented due to inefficient depth update, remove and insert
    ////instead save the curve for the "after" function.
    //m_cv_before_split = e->curve();
    //td.before_split_edge(m_cv_before_split, cv1, cv2);

    td.remove(e);
  }

  virtual void after_split_edge(Halfedge_handle e1, Halfedge_handle e2)
    override {
    //MICHAL: commented due to inefficient depth update, remove and insert instead
    //td.split_edge(m_cv_before_split,e1,e2);

    td.insert(e1);
    td.insert(e2);
  }

  virtual void before_merge_edge(Halfedge_handle e1, Halfedge_handle e2,
                                 const X_monotone_curve_2& cv) override {
    //save the halfedge handle for the "after" function.
    m_he_after_merge = e1;
    td.merge_edge (e1, e2, cv);
  }

  virtual void after_merge_edge(Halfedge_handle e) override
  { td.after_merge_edge(e, m_he_after_merge); }

  virtual void before_remove_edge(Halfedge_handle e) override { td.remove(e); }
  //@}

public:
//#ifdef CGAL_TD_DEBUG
//  void debug()
//  {
//    td.debug();
//  }
//#endif

protected:
  /*! Construct the trapezoidal decomposition. */
  void _construct_td() {
    td.clear();

    std::vector<Halfedge_const_handle> he_container;
    auto* arr = this->arrangement();
    //collect the arrangement halfedges
    for (auto eit = arr->edges_begin(); eit != arr->edges_end(); ++eit) {
      Halfedge_const_handle he_cst = eit;
      he_container.push_back(he_cst);
    }
    //container insertion
    td.insert(he_container.begin(), he_container.end());
  }

  /*! Obtain the unbounded face that contains the point when the trapezoid is
   * unbounded
   * \param tr The unbounded trapezoid whose face we should get
   * \param p  The query point.
   * \param Arr_all_sides_oblivious_tag
   * \return A Face_const_handle representing the arrangement unbounded face in
   *         which the point p lies
   */
  Face_const_handle _get_unbounded_face(const Td_map_item& tr,
                                        const Point_2& p,
                                        Arr_all_sides_oblivious_tag) const;

  /*! Obtain the unbounded face that contains the point when the trapezoid is
   * unbounded
   * \param tr The unbounded trapezoid whose face we should get
   * \param p  The query point.
   * \param Arr_not_all_sides_oblivious_tag
   * \return A Face_const_handle representing the arrangement unbounded face in
   *         which the point p lies
   */
  Face_const_handle _get_unbounded_face(const Td_map_item& tr,
                                        const Point_2& p,
                                        Arr_not_all_sides_oblivious_tag) const;

  /*! Locate the arrangement feature which a vertical ray emanating from the
   * given point hits, considering isolated vertices.
   * \param p The query point.
   * \param shoot_up Indicates whether the ray is directed upward or downward.
   * \return An object representing the arrangement feature the ray hits.
   *         This object is either a Halfedge_const_handle,
   *         a Vertex_const_handle or an empty object.
   */
  result_type _vertical_ray_shoot(const Point_2& p, bool shoot_up) const;

  /*! In vertical ray shoot, when the closest halfedge is found
   * (or unbounded face)
   * we check the isolated vertices inside the face to check whether there
   * is an isolated vertex right above/below the query point.
   */
  result_type _check_isolated_for_vertical_ray_shoot
  (Halfedge_const_handle halfedge_found,
   const Point_2& p, bool shoot_up,
   const Td_map_item& tr) const;
};

} //namespace CGAL

// The member-function definitions can be found under:
#include <CGAL/Arr_point_location/Arr_trapezoid_ric_pl_impl.h>

#include <CGAL/enable_warnings.h>

#endif
