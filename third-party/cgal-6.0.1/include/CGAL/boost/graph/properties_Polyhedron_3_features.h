// Copyright (c) 2017  GeometryFactory (France).  All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Polyhedron/include/CGAL/boost/graph/properties_Polyhedron_3_features.h $
// $Id: include/CGAL/boost/graph/properties_Polyhedron_3_features.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Andreas Fabri

#ifndef CGAL_PROPERTIES_POLYHEDRON_3_FEATURES_H
#define CGAL_PROPERTIES_POLYHEDRON_3_FEATURES_H

#include <CGAL/license/Polyhedron.h>


#include <CGAL/Polyhedron_3.h>
#include <set>

#define CGAL_HDS_PARAM_ template < class Traits, class Items, class Alloc> class HDS

namespace CGAL {


namespace internal{
BOOST_MPL_HAS_XXX_TRAIT_DEF(Plane_3)


template <class Gt, class I, CGAL_HDS_PARAM_, class A>
struct Get_static_property_map {
  typedef boost::graph_traits<CGAL::Polyhedron_3<Gt,I,HDS,A> > Graph_traits;
  typedef CGAL::Constant_property_map<typename Graph_traits::face_descriptor,
                                      std::pair<int,int> > type;
};

} // end namespace internal



template <typename Patch_id>
struct Polyhedron_face_patch_id_pmap {
  typedef void                               key_type;
  typedef Patch_id                           value_type;
  typedef Patch_id                           reference;
  typedef boost::read_write_property_map_tag category;
};

template <typename Patch_id, typename Handle_type>
Patch_id get(Polyhedron_face_patch_id_pmap<Patch_id>, Handle_type h)
{
  return h->patch_id();
}

template <typename Patch_id, typename Handle_type>
void put(Polyhedron_face_patch_id_pmap<Patch_id>, Handle_type h, Patch_id pid)
{
  h->set_patch_id(pid);
}

template <class Gt, class I, CGAL_HDS_PARAM_, class A, typename Patch_id>
struct HDS_property_map<CGAL::Polyhedron_3<Gt, I, HDS, A>, CGAL::face_patch_id_t<Patch_id> >
{
  struct bind_
  {
    typedef Polyhedron_face_patch_id_pmap<Patch_id> type;
    typedef type const_type;
  };
};



template <class Gt, class I, CGAL_HDS_PARAM_, class A>
typename boost::lazy_enable_if<
  internal::has_Plane_3<Gt>,
  internal::Get_static_property_map<Gt, I, HDS, A>
  >::type
get(CGAL::face_patch_id_t<void>, const Polyhedron_3<Gt,I,HDS,A>&)
{
  typedef typename internal::Get_static_property_map<Gt, I, HDS, A>::type Pmap;
  return Pmap( std::pair<int,int>(0,1) );
}

template<class Gt, class I, CGAL_HDS_PARAM_, class A>
struct HDS_property_map<CGAL::Polyhedron_3<Gt, I, HDS, A>, CGAL::face_patch_id_t<void> >
{
  struct bind_
  {
    typedef typename internal::Get_static_property_map<Gt,I,HDS,A>::type type;
    typedef type const_type;
  };
};

// Compatibility: when the `Patch_id` template argument of
// `Polyhedron_mesh_domain` is `Tag_true` (because that argument was named
// `UsePatchId` in previous versions of CGAL.
template<class Gt, class I, CGAL_HDS_PARAM_, class A>
struct HDS_property_map<CGAL::Polyhedron_3<Gt, I, HDS, A>, CGAL::face_patch_id_t<CGAL::Tag_true> >
{
  struct bind_
  {
    typedef Polyhedron_3<Gt,I,HDS,A> Polyhedron;
    typedef Polyhedron_face_patch_id_pmap<typename Polyhedron::Face::Patch_id> type;
    typedef type const_type;
  };
};

// Compatibility: when the `Patch_id` template argument of
// `Polyhedron_mesh_domain` is `Tag_false` (because that argument was named
// `UsePatchId` in previous versions of CGAL.
template<class Gt, class I, CGAL_HDS_PARAM_, class A>
struct HDS_property_map<CGAL::Polyhedron_3<Gt, I, HDS, A>, CGAL::face_patch_id_t<CGAL::Tag_false> >
  : public HDS_property_map<CGAL::Polyhedron_3<Gt, I, HDS, A>, CGAL::face_patch_id_t<void> >
{
};

template <class Gt, class I, CGAL_HDS_PARAM_, class A>
typename boost::lazy_enable_if<
  internal::has_Plane_3<Gt>,
  internal::Get_static_property_map<Gt, I, HDS, A>
  >::type
get(CGAL::face_patch_id_t<Tag_false>, const Polyhedron_3<Gt,I,HDS,A>& p)
{
  return get(CGAL::face_patch_id_t<void>(), p);
}

struct Polyhedron_num_feature_edges_pmap {
  typedef void                               key_type;
  typedef int                                value_type;
  typedef int                                reference;
  typedef boost::read_write_property_map_tag category;
};

template <typename Handle_type>
int get(Polyhedron_num_feature_edges_pmap, Handle_type h)
{
  return h->nb_of_feature_edges;
}

template <typename Handle_type>
void put(Polyhedron_num_feature_edges_pmap, Handle_type h, int n)
{
  h->nb_of_feature_edges = n;
}


template<class Gt, class I, CGAL_HDS_PARAM_, class A>
struct HDS_property_map<CGAL::Polyhedron_3<Gt, I, HDS, A>, CGAL::vertex_feature_degree_t>
{
  struct bind_
  {
    typedef Polyhedron_num_feature_edges_pmap type;
    typedef type const_type;
  };
};

struct Polyhedron_is_feature_edge_pmap {
  typedef void                               key_type;
  typedef bool                               value_type;
  typedef bool                               reference;
  typedef boost::read_write_property_map_tag category;

};

template <typename Handle_type>
bool get(Polyhedron_is_feature_edge_pmap, Handle_type e)
{
  return e.halfedge()->is_feature_edge()
          || e.halfedge()->opposite()->is_feature_edge();
}

template <typename Handle_type>
void put(Polyhedron_is_feature_edge_pmap, Handle_type e, bool b)
{
  e.halfedge()->set_feature_edge(b);
  e.halfedge()->opposite()->set_feature_edge(b);
}

template<class Gt, class I, CGAL_HDS_PARAM_, class A>
struct HDS_property_map<CGAL::Polyhedron_3<Gt, I, HDS, A>, CGAL::edge_is_feature_t>
{
  struct bind_
  {
    typedef Polyhedron_is_feature_edge_pmap type;
    typedef type const_type;
  };
};



template <typename Patch_id>
struct Polyhedron_incident_patches_pmap {
  typedef void                               key_type;
  typedef std::set<Patch_id>                 value_type;
  typedef std::set<Patch_id>&                reference;
  typedef boost::lvalue_property_map_tag     category;

  template <typename Handle_type>
  reference operator[](Handle_type h) const
  {
    return get(*this, h);
  }
};

template <typename Patch_id, typename Handle_type>
std::set<Patch_id>& get(Polyhedron_incident_patches_pmap<Patch_id>,
                        Handle_type h)
{
  return h->incident_patches_ids_set();
}

template <typename Patch_id, typename Handle_type>
void put(Polyhedron_incident_patches_pmap<Patch_id>,
         Handle_type h, const std::set<Patch_id>& v)
{
  h->clear_incident_patches();
  for(Patch_id n : v)
    h->add_incident_patch(n);
}

template<class Gt, class I, CGAL_HDS_PARAM_, class A, class Patch_id>
struct HDS_property_map<CGAL::Polyhedron_3<Gt, I, HDS, A>, CGAL::vertex_incident_patches_t<Patch_id> >
{
  struct bind_
  {
    typedef Polyhedron_incident_patches_pmap<Patch_id> type;
    typedef type const_type;
  };
};


} // end namespace CGAL

#undef CGAL_HDS_PARAM_

#endif // CGAL_PROPERTIES_POLYHEDRON_3_FEATURES_H
