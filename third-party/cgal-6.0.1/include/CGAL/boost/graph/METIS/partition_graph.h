// Copyright (c) 2017 GeometryFactory (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/BGL/include/CGAL/boost/graph/METIS/partition_graph.h $
// $Id: include/CGAL/boost/graph/METIS/partition_graph.h 50cfbde3b84 $
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Mael Rouxel-Labbé

#ifndef CGAL_BGL_PARTITION_GRAPH_H
#define CGAL_BGL_PARTITION_GRAPH_H

#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/Named_function_parameters.h>
#include <CGAL/boost/graph/named_params_helper.h>

#include <CGAL/assertions.h>

#include <metis.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/tuple/tuple.hpp>

#include <cstdlib>

namespace CGAL {

namespace METIS {

#ifndef DOXYGEN_RUNNING

struct Output_vertex_partition_ids
{
  template<typename TriangleMesh, typename Indices>
  void operator()(const TriangleMesh&, const Indices,
                  idx_t const * const, internal_np::Param_not_found) { }

  template<typename TriangleMesh,
           typename Indices,
           typename VertexPartitionIDPmap>
  void operator()(const TriangleMesh& tm, const Indices indices,
                  idx_t const * const npart,
                  VertexPartitionIDPmap vertex_partition_id_map)
  {
    typename boost::graph_traits<TriangleMesh>::vertex_iterator vit, ve;
    boost::tie(vit, ve) = vertices(tm);
    for(; vit!=ve; ++vit)
      put(vertex_partition_id_map, *vit, npart[get(indices, *vit)]);
  }
};

struct Output_face_partition_ids
{
  template<typename TriangleMesh>
  void operator()(const TriangleMesh&,
                  idx_t const * const, internal_np::Param_not_found) { }

  template<typename TriangleMesh, typename FacePartitionIDPmap>
  void operator()(const TriangleMesh& tm, idx_t const * const epart,
                  FacePartitionIDPmap face_partition_id_map)
  {
    typename boost::graph_traits<TriangleMesh>::face_iterator fit, fe;
    boost::tie(fit, fe) = faces(tm);
    for(int i=0; fit!=fe; ++fit, ++i)
      put(face_partition_id_map, *fit, epart[i]);
  }
};



template<typename TriangleMesh, typename METIS_options, typename NamedParameters>
void partition_graph(const TriangleMesh& tm,
                     int nparts,
                     METIS_options options, // pointer to the options array
                     const NamedParameters& np)
{
  CGAL_precondition(CGAL::is_triangle_mesh(tm));
  CGAL_precondition_msg(nparts > 1, ("Partitioning requires a number of parts > 1"));

  using parameters::get_parameter;

  typedef typename boost::graph_traits<TriangleMesh>::vertex_descriptor   vertex_descriptor;
  typedef typename boost::graph_traits<TriangleMesh>::halfedge_descriptor halfedge_descriptor;
  typedef typename boost::graph_traits<TriangleMesh>::face_iterator       face_iterator;

  typedef typename CGAL::GetInitializedVertexIndexMap<TriangleMesh, NamedParameters>::type Indices;
  Indices indices = CGAL::get_initialized_vertex_index_map(tm, np);

  idx_t nn = static_cast<idx_t>(num_vertices(tm));
  idx_t ne = static_cast<idx_t>(num_faces(tm));
  idx_t d = 3; // number of nodes per element
  idx_t* eptr = new idx_t[ne + 1];
  idx_t* eind = new idx_t[d * ne];

  // fill the adjacency info
  face_iterator fit, fe;
  boost::tie(fit, fe) = faces(tm);
  for(int i=0, j=0; fit!=fe; ++fit, ++i)
  {
    eptr[i] = j;

    halfedge_descriptor h = halfedge(*fit, tm), done = h;
    do
    {
      vertex_descriptor v = target(h, tm);
      CGAL_assertion(j < d * ne);
      eind[j++] = static_cast<idx_t>(get(indices, v));
      h = next(h, tm);
    } while (h != done);

    CGAL_assertion(i < ne);
    eptr[i + 1] = j;
  }

  idx_t nparts_as_idx_t = nparts;

  // either the edgecut or the total communication volume of the dual graph’s partitioning
  idx_t objval;

  // partition info for the nodes
  idx_t* npart = (idx_t*) calloc(num_vertices(tm), sizeof(idx_t));
  CGAL_assertion(npart != nullptr);

  // partition info for the elements
  idx_t* epart = (idx_t*) calloc(num_faces(tm), sizeof(idx_t));
  CGAL_assertion(epart != nullptr);

  // do not support Fortran-style arrays
  CGAL_assertion((*options)[METIS_OPTION_NUMBERING] == -1 || // default initialization is '-1'
                 (*options)[METIS_OPTION_NUMBERING] == 0);

  CGAL_assertion_code(int ret =)
    METIS_PartMeshNodal(&ne, &nn, eptr, eind,
                        nullptr /* nodes weights */, nullptr /* nodes sizes */,
                        &nparts_as_idx_t,
                        nullptr /* partitions weights */,
                        *options,
                        &objval, epart, npart);

  CGAL_assertion(ret == METIS_OK);

  Output_vertex_partition_ids vo;
  Output_face_partition_ids fo;
  vo(tm, indices, npart, get_parameter(np, internal_np::vertex_partition_id));
  fo(tm, epart, get_parameter(np, internal_np::face_partition_id));

  delete[] eptr;
  delete[] eind;

  (std::free)(npart);
  (std::free)(epart);
}

template<typename TriangleMesh, typename NamedParameters>
void partition_graph(const TriangleMesh& tm, int nparts,
                     const internal_np::Param_not_found, // no METIS options were passed
                     const NamedParameters& np)
{
  idx_t options[METIS_NOPTIONS];
  METIS_SetDefaultOptions(options);
  return partition_graph(tm, nparts, &options, np);
}

#endif


/// \ingroup PkgBGLPartition
///
/// computes a partition of the input triangular mesh into `nparts` parts, based on the
/// mesh's nodal graph. The resulting partition is stored in the vertex and/or face
/// property maps that are passed as parameters using \ref bgl_namedparameters "Named Parameters".
///
/// \param tm a triangle mesh
/// \param nparts the number of parts in the final partition
/// \param np an optional sequence of \ref bgl_namedparameters "Named Parameters" among the ones listed below
///
/// \tparam TriangleMesh is a model of the `FaceListGraph` concept.
/// \tparam NamedParameters a sequence of \ref bgl_namedparameters "Named Parameters"
///
/// \cgalNamedParamsBegin
///   \cgalParamNBegin{vertex_index_map}
///     \cgalParamDescription{a property map associating to each vertex of `tm` a unique index between `0` and `num_vertices(tm) - 1`}
///     \cgalParamType{a class model of `ReadablePropertyMap` with `boost::graph_traits<TriangleMesh>::%vertex_descriptor`
///                    as key type and `std::size_t` as value type}
///     \cgalParamDefault{an automatically indexed internal map}
///     \cgalParamExtra{If this parameter is not passed, internal machinery will create and initialize
///                     a face index property map, either using the internal property map if it exists
///                     or using an external map. The latter might result in  - slightly - worsened performance
///                     in case of non-constant complexity for index access.}
///   \cgalParamNEnd
///
///   \cgalParamNBegin{METIS_options}
///     \cgalParamDescription{a parameter used in to pass options to the METIS mesh partitioner}
///     \cgalParamType{an array of size `METIS_NOPTIONS` with value type `idx_t` (an integer type defined by METIS)}
///     \cgalParamDefault{an array of size `METIS_NOPTIONS` with value type `idx_t`,
///                       initialized using the function `METIS_SetDefaultOptions()`}
///     \cgalParamExtra{The many options of METIS are not described here. Instead, users should refer
///                     to the <a href="http://glaros.dtc.umn.edu/gkhome/fetch/sw/metis/manual.pdf">documentation</a>
///                     of METIS directly.}
///   \cgalParamNEnd
///
///   \cgalParamNBegin{vertex_partition_id_map}
///     \cgalParamDescription{a property map that contains (after the function has been run)
///                           the ID of the subpart for each vertex of `tm`}
///     \cgalParamType{a class model of `ReadWritePropertyMap` with
///                    `boost::graph_traits<TriangleMesh>::%vertex_descriptor` as key type and
///                    `int` as value type}
///     \cgalParamDefault{unused}
///   \cgalParamNEnd
///
///   \cgalParamNBegin{face_partition_id_map}
///     \cgalParamDescription{a property map that contains (after the function has been run)
///                           the ID of the subpart for each face of `tm`}
///     \cgalParamType{a class model of `ReadWritePropertyMap` with
///                    `boost::graph_traits<TriangleMesh>::%face_descriptor` as key type and
///                    `int` as value type}
///     \cgalParamDefault{unused}
///   \cgalParamNEnd
/// \cgalNamedParamsEnd
///
/// \pre `tm` is a pure triangular surface mesh: there are no edges
///       without at least one incident face
template<typename TriangleMesh, typename NamedParameters = parameters::Default_named_parameters>
void partition_graph(const TriangleMesh& tm, int nparts, const NamedParameters& np = parameters::default_values())
{
  using parameters::get_parameter;

  return partition_graph(tm, nparts, get_parameter(np, internal_np::METIS_options), np);
}

} // end namespace METIS

} // end namespace CGAL

#endif // CGAL_BGL_PARTITION_GRAPH_H
