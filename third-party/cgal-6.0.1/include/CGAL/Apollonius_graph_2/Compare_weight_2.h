// Copyright (c) 2003,2004,2006  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Apollonius_graph_2/include/CGAL/Apollonius_graph_2/Compare_weight_2.h $
// $Id: include/CGAL/Apollonius_graph_2/Compare_weight_2.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Menelaos Karavelas <mkaravel@iacm.forth.gr>



#ifndef CGAL_APOLLONIUS_GRAPH_2_COMPARE_WEIGHT_2_H
#define CGAL_APOLLONIUS_GRAPH_2_COMPARE_WEIGHT_2_H

#include <CGAL/license/Apollonius_graph_2.h>


#include <CGAL/Apollonius_graph_2/basic.h>

//--------------------------------------------------------------------

namespace CGAL {

namespace ApolloniusGraph_2 {

template<class K>
class Compare_weight_2
{
public:
  typedef K                               Kernel;
  typedef typename K::Site_2              Site_2;

  typedef typename K::Comparison_result   result_type;
  typedef Site_2                          argument_type;

  inline
  result_type operator()(const Site_2& s1, const Site_2& s2) const
  {
    return CGAL::compare(s1.weight(), s2.weight());
  }
};

//--------------------------------------------------------------------

} //namespace ApolloniusGraph_2

} //namespace CGAL

#endif // CGAL_APOLLONIUS_GRAPH_2_COMPARE_WEIGHT_2_H
