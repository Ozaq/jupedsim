// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>

#include <cstddef>
#include <functional>
#include <list>
#include <sstream>
#include <string>

// using K = CGAL::Exact_predicates_exact_constructions_kernel;
using K = CGAL::Simple_cartesian<double>;

using Poly = CGAL::Polygon_2<K>;
using PolyWithHoles = CGAL::Polygon_with_holes_2<K>;
using PolyWithHolesList = std::list<PolyWithHoles>;
using PolyList = std::list<Poly>;
using Vb = CGAL::Triangulation_vertex_base_2<K>;

template <class Gt, class Fb = CGAL::Constrained_triangulation_face_base_2<Gt>>
class MyFace : public Fb
{
    bool in{false};
    using Base = Fb;
    using TDS = typename Fb::Triangulation_data_structure;

public:
    using Fb::Fb;
    template <typename TDS2>
    struct Rebind_TDS {
        using Fb2 = typename Fb::template Rebind_TDS<TDS2>::Other;
        using Other = MyFace<Gt, Fb2>;
    };
    // NOLINTNEXTLINE(readability-identifier-naming)
    void set_in_domain(bool v) { in = v; }
    // NOLINTNEXTLINE(readability-identifier-naming)
    bool get_in_domain() const { return in; }
};
using TDS = CGAL::Triangulation_data_structure_2<Vb, MyFace<K>>;
using Itag = CGAL::Exact_predicates_tag;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>;

namespace std
{
template <>
class hash<PolyWithHoles>
{
public:
    size_t operator()(const PolyWithHoles& p) const
    {
        std::stringstream ss{};
        ss << p;
        return std::hash<std::string>()(ss.str());
    }
};
} // namespace std
