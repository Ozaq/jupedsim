// SPDX-License-Identifier: LGPL-3.0-or-later
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Surface_mesh.h>
#include <fmt/base.h>

#include <filesystem>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = K::Point_3;
using Vector = K::Vector_3;
using Mesh = CGAL::Surface_mesh<Point>;
namespace PMP = CGAL::Polygon_mesh_processing;

template <class MeshT>
bool is_face_planar(typename boost::graph_traits<MeshT>::face_descriptor face, const MeshT& mesh)
{
    std::vector<typename boost::graph_traits<MeshT>::vertex_descriptor> vs{};
    vs.reserve(3);
    return is_face_planar(face, mesh, {});
}

template <class MeshT>
bool is_face_planar(
    typename boost::graph_traits<MeshT>::face_descriptor face,
    const MeshT& mesh,
    std::vector<typename boost::graph_traits<MeshT>::vertex_descriptor>& vs)
{
    for(auto half_edge : CGAL::halfedges_around_face(CGAL::halfedge(face, mesh), mesh)) {
        vs.emplace_back(CGAL::source(half_edge, mesh));
    }
    if(vs.size() == 3) {
        return true;
    }
    if(vs.size() < 3) {
        // ERROR
    }

    // TODO(kkratz): Add check for collinearity of all points -> degenerate face
    // TODO(kkratz): Add check for identical vertex locations of all points -> degenerate face

    const auto& p0 = mesh.point(vs[0]);
    const auto& p1 = mesh.point(vs[1]);
    const auto& p2 = mesh.point(vs[2]);
    return std::all_of(std::begin(vs), std::end(vs), [&p0, &p1, &p2, &mesh](auto v) {
        return CGAL::coplanar(p0, p1, p2, mesh.point(v));
    });
}

bool all_faces_planar_exact(const Mesh& mesh)
{
    using vert_desc = typename boost::graph_traits<Mesh>::vertex_descriptor;
    std::vector<vert_desc> vs{};
    vs.reserve(3);
    return std::all_of(
        std::begin(CGAL::faces(mesh)), std::end(CGAL::faces(mesh)), [&vs, &mesh](auto face) {
            vs.clear();
            return is_face_planar(face, mesh, vs);
        });
}

void print_usage()
{
    fmt::println("Usage:\n\tplane-splitter <OBJ file>");
}

struct Opts {
    std::filesystem::path geo_path{};
};

Opts parse_opts(int argc, char** argv)
{
    if(argc != 2) {
        fmt::println("Missing positional argument.");
        print_usage();
        std::exit(1);
    }
    Opts opts{};
    opts.geo_path = std::filesystem::path(argv[1]);
    return opts;
}

Mesh load_obj(const std::filesystem::path& file)
{
    std::vector<K::Point_3> points;
    std::vector<std::vector<std::size_t>> faces;

    std::ifstream in(file);
    if(!in || !CGAL::IO::read_OBJ(in, points, faces)) {
        fmt::println("Error parsing OBJ file");
        std::exit(1);
    }

    if(!PMP::orient_polygon_soup(points, faces)) {
        fmt::println("Error processig input mesh, cannot orient all facets uniformly.");
        std::exit(1);
    }

    Mesh mesh{};
    PMP::polygon_soup_to_polygon_mesh(points, faces, mesh);
    return mesh;
}

int main(int argc, char** argv)
{
    const auto opts = parse_opts(argc, argv);
    const auto mesh = load_obj(opts.geo_path);
    fmt::println("Validating Mesh...");
    const auto result = all_faces_planar_exact(mesh);
    fmt::println("Result: {}", result ? "OK" : "Invalid");
    return 0;
}
