add_executable(plane-splitter
    plane_splitter.cpp
)

target_link_libraries(plane-splitter
    fmt::fmt
    CGAL::CGAL
)
