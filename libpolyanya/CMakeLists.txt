add_library(polyanya STATIC
    src/cfg.cpp
    src/cfg.h
    src/consts.h
    src/cpool.h
    src/expansion.cpp
    src/expansion.h
    src/geometry.cpp
    src/geometry.h
    src/mesh.cpp
    src/mesh.h
    src/point.h
    src/polygon.h
    src/scenario.cpp
    src/scenario.h
    src/searchinstance.cpp
    src/searchinstance.h
    src/searchnode.h
    src/successor.h
    src/timer.cpp
    src/timer.h
    src/vertex.h
)

target_include_directories(polyanya INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set_property(TARGET polyanya PROPERTY INTERPROCEDURAL_OPTIMIZATION ${USE_IPO})
set_property(TARGET polyanya PROPERTY INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)
