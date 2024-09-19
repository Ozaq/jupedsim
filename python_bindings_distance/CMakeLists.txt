add_library(py_distance MODULE
    bindings_distance.cpp
)

target_link_libraries(py_distance
    PRIVATE
        pybind11::module
        pybind11::lto
        pybind11::windows_extras
        fmt::fmt
        distance
)

pybind11_extension(py_distance)

if(NOT MSVC AND NOT ${CMAKE_BUILD_TYPE} MATCHES Debug|RelWithDebInfo)
    # Strip unnecessary sections of the binary on Linux/macOS
    pybind11_strip(py_distance)
endif()

set_target_properties(py_distance
    PROPERTIES
        CXX_VISIBILITY_PRESET "hidden"
        CUDA_VISIBILITY_PRESET "hidden"
)

