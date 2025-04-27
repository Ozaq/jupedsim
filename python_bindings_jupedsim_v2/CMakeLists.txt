add_library(py_jupedsim_v2 MODULE
    bindings_jupedsim.cpp
    build_info.cpp
    conversion.cpp
    conversion.hpp
    generalized_centrifugal_force_model.cpp
    collision_free_speed_model.cpp
    collision_free_speed_model_v2.cpp
    anticipation_velocity_model.cpp
    social_force_model.cpp
    logging.cpp
    logging.hpp
    trace.cpp
    wrapper.hpp
    geometry.cpp
    routing.cpp
    simulation.cpp
    agent.cpp
    stage.cpp
    journey.cpp
    transition.cpp
)

target_link_libraries(py_jupedsim_v2
    PRIVATE
        pybind11::module
        pybind11::windows_extras
        fmt::fmt
        jupedsim_obj
        simulator
        common
)

pybind11_extension(py_jupedsim_v2)

if(NOT MSVC AND NOT ${CMAKE_BUILD_TYPE} MATCHES Debug|RelWithDebInfo)
    # Strip unnecessary sections of the binary on Linux/macOS
    pybind11_strip(py_jupedsim_v2)
endif()

set_target_properties(py_jupedsim_v2
    PROPERTIES
        CXX_VISIBILITY_PRESET "hidden"
        CUDA_VISIBILITY_PRESET "hidden"
        INTERPROCEDURAL_OPTIMIZATION ${USE_IPO}
        INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF
)
