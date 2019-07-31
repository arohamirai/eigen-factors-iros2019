# locate the necessary dependencies, if any

# extra source files
SET(sources
    arun.cpp
    gicp.cpp
    plane.cpp
    plane_registration.cpp
    create_points.cpp
)

# extra header files
SET(headers
    mrob/pc_registration.hpp
    mrob/plane.hpp
    mrob/plane_registration.hpp
    mrob/create_points.hpp
)

# create the shared library
ADD_LIBRARY(PCRegistration SHARED  ${sources} )
TARGET_LINK_LIBRARIES(PCRegistration SE3 )

