# locate the additional necessary dependencies, if any

# extra source files
SET(sources
    SO3.cpp
    SE3.cpp
)

# extra header files
SET(headers
	mrob/SO3.hpp
	mrob/SE3.hpp
)

# create the shared library
ADD_LIBRARY(SE3 SHARED  ${sources})
