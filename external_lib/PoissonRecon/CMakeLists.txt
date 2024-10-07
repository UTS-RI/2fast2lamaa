cmake_minimum_required(VERSION 3.20)
project(PoissonRecon)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mavx -mbmi2")


# Make release build the default
set(CMAKE_BUILD_TYPE Release)

include_directories(${CMAKE_CURRENT_LIST_DIR})

# Add the source files
file(GLOB_RECURSE SRCS_ZLIB "ZLIB/*.c")
file(GLOB_RECURSE SRCS_PNG "PNG/*.c")
file(GLOB_RECURSE SRCS_JPEG "JPEG/*.cpp")

# Add the libraries
add_library(ZLIB STATIC ${SRCS_ZLIB})
add_library(PNG STATIC ${SRCS_PNG})
add_library(JPEG STATIC ${SRCS_JPEG})


add_library(PoissonRecon STATIC Src/PoissonReconWrapped.cpp)
target_link_libraries(PoissonRecon PRIVATE PNG ZLIB JPEG )


#add_executable(PoissonReconApp main.cpp)
#target_link_libraries(PoissonReconApp PRIVATE PoissonRecon)


# Add openmp
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")



