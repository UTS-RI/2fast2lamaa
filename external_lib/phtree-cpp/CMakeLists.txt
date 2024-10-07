cmake_minimum_required(VERSION 3.14)

project(phtree VERSION 1.6.1
        DESCRIPTION "PH-Tree C++"
        HOMEPAGE_URL "https://github.com/tzaeschke/phtree-cpp"
        LANGUAGES CXX)


cmake_policy(SET CMP0077 NEW)

# ---------------------------------------------------------------------------------------
# Set default build to release
# ---------------------------------------------------------------------------------------
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif ()


# ---------------------------------------------------------------------------------------
# Build options
# ---------------------------------------------------------------------------------------
option(PHTREE_BUILD_ALL "Build examples, tests and benchmarks" OFF)

# example options
option(PHTREE_BUILD_EXAMPLES "Build examples" OFF)

# testing options
option(PHTREE_BUILD_TESTS "Build tests" OFF)
option(PHTREE_CODE_COVERAGE "Collect coverage from test library" OFF)
if (PHTREE_CODE_COVERAGE)
    set(PHTREE_BUILD_TESTS ON)
endif ()

# bench options
option(PHTREE_BUILD_BENCHMARKS "Build benchmarks (Requires https://github.com/google/benchmark.git to be installed)" OFF)

# install options
option(PHTREE_INSTALL "Generate the install target" OFF)


# ---------------------------------------------------------------------------------------
# Compiler config
# ---------------------------------------------------------------------------------------

# specify the C++ standard
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if (MSVC)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall")
    set(CMAKE_CXX_FLAGS_RELEASE "/O2")

    if (PHTREE_BUILD_TESTS OR PHTREE_BUILD_ALL)
        add_compile_options(/bigobj)
    endif ()

    # For google benchmark
    if (PHTREE_BUILD_BENCHMARKS) # OR PHTREE_BUILD_ALL)
        # This still doesn't work. This also breaks gtest
        # See for example
        # https://stackoverflow.com/questions/55376111/how-to-build-and-link-google-benchmark-using-cmake-in-windows
        # https://github.com/google/benchmark/issues/1348
        # https://github.com/google/benchmark/issues/639
        # set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
        # set(BUILD_SHARED_LIBS TRUE) #=TRUE
        # set(BENCHMARK_DOWNLOAD_DEPENDENCIES on)
        # set(BENCHMARK_ENABLE_GTEST_TESTS OFF)
    endif ()
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")
    if (PHTREE_BUILD_BENCHMARKS)
        # Enable vectorization and TZCNT/CTZ
        set(CMAKE_CXX_FLAGS_RELEASE "-O3 -mavx -mbmi2 -pthread")
    else ()
        # Enable vectorization and TZCNT/CTZ
        set(CMAKE_CXX_FLAGS_RELEASE "-O3 -mavx -mbmi2 ")
    endif ()
    if (PHTREE_CODE_COVERAGE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage") # -Wa,-mbig-obj")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    endif ()
endif ()

# ---------------------------------------------------------------------------------------
# Build binaries
# ---------------------------------------------------------------------------------------

# ---------------------------------------------------------------------------------------
# Header only version
# ---------------------------------------------------------------------------------------
add_library(phtree INTERFACE)
add_library(phtree::phtree ALIAS phtree)
target_compile_features(phtree INTERFACE cxx_std_17)

target_include_directories(phtree INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

if (PHTREE_BUILD_EXAMPLES OR PHTREE_BUILD_ALL)
    message(STATUS "Generating examples")
    add_subdirectory(examples)
endif ()

if ((PHTREE_BUILD_BENCHMARKS OR PHTREE_BUILD_ALL) AND NOT MSVC)
    message(STATUS "Generating benchmarks")
    add_subdirectory(benchmark)
endif ()

if (PHTREE_BUILD_TESTS OR PHTREE_BUILD_ALL)
    message(STATUS "Generating tests")
    enable_testing()
    include(GoogleTest)
    add_subdirectory(test)
endif ()

# ---------------------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------------------
if (PHTREE_INSTALL)
    include(GNUInstallDirs)

    install(TARGETS phtree
            EXPORT ${PROJECT_NAME}_Targets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

    include(CMakePackageConfigHelpers)
    write_basic_package_version_file("phtreeConfigVersion.cmake"
            VERSION ${PROJECT_VERSION}
            COMPATIBILITY SameMajorVersion)

    configure_package_config_file(
            "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
            "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
            INSTALL_DESTINATION
            ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

    install(EXPORT ${PROJECT_NAME}_Targets
            FILE ${PROJECT_NAME}Targets.cmake
            NAMESPACE phtree::
            DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

    install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
            "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
            DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake)

    install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/phtree
            DESTINATION include
            PATTERN "BUILD" EXCLUDE
            PATTERN "*.md" EXCLUDE)

endif ()
