cmake_minimum_required(VERSION 3.14)
project(phtree-tests LANGUAGES CXX)

include(FetchContent)
include(common/scripts.cmake)

FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
)
if (MSVC)
    # Avoids LNK2038 Error with MSVC
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif ()
FetchContent_MakeAvailable(googletest)

# The next line is optional, but keeps your CACHE cleaner:
mark_as_advanced(
        BUILD_GTEST BUILD_SHARED_LIBS
        gtest_build_samples gtest_build_tests
        gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

# If you are interested in keeping IDEs that support folders clean, I would also add these lines:
set_target_properties(gtest PROPERTIES FOLDER extern)
set_target_properties(gtest_main PROPERTIES FOLDER extern)

#include(GoogleTest)
#gtest_discover_tests(all_tests_driver)

if (PHTREE_CODE_COVERAGE)
    package_add_test_main(all_tests
            all_tests.cc
            converter_test.cc
            distance_test.cc
            filter_test.cc
            phtree_test.cc
            phtree_test_const_values.cc
            phtree_test_issues.cc
            phtree_test_ptr_values.cc
            phtree_test_unique_ptr_values.cc
            phtree_f_test.cc
            phtree_d_test.cc
            phtree_d_test_copy_move.cc
            phtree_d_test_custom_key.cc
            phtree_d_test_filter.cc
            phtree_d_test_preprocessor.cc
            phtree_box_f_test.cc
            phtree_box_d_test.cc
            phtree_box_d_test_filter.cc
            phtree_box_d_test_query_types.cc
            phtree_multimap_d_test.cc
            phtree_multimap_d_test_copy_move.cc
            phtree_multimap_d_test_filter.cc
            phtree_multimap_d_test_unique_ptr_values.cc
            phtree_multimap_box_d_test.cc
            common/b_plus_tree_hash_map_test.cc
            common/b_plus_tree_map_test.cc
            common/b_plus_tree_multimap_test.cc
            # Disabled, see issue #147
            # common/b_priority_queue_test.cc
            # common/b_vector_test.cc
            common/base_types_test.cc
            common/bits_test.cc
            common/common_test.cc
            common/flat_array_map_test.cc
            common/flat_sparse_map_test.cc)
    target_compile_definitions(all_tests PUBLIC SKIP_TEST_MEMORY_LEAKS=ON)
else ()
    package_add_test(phtree_test phtree_test.cc)
    package_add_test(phtree_test_const_values phtree_test_const_values.cc)
    package_add_test(phtree_test_issues phtree_test_issues.cc)
    target_compile_definitions(phtree_test_issues PUBLIC SKIP_TEST_MEMORY_LEAKS=ON)
    package_add_test(phtree_test_ptr_values phtree_test_ptr_values.cc)
    package_add_test(phtree_test_unique_ptr_values phtree_test_unique_ptr_values.cc)

    package_add_test(phtree_f_test phtree_f_test.cc)

    package_add_test(phtree_d_test phtree_d_test.cc)
    package_add_test(phtree_d_test_copy_move phtree_d_test_copy_move.cc)
    package_add_test(phtree_d_test_custom_key phtree_d_test_custom_key.cc)
    package_add_test(phtree_d_test_filter phtree_d_test_filter.cc)
    package_add_test(phtree_d_test_preprocessor phtree_d_test_preprocessor.cc)

    package_add_test(phtree_box_f_test phtree_box_f_test.cc)

    package_add_test(phtree_box_d_test phtree_box_d_test.cc)
    package_add_test(phtree_box_d_test_filter phtree_box_d_test_filter.cc)
    package_add_test(phtree_box_d_test_query_types phtree_box_d_test_query_types.cc)

    package_add_test(phtree_multimap_d_test phtree_multimap_d_test.cc)
    package_add_test(phtree_multimap_d_test_copy_move phtree_multimap_d_test_copy_move.cc)
    package_add_test(phtree_multimap_d_test_filter phtree_multimap_d_test_filter.cc)
    package_add_test(phtree_multimap_d_test_unique_ptr_values phtree_multimap_d_test_unique_ptr_values.cc)

    package_add_test(phtree_multimap_box_d_test phtree_multimap_box_d_test.cc)

    package_add_test(converter_test converter_test.cc)
    package_add_test(distance_test distance_test.cc)
    package_add_test(filter_test filter_test.cc)

    add_subdirectory(common)
endif ()
