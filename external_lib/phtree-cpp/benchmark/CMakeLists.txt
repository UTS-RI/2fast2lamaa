cmake_minimum_required(VERSION 3.14)
project(phtree-benchmarks)

set(BENCHMARK_ENABLE_TESTING OFF)

include(FetchContent)

FetchContent_Declare(
        googlebenchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v1.7.0
)
FetchContent_MakeAvailable(googlebenchmark)

FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.10.0
)
FetchContent_MakeAvailable(spdlog)

macro(package_add_benchmark TESTNAME)
    add_executable(${TESTNAME} ${ARGN} benchmark_util.h logging.h)
    target_link_libraries(${TESTNAME} PRIVATE benchmark::benchmark)
    target_link_libraries(${TESTNAME} PRIVATE spdlog::spdlog)
    target_link_libraries(${TESTNAME} PRIVATE phtree::phtree)
endmacro()

add_compile_definitions(RUN_HAVE_STD_REGEX=0 RUN_HAVE_POSIX_REGEX=0 COMPILE_HAVE_GNU_POSIX_REGEX=0)

package_add_benchmark(bpt_erase_benchmark bpt_erase_benchmark.cc)
package_add_benchmark(bpt_erase_it_benchmark bpt_erase_it_benchmark.cc)
package_add_benchmark(bpt_insert_benchmark bpt_insert_benchmark.cc)
package_add_benchmark(bpt_iter_benchmark bpt_iter_benchmark.cc)
package_add_benchmark(bpt_lower_bound_benchmark bpt_lower_bound_benchmark.cc)

package_add_benchmark(count_mm_d_benchmark count_mm_d_benchmark.cc)
package_add_benchmark(erase_benchmark erase_benchmark.cc)
package_add_benchmark(erase_d_benchmark erase_d_benchmark.cc)
package_add_benchmark(extent_benchmark extent_benchmark.cc)
package_add_benchmark(extent_benchmark_weird extent_benchmark_weird.cc)
package_add_benchmark(find_benchmark find_benchmark.cc)
package_add_benchmark(hd_erase_d_benchmark hd_erase_d_benchmark.cc)
package_add_benchmark(hd_insert_d_benchmark hd_insert_d_benchmark.cc)
package_add_benchmark(hd_knn_d_benchmark hd_knn_d_benchmark.cc)
package_add_benchmark(hd_query_d_benchmark hd_query_d_benchmark.cc)
package_add_benchmark(insert_benchmark insert_benchmark.cc)
package_add_benchmark(insert_box_d_benchmark insert_box_d_benchmark.cc)
package_add_benchmark(insert_d_benchmark insert_d_benchmark.cc)
package_add_benchmark(knn_d_benchmark knn_d_benchmark.cc)
package_add_benchmark(query_benchmark query_benchmark.cc)
package_add_benchmark(query_box_d_benchmark query_box_d_benchmark.cc)
package_add_benchmark(query_d_benchmark query_d_benchmark.cc)
package_add_benchmark(query_mm_box_d_benchmark query_mm_box_d_benchmark.cc)
package_add_benchmark(query_mm_d_benchmark query_mm_d_benchmark.cc)
package_add_benchmark(query_mm_d_filter_benchmark query_mm_d_filter_benchmark.cc)
package_add_benchmark(update_box_d_benchmark update_box_d_benchmark.cc)
package_add_benchmark(update_d_benchmark update_d_benchmark.cc)
package_add_benchmark(update_mm_box_d_benchmark update_mm_box_d_benchmark.cc)
package_add_benchmark(update_mm_d_benchmark update_mm_d_benchmark.cc)
