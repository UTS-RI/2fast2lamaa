cmake_minimum_required(VERSION 3.14)
project(phtree-examples)

add_executable(Example example.cc)
target_link_libraries(Example phtree)
