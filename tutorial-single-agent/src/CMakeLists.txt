cmake_minimum_required(VERSION 3.10.1)
project(DEMO)

# find_package(Boost REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++20 ${CMAKE_CXX_FLAGS}")

# 添加 fmt 库
find_package(fmt REQUIRED)

file(GLOB_RECURSE ALL_HDRS "include/*.hpp")
file(GLOB_RECURSE ALL_SRCS "source/*.cpp")

# message("ALL_HDRS:", ${ALL_HDRS})

include_directories(DIRECTORY 
  # include/${PROJECT_NAME}
  include
  # include/common
)

add_library(${PROJECT_NAME} SHARED
  ${ALL_SRCS}
)

set(test_cpp_dir "test/")
set(test_cpp_files "")
file(GLOB_RECURSE test_cpp_files "${test_cpp_dir}/*.cpp")
foreach(test_cpp_file ${test_cpp_files})
  get_filename_component(test_cpp_name ${test_cpp_file} NAME_WE)
  ADD_EXECUTABLE(${test_cpp_name} ${ALL_SRCS} ${test_cpp_dir}/${test_cpp_name}.cpp)
  # TARGET_INCLUDE_DIRECTORIES(${test_cpp_name} PUBLIC include/common)
  TARGET_LINK_LIBRARIES(${test_cpp_name} 
    ${PROJECT_NAME} 
  )
  
  # 为 run_stastar 添加 fmt 库链接
  if(${test_cpp_name} STREQUAL "run_stastar")
    target_link_libraries(${test_cpp_name} fmt::fmt)
  endif()
endforeach(test_cpp_file ${test_cpp_files})
