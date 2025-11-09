cmake_minimum_required(VERSION 3.15)

set(project_name consens)

project(${project_name} VERSION 0.0.1 LANGUAGES CXX)

add_compile_options(-Wall -Wextra -Wpedantic)
cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0135 NEW)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(params
    -Wno-reorder
    -Wno-unused-parameter
)

string(TOUPPER ${project_name} project_name_upper)
option(${project_name_upper}_BUILD_EXAMPLES "Build examples" OFF)
option(${project_name_upper}_ENABLE_TESTS "Enable tests" OFF)
include(FetchContent)

# --------------------------------------------------------------------------------------------------
set(ext_deps)

FetchContent_Declare(concord GIT_REPOSITORY https://github.com/smolfetch/concord.git GIT_TAG 2.2.0)
FetchContent_MakeAvailable(concord)
list(APPEND ext_deps concord::concord)

find_package(Boost REQUIRED)
list(APPEND ext_deps Boost::boost)


find_package(spdlog REQUIRED)
list(APPEND ext_deps spdlog::spdlog)


# Optional dependencies via pkg-config (for MPC, etc.)
# find_package(PkgConfig)
# set(pkg_packs libsodium)
# foreach(pkg IN LISTS pkg_packs)
#     string(TOUPPER ${pkg} pkg_upper)
#     pkg_check_modules(${pkg_upper} ${pkg})
#     if(${pkg_upper}_FOUND)
#         message(STATUS "Found ${pkg}: ${${pkg_upper}_LIBRARIES}")
#         message(STATUS "  Include dirs: ${${pkg_upper}_INCLUDE_DIRS}")
#         message(STATUS "  Library dirs: ${${pkg_upper}_LIBRARY_DIRS}")
#         include_directories(${${pkg_upper}_INCLUDE_DIRS})
#         link_directories(${${pkg_upper}_LIBRARY_DIRS})
#         list(APPEND ext_deps ${${pkg_upper}_LIBRARIES})
#     else()
#         message(WARNING "${pkg} not found")
#     endif()
# endforeach()

# --------------------------------------------------------------------------------------------------
# Build library
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)

add_library(${project_name} ${SOURCES})
add_library(${project_name}::${project_name} ALIAS ${project_name})

target_include_directories(${project_name} 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${project_name} PRIVATE ${ext_deps})

# Export dependencies as public (headers use them)
target_link_libraries(${project_name} PUBLIC ${ext_deps})

target_compile_options(${project_name} PRIVATE ${params})

install(
  DIRECTORY include/
  DESTINATION include
)
include_directories(include)

# --------------------------------------------------------------------------------------------------
include(GNUInstallDirs)

#Install headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install library and export targets
install(TARGETS ${project_name} 
    EXPORT ${project_name}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

#Install the export file so that `find_package(${project_name})` works
install(EXPORT ${project_name}Targets
  FILE ${project_name}Targets.cmake
  NAMESPACE ${project_name}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${project_name}
)

#Also export it for in‐source FetchContent usage
export(EXPORT ${project_name}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${project_name}Targets.cmake"
  NAMESPACE ${project_name}::
)


# --------------------------------------------------------------------------------------------------
if(${project_name_upper}_BUILD_EXAMPLES)
  file(GLOB exe CONFIGURE_DEPENDS examples/*.cpp)
  # ----------------------------------------------
  FetchContent_Declare(cli11 GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git GIT_TAG v2.5.0)
  FetchContent_MakeAvailable(cli11)
  # Note: farmtrax is optional and only needed if examples use field generation
  # FetchContent_Declare(farmtrax GIT_REPOSITORY https://github.com/onlyhead/farmtrax.git GIT_TAG 1.0.1)
  # FetchContent_MakeAvailable(farmtrax)
  # list(APPEND ext_deps farmtrax::farmtrax)

  set(exec_names)
  foreach(src_file IN LISTS exe)
    get_filename_component(exec_name "${src_file}" NAME_WE)
    add_executable(${exec_name} "${src_file}")
      target_compile_options(${exec_name} PRIVATE ${params})
      foreach(lib_file IN LISTS internal_deps)
        target_compile_options(${exec_name} PRIVATE ${params})
        target_sources(${exec_name} PRIVATE "${lib_file}")
      endforeach()
    target_compile_options(${exec_name} PRIVATE ${CURL_CFLAGS_OTHER})
    target_link_libraries(${exec_name} ${project_name}::${project_name} ${ext_deps})
    target_link_options(${exec_name} PRIVATE ${link_options})
    target_include_directories(${exec_name} PRIVATE ${CURL_INCLUDE_DIRS})
    install(TARGETS ${exec_name} DESTINATION bin)
    list(APPEND exec_names ${exec_name})
  endforeach()
  # ----------------------------------------------
  foreach(exec IN LISTS exec_names)
    file(REMOVE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    file(WRITE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    get_filename_component(exec_name "${exec}" NAME)
    file(APPEND "${CMAKE_CURRENT_LIST_DIR}/.execs" "${CMAKE_CURRENT_LIST_DIR}/build/${exec_name}\n")
  endforeach()
  # ----------------------------------------------
  install(TARGETS
    ${exec_names}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# --------------------------------------------------------------------------------------------------
if(${project_name_upper}_ENABLE_TESTS)
  enable_testing()
  add_definitions(-DENABLE_DOCTEST_IN_LIBRARY)
  include(FetchContent)
  FetchContent_Declare(DocTest GIT_REPOSITORY "https://github.com/onqtam/doctest")
  FetchContent_MakeAvailable(DocTest)

  file(GLOB test_src test/*.cpp)

  foreach(src_file IN LISTS test_src)
    get_filename_component(test_name "${src_file}" NAME_WE)
    add_executable(${test_name} "${src_file}")
    target_compile_options(${test_name} PRIVATE ${params})
    target_link_libraries(${test_name} ${project_name}::${project_name} ${ext_deps} doctest_with_main)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
