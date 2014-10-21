# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^debug|optimized|general$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(turtlebot_follower_CONFIG_INCLUDED)
  return()
endif()
set(turtlebot_follower_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(turtlebot_follower_SOURCE_PREFIX /home/marshall/hcr_term_project/turtlebot/src/turtlebot_apps/turtlebot_follower)
  set(turtlebot_follower_DEVEL_PREFIX /home/marshall/hcr_term_project/turtlebot/devel)
  set(turtlebot_follower_INSTALL_PREFIX "")
  set(turtlebot_follower_PREFIX ${turtlebot_follower_DEVEL_PREFIX})
else()
  set(turtlebot_follower_SOURCE_PREFIX "")
  set(turtlebot_follower_DEVEL_PREFIX "")
  set(turtlebot_follower_INSTALL_PREFIX /home/marshall/hcr_term_project/turtlebot/install)
  set(turtlebot_follower_PREFIX ${turtlebot_follower_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'turtlebot_follower' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(turtlebot_follower_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/marshall/hcr_term_project/turtlebot/devel/include;/usr/include;/usr/include/pcl-1.7;/usr/include/eigen3;/usr/include/ni;/usr/include/vtk-5.8" STREQUAL "")
  set(turtlebot_follower_INCLUDE_DIRS "")
  set(_include_dirs "/home/marshall/hcr_term_project/turtlebot/devel/include;/usr/include;/usr/include/pcl-1.7;/usr/include/eigen3;/usr/include/ni;/usr/include/vtk-5.8")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir}" STREQUAL "include")
      get_filename_component(include "${turtlebot_follower_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'turtlebot_follower' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'Daniel Stonier <stonier@rnd.yujinrobot.com>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'turtlebot_follower' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/marshall/hcr_term_project/turtlebot/src/turtlebot_apps/turtlebot_follower/${idir}'.  Ask the maintainer 'Daniel Stonier <stonier@rnd.yujinrobot.com>' to fix it.")
    endif()
    _list_append_unique(turtlebot_follower_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "turtlebot_follower;optimized;/usr/lib/libpcl_common.so;debug;/usr/lib/libpcl_common.so;optimized;/usr/lib/libpcl_kdtree.so;debug;/usr/lib/libpcl_kdtree.so;optimized;/usr/lib/libpcl_octree.so;debug;/usr/lib/libpcl_octree.so;optimized;/usr/lib/libpcl_search.so;debug;/usr/lib/libpcl_search.so;optimized;/usr/lib/libpcl_surface.so;debug;/usr/lib/libpcl_surface.so;optimized;/usr/lib/libpcl_sample_consensus.so;debug;/usr/lib/libpcl_sample_consensus.so;optimized;/usr/lib/libpcl_filters.so;debug;/usr/lib/libpcl_filters.so;optimized;/usr/lib/libpcl_features.so;debug;/usr/lib/libpcl_features.so;optimized;/usr/lib/libpcl_segmentation.so;debug;/usr/lib/libpcl_segmentation.so;optimized;/usr/lib/libpcl_io.so;debug;/usr/lib/libpcl_io.so;optimized;/usr/lib/libpcl_registration.so;debug;/usr/lib/libpcl_registration.so;optimized;/usr/lib/libpcl_keypoints.so;debug;/usr/lib/libpcl_keypoints.so;optimized;/usr/lib/libpcl_recognition.so;debug;/usr/lib/libpcl_recognition.so;optimized;/usr/lib/libpcl_visualization.so;debug;/usr/lib/libpcl_visualization.so;optimized;/usr/lib/libpcl_people.so;debug;/usr/lib/libpcl_people.so;optimized;/usr/lib/libpcl_outofcore.so;debug;/usr/lib/libpcl_outofcore.so;optimized;/usr/lib/libpcl_tracking.so;debug;/usr/lib/libpcl_tracking.so;optimized;/usr/lib/libpcl_apps.so;debug;/usr/lib/libpcl_apps.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libpthread.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;/usr/lib/libOpenNI.so;optimized;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a;debug;/usr/lib/x86_64-linux-gnu/libflann_cpp_s.a;/usr/lib/libvtkCommon.so.5.8.0;/usr/lib/libvtkRendering.so.5.8.0;/usr/lib/libvtkHybrid.so.5.8.0;/usr/lib/libvtkCharts.so.5.8.0")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^debug|optimized|general$")
    list(APPEND turtlebot_follower_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND turtlebot_follower_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND turtlebot_follower_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/marshall/hcr_term_project/turtlebot/devel/lib;/home/marshall/hcr_term_project/kobuki/devel/lib;/home/marshall/hcr_term_project/rocon/devel/lib;/home/marshall/turtlebot/devel/lib;/home/marshall/kobuki/devel/lib;/home/marshall/rocon/devel/lib;/opt/ros/indigo/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(turtlebot_follower_LIBRARY_DIRS ${lib_path})
      list(APPEND turtlebot_follower_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'turtlebot_follower'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND turtlebot_follower_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(turtlebot_follower_EXPORTED_TARGETS "turtlebot_follower_gencfg")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${turtlebot_follower_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "nodelet;pcl_ros;roscpp;visualization_msgs;turtlebot_msgs;dynamic_reconfigure")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 turtlebot_follower_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${turtlebot_follower_dep}_FOUND)
      find_package(${turtlebot_follower_dep} REQUIRED)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${turtlebot_follower_dep} REQUIRED ${depend_list})
  endif()
  _list_append_unique(turtlebot_follower_INCLUDE_DIRS ${${turtlebot_follower_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(turtlebot_follower_LIBRARIES ${turtlebot_follower_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${turtlebot_follower_dep}_LIBRARIES})
  _list_append_deduplicate(turtlebot_follower_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(turtlebot_follower_LIBRARIES ${turtlebot_follower_LIBRARIES})

  _list_append_unique(turtlebot_follower_LIBRARY_DIRS ${${turtlebot_follower_dep}_LIBRARY_DIRS})
  list(APPEND turtlebot_follower_EXPORTED_TARGETS ${${turtlebot_follower_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${turtlebot_follower_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
