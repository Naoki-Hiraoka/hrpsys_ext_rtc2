set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  choreonoid_qhull
  choreonoid_vclip
  cpp_filters
  )

# find_package(choreonoid)を2回呼ぶとadd_library cannot create imported target "Choreonoid::Cnoid*" because another target with the same name already exists.エラーになるので、親のCMakeLists.txtで1回だけ呼ぶ.
# find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  program_options)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${hrpsys_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  )

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  ${openrtm_aist_LIBRARY_DIRS}
  )

add_library(CollisionDetector2 SHARED
  CollisionDetector2.cpp
  CollisionDetector2Service_impl.cpp
  ../SoftErrorLimiter2/beep.cpp
  )
target_link_libraries(CollisionDetector2
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
  HRPDataTypesSkel HRPDataTypesStub
  )
set_target_properties(CollisionDetector2 PROPERTIES PREFIX "") # libCollisionDetector2.so -> CollisionDetector2.so
add_dependencies(CollisionDetector2 RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

add_executable(CollisionDetector2Comp CollisionDetector2Comp.cpp)
target_link_libraries(CollisionDetector2Comp CollisionDetector2)

add_executable(SetupCollisionPair SetupCollisionPair.cpp)
target_link_libraries(SetupCollisionPair
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  Boost::program_options
  )
