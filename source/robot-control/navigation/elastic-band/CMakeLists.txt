set(hdrs_g2o_actions
    g2o_actions/OrientateAction.hpp
    g2o_actions/TerminateAction.hpp
)

set(hdrs_g2o_types
    g2o_types/BaseTebEdges.hpp
    g2o_types/EdgeAcceleration.hpp
    g2o_types/EdgeDynamicObstacle.hpp
    g2o_types/EdgeKinematics.hpp
    g2o_types/EdgeObstacle.hpp
    g2o_types/EdgePreferRotDir.hpp
    g2o_types/EdgeProfileFidelity.hpp
    g2o_types/EdgeShortestPath.hpp
    g2o_types/EdgeTimeOptimal.hpp
    g2o_types/EdgeVelocity.hpp
    g2o_types/EdgeViaPoint.hpp
    g2o_types/Penalties.hpp
    g2o_types/VertexPose.hpp
    g2o_types/VertexTimeDiff.hpp
)

set(hdrs_kinematics
    kinematics/Acceleration.hpp
    kinematics/PoseSE2.hpp
    kinematics/Timestamp.hpp
    kinematics/Trajectory.hpp
    kinematics/Velocity.hpp
)

set(hdrs
  ${hdrs_g2o_actions}
  ${hdrs_g2o_types}
  ${hdrs_kinematics}
    Distances.hpp
    EquivalenceClass.hpp
    GraphSearchInterface.hpp
    HomotopyClassPlanner.hpp
    HSignature.hpp
    Misc.hpp
    Obstacles.hpp
    PlannerInterface.hpp
    RecoveryBehaviors.hpp
    RobotFootprintModel.hpp
    TebConfig.hpp
    TebPlanner.hpp
    TebPlot.hpp
    TebVisualization.hpp
    TimedElasticBand.hpp
)

set(srcs_g2o_actions)

set(srcs_g2o_types
    g2o_types/CustomTypes.cpp
)

set(srcs_kinematics)

set(srcs
  ${srcs_g2o_actions}
  ${srcs_g2o_types}
  ${srcs_kinematics}
    GraphSearchInterface.cpp
    HomotopyClassPlanner.cpp
    Obstacles.cpp
    RecoveryBehaviors.cpp
    TebConfig.cpp
    TebPlanner.cpp
    TebPlot.cpp
    TebVisualization.cpp
    TimedElasticBand.cpp
)

set(ros /opt/ros/kinetic)
list(APPEND CMAKE_PREFIX_PATH ${ros})

find_package(catkin      REQUIRED COMPONENTS teb_local_planner)
find_package(Qt5         REQUIRED COMPONENTS Widgets Charts)
find_package(SUITESPARSE REQUIRED)
find_package(G2O         REQUIRED)
find_package(Eigen3      REQUIRED)

list(REMOVE_ITEM catkin_LIBRARIES ${ros}/lib/libteb_local_planner.so)

set(root "")
if(EXISTS /usr/local/include/g2o)
    set(root /usr/local)
elseif(EXISTS   /usr/include/g2o)
    set(root /usr)
endif()

function(g2o_replace inc lib)
    if(NOT "${root}" STREQUAL "")
        string(REPLACE ${ros}/include/g2o ${root}/include/g2o ${inc} "${${inc}}")
        string(REPLACE ${ros}/lib/libg2o_ ${root}/lib/libg2o_ ${lib} "${${lib}}")
        set(${inc} ${${inc}} PARENT_SCOPE)
        set(${lib} ${${lib}} PARENT_SCOPE)
    endif()
endfunction()

g2o_replace(catkin_INCLUDE_DIRS catkin_LIBRARIES)
g2o_replace(   G2O_INCLUDE_DIR     G2O_LIBRARIES)

set(EXTERNAL_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
set(EXTERNAL_LIBS         ${catkin_LIBRARIES}    ${SUITESPARSE_LIBRARIES}    ${G2O_LIBRARIES})
list(APPEND EXTERNAL_LIBS Qt5::Widgets Qt5::Charts)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

set(lib g2o_custom_types)
add_library               (${lib}        SHARED ${srcs_g2o_types} ${hdrs_g2o_types})
target_include_directories(${lib} SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/g2o_types)
target_include_directories(${lib} SYSTEM PUBLIC ${EXTERNAL_INCLUDE_DIRS})
target_link_libraries     (${lib}        PUBLIC ${EXTERNAL_LIBS} -fsanitize=address)
target_compile_options    (${lib}        PUBLIC                  -std=c++14)

set(lib elastic-band)
add_library               (${lib}        SHARED ${srcs} ${hdrs})
target_include_directories(${lib} SYSTEM PUBLIC ${EXTERNAL_INCLUDE_DIRS})
target_link_libraries     (${lib}        PUBLIC ${EXTERNAL_LIBS} -fsanitize=address)
target_compile_options    (${lib}        PRIVATE                 -std=c++14)

install(TARGETS ${lib}  DESTINATION .)
install(FILES   ${hdrs} DESTINATION include/${lib})
