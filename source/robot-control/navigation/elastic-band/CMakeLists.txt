set(srcs
    GraphSearchInterface.cpp
    HomotopyClassPlanner.cpp
    Obstacles.cpp
    RecoveryBehaviors.cpp
    TebConfig.cpp
    TebLocalPlannerROS.cpp
    TebPlanner.cpp
    TebPlot.cpp
    TebVisualization.cpp
    TimedElasticBand.cpp
)

set(hdrs
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
    kinematics/Acceleration.hpp
    kinematics/PoseSE2.hpp
    kinematics/Timestamp.hpp
    kinematics/Trajectory.hpp
    kinematics/Velocity.hpp
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
    TebLocalPlannerROS.hpp
    TebPlanner.hpp
    TebPlot.hpp
    TebVisualization.hpp
    TimedElasticBand.hpp
)

find_package(catkin      REQUIRED COMPONENTS teb_local_planner)
find_package(Qt5         REQUIRED COMPONENTS Widgets Charts)
find_package(SUITESPARSE REQUIRED)
find_package(G2O         REQUIRED)
find_package(Eigen3      REQUIRED)

set(root "")
if(EXISTS /usr/local/include/g2o)
    set(root /usr/local)
elseif(EXISTS   /usr/include/g2o)
    set(root /usr)
endif()

function(g2o_replace inc lib)
    if(NOT "${root}" STREQUAL "")
        string(REPLACE /opt/ros/kinetic/include/g2o ${root}/include/g2o ${inc} "${${inc}}")
        string(REPLACE /opt/ros/kinetic/lib/libg2o_ ${root}/lib/libg2o_ ${lib} "${${lib}}")
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

set(lib elastic-band)

add_library               (${lib}        SHARED  ${srcs} ${hdrs})
target_include_directories(${lib} SYSTEM PUBLIC  ${EXTERNAL_INCLUDE_DIRS})
target_link_libraries     (${lib}        PRIVATE ${EXTERNAL_LIBS} -fsanitize=address)
target_compile_options    (${lib}        PRIVATE                  -std=c++14)

install(TARGETS ${lib}  DESTINATION .)
install(FILES   ${hdrs} DESTINATION include/${lib})
