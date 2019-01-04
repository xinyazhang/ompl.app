if (OMPL_BUILD_DEMOS)
    set(OMPLAPP_DEMO_LIBRARIES ${OMPLAPP_LIBRARIES} ompl ompl_app_base)

    macro(add_omplapp_demo demo_name)
      add_executable(${ARGV})
      target_link_libraries(${demo_name} ${OMPLAPP_DEMO_LIBRARIES})
    endmacro(add_omplapp_demo)

    add_omplapp_demo(demo_SE3RigidBodyPlanningBenchmark SE3RigidBodyPlanning/SE3RigidBodyPlanningBenchmark.cpp)
    add_omplapp_demo(demo_SE3RigidBodyPlanning SE3RigidBodyPlanning/SE3RigidBodyPlanning.cpp)
    add_omplapp_demo(demo_SE3RigidBodyPlanningWithOptimization SE3RigidBodyPlanning/SE3RigidBodyPlanningWithOptimization.cpp)
    add_omplapp_demo(demo_SE2RigidBodyPlanning SE2RigidBodyPlanning/SE2RigidBodyPlanning.cpp)
    add_omplapp_demo(demo_KinematicCarPlanning SE2RigidBodyPlanning/KinematicCarPlanning.cpp)
    add_omplapp_demo(demo_DynamicCarPlanning SE2RigidBodyPlanning/DynamicCarPlanning.cpp)
    add_omplapp_demo(demo_BlimpPlanning SE3RigidBodyPlanning/BlimpPlanning.cpp)
    add_omplapp_demo(demo_QuadrotorPlanning SE3RigidBodyPlanning/QuadrotorPlanning.cpp)
    add_omplapp_demo(demo_SE2MultiRigidBodyPlanning SE2RigidBodyPlanning/SE2MultiRigidBodyPlanning.cpp)
    add_omplapp_demo(demo_SE3MultiRigidBodyPlanning SE3RigidBodyPlanning/SE3MultiRigidBodyPlanning.cpp)
    add_omplapp_demo(demo_CollisionCheckers SE3RigidBodyPlanning/CollisionCheckers.cpp)
    add_omplapp_demo(demo_AnytimePathShortening AnytimePathShorteningDemo.cpp)
    add_omplapp_demo(demo_Alpha SE3RigidBodyPlanning/Alpha.cpp
    	    SE3RigidBodyPlanning/config_planner.cc)
    add_omplapp_demo(demo_Alpha-1.0 SE3RigidBodyPlanning/Alpha-1.0.cpp
    	    SE3RigidBodyPlanning/config_planner.cc)
    add_omplapp_demo(demo_Alpha-1.1 SE3RigidBodyPlanning/Alpha-1.1.cpp
    	    SE3RigidBodyPlanning/config_planner.cc)
    add_omplapp_demo(demo_Alpha-1.2 SE3RigidBodyPlanning/Alpha-1.2.cpp
    	    SE3RigidBodyPlanning/config_planner.cc)
    add_omplapp_demo(demo_Elk SE3RigidBodyPlanning/Elk.cpp
    	    SE3RigidBodyPlanning/config_planner.cc)
    add_omplapp_demo(demo_Elk-1.1 SE3RigidBodyPlanning/Elk-1.1.cc
    	    SE3RigidBodyPlanning/config_planner.cc)
    add_omplapp_demo(demo_Elk-ls1.0 SE3RigidBodyPlanning/Elk-ls1.0.cc
    	    SE3RigidBodyPlanning/config_planner.cc)
    add_omplapp_demo(demo_Elk-Stage2-ls1.0 SE3RigidBodyPlanning/Elk-Stage2-ls1.0.cc
    	    SE3RigidBodyPlanning/config_planner.cc)
    add_omplapp_demo(demo_Elk-Stage1.5-ls1.0 SE3RigidBodyPlanning/Elk-Stage1.5-ls1.0.cc
    	    SE3RigidBodyPlanning/config_planner.cc)
    add_omplapp_demo(demo_Dual_tiny SE3RigidBodyPlanning/Dual.cc
                     SE3RigidBodyPlanning/config_planner.cc)
    add_omplapp_demo(demo_Dual_full SE3RigidBodyPlanning/DualFull.cc
                     SE3RigidBodyPlanning/config_planner.cc)
    aux_source_directory(mtblender/ MTBLENDER_SRC)
    add_omplapp_demo(mtblender ${MTBLENDER_SRC})

    # create executable for planning with OPENDE
    if (OPENDE_FOUND AND DRAWSTUFF_FOUND)
        if(APPLE)
            find_package(GLUT)
        endif(APPLE)
        add_definitions(${OPENDE_DEFINITIONS})
        include_directories(${OPENDE_INCLUDE} ${DRAWSTUFF_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})

        aux_source_directory(OpenDEPlanning OpenDEPlanning_SRC)
        add_executable(demo_OpenDEPlanning ${OpenDEPlanning_SRC})
        target_link_libraries(demo_OpenDEPlanning ${DRAWSTUFF_LIBRARY} ${OPENDE_LIBRARY} ${OMPLAPP_DEMO_LIBRARIES} ${GLUT_LIBRARIES})
    endif()
endif(OMPL_BUILD_DEMOS)

file(GLOB_RECURSE OMPLAPP_DEMO_FILES "*/*")
install(FILES ${OMPLAPP_DEMO_FILES}
    DESTINATION ${OMPL_DEMO_INSTALL_DIR}
    COMPONENT omplapp)
