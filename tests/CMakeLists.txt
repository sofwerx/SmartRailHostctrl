set ( CMAKE_BUILD_TYPE Debug )

## library include
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

catkin_add_gtest(${PROJECT_NAME}-fletcher32-test test_fletcher32.cpp)
if (TARGET ${PROJECT_NAME}-fletcher32-test)
    target_link_libraries(${PROJECT_NAME}-fletcher32-test
        ${GTEST_LIBRARIES}
        ${Boost_LIBRARIES}
)
endif()

catkin_add_gtest(${PROJECT_NAME}-serial-test test_serial.cpp)
if (TARGET ${PROJECT_NAME}-serial-test)
    target_link_libraries(${PROJECT_NAME}-serial-test
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES}
        ${Boost_LIBRARIES}
    )
endif()
