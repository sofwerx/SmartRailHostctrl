## library include
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## testing utility Sources
set(hostctrl_SRCS
    hostserial.cpp
)

## Add the testing utilities library
add_library(hostctrl_lib ${hostctrl_SRCS})
target_link_libraries(hostctrl_lib ${Boost_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}-serial-test test_serial.cpp)
if (TARGET ${PROJECT_NAME}-serial-test)
    target_link_libraries(${PROJECT_NAME}-serial-test
        ${catkin_LIBRARIES}
        ${GTEST_LIBRARIES}
        ${Boost_LIBRARIES}
    )
endif()
