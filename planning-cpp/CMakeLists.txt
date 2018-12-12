cmake_minimum_required( VERSION 3.1 )

project(planning_project)

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_SHARED_LIBRARY_PREFIX "")

find_package( OpenCV REQUIRED)
find_package( PythonLibs 2.7 REQUIRED )

include_directories ("${PROJECT_SOURCE_DIR}/lib/json/include")


include_directories(${PYTHON_INCLUDE_DIRS})


add_definitions(-DMATPLOTLIBCPP_PYTHON_HEADER=Python.h)

add_executable(planner
    src/planner.cpp
    src/CollisionChecker.cpp
    src/actions.cpp
    src/main.cpp
    src/Timer.cpp
)

target_link_libraries(planner
    ${OpenCV_LIBS}
    ${PYTHON_LIBRARIES}
)


