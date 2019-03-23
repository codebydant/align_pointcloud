##############################################################################
# CMAKE CONFIGURATION
##############################################################################
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

# set project name
project(alignmentCloud VERSION 1.0.0 LANGUAGES CXX)

# set build type = Release mode
set(CMAKE_BUILD_TYPE Release)

message("\n" "=========================================")
message("Project: ${PROJECT_NAME} ")
message("=========================================")

#set(CMAKE_C_COMPILER /usr/bin/gcc)
#set(CMAKE_CXX_COMPILER /usr/bin/g++)
#set(QT_QMAKE_EXECUTABLE "/usr/lib/x86_64-linux-gnu/qt5/bin/qmake")

# set the include directive in the same project folder
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#set(${PROJECT_SOURCE_DIR} src)
#message(STATUS "    source dir: ${PROJECT_SOURCE_DIR}")

# set corresponding package directories
set(PCL_DIR /opt/pcl-1.8.1/build)


#set the CMP0074 policy to old behavior (disable warnings) (CMake 3.12.0-rc1)
if(${CMAKE_VERSION} MATCHES 3.12.0)
    cmake_policy(SET CMP0074 OLD)
    if(POLICY CMP0048)
        cmake_policy(SET CMP0048 NEW)
    endif(POLICY CMP0048)
endif()

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#message(STATUS "path work: ${CMAKE_BINARY_DIR}")

# Include dependencies of pcl 1.8.1 in project directorie
set(CMAKE_MODULE_PATH ${PCL_DIR}/../cmake/Modules)

# set cmake for use std c++11 and output executable folder to bin
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)

# set turn off the output rule messages of cmake
set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#SET(LIBRARIES_EXTERNAL_SOURCE_DIR ${PROJECT_SOURCE_DIR}/libraries CACHE PATH "Libs")
#add_subdirectory(${LIBRARIES_EXTERNAL_SOURCE_DIR} libraries)
#set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})
#set(CMAKE_MODULE_PATH ${OpenMVG_PATH}/../src)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#set(CMAKE_C_COMPILER /usr/bin/gcc)
#set(CMAKE_CXX_COMPILER /usr/bin/g++)
#set(QT_QMAKE_EXECUTABLE "/usr/lib/x86_64-linux-gnu/qt5/bin/qmake")
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp -lpthread ")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -lpthread ")

##############################################################################
# PACKAGES
##############################################################################
message("***********************")
message("point cloud library pkg")
message("***********************")

find_package(PCL 1.8 PATHS ${PCL_DIR})
if(PCL_FOUND)
   message(STATUS "PCL status:")
   message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    pcl directorie: ${PCL_INCLUDE_DIRS}")
else()
    message(WARNING "   PCL 1.8 not found, attempting 1.7...")
    find_package(PCL 1.7 REQUIRED)
    if(PCL_FOUND)
        message(STATUS "PCL status:")
        message(STATUS "    version: ${PCL_VERSION}")
        message(STATUS "    pcl directorie: ${PCL_INCLUDE_DIRS}")
    else()
        message(FATAL_ERROR "    ERROR: PCL minimum required version 1.7. Not found")
    endif()
endif()

#if(${PCL_VERBOSITY_LEVEL} STREQUAL Error)
#set(VERBOSITY_LEVEL_ERROR 1)
#endif()

##############################################################################
# HEADERS
##############################################################################
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}

)


include(CheckFunctionExists)

# Use the compile definitions defined in PCL
add_definitions(${PCL_DEFINITIONS})


##############################################################################
# LIBRARIES PATH
##############################################################################
link_directories(${PCL_LIBRARY_DIRS}

)

##############################################################################
# SOURCE CODE
##############################################################################
set(MAIN_SOURCE "main.cpp")

##############################################################################
# EXECUTABLES
##############################################################################
add_executable(${PROJECT_NAME}               
               ${MAIN_SOURCE}
               )
               
##############################################################################
# TARGET LIBRARIES
##############################################################################
target_link_libraries(${PROJECT_NAME}                     
                      ${PCL_LIBRARIES}                  
                      ${Boost_LIBRARIES}
                               
                      )#Eigen3::Eigen)
                      

message("=========================================")
message("Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("=========================================")
