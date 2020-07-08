cmake_minimum_required (VERSION 2.8)
cmake_policy(VERSION 2.8)

project (170CPG01_DaeunSong)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

link_directories(${CMAKE_SOURCE_DIR}/lib)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

# find the required packages
find_package(GLM REQUIRED)
message(STATUS "GLM included at ${GLM_INCLUDE_DIR}")
find_package(GLFW3 REQUIRED)
message(STATUS "Found GLFW3 in ${GLFW3_INCLUDE_DIR}")
find_package(ASSIMP REQUIRED)
message(STATUS "Found ASSIMP in ${ASSIMP_INCLUDE_DIR}")


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
find_package(OpenGL REQUIRED)
add_definitions(${OPENGL_DEFINITIONS})
find_package(X11 REQUIRED)
set(LIBS ${GLFW3_LIBRARY} X11 Xrandr Xinerama Xi Xxf86vm Xcursor GL dl pthread ${ASSIMP_LIBRARY})
set (CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

configure_file(configuration/root_directory.h.in configuration/root_directory.h)
include_directories(${CMAKE_BINARY_DIR}/configuration)

# first create relevant static libraries requried for other projects
add_library(STB_IMAGE "includes/stb_image.cpp")
set(LIBS ${LIBS} STB_IMAGE)

add_library(GLAD "includes/glad.c")
set(LIBS ${LIBS} GLAD)

macro(makeLink src dest target)
  add_custom_command(TARGET ${target} POST_BUILD COMMAND ${CMAKE_COMMAND} -E create_symlink ${src} ${dest}  DEPENDS  ${dest} COMMENT "mklink ${src} -> ${dest}")
endmacro()

# add executables
set(DEMO_FETCH_SAY_HI "demo_fetch_say_hi")
add_executable(${DEMO_FETCH_SAY_HI} src/${DEMO_FETCH_SAY_HI}.cpp)
target_link_libraries(${DEMO_FETCH_SAY_HI} ${LIBS} urdfdom_model)

set(DEMO_TALOS_SAY_HI "demo_talos_say_hi")
add_executable(${DEMO_TALOS_SAY_HI} src/${DEMO_TALOS_SAY_HI}.cpp)
target_link_libraries(${DEMO_TALOS_SAY_HI} ${LIBS} urdfdom_model)

set(DEMO_TALOS "demo_talos")
add_executable(${DEMO_TALOS} src/${DEMO_TALOS}.cpp)
target_link_libraries(${DEMO_TALOS} ${LIBS} urdfdom_model)

set(glURDF "glURDF")
add_executable(${glURDF} src/${glURDF}.cpp)
target_link_libraries(${glURDF} ${LIBS} urdfdom_model)

set_target_properties(${DEMO_FETCH_SAY_HI} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

set_target_properties(${DEMO_TALOS_SAY_HI} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

set_target_properties(${DEMO_TALOS} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

set_target_properties(${glURDF} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

# copy shader files to build directory
file(GLOB SHADERS
  "src/*.vs"
  "src/*.fs"
  "src/*.gs"
)
foreach(SHADER ${SHADERS})
  file(COPY ${SHADER} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/bin)
endforeach(SHADER)
# if compiling for visual studio, also use configure file for each project (specifically to set up working directory)

if(MSVC)
    configure_file(${CMAKE_SOURCE_DIR}/configuration/visualstudio.vcxproj.user.in ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.vcxproj.user @ONLY)
endif(MSVC)

include_directories(${CMAKE_SOURCE_DIR}/includes)
