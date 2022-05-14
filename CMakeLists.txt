cmake_minimum_required(VERSION  2.8.3)
project(GhostWorldServer)

add_compile_options(-std=c++14)

set(Bullet_INCLUDE_DIRS /usr/local/include/bullet)


find_package(Boost REQUIRED system filesystem date_time thread)
find_package(OpenGL REQUIRED)
find_package( Bullet REQUIRED )
find_package(Com)

if (Bullet_FOUND)
  message("bullet found")
else()
  message(FATAL_ERORR "Bullet not found")
endif()

include_directories(
    ${Bullet_INCLUDE_DIRS}
    ${Bullet_INCLUDE_DIRS}/ThirdPartyLibs/glad
    ${Bullet_INCLUDE_DIRS}/Bullet3Common                  
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR}  ${OPENGL_INCLUDE_DIRS} include)

LINK_LIBRARIES(
BulletDynamics BulletCollision Bullet3Common LinearMath OpenGLWindow
)
##################################Testing##################################

enable_testing()
add_subdirectory(unit_test)
add_subdirectory(googletest)

#making a library for testing
add_library(GhostWorldServer_Test
                src/GhostWorldServer.cpp
                src/b3Clock.cpp
                src/GhostExample_shrededws.cpp
                src/OpenGLGuiHelper.cpp
                src/b3ResourcePath.cpp
                src/stb_image_write.cpp
                src/fontstash.cpp
                src/TwFonts.cpp
                src/CollisionShape2TriangleMesh.cpp
              
)
target_link_libraries(GhostWorldServer_Test  Com jsoncpp msg   ${OPENGL_INCLUDE_DIRS}  ${CMAKE_DL_LIBS} ${CMAKE_THREAD_LIBS_INIT})

########################################USE#################################
#making an excutable for use
add_executable(GhostWorldServer
                src/GhostWorldServer.cpp
                src/b3Clock.cpp
                src/GhostExample_shrededws.cpp
                src/OpenGLGuiHelper.cpp
                src/b3ResourcePath.cpp
                src/stb_image_write.cpp
                src/fontstash.cpp
                src/TwFonts.cpp
                
                src/CollisionShape2TriangleMesh.cpp
               
)


target_link_libraries(GhostWorldServer  Com jsoncpp msg  ${OPENGL_INCLUDE_DIRS}  ${CMAKE_DL_LIBS} ${CMAKE_THREAD_LIBS_INIT})

