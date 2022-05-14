
cmake_minimum_required(VERSION  2.8.3)

add_compile_options(-std=c++14)



add_executable(Test unitTest.cpp)


target_link_libraries(Test gtest_main GhostWorldServer_Test)

add_test(
    NAME Test
    COMMAND Test
)