cmake_minimum_required(VERSION  2.8.3)
project(Game)

add_compile_options(-std=c++14)


set(PANDA_INCLUDE_DIRS /usr/include/panda3d)
set(Eigen_INCLUDE_DIRS /usr/include/eigen3)
set(PANDA_LIB /usr/lib/x86_64-linux-gnu/panda3d/)
set(LIBS_LIST libp3assimp.so;libp3interrogatedb.so.1.11;libpandaegg.so.1.11;libp3direct.so;libp3openal_audio.so;libpandaexpress.so;libp3direct.so.1.11;libp3ptloader.so;libpandaexpress.so.1.11;
libp3dtool.so; libp3tinydisplay.so;         libpandafx.so;
libp3dtool.so.1.11  ;      libp3tinydisplay.so.1.11 ;   libpandafx.so.1.11;
libp3dtoolconfig.so    ;   libp3vision.so ;             libpandagl.so;
libp3dtoolconfig.so.1.11;  libp3vision.so.1.11   ;      libpandaode.so;
libp3ffmpeg.so        ;    libpanda.so        ;         libpandaode.so.1.11;
libp3ffmpeg.so.1.11  ;     libpanda.so.1.11      ;      libpandaphysics.so;
libp3framework.so  ;       libpandaai.so  ;             libpandaphysics.so.1.11;
libp3framework.so.1.11  ;  libpandaai.so.1.11    ;      libpandaskel.so;
libp3interrogatedb.so   ;  libpandaegg.so   ;           libpandaskel.so.1.11
 )


# find_package()





include_directories( ${PANDA_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})



########################################USE#################################
#making an excutable for use
add_executable(Game
                src/game.cpp
      
               
)
foreach(LIB ${LIBS_LIST})
    target_link_libraries(Game ${PANDA_LIB}${LIB})
endforeach()






