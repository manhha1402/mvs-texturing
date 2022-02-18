externalproject_add(ext_mapmap
    PREFIX          ext_mapmap
    GIT_REPOSITORY  https://github.com/dthuerck/mapmap_cpu.git
    GIT_TAG         4adffe0a7a9283f092f9b8d0902adc3c144b0984
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${CMAKE_SOURCE_DIR}/elibs/mapmap
    CONFIGURE_COMMAND ""
    BUILD_COMMAND   ""
    INSTALL_COMMAND ""
)

externalproject_add(ext_rayint
    PREFIX          ext_rayint
    GIT_REPOSITORY  https://github.com/nmoehrle/rayint.git
    GIT_TAG         cuda
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${CMAKE_SOURCE_DIR}/elibs/rayint
    CONFIGURE_COMMAND ""
    BUILD_COMMAND   ""
    INSTALL_COMMAND ""
)

externalproject_add(ext_eigen
    PREFIX          ext_eigen
    URL             https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
    SOURCE_DIR      ${CMAKE_SOURCE_DIR}/elibs/eigen
    CONFIGURE_COMMAND ""
    BUILD_COMMAND   ""
    INSTALL_COMMAND ""
)

externalproject_add(ext_mve
    PREFIX          mve
    GIT_REPOSITORY  https://github.com/nmoehrle/mve.git
    UPDATE_COMMAND  ""
    SOURCE_DIR      ${CMAKE_SOURCE_DIR}/elibs/mve
    CONFIGURE_COMMAND ""
    BUILD_COMMAND   make -C libs/mve && make -C libs/util #not platform independent
    BUILD_IN_SOURCE 1
    INSTALL_COMMAND ""
)
