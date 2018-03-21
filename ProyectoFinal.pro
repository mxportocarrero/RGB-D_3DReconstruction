TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    posegraph.cpp \
    pointcloud.cpp \
    main.cpp \
    pointcloud.cpp \
    posegraph.cpp \
    loadshader.cpp \
    image.cpp \
    camera.cpp \
    dataset.cpp \
    odometry.cpp \
    addfunctions.cpp \
    volumeintegrator.cpp

HEADERS += \
    posegraph.h \
    includes.h \
    pointcloud.h \
    includes.h \
    pointcloud.h \
    posegraph.h \
    loadshader.h \
    image.h \
    camera.h \
    dataset.h \
    odometry.h \
    volumeintegrator.h \
    octree.h \
    visualoctree.h

DISTFILES += \
    TODO \
    makefile \
    README.md \
    vertex.shader \
    fragment.shader
