TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG += qt

INCLUDEPATH += "C:/Program Files/PCL 1.7.2/3rdParty/Boost/include/boost-1_57"
INCLUDEPATH += "C:/Program Files/PCL 1.7.2/include/pcl-1.7"
INCLUDEPATH += "C:/Program Files/PCL 1.7.2/3rdParty/Eigen/eigen3"
INCLUDEPATH += "C:/Program Files/PCL 1.7.2/3rdParty/Qhull/include"

LIBS += "-LC:/Program Files/PCL 1.7.2/lib"
LIBS += "-LC:/Program Files/PCL 1.7.2/3rdParty/Boost/lib"


LIBS += -lpcl_common_debug
LIBS += -lpcl_filters_debug
LIBS += -lpcl_keypoints_debug
LIBS += -lpcl_kdtree_debug
LIBS += -lpcl_search_debug
LIBS += -lpcl_features_debug
LIBS += -lpcl_io_debug
LIBS += -lpcl_io_ply_debug
LIBS += -lpcl_visualization_debug


LIBS += -lpcl_common_release
LIBS += -lpcl_filters_release
LIBS += -lpcl_keypoints_release
LIBS += -lpcl_kdtree_release
LIBS += -lpcl_search_release
LIBS += -lpcl_features_release
LIBS += -lpcl_io_release
LIBS += -lpcl_io_ply_release
LIBS += -lpcl_visualization_release

SOURCES += main.cpp \
    src/model/cylinder.cpp \
    src/model/segment.cpp \
    src/math/simplemath.cpp \
    src/test/model/testcylinder.cpp \
    src/test/testsimplemath.cpp \
    src/test/model/testsegment.cpp \
    src/model/tree.cpp \
    src/test/model/testtree.cpp \
    src/model/build_tree/buildtree.cpp \
    src/model/build_tree/improvebymedian.cpp \
    src/model/build_tree/removefalsecylinders.cpp \
    src/model/build_tree/reordertree.cpp \
    src/model/build_tree/improvebranchjunctions.cpp \
    src/model/build_tree/improvefit.cpp

HEADERS += \
    src/model/cylinder.h \
    src/model/pointsimpletree.h \
    src/model/segment.h \
    src/typedef.h \
    src/math/simplemath.h \
    src/test/model/testcylinder.h \
    src/test/testsimplemath.h \
    src/test/model/testsegment.h \
    src/math/simplemath.hpp \
    src/model/tree.h \
    src/test/model/testtree.h \
    src/model/build_tree/buildtree.h \
    src/model/build_tree/improvebymedian.h \
    src/model/build_tree/removefalsecylinders.h \
    src/model/build_tree/reordertree.h \
    src/model/build_tree/improvebranchjunctions.h \
    src/model/build_tree/improvefit.h












