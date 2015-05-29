#-------------------------------------------------
#
# Project created by QtCreator 2015-04-07T13:42:41
#
#-------------------------------------------------

QT += core gui
QT += opengl
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
#QMAKE_LFLAGS += /INCREMENTAL:NO
TARGET = SCN_reader
TEMPLATE = app
#icon
RC_FILE = ico.rc

OTHER_FILES += \
    ic.ico \
    ico.rc

# ---------- CONFIG
CONFIG += opengl
CONFIG += opencv
CONFIG += pcl
# ---------- end CONFIG


# -------------------  SOURCES -------------------
SOURCES += main.cpp\
    mainwindow.cpp \
    modules/exceptions/erreur.cpp \
    works/scnreader_view.cpp \
    works/scnreader_model.cpp \
    works/scn_reader/sources/scndata.cpp \
    works/scn_reader/sources/datapackage.cpp \
    works/vueparetape.cpp \
    works/railcluster.cpp \
    works/listerail.cpp \
    modules/openGL/tools_models/Point.cpp \
    works/regionsmanager.cpp \
    works/colorsmanager.cpp \
    works/regiongrowing.cpp\
    works/imageprocessing.cpp
# if you use openGL
opengl {
    SOURCES+=modules/openGL/ground/controller/controller_ground_gl.cpp \
        modules/openGL/ground/model/model_ground_gl.cpp \
        modules/openGL/ground/model/GLWidget.cpp \
        modules/openGL/ground/model/LesTextures.cpp \
        modules/openGL/ground/model/Materiel.cpp \
        modules/openGL/ground/model/Scene.cpp \
        modules/openGL/ground/model/Texture.cpp \
        modules/openGL/ground/view/view_ground_GL.cpp \
        modules/openGL/ground/groundglwidget.cpp
}
#if you use openCV
opencv {
    SOURCES+=modules/openCV/ground/model/toolopencv.cpp \
        modules/openCV/magicfish.cpp
}
#if you use pcl
pcl {
    SOURCES+=modules/pcl/ground/ToolsPCL.cpp \
        modules/pcl/view/view_pcl.cpp
}
# -------------------  HEADERS -------------------

HEADERS  += mainwindow.h \
    modules/exceptions/erreur.h \
    works/scnreader_view.h \
    works/scnreader_model.h \
    works/scn_reader/sources/scndata.h \
    works/scn_reader/sources/datapackage.h \
    works/vueparetape.h \
    works/railcluster.h \
    works/listerail.h \
    modules/openGL/tools_models/Point.h \
    works/regionsmanager.h \
    works/colorsmanager.h \
    works/regiongrowing.h\
    works/imageprocessing.h

# if you use openGL
opengl {
    HEADERS  += modules/openGL/ground/model/GLWidget.h \
        modules/openGL/ground/model/LesTextures.h \
        modules/openGL/ground/model/Materiel.h \
        modules/openGL/ground/model/Scene.h \
        modules/openGL/ground/model/Texture.h \
        modules/openGL/ground/view/view_ground_GL.h \
        modules/openGL/ground/controller/controller_ground_gl.h \
        modules/openGL/ground/model/model_ground_gl.h \
        modules/openGL/ground/groundglwidget.h
}
# if you use openCV
opencv {
    HEADERS  +=modules/openCV/ground/model/toolopencv.h \
        modules/openCV/magicfish.h

}
#if you use pcl
pcl {
    HEADERS  += modules/pcl/ground/ToolsPCL.h \
            modules/pcl/view/view_pcl.h
}

FORMS    += mainwindow.ui


# -------------------  LIBS -------------------

# ===== PCL
#if you use pcl
pcl {
    unix{
         LIBS += -L$$PWD/usr/lib/  -lpcl_common
         LIBS += -L$$PWD/usr/lib/ -lpcl_search
         LIBS += -L$$PWD/usr/lib/ -lpcl_outofcore
         LIBS += -L$$PWD/usr/lib/ -lpcl_octree
         LIBS += -L$$PWD/usr/lib/ -lpcl_recognition
         LIBS += -L$$PWD/usr/lib/ -lpcl_sample_consensus
         LIBS += -L$$PWD/usr/lib/ -lpcl_filters
         LIBS += -L$$PWD/usr/lib/ -lpcl_kdtree
         LIBS += -L$$PWD/usr/lib/ -lpcl_segmentation
         LIBS += -L$$PWD/usr/lib/ -lpcl_search
         LIBS += -L$$PWD/usr/lib/ -lpcl_io
         LIBS += -L$$PWD/usr/lib/ -lpcl_keypoints
         LIBS += -L$$PWD/usr/lib/ -lpcl_features
         LIBS += -L$$PWD/usr/lib/ -lpcl_visualization
         LIBS += -L$$PWD/usr/lib/x86_64-linux-gnu/ -lboost_system
    }#end unix

    else:win32 {
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_common_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_common_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_search_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_search_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_recognition_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_recognition_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_outofcore_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_outofcore_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_octree_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_octree_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_ml_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_ml_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_sample_consensus_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_sample_consensus_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_filters_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_filters_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_kdtree_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_kdtree_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_segmentation_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_segmentation_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_common_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_search_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_io_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_io_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_keypoints_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_keypoints_debug
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_features_release
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/PC_1.7.1/lib/ -lpcl_features_debug
        CONFIG(release, debug|release):LIBS += -L$$PWD/../../Libs/PC_1.7.1/3rdParty/Boost/lib/ -lboost_thread-vc100-mt-1_49
        else:CONFIG(debug, debug|release):LIBS += -L$$PWD/../../Libs/PC_1.7.1/3rdParty/Boost/lib/ -lboost_thread-vc100-mt-gd-1_49
    }#end win32

}#end pcl

# ==== OpenCV

#if you use opencv
opencv {
    unix{
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_calib3d
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_contrib
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_features2d
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_core
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_flann
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_gpu
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_highgui
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_imgproc
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_legacy
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_ml
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_nonfree
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_objdetect
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_photo
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_stitching
        LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lopencv_video
    }#end unix
    else:win32 {
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_calib3d242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_calib3d242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_contrib242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_contrib242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_core242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_core242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_features2d242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_features2d242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_flann242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_flann242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_gpu242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_gpu242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_highgui242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_highgui242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_imgproc242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_imgproc242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_legacy242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_legacy242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_ml242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_ml242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_nonfree242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_nonfree242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_objdetect242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_objdetect242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_photo242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_photo242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_stitching242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_stitching242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_ts242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_ts242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_video242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_video242d
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_videostab242
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/OpenCV_2.4/lib/ -lopencv_videostab242d
    }#end win32

}#end opencv

# ===== OpenGL
#if you use freeglut and openGL
opengl{
    win32: LIBS += -lQt5OpenGL
    unix{
         LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lGL
         LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lglut
         LIBS +=  -L$$PWD/usr/lib/x86_64-linux-gnu/ -lGLU
    }#end unix
    else:win32 {
        CONFIG(release, debug|release): LIBS += -L$$PWD/../../Libs/freeglut/lib/ -lfreeglut
        else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Libs/freeglut/lib/ -lfreeglutd
    }#end win32
}#end opengl
# -------------------  INCLUDEPATH and DEPENDPATH -------------------
# ===== PCL
#if you use pcl
pcl {
    unix{
        INCLUDEPATH += /usr/include/pcl-1.7
        DEPENDPATH += /usr/include/pcl-1.7
        INCLUDEPATH +=$$PWD /usr/include/eigen3
        DEPENDPATH += $$PWD/usr/include/eigen3
        INCLUDEPATH += $$PWD/usr/include/flann
        DEPENDPATH += $$PWD/usr/include/flann
        INCLUDEPATH += $$PWD/usr/include/boost
        DEPENDPATH += $$PWD/usr/include/boost
    }#end unix
    else:win32 {
        INCLUDEPATH += $$PWD/../../Libs/PC_1.7.1/3rdParty/Boost/include
        DEPENDPATH += $$PWD/../../Libs/PC_1.7.1/3rdParty/Boost/include
        INCLUDEPATH += $$PWD/../../Libs/PC_1.7.1/include/pcl-1.7
        DEPENDPATH += $$PWD/../../Libs/PC_1.7.1/include/pcl-1.7
        INCLUDEPATH += $$PWD/../../Libs/PC_1.7.1/3rdParty/eigen/include
        DEPENDPATH += $$PWD/../../Libs/PC_1.7.1/3rdParty/eigen/include
        INCLUDEPATH += $$PWD/../../Libs/PC_1.7.1/3rdParty/FLANN/include
        DEPENDPATH += $$PWD/../../Libs/PC_1.7.1/3rdParty/FLANN/include
    }#end win32

}#end pcl
# ===== OpenCV
#if you use opencv
opencv {
    unix {
        INCLUDEPATH += $$PWD/usr/include/opencv2
        DEPENDPATH += $$PWD/usr/include/opencv2
        INCLUDEPATH += $$PWD/usr/include/opencv
        DEPENDPATH += $$PWD/usr/include/opencv
    }#end unix
    else:win32 {
        INCLUDEPATH += $$PWD/../../Libs/OpenCV_2.4/lib
        DEPENDPATH += $$PWD/../../Libs/OpenCV_2.4/lib
        INCLUDEPATH += $$PWD/../../Libs/OpenCV_2.4/include
        DEPENDPATH += $$PWD/../../Libs/OpenCV_2.4/include
    }#end win32

}#end opencv

# ===== OpenGL
#if you use freeglut and openGL
opengl{
    unix{
        INCLUDEPATH += /usr/include
        DEPENDPATH += /usr/include
    }#end unix
    else:win32{
        INCLUDEPATH += $$PWD/../../Libs/freeglut/include
        DEPENDPATH += $$PWD/../../Libs/freeglut/include
    }#end win32
}#end opengl
