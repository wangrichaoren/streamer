cmake_minimum_required(VERSION 3.8)

cmake_policy(SET CMP0072 NEW) # new in 3.11. The NEW behavior for this policy is to set OpenGL_GL_PREFERENCE to GLVND.
cmake_policy(SET CMP0068 NEW) # new in 3.9. The NEW behavior of this policy is to ignore the RPATH settings for install_name on macOS.


project(Streamer CXX)

#add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)

if (CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Debug"))
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -O0")
    message("Debug mode:${CMAKE_C_FLAGS_DEBUG}")
elseif (CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Release"))
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall -O3 ")
    message("Release mode:${CMAKE_C_FLAGS_RELEASE}")
endif ()

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(OpenGL_GL_PREFERENCE LEGACY)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

get_directory_property(_has_parent PARENT_DIRECTORY)
if (_has_parent)
    set(is_root_project OFF)
else ()
    set(is_root_project ON)
endif ()

set(QT_NODES_DEVELOPER_DEFAULTS "${is_root_project}" CACHE BOOL "Turns on default settings for development of QtNodes")

#option(BUILD_TESTING "Build tests" "${QT_NODES_DEVELOPER_DEFAULTS}")
#option(BUILD_EXAMPLES "Build Examples" "${QT_NODES_DEVELOPER_DEFAULTS}")
#option(BUILD_DOCS "Build Documentation" "${QT_NODES_DEVELOPER_DEFAULTS}")
option(BUILD_SHARED_LIBS "Build as shared library" ON)
option(BUILD_DEBUG_POSTFIX_D "Append d suffix to debug libraries" OFF)
option(QT_NODES_FORCE_TEST_COLOR "Force colorized unit test output" OFF)

enable_testing()

if (QT_NODES_DEVELOPER_DEFAULTS)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
endif ()

if (BUILD_DEBUG_POSTFIX_D)
    set(CMAKE_DEBUG_POSTFIX "d")
    set(CMAKE_RELEASE_POSTFIX "")
    set(CMAKE_RELWITHDEBINFO_POSTFIX "rd")
    set(CMAKE_MINSIZEREL_POSTFIX "s")
endif ()

add_subdirectory(external)

# QT
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets Gui OpenGL)
message(STATUS "QT_VERSION: ${QT_VERSION}, QT_DIR: ${QT_DIR}")

if (${QT_VERSION} VERSION_LESS 5.11.0)
    message(FATAL_ERROR "Requires qt version >= 5.11.0, Your current version is ${QT_VERSION}")
endif ()

if (${QT_VERSION_MAJOR} EQUAL 6)
    qt_add_resources(RESOURCES ./resources/resources.qrc)
else ()
    qt5_add_resources(RESOURCES ./resources/resources.qrc)
endif ()

# OPENCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
MESSAGE(STATUS "OPENCV_VERSION: ${OpenCV_VERSION}")

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
MESSAGE(STATUS "PCL_VERSION: ${PCL_VERSION}")
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")#不加这个会报错,必须在下面这行之前
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Unfortunately, as we have a split include/src, AUTOMOC doesn't work.
# We'll have to manually specify some files
set(CMAKE_AUTOMOC ON)

set(CPP_SOURCE_FILES
        src/QtNodes/AbstractGraphModel.cpp
        src/QtNodes/AbstractNodeGeometry.cpp
        src/QtNodes/BasicGraphicsScene.cpp
        src/QtNodes/ConnectionGraphicsObject.cpp
        src/QtNodes/ConnectionPainter.cpp
        src/QtNodes/ConnectionState.cpp
        src/QtNodes/ConnectionStyle.cpp
        src/QtNodes/DataFlowGraphModel.cpp
        src/QtNodes/DataFlowGraphicsScene.cpp
        src/QtNodes/DefaultHorizontalNodeGeometry.cpp
        src/QtNodes/DefaultVerticalNodeGeometry.cpp
        src/QtNodes/Definitions.cpp
        src/QtNodes/GraphicsView.cpp
        src/QtNodes/GraphicsViewStyle.cpp
        src/QtNodes/NodeDelegateModelRegistry.cpp
        src/QtNodes/NodeConnectionInteraction.cpp
        src/QtNodes/NodeDelegateModel.cpp
        src/QtNodes/NodeGraphicsObject.cpp
        src/QtNodes/DefaultNodePainter.cpp
        src/QtNodes/NodeState.cpp
        src/QtNodes/NodeStyle.cpp
        src/QtNodes/StyleCollection.cpp
        src/QtNodes/UndoCommands.cpp
        src/QtNodes/locateNode.cpp
        )

set(HPP_HEADER_FILES
        include/QtNodes/internal/AbstractGraphModel.hpp
        include/QtNodes/internal/AbstractNodeGeometry.hpp
        include/QtNodes/internal/AbstractNodePainter.hpp
        include/QtNodes/internal/BasicGraphicsScene.hpp
        include/QtNodes/internal/Compiler.hpp
        include/QtNodes/internal/ConnectionGraphicsObject.hpp
        include/QtNodes/internal/ConnectionIdHash.hpp
        include/QtNodes/internal/ConnectionIdUtils.hpp
        include/QtNodes/internal/ConnectionState.hpp
        include/QtNodes/internal/ConnectionStyle.hpp
        include/QtNodes/internal/DataFlowGraphicsScene.hpp
        include/QtNodes/internal/DataFlowGraphModel.hpp
        include/QtNodes/internal/DefaultNodePainter.hpp
        include/QtNodes/internal/Definitions.hpp
        include/QtNodes/internal/Export.hpp
        include/QtNodes/internal/GraphicsView.hpp
        include/QtNodes/internal/GraphicsViewStyle.hpp
        include/QtNodes/internal/locateNode.hpp
        include/QtNodes/internal/NodeData.hpp
        include/QtNodes/internal/NodeDelegateModel.hpp
        include/QtNodes/internal/NodeDelegateModelRegistry.hpp
        include/QtNodes/internal/NodeGraphicsObject.hpp
        include/QtNodes/internal/NodeState.hpp
        include/QtNodes/internal/NodeStyle.hpp
        include/QtNodes/internal/OperatingSystem.hpp
        include/QtNodes/internal/QStringStdHash.hpp
        include/QtNodes/internal/QUuidStdHash.hpp
        include/QtNodes/internal/Serializable.hpp
        include/QtNodes/internal/Style.hpp
        include/QtNodes/internal/StyleCollection.hpp
        src/QtNodes/ConnectionPainter.hpp
        src/QtNodes/DefaultHorizontalNodeGeometry.hpp
        src/QtNodes/DefaultVerticalNodeGeometry.hpp
        src/QtNodes/NodeConnectionInteraction.hpp
        src/QtNodes/UndoCommands.hpp
        )

# If we want to give the option to build a static library,
# set BUILD_SHARED_LIBS option to OFF
add_library(QtNodes
        ${CPP_SOURCE_FILES}
        ${HPP_HEADER_FILES}
        ${RESOURCES}
        )

add_library(QtNodes::QtNodes ALIAS QtNodes)


target_include_directories(QtNodes
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/QtNodes/internal>
        )

target_link_libraries(QtNodes
        PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::OpenGL
        )

target_compile_definitions(QtNodes
        PUBLIC
        NODE_EDITOR_SHARED
        PRIVATE
        NODE_EDITOR_EXPORTS
        #NODE_DEBUG_DRAWING
        QT_NO_KEYWORDS
        )


target_compile_options(QtNodes
        PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /wd4127 /EHsc /utf-8>
        $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra>
        $<$<CXX_COMPILER_ID:AppleClang>:-Wall -Wextra -Werror>
        )
if (NOT "${CMAKE_CXX_SIMULATE_ID}" STREQUAL "MSVC")
    # Clang-Cl on MSVC identifies as "Clang" but behaves more like MSVC:
    target_compile_options(QtNodes
            PRIVATE
            $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra>
            )
endif ()

if (QT_NODES_DEVELOPER_DEFAULTS)
    target_compile_features(QtNodes PUBLIC cxx_std_14)
    set_target_properties(QtNodes PROPERTIES CXX_EXTENSIONS OFF)
endif ()


set_target_properties(QtNodes
        PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        )

######
# Moc
##

file(GLOB_RECURSE HEADERS_TO_MOC include/QtNodes/internal/*.hpp)

if (${QT_VERSION_MAJOR} EQUAL 6)
    qt_wrap_cpp(nodes_moc
            ${HEADERS_TO_MOC}
            TARGET QtNodes
            OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
            )
else ()
    qt5_wrap_cpp(nodes_moc
            ${HEADERS_TO_MOC}
            TARGET QtNodes
            OPTIONS --no-notes # Don't display a note for the headers which don't produce a moc_*.cpp
            )
endif ()

target_sources(QtNodes PRIVATE ${nodes_moc})

###########
# Examples
##

##if (BUILD_EXAMPLES)
#add_subdirectory(examples)
##endif ()

##################
# Automated Tests
##

#if (BUILD_TESTING)
#    #add_subdirectory(test)
#endif ()

###############
# Installation
##

include(GNUInstallDirs)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/QtNodes)

install(TARGETS QtNodes
        EXPORT QtNodesTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        )

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT QtNodesTargets
        FILE QtNodesTargets.cmake
        NAMESPACE QtNodes::
        DESTINATION ${INSTALL_CONFIGDIR}
        )

include(CMakePackageConfigHelpers)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/QtNodesConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/QtNodesConfig.cmake
        INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
        )

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/QtNodesConfig.cmake
        DESTINATION ${INSTALL_CONFIGDIR}
        )


# header and source

# include MIPP headers / linemod2d
include_directories("include/MIPP/")
set(LINE2DUP_HEADER_FILES
        include/Line2Dup/line2Dup.hpp
        )

set(LINE2DUP_SOURCE_FILES
        src/Line2Dup/line2Dup.cpp
        )

# ----------工具--------------
# ---------------------------
# ---------------------------
set(UTIL_HEADER_FILES
        include/Utils/Utils.hpp
        )

set(UTIL_SOURCE_FILES
        src/Utils/Utils.cpp
        )
# ---------------------------
# ---------------------------
# ---------------------------


# ---------数据类型---------
# -------------------------
# -------------------------
set(HEADLESS_DATA_SOURCE_FILES
        include/DataTypes/DecimalData.hpp
        include/DataTypes/ImageData.hpp
        include/DataTypes/StringData.hpp
        include/DataTypes/PixmapData.hpp
        include/DataTypes/ContoursData.hpp
        include/DataTypes/ResultData.hpp
        include/DataTypes/PointCloudData.hpp
        include/DataTypes/NormalData.hpp
        include/DataTypes/Matrix4fData.hpp
        include/DataTypes/FPFHSignature33Data.hpp
        )
# ------------------------
# ------------------------
# ------------------------


# -------------------通用模型-------------------
# ---------------------------------------------
# ---------------------------------------------
# 通用模型头文件
set(GENERAL_MODEL_HEADER_FILES
        include/GeneralModels/AdditionModel.hpp
        include/GeneralModels/DivisionModel.hpp
        include/GeneralModels/SubtractionModel.hpp

        include/GeneralModels/MathOperationDataModel.hpp
        include/GeneralModels/MultiplicationModel.hpp
        include/GeneralModels/NumberDisplayDataModel.hpp
        include/GeneralModels/NumberSourceDataModel.hpp
        include/GeneralModels/ImageLoaderModel.hpp
        include/GeneralModels/ImageShowModel.hpp
        include/GeneralModels/ResultShowerModel.hpp
        include/GeneralModels/MakeMat4fModel.hpp
        )

# 通用模型源文件
set(GENERAL_MODEL_SOURCE_FILES
        src/GeneralModels/MathOperationDataModel.cpp
        src/GeneralModels/NumberDisplayDataModel.cpp
        src/GeneralModels/NumberSourceDataModel.cpp
        src/GeneralModels/ImageLoaderModel.cpp
        src/GeneralModels/ImageShowModel.cpp
        src/GeneralModels/ResultShowerModel.cpp
        src/GeneralModels/MakeMat4fModel.cpp
        )
# -------------------------------------------
# -------------------------------------------
# -------------------------------------------


# -----------------视觉模型-----------------
# ----------------------------------------
# ----------------------------------------
# 视觉模型头文件
set(CV_MODEL_HEADER_FILES
        # 二维
        include/CvModels/CvImageLoaderModel.hpp
        include/CvModels/CvImageShowModel.hpp
        include/CvModels/CvRGB2GrayModel.hpp
        include/CvModels/CvBinaryModel.hpp
        include/CvModels/CvMorphModel.hpp
        include/CvModels/CvFilter2dModel.hpp
        include/CvModels/CvBlurModel.hpp
        include/CvModels/CvMedianBlurModel.hpp
        include/CvModels/CvGaussianBlurModel.hpp
        include/CvModels/CvEdgeModel.hpp
        include/CvModels/CvFindContoursModel.hpp
        include/CvModels/CvDrawContoursModel.hpp
        include/CvModels/CvShapeBaseDetectorModel.hpp

        # 三维
        include/CvModels/CvPointCloudLoaderModel.hpp
        include/CvModels/CvPointCloudPassThroughModel.hpp
        include/CvModels/CvPointCloudDownsampleVoxelGridModel.hpp
        include/CvModels/CvPointCloudStatisticalOutlierRemovalModel.hpp
        include/CvModels/CvPointCloudSampleConsensusModel.hpp
        include/CvModels/CvPointCloudNormalEstimationModel.hpp
        include/CvModels/CvPointCloudNormalEstimationOMPModel.hpp
        include/CvModels/CvPointCloudApproximateVoxelFilterModel.hpp
        include/CvModels/CvPointCloudNDTModel.hpp
        include/CvModels/CvPointCloudFPFHEstimationOMPModel.hpp
        include/CvModels/CvPointCloudSampleConsensusPrerejectiveModel.hpp
        include/CvModels/CvPointIaRansacModel.hpp
        include/CvModels/CvPointCloudTransformModel.hpp

        )

# 视觉模型源文件
set(CV_MODEL_SOURCE_FILES
        # 二维
        src/CvModels/CvImageLoaderModel.cpp
        src/CvModels/CvImageShowModel.cpp
        src/CvModels/CvRGB2GrayModel.cpp
        src/CvModels/CvBinaryModel.cpp
        src/CvModels/CvMorphModel.cpp
        src/CvModels/CvFilter2dModel.cpp
        src/CvModels/CvBlurModel.cpp
        src/CvModels/CvMedianBlurModel.cpp
        src/CvModels/CvGaussianBlurModel.cpp
        src/CvModels/CvEdgeModel.cpp
        src/CvModels/CvFindContoursModel.cpp
        src/CvModels/CvDrawContoursModel.cpp
        src/CvModels/CvShapeBaseDetectorModel.cpp

        # 三维
        src/CvModels/CvPointCloudLoaderModel.cpp
        src/CvModels/CvPointCloudPassThroughModel.cpp
        src/CvModels/CvPointCloudDownsampleVoxelGridModel.cpp
        src/CvModels/CvPointCloudStatisticalOutlierRemovalModel.cpp
        src/CvModels/CvPointCloudSampleConsensusModel.cpp
        src/CvModels/CvPointCloudNormalEstimationModel.cpp
        src/CvModels/CvPointCloudNormalEstimationOMPModel.cpp
        src/CvModels/CvPointCloudApproximateVoxelFilterModel.cpp
        src/CvModels/CvPointCloudNDTModel.cpp
        src/CvModels/CvPointCloudFPFHEstimationOMPModel.cpp
        src/CvModels/CvPointCloudSampleConsensusPrerejectiveModel.cpp
        src/CvModels/CvPointCloudIaRansacModel.cpp
        src/CvModels/CvPointCloudTransformModel.cpp

        )
# ---------------------------------------
# ---------------------------------------
# ---------------------------------------


# --------------外部控件------------------
set(WIDGET_UI_HEADER_FILES
        include/Widget/ui_StreamerMainWindow.h
        include/Widget/StreamerMainWindow.ui
        include/Widget/StreamerMainWindow.hpp

        include/Widget/MyGraphicsView.h

        include/Widget/ui_Full2DDialog.h
        include/Widget/Full2DDialog.ui
        include/Widget/Full2DDialog.h

        include/Widget/ui_ShapeBaseTrainer.h
        include/Widget/ShapeBaseTrainer.ui
        include/Widget/ShapeBaseTrainer.h

        include/Widget/ui_PCViewer.h
        include/Widget/PCViewer.ui
        include/Widget/PCViewer.h

        include/Widget/LoadingDialog.h
        )

set(WIDGET_UI_SOURCE_FILES
        src/Widget/StreamerMainWindow.cpp

        src/Widget/MyGraphicsView.cpp

        src/Widget/Full2DDialog.cpp

        src/Widget/ShapeBaseTrainer.cpp

        src/Widget/PCViewer.cpp

        src/Widget/LoadingDialog.cpp
        )
# ---------------------------------------
# ---------------------------------------
# ---------------------------------------


# 主程序
set(MAIN_SOURCE_FILE
        main.cpp
        )

# UTIL_HEADER_FILES UTIL_SOURCE_FILES
# HEADLESS_DATA_SOURCE_FILES
# GENERAL_MODEL_HEADER_FILES GENERAL_MODEL_SOURCE_FILES
# CV_MODEL_HEADER_FILES CV_MODEL_SOURCE_FILES
add_executable(streamer
        ${MAIN_SOURCE_FILE}

        ${LINE2DUP_HEADER_FILES}
        ${LINE2DUP_SOURCE_FILES}

        ${UTIL_HEADER_FILES}
        ${UTIL_SOURCE_FILES}

        ${HEADLESS_DATA_SOURCE_FILES}

        ${GENERAL_MODEL_HEADER_FILES}
        ${GENERAL_MODEL_SOURCE_FILES}

        ${CV_MODEL_HEADER_FILES}
        ${CV_MODEL_SOURCE_FILES}

        ${WIDGET_UI_HEADER_FILES}
        ${WIDGET_UI_SOURCE_FILES}
        )

target_link_libraries(streamer QtNodes ${OpenCV_LIBS} ${PCL_LIBRARIES})
