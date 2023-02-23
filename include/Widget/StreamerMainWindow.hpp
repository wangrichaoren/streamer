//
// Created by wrc on 23-2-22.
//

#ifndef STREAMER_STREAMERMAINWINDOW_HPP
#define STREAMER_STREAMERMAINWINDOW_HPP

#pragma once

#include <QMainWindow>

#include <QtNodes/ConnectionStyle>
#include <QtNodes/DataFlowGraphModel>
#include <QtNodes/DataFlowGraphicsScene>
#include <QtNodes/GraphicsView>
#include <QtNodes/GraphicsViewStyle>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtNodes/NodeStyle>

#include <memory>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>

#include "Utils/Utils.hpp"
#include "Widget/PCViewer.h"
#include "Widget/ShapeBaseTrainer.h"

#include "GeneralModels/AdditionModel.hpp"
#include "GeneralModels/DivisionModel.hpp"
#include "GeneralModels/MultiplicationModel.hpp"
#include "GeneralModels/NumberDisplayDataModel.hpp"
#include "GeneralModels/NumberSourceDataModel.hpp"
#include "GeneralModels/ResultShowerModel.hpp"
#include "GeneralModels/SubtractionModel.hpp"

#include "CvModels/CvBinaryModel.hpp"
#include "CvModels/CvBlurModel.hpp"
#include "CvModels/CvDrawContoursModel.hpp"
#include "CvModels/CvEdgeModel.hpp"
#include "CvModels/CvFilter2dModel.hpp"
#include "CvModels/CvFindContoursModel.hpp"
#include "CvModels/CvGaussianBlurModel.hpp"
#include "CvModels/CvImageLoaderModel.hpp"
#include "CvModels/CvImageShowModel.hpp"
#include "CvModels/CvMedianBlurModel.hpp"
#include "CvModels/CvMorphModel.hpp"
#include "CvModels/CvPointCloudLoaderModel.hpp"
#include "CvModels/CvPointCloudPassThroughModel.hpp"
#include "CvModels/CvRGB2GrayModel.hpp"
#include "CvModels/CvShapeBaseDetectorModel.hpp"
#include "CvModels/CvPointCloudDownsampleVoxelGridModel.hpp"
#include "CvModels/CvPointCloudStatisticalOutlierRemovalModel.hpp"

using QtNodes::ConnectionStyle;
using QtNodes::DataFlowGraphicsScene;
using QtNodes::DataFlowGraphModel;
using QtNodes::GraphicsView;
using QtNodes::GraphicsViewStyle;
using QtNodes::NodeDelegateModelRegistry;
using QtNodes::NodeFlag;
using QtNodes::NodeStyle;

QT_BEGIN_NAMESPACE
namespace Ui {
class StreamerMainWindow;
}
QT_END_NAMESPACE

class StreamerMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit StreamerMainWindow(QWidget *parent = nullptr);

    ~StreamerMainWindow() override;

    void setupStyle()
    {
        GraphicsViewStyle::setStyle(
            R"(
      {
        "GraphicsViewStyle": {
          "BackgroundColor": [255, 255, 240],
          "FineGridColor": [245, 245, 230],
          "CoarseGridColor": [235, 235, 220]
        }
      }
      )");

        NodeStyle::setNodeStyle(
            R"(
      {
        "NodeStyle": {
          "NormalBoundaryColor": "darkgray",
          "SelectedBoundaryColor": "dark",
          "GradientColor0": "mintcream",
          "GradientColor1": "mintcream",
          "GradientColor2": "mintcream",
          "GradientColor3": "mintcream",
          "ShadowColor": [150, 150, 150],
          "FontColor": [10, 10, 10],
          "FontColorFaded": [100, 100, 100],
          "ConnectionPointColor": "white",
          "PenWidth": 2.0,
          "HoveredPenWidth": 2.5,
          "ConnectionPointDiameter": 10.0,
          "Opacity": 1.0
        }
      }
      )");

        ConnectionStyle::setConnectionStyle(
            R"(
  {
    "ConnectionStyle": {
      "ConstructionColor": "gray",
      "NormalColor": "black",
      "SelectedColor": "gray",
      "SelectedHaloColor": "deepskyblue",
      "HoveredColor": "deepskyblue",

      "LineWidth": 3.0,
      "ConstructionLineWidth": 2.0,
      "PointDiameter": 10.0,

      "UseDataDefinedColors": true
    }
  }
  )");
    }

    std::shared_ptr<NodeDelegateModelRegistry> registerDataModels()
    {
        auto ret = std::make_shared<NodeDelegateModelRegistry>();

        // 通用模型
        ret->registerModel<NumberSourceDataModel>("通用");
        ret->registerModel<NumberDisplayDataModel>("通用");
        ret->registerModel<AdditionModel>("通用");
        ret->registerModel<SubtractionModel>("通用");
        ret->registerModel<MultiplicationModel>("通用");
        ret->registerModel<DivisionModel>("通用");
        ret->registerModel<ResultShowerModel>("通用");

        // 2d模型
        ret->registerModel<CvImageLoaderModel>("2D");
        ret->registerModel<CvImageShowModel>("2D");
        ret->registerModel<CvRGB2GrayModel>("2D");
        ret->registerModel<CvBinaryModel>("2D");
        ret->registerModel<CvMorphModel>("2D");
        ret->registerModel<CvFilter2dModel>("2D");
        ret->registerModel<CvBlurModel>("2D");
        ret->registerModel<CvMedianBlurModel>("2D");
        ret->registerModel<CvGaussianBlurModel>("2D");
        ret->registerModel<CvEdgeModel>("2D");
        ret->registerModel<CvFindContoursModel>("2D");
        ret->registerModel<CvDrawContoursModel>("2D");
        ret->registerModel<CvShapeBaseDetectorModel>("2D");

        // 3d模型
        ret->registerModel<CvPointCloudLoaderModel>("3D");
        ret->registerModel<CvPointCloudPassThroughModel>("3D");
        ret->registerModel<CvPointCloudDownSampleVoxelGridModel>("3D");
        ret->registerModel<CvPointCloudStatisticalOutlierRemovalModel>("3D");

        return ret;
    }

    bool eventFilter(QObject *watched, QEvent *event) override;

    void initialVtkWidget();

signals:
    void updateVTK(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc);

private slots:
    void VtkRender(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc);

private:
    DataFlowGraphModel *data_flow_graphics_model;
    DataFlowGraphicsScene *scene;
    GraphicsView *view;
    QWidget *view_mask;
    pcl::visualization::PCLVisualizer *pcl_viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc{new pcl::PointCloud<pcl::PointXYZRGB>};

    bool _lock_state = false;
    bool _pc_display_state = false;

    QPushButton *open_flow_btn;
    QPushButton *new_flow_btn;
    QPushButton *save_flow_btn;
    QPushButton *lock_btn;
    QPushButton *pcl_viewer_btn;
    Ui::StreamerMainWindow *ui;
};

#endif //STREAMER_STREAMERMAINWINDOW_HPP
