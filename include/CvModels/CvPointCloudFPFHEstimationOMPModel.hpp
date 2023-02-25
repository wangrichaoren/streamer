#pragma once

#include <iostream>

#include <vtkRenderWindow.h>
#include <QPushButton>
#include <QRadioButton>
#include <QVTKWidget.h>
#include <QtCore/QObject>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>

#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <QTimer>

#include <QtNodes/NodeDelegateModel>
#include <QtNodes/NodeDelegateModelRegistry>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "DataTypes/FPFHSignature33Data.hpp"
#include "DataTypes/Matrix4fData.hpp"
#include "DataTypes/NormalData.hpp"
#include "DataTypes/PointCloudData.hpp"
#include "DataTypes/ResultData.hpp"
#include "Utils/Utils.hpp"

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;
using NodeId = QtNodes::NodeId;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class CvPointCloudFPFHEstimationOMPModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvPointCloudFPFHEstimationOMPModel();

    ~CvPointCloudFPFHEstimationOMPModel() override
    {
        delete _box;
        std::cout << "delete CvPointCloudFPFHEstimationOMPModel" << std::endl;
    };

public:
    QString caption() const override { return QString("PC FPFHEstimationOMPModel"); }

    QString name() const override
    {
        return QString("FPFHEstimationOMPModel:快速点特征直方图描述子");
    }

public:
    virtual QString modelName() const { return QString("FPFHEstimationOMPModel"); }

    unsigned int nPorts(PortType const portType) const override;

    NodeDataType dataType(PortType const portType, PortIndex const portIndex) const override;

    std::shared_ptr<NodeData> outData(PortIndex const port) override;

    void setInData(std::shared_ptr<NodeData>, PortIndex const portIndex) override;

    QWidget *embeddedWidget() override { return _box; }

    bool resizable() const override { return true; }

    QJsonObject save() const override;

    void load(QJsonObject const &) override;

    void compute();

protected:
    bool eventFilter(QObject *object, QEvent *event) override;

private:
    QGroupBox *_box;
    QVBoxLayout *_layout;
    QLineEdit *r;

    std::string _outRes;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _outPc{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr _outFeature{
        new pcl::PointCloud<pcl::FPFHSignature33>};

    std::shared_ptr<NodeData> _inNodeData = nullptr;
    std::shared_ptr<NodeData> _inNormal = nullptr;
};
