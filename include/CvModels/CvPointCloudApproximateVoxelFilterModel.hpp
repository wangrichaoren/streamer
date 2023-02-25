#pragma once

#include <iostream>

#include <vtkRenderWindow.h>
#include <QPushButton>
#include <QVTKWidget.h>
#include <QtCore/QObject>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>
#include <QRadioButton>

#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <QTimer>

#include <QtNodes/NodeDelegateModel>
#include <QtNodes/NodeDelegateModelRegistry>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/approximate_voxel_grid.h>

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
class CvPointCloudApproximateVoxelFilterModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvPointCloudApproximateVoxelFilterModel();

    ~CvPointCloudApproximateVoxelFilterModel() override
    {
        delete _box;
        std::cout << "delete CvPointCloudApproximateVoxelFilterModel" << std::endl;
    };

public:
    QString caption() const override { return QString("PC ApproximateVoxelFilterModel"); }

    QString name() const override { return QString("ApproximateVoxelFilterModel:近似体素过滤器"); }

public:
    virtual QString modelName() const { return QString("ApproximateVoxelFilterModel"); }

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
    QLineEdit *x;
    QLineEdit *y;
    QLineEdit *z;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pc{new pcl::PointCloud<pcl::PointXYZRGB>};
    std::string _res;
    std::shared_ptr<NodeData> _nodeData;
};
