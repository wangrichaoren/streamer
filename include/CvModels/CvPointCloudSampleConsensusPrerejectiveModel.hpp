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

#include <pcl/common/time.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "DataTypes/FPFHSignature33Data.hpp"
#include "DataTypes/Matrix4fData.hpp"
#include "DataTypes/NormalData.hpp"
#include "DataTypes/PointCloudData.hpp"
#include "DataTypes/ResultData.hpp"
#include "Utils/Utils.hpp"
#include "Widget/LoadingDialog.h"
#include <thread>

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;
using NodeId = QtNodes::NodeId;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class CvPointCloudSampleConsensusPrerejectiveModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvPointCloudSampleConsensusPrerejectiveModel();

    ~CvPointCloudSampleConsensusPrerejectiveModel() override
    {
        delete _box;
        loading_dialog->deleteLater();
        std::cout << "delete CvPointCloudSampleConsensusPrerejectiveModel" << std::endl;
    };

public:
    QString caption() const override { return QString("PC SampleConsensusPrerejectiveModel"); }

    QString name() const override
    {
        return QString("SampleConsensusPrerejectiveModel:刚性物体的鲁棒姿态估计");
    }

public:
    virtual QString modelName() const { return QString("SampleConsensusPrerejectiveModel"); }

    unsigned int nPorts(PortType const portType) const override;

    NodeDataType dataType(PortType const portType, PortIndex const portIndex) const override;

    std::shared_ptr<NodeData> outData(PortIndex const port) override;

    void setInData(std::shared_ptr<NodeData>, PortIndex const portIndex) override;

    QWidget *embeddedWidget() override { return _box; }

    bool resizable() const override { return true; }

    QJsonObject save() const override;

    void load(QJsonObject const &) override;

    void compute();

    void calc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr d1,
              pcl::PointCloud<pcl::FPFHSignature33>::Ptr d2,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr d3,
              pcl::PointCloud<pcl::FPFHSignature33>::Ptr d4,
              float nos_v,
              float cr_v,
              float st_v,
              float mcd_v,
              float ift_v,
              int mi_v);

protected:
    bool eventFilter(QObject *object, QEvent *event) override;

private:
    QGroupBox *_box;
    QVBoxLayout *_layout;

    QLineEdit *nos;
    QLineEdit *cr;
    QLineEdit *st;
    QLineEdit *mcd;
    QLineEdit *ift;
    QLineEdit *mi;

    LoadingDialog *loading_dialog;

    std::string _outRes;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _outPc{new pcl::PointCloud<pcl::PointXYZRGB>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _showPc{new pcl::PointCloud<pcl::PointXYZRGB>};
    Eigen::Matrix4f _outMat4f;

    std::shared_ptr<NodeData> _inNodeData = nullptr;
    std::shared_ptr<NodeData> _inFeature = nullptr;

    std::shared_ptr<NodeData> _inTargetNodeData = nullptr;
    std::shared_ptr<NodeData> _inTargetFeature = nullptr;

    std::shared_ptr<NodeData> _inMat4f = nullptr;
};
