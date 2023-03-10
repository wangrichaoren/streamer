#pragma once

#include <iostream>

#include <QtCore/QObject>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

#include "../include/Widget/Full2DDialog.h"
#include "Utils/Utils.hpp"
#include <opencv2/opencv.hpp>
#include <QtNodes/NodeDelegateModel>
#include <QtNodes/NodeDelegateModelRegistry>

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class CvImageShowModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvImageShowModel();

    ~CvImageShowModel() override
    {
        delete _box;
        std::cout << "delete CvImageShowModel" << std::endl;
    };

public:
    QString caption() const override { return QString("Image Display"); }

    QString name() const override { return QString("ImageShowModel:图像显示器"); }

public:
    virtual QString modelName() const { return QString("ImageShowModel"); }

    unsigned int nPorts(PortType const portType) const override;

    NodeDataType dataType(PortType const portType, PortIndex const portIndex) const override;

    std::shared_ptr<NodeData> outData(PortIndex const port) override;

    void setInData(std::shared_ptr<NodeData> nodeData, PortIndex const port) override;

    QWidget *embeddedWidget() override { return _box; }

    bool resizable() const override { return true; }

protected:
    bool eventFilter(QObject *object, QEvent *event) override;

private:
    QGroupBox *_box;
    QLabel *_label;
    QPushButton *_saveBtn;
    QVBoxLayout *_layout;

    cv::Mat _mat;
    std::shared_ptr<NodeData> _nodeData;
};
