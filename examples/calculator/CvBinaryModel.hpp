#pragma once

#include <iostream>

#include <QtCore/QObject>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>

#include <QtNodes/NodeDelegateModel>
#include <QtNodes/NodeDelegateModelRegistry>

#include "ImageData.hpp"
#include "Utils.hpp"

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class CvBinaryModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvBinaryModel();

    ~CvBinaryModel() = default;

public:
    QString caption() const override { return QString("Binary Image"); }

    QString name() const override { return QString("BinaryImage:二值化"); }

public:
    virtual QString modelName() const { return QString("BinaryImage"); }

    unsigned int nPorts(PortType const portType) const override;

    NodeDataType dataType(PortType const portType, PortIndex const portIndex) const override;

    std::shared_ptr<NodeData> outData(PortIndex const port) override;

    void setInData(std::shared_ptr<NodeData>, PortIndex const portIndex) override ;

    QWidget *embeddedWidget() override { return _label; }

    bool resizable() const override { return true; }

protected:
    bool eventFilter(QObject *object, QEvent *event) override;

private:
    QLabel *_label;
    cv::Mat _mat;
    QPixmap _q_pix;
    std::shared_ptr<NodeData> _nodeData;
};
