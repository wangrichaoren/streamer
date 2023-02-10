#pragma once

#include <iostream>

#include <QDoubleValidator>
#include <QtCore/QObject>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>

#include <QtNodes/NodeDelegateModel>
#include <QtNodes/NodeDelegateModelRegistry>

#include "DataTypes/ImageData.hpp"
#include "Utils/Utils.hpp"

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class CvFilter2dModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvFilter2dModel();

    ~CvFilter2dModel() = default;

public:
    QString caption() const override { return QString("Filter2d"); }

    QString name() const override { return QString("Filter2dModel:2d濾波器"); }

public:
    virtual QString modelName() const { return QString("Filter2dModel"); }

    unsigned int nPorts(PortType const portType) const override;

    NodeDataType dataType(PortType const portType, PortIndex const portIndex) const override;

    std::shared_ptr<NodeData> outData(PortIndex const port) override;

    void setInData(std::shared_ptr<NodeData>, PortIndex const portIndex) override;

    QWidget *embeddedWidget() override { return _box; };

    bool resizable() const override { return true; }

    void compute();

protected:
    bool eventFilter(QObject *object, QEvent *event) override;

private:
    QLabel *_label;
    QGroupBox *_box;
    QLineEdit *dep_edit;
    QLineEdit *n00;
    QLineEdit *n01;
    QLineEdit *n02;
    QLineEdit *n10;
    QLineEdit *n11;
    QLineEdit *n12;
    QLineEdit *n20;
    QLineEdit *n21;
    QLineEdit *n22;

    cv::Mat _mat;
    QPixmap _q_pix;
    std::shared_ptr<NodeData> _nodeData;
};
