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

#include "DataTypes/ContoursData.hpp"
#include "DataTypes/ImageData.hpp"
#include "Utils/Utils.hpp"
#include "Widget/Full2DDialog.h"

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class CvDrawContoursModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvDrawContoursModel();

    ~CvDrawContoursModel() = default;

public:
    QString caption() const override { return QString("DrawContours"); }

    QString name() const override { return QString("DrawContoursModel:轮廓绘制"); }

public:
    virtual QString modelName() const { return QString("DrawContoursModel"); }

    unsigned int nPorts(PortType const portType) const override;

    NodeDataType dataType(PortType const portType, PortIndex const portIndex) const override;

    std::shared_ptr<NodeData> outData(PortIndex const port) override;

    void setInData(std::shared_ptr<NodeData>, PortIndex const portIndex) override;

    QWidget *embeddedWidget() override { return _box; };

    bool resizable() const override { return true; }

    void compute();

    void load(const QJsonObject &) override;

    QJsonObject save() const override;

protected:
    bool eventFilter(QObject *object, QEvent *event) override;

private:
    QLabel *_label;
    QGroupBox *_box;

    cv::Mat _mat;
    QPixmap _q_pix;
    std::shared_ptr<NodeData> _imageNodeData;
    std::shared_ptr<NodeData> _contoursNodeData;
    bool port0_ok = false;
    bool port1_ok = false;
};
