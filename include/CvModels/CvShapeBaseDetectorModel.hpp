#pragma once

#include <iostream>

#include <QDoubleValidator>
#include <QPushButton>
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
#include "DataTypes/ResultData.hpp"
#include "Line2Dup/line2Dup.h"
#include "Utils/Utils.hpp"
#include "Widget/Full2DDialog.h"

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class CvShapeBaseDetectorModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvShapeBaseDetectorModel();

    ~CvShapeBaseDetectorModel()
    {
        delete _box;
        std::cout << "delete CvShapeBaseDetectorModel" << std::endl;
    };

public:
    QString caption() const override { return QString("ShapeBaseDetector"); }

    QString name() const override { return QString("ShapeBaseDetectorModel:形状匹配检测器"); }

public:
    virtual QString modelName() const { return QString("ShapeBaseDetectorModel"); }

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
    QGroupBox *_box;
    QVBoxLayout *all_lay;
    QLabel *_label;
    QLineEdit *path_show;
    QLineEdit *threshold_edit;

    cv::Mat _mat;
    std::string _res;
    std::shared_ptr<NodeData> _nodeData;
};
