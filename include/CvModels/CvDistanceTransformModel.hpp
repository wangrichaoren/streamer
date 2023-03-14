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
#include "DataTypes/ResultData.hpp"
#include "Utils/Utils.hpp"
#include "Widget/Full2DDialog.h"
#include <QComboBox>


using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class CvDistanceTransformModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvDistanceTransformModel();

    ~CvDistanceTransformModel() override
    {
        delete _box;
        std::cout << "delete CvDistanceTransformModel" << std::endl;
    };

public:
    QString caption() const override { return QString("DistanceTransform"); }

    QString name() const override { return QString("DistanceTransform:距离变换"); }

public:
    virtual QString modelName() const { return QString("DistanceTransform"); }

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
    bool eventFilter(QObject *object, QEvent *event) override { return false; };

private:
    QGroupBox *_box;

    QRadioButton *l1;
    QRadioButton *l2;
    QRadioButton *l3;
    QRadioButton *l4;
    QRadioButton *l5;
    QRadioButton *l6;
    QRadioButton *l7;

    QComboBox *mask_size_combox;

        cv::Mat _mat;
    std::string res;
    std::shared_ptr<NodeData> _nodeData;
};
