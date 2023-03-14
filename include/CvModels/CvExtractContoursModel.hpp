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

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class CvExtractContoursModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvExtractContoursModel();

    ~CvExtractContoursModel() override
    {
        delete _box;
        std::cout << "delete CvExtractContoursModel" << std::endl;
    };

public:
    QString caption() const override { return QString("ExtractContours"); }

    QString name() const override { return QString("ExtractContours:轮廓提取"); }

public:
    virtual QString modelName() const { return QString("ExtractContours"); }

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
    bool eventFilter(QObject *object, QEvent *event) override{return false;};

private:
    QGroupBox *_box;

    QLineEdit *area_min;
    QLineEdit *area_max;

    std::vector<std::vector<cv::Point>> contours = {};
    std::vector<cv::Vec4i> hierachy = {};
    std::string res;

    cv::Mat _mat;
    std::shared_ptr<NodeData> _inContoursData;
};
