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
class CvGaussianBlurModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvGaussianBlurModel();

    ~CvGaussianBlurModel() override{delete _box;std::cout<<"delete CvGaussianBlurModel"<<std::endl;};

public:
    QString caption() const override { return QString("GaussianBlur"); }

    QString name() const override { return QString("GaussianBlurModel:高斯滤波"); }

public:
    virtual QString modelName() const { return QString("GaussianBlurModel"); }

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
    QLabel *_label;
    QLineEdit *_kredi;
    QLineEdit *_kcedi;
    QLineEdit *_sigmx;
    QLineEdit *_sigmy;

    cv::Mat _mat;
    QPixmap _q_pix;
    std::shared_ptr<NodeData> _nodeData;
};
