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
class CvBinaryModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvBinaryModel();

    ~CvBinaryModel() override
    {
        delete _box;
        std::cout << "delete CvBinaryModel" << std::endl;
    };

public:
    QString caption() const override { return QString("Binary"); }

    QString name() const override { return QString("BinaryModel:二值化"); }

public:
    virtual QString modelName() const { return QString("BinaryModel"); }

    unsigned int nPorts(PortType const portType) const override;

    NodeDataType dataType(PortType const portType, PortIndex const portIndex) const override;

    std::shared_ptr<NodeData> outData(PortIndex const port) override;

    void setInData(std::shared_ptr<NodeData>, PortIndex const portIndex) override;

    QWidget *embeddedWidget() override { return _box; };

    bool resizable() const override { return true; }

    void compute();

    QJsonObject save() const override;

    void load(QJsonObject const &) override;

protected:
    bool eventFilter(QObject *object, QEvent *event) override;

private:
    QGroupBox *_box;
    QLabel *_label;
    QLineEdit *_thresh_val;
    QLineEdit *_max_val;
    QVBoxLayout *_vlayout;
    QRadioButton *_binary_r;
    QRadioButton *_tozero_r;
    QRadioButton *_trunc_r;
    QRadioButton *_ostu_r;
    QRadioButton *_inv_r;
    QRadioButton *_not_inv_r;

    cv::Mat _mat;
    QPixmap _q_pix;
    std::shared_ptr<NodeData> _nodeData;
};
