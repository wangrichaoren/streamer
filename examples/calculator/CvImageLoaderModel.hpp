#pragma once

#include <iostream>

#include <QtCore/QObject>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>

#include <QtNodes/NodeDelegateModel>
#include <QtNodes/NodeDelegateModelRegistry>

#include "ImageData.hpp"

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class CvImageLoaderModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvImageLoaderModel();

    ~CvImageLoaderModel() = default;

public:
    QString caption() const override { return QString("Cv Image Source"); }

    QString name() const override { return QString("CvImageLoaderModel:Mat格式图像加载"); }

public:
    virtual QString modelName() const { return QString("Cv Source Image"); }

    unsigned int nPorts(PortType const portType) const override;

    NodeDataType dataType(PortType const portType, PortIndex const portIndex) const override;

    std::shared_ptr<NodeData> outData(PortIndex const port) override;

    void setInData(std::shared_ptr<NodeData>, PortIndex const portIndex) override {}

    QWidget *embeddedWidget() override { return _box; }

    bool resizable() const override { return true; }

    QImage MatToQImage(const cv::Mat &mat);

protected:
    bool eventFilter(QObject *object, QEvent *event) override;

private:
    QLabel *_label;
    QLineEdit *_path_lineedit;
    QVBoxLayout *_layout;
    QGroupBox *_box;

    cv::Mat _mat;

    QPixmap _q_pix;
};
