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
class CvEdgeModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvEdgeModel();

    ~CvEdgeModel() = default;

public:
    QString caption() const override { return QString("Edge"); }

    QString name() const override { return QString("EdgeModel:边缘提取"); }

public:
    virtual QString modelName() const { return QString("EdgeModel"); }

    unsigned int nPorts(PortType const portType) const override;

    NodeDataType dataType(PortType const portType, PortIndex const portIndex) const override;

    std::shared_ptr<NodeData> outData(PortIndex const port) override;

    void setInData(std::shared_ptr<NodeData>, PortIndex const portIndex) override;

    QWidget *embeddedWidget() override { return _box; };

    bool resizable() const override { return true; }

    void compute();

    void load(const QJsonObject &) override;

    QJsonObject save() const override;

    void createCannyGroupBox();

    void createLaplacianGroupBox();

    void createSobelGroupBox();

protected:
    bool eventFilter(QObject *object, QEvent *event) override;

private:
    QLabel *_label;
    QGroupBox *_box;

    QRadioButton *canny_radio;
    QLineEdit *threshold1_canny;
    QLineEdit *threshold2_canny;
    QLineEdit *aperture_size_canny;
    QRadioButton *l2gradient_canny_true;
    QRadioButton *l2gradient_canny_false;


    QRadioButton *laplacian_radio;
    QLineEdit *ddepth_laplacian;
    QLineEdit *ksize_laplacian;
    QLineEdit *scale_laplacian;
    QLineEdit *delta_laplacian;

    QRadioButton *sobel_radio;
    QLineEdit *ddepth_sobel;
    QLineEdit *ksize_sobel;
    QLineEdit *dx_sobel;
    QLineEdit *dy_sobel;
    QLineEdit *scale_sobel;
    QLineEdit *delta_sobel;

    QGroupBox *laplacian_group;
    QGroupBox *sobel_group;
    QGroupBox *canny_group;

    QVBoxLayout *all_lay;

    cv::Mat _mat;
    QPixmap _q_pix;
    std::shared_ptr<NodeData> _nodeData;
};
