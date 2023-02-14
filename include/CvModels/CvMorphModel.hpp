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
class CvMorphModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    CvMorphModel();

    ~CvMorphModel() override{delete _box;std::cout<<"delete CvMorphModel"<<std::endl;};

public:
    QString caption() const override { return QString("Morph"); }

    QString name() const override { return QString("MorphModel:形态学"); }

public:
    virtual QString modelName() const { return QString("MorphModel"); }

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
    QLineEdit *_row_edit;
    QLineEdit *_col_edit;

    QRadioButton *_cross_R;
    QRadioButton *_ellipse_R;

    QRadioButton *_dilation_R;
    QRadioButton *_erosion_R;
    QRadioButton *_open_R;
    QRadioButton *_close_R;

    cv::Mat _mat;
    QPixmap _q_pix;
    std::shared_ptr<NodeData> _nodeData;
};
