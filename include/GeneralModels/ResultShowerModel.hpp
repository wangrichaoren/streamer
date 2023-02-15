#pragma once

#include <iostream>

#include <QDoubleValidator>
#include <QPlainTextEdit>
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
#include "Utils/Utils.hpp"
#include "Widget/Full2DDialog.h"

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class ResultShowerModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    ResultShowerModel();

    ~ResultShowerModel() override
    {
        delete _box;
        std::cout << "delete ResultShowerModel" << std::endl;
    };

public:
    QString caption() const override { return QString("ResultShower"); }

    QString name() const override { return QString("ResultShowerModel:结果"); }

public:
    virtual QString modelName() const { return QString("ResultShowerModel"); }

    unsigned int nPorts(PortType const portType) const override;

    NodeDataType dataType(PortType const portType, PortIndex const portIndex) const override;

    std::shared_ptr<NodeData> outData(PortIndex const port) override { return NULL; };

    void setInData(std::shared_ptr<NodeData>, PortIndex const portIndex) override;

    QWidget *embeddedWidget() override { return _box; };

    bool resizable() const override { return true; }

protected:
    bool eventFilter(QObject *object, QEvent *event) override { return false; };

private:
    QGroupBox *_box;
    QVBoxLayout *all_lay;
    QPlainTextEdit *result_edit;
};
