#pragma once

#include <QtNodes/NodeDelegateModel>

#include <QtCore/QObject>
#include <QtWidgets/QLabel>

#include "MathOperationDataModel.hpp"

#include "DecimalData.hpp"

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class MultiplicationModel : public MathOperationDataModel
{
public:
    virtual ~MultiplicationModel() {}

public:
    QString caption() const override { return QStringLiteral("Multiplication Number"); }

    QString name() const override { return QStringLiteral("MultiplicationNumber:数字乘法运算"); }

private:
    void compute() override
    {
        PortIndex const outPortIndex = 0;

        auto n1 = _number1.lock();
        auto n2 = _number2.lock();

        if (n1 && n2) {
            //modelValidationState = NodeValidationState::Valid;
            //modelValidationError = QString();
            _result = std::make_shared<DecimalData>(n1->number() * n2->number());
        } else {
            //modelValidationState = NodeValidationState::Warning;
            //modelValidationError = QStringLiteral("Missing or incorrect inputs");
            _result.reset();
        }

        Q_EMIT dataUpdated(outPortIndex);
    }
};
