#include "GeneralModels/ResultShowerModel.hpp"
#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

ResultShowerModel::ResultShowerModel()
    : _box(new QGroupBox())
{
    all_lay = new QVBoxLayout(_box);
    auto bf = _box->font();
    bf.setBold(true);
    _box->setFont(bf);

    result_edit = new QPlainTextEdit(_box);
    all_lay->addWidget(result_edit);
    result_edit->setReadOnly(true);
    _box->setLayout(all_lay);
    _box->setMinimumSize(200, 200);
    _box->resize(200, 300);
}

unsigned int ResultShowerModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 1;
        break;
    case PortType::Out:
        result = 0;
        break;
    default:
        break;
    };

    return result;
}

NodeDataType ResultShowerModel::dataType(PortType const portType, PortIndex const portIndex) const
{
    if (portType == PortType::In) {
        return ResultData().type();
    } else {
        return {};
    }
}

void ResultShowerModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const portIndex)
{
    if (portIndex == 0) {
        if (nodeData) {
            auto res = std::dynamic_pointer_cast<ResultData>(nodeData);
            auto show_res = res->getData();
            if (show_res.empty()) {
                return;
            }
            result_edit->setPlainText(QString(show_res.data()));
        } else {
            result_edit->clear();
            return;
        }
    }
}
