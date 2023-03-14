#include "../include/CvModels/CvExtractContoursModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include <QComboBox>
#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvExtractContoursModel::CvExtractContoursModel()
    : _box(new QGroupBox())
{
    auto bf = _box->font();
    bf.setBold(true);
    _box->setFont(bf);

    auto com_group = new QGroupBox(_box);
    auto com_lay = new QVBoxLayout(_box);
    auto com = new QComboBox(_box);
    com_lay->addWidget(com);
    com_group->setLayout(com_lay);

    // todo 其他方式待添加
    QStringList com_list;
    com_list << "按面积条件"
             << "按父子关系";
    com->addItems(com_list);

    // area option
    auto area_group = new QGroupBox(_box);
    auto area_lay = new QHBoxLayout(_box);
    auto area_label = new QLabel("~", _box);
    area_min = new QLineEdit("0", _box);
    area_max = new QLineEdit("10000", _box);
    area_lay->addWidget(area_min);
    area_lay->addWidget(area_label);
    area_lay->addWidget(area_max);
    area_group->setLayout(area_lay);

    auto all_lay = new QVBoxLayout(_box);
    all_lay->addWidget(com_group);
    all_lay->addWidget(area_group);

    _box->setLayout(all_lay);

    _box->resize(200, 200);

    connect(area_min, &QLineEdit::editingFinished, [=] { compute(); });
    connect(area_max, &QLineEdit::editingFinished, [=] { compute(); });
}

unsigned int CvExtractContoursModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 1;
        break;

    case PortType::Out:
        result = 2;

    default:
        break;
    };

    return result;
}

NodeDataType CvExtractContoursModel::dataType(PortType const portType,
                                              PortIndex const portIndex) const
{
    switch (portType) {
    case PortType::In:
        return ContoursData().type();

    case PortType::Out:
        switch (portIndex) {
        case 0:
            return ContoursData().type();
        case 1:
            return ResultData().type();
        }

    case PortType::None:
        break;
    }
    // FIXME: control may reach end of non-void function [-Wreturn-type]
    return NodeDataType();
}

std::shared_ptr<NodeData> CvExtractContoursModel::outData(PortIndex p)
{
    if (p == 0) {
        ContoursDataStruct dataStruct;
        dataStruct.contours = contours;
        dataStruct.hierachy = hierachy;
        ContoursData data(dataStruct);
        return std::make_shared<ContoursData>(data);
    } else if (p == 1) {
        return std::make_shared<ResultData>(res);
    }
}

void CvExtractContoursModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const portIndex)
{
    if (nodeData) {
        _inContoursData = nodeData;
    } else {
        _inContoursData = nullptr;
    }
    compute();
}
void CvExtractContoursModel::compute()
{
    auto d = std::dynamic_pointer_cast<ContoursData>(_inContoursData);
    if (!d) {
        return;
    }

    bool f;
    auto area_min_val = area_min->text().toFloat(&f);
    auto area_max_val = area_max->text().toFloat(&f);
    if (!f) {
        return;
    }

    auto _contours = d->getData().contours;
    auto _hierachy = d->getData().hierachy;

    vector<vector<cv::Point_<int>>> new_contours;
    for (auto c : _contours) {
        auto area = cv::contourArea(c);
        if (area_min_val <= area && area <= area_max_val) {
            new_contours.push_back(c);
        }
    }
    contours = new_contours;
    hierachy = _hierachy;
    //    ContoursDataStruct s;
    //    s.contours = new_contours;
    //    s.hierachy = hierachy;
    Q_EMIT dataUpdated(0);
}
void CvExtractContoursModel::load(const QJsonObject &s)
{
    area_min->setText(s["min"].toString());
    area_max->setText(s["max"].toString());
}

QJsonObject CvExtractContoursModel::save() const
{
    auto s = NodeDelegateModel::save();
    s["min"] = area_min->text();
    s["max"] = area_max->text();
    return s;
}