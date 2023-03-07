#include "CvModels/CvPointCloudPassThroughModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudPassThroughModel::CvPointCloudPassThroughModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    auto name_group = new QGroupBox("FilterFieldName", _box);
    auto radio_lay = new QHBoxLayout(_box);
    x = new QRadioButton("x", _box);
    y = new QRadioButton("y", _box);
    z = new QRadioButton("z", _box);
    radio_lay->addWidget(x);
    radio_lay->addWidget(y);
    radio_lay->addWidget(z);
    name_group->setLayout(radio_lay);
    x->setChecked(true);

    auto negative_group = new QGroupBox("Negative", _box);
    auto negative_lay = new QHBoxLayout(_box);
    f = new QRadioButton("false", _box);
    t = new QRadioButton("true", _box);
    negative_lay->addWidget(f);
    negative_lay->addWidget(t);
    negative_group->setLayout(negative_lay);
    f->setChecked(true);

    auto limit_group = new QGroupBox("FilterLimits", _box);
    auto limit_lay = new QHBoxLayout(_box);
    mined = new QLineEdit("0", _box);
    maxed = new QLineEdit("1.0", _box);
    auto lb = new QLabel("~", _box);
    limit_lay->addWidget(mined);
    limit_lay->addWidget(lb);
    limit_lay->addWidget(maxed);
    limit_group->setLayout(limit_lay);
    _layout->addWidget(name_group);
    _layout->addWidget(limit_group);
    _layout->addWidget(negative_group);
    _box->setLayout(_layout);
    _box->resize(200, 300);
    _box->installEventFilter(this);
    limit_group->setToolTip("根据filter name, 过滤掉min~max间的区域");

    connect(mined, &QLineEdit::editingFinished, [=] { compute(); });
    connect(maxed, &QLineEdit::editingFinished, [=] { compute(); });
    connect(x, &QRadioButton::clicked, [=] { compute(); });
    connect(y, &QRadioButton::clicked, [=] { compute(); });
    connect(z, &QRadioButton::clicked, [=] { compute(); });
    connect(f, &QRadioButton::clicked, [=] { compute(); });
    connect(t, &QRadioButton::clicked, [=] { compute(); });
}

unsigned int CvPointCloudPassThroughModel::nPorts(PortType portType) const
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
    }
    return result;
}

bool CvPointCloudPassThroughModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _box) {
        if (_pc->empty()) {
            return false;
        }
        if (event->type() == QEvent::MouseButtonPress) {
            extern StreamerMainWindow *smw;
            smw->updateVTK(_pc);
            return true;
        }
    }
    return false;
}

NodeDataType CvPointCloudPassThroughModel::dataType(PortType const portType,
                                                    PortIndex const portIndex) const
{
    if (portType == PortType::In) {
        return PointCloudData().type();
    } else if (portType == PortType::Out) {
        if (portIndex == 0) {
            return PointCloudData().type();
        } else if (portIndex == 1) {
            return ResultData().type();
        }
    } else {
        return NodeDataType();
    }
}

std::shared_ptr<NodeData> CvPointCloudPassThroughModel::outData(PortIndex portIndex)
{
    if (portIndex == 0) {
        return std::make_shared<PointCloudData>(_pc);
    } else if (portIndex == 1) {
        return std::make_shared<ResultData>(_res);
    }
}
QJsonObject CvPointCloudPassThroughModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["x"] = x->isChecked();
    saver["y"] = x->isChecked();
    saver["z"] = x->isChecked();
    saver["f"] = f->isChecked();
    saver["t"] = t->isChecked();
    saver["mined"] = mined->text();
    saver["maxed"] = maxed->text();
    return saver;
}

void CvPointCloudPassThroughModel::load(const QJsonObject &js)
{
    x->setChecked(js["x"].toBool());
    y->setChecked(js["y"].toBool());
    z->setChecked(js["z"].toBool());
    f->setChecked(js["f"].toBool());
    t->setChecked(js["t"].toBool());
    mined->setText(js["mined"].toString());
    maxed->setText(js["maxed"].toString());
}

void CvPointCloudPassThroughModel::compute()
{
    bool min_f;
    bool max_f;
    auto min_v = mined->text().toFloat(&min_f);
    auto max_v = maxed->text().toFloat(&max_f);
    if ((min_f + max_f) != 2) {
        createMessageBox(nullptr,
                         ":icons/error.png",
                         "错误",
                         "输入的limits必须为数字，单位m",
                         1,
                         {"返回"});
        return;
    }
    auto d = std::dynamic_pointer_cast<PointCloudData>(_nodeData);
    if (!d) {
        return;
    }
    auto in_pc = d->getData();
    if (in_pc->empty()) {
        return;
    }
    std::string filter_name;
    if (x->isChecked()) {
        filter_name = "x";
    } else if (y->isChecked()) {
        filter_name = "y";
    } else {
        filter_name = "z";
    }

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(in_pc);
    pass.setFilterFieldName(filter_name);         // x y z
                                                  //    std::cout<<"min_v"<<min_v<<std::endl;
                                                  //    std::cout<<"max_v"<<max_v<<std::endl;
    pass.setFilterLimits(min_v, max_v);           // 3. 设置过滤范围
    pass.setFilterLimitsNegative(f->isChecked()); // 设置获取Limits之外的内容
    pass.filter(*_pc); // 4. 执行过滤，并将结果输出到cloud_filtered

    extern StreamerMainWindow *smw;
    smw->updateVTK(_pc);

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
}

void CvPointCloudPassThroughModel::setInData(std::shared_ptr<NodeData> nodeData,
                                             PortIndex const portIndex)
{
    auto d = std::dynamic_pointer_cast<PointCloudData>(nodeData);
    if (!d) {
        return;
    }
    _nodeData = nodeData;
    compute();
};
