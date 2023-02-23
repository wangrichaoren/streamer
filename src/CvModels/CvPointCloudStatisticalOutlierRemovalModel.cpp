#include "CvModels/CvPointCloudStatisticalOutlierRemovalModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudStatisticalOutlierRemovalModel::CvPointCloudStatisticalOutlierRemovalModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    auto k_group = new QGroupBox("MeanK", _box);
    auto klay = new QHBoxLayout(_box);
    k = new QLineEdit("50");
    klay->addWidget(k);
    k_group->setLayout(klay);

    auto s_group = new QGroupBox("StddevMulThresh", _box);
    auto slay = new QVBoxLayout(_box);
    s = new QLineEdit("1.0");
    slay->addWidget(s);
    s_group->setLayout(slay);

    auto n_group = new QGroupBox("Negative", _box);
    f = new QRadioButton("false", _box);
    t = new QRadioButton("true", _box);
    auto nlay = new QHBoxLayout(_box);
    nlay->addWidget(f);
    nlay->addWidget(t);
    n_group->setLayout(nlay);
    f->setChecked(true);

    _layout->addWidget(k_group);
    _layout->addWidget(s_group);
    _layout->addWidget(n_group);

    _box->setLayout(_layout);
    _box->resize(200, 300);
    _box->installEventFilter(this);

    connect(k,&QLineEdit::editingFinished,[&]{compute();});
    connect(s,&QLineEdit::editingFinished,[&]{compute();});
    connect(f,&QRadioButton::clicked,[&]{compute();});
    connect(t,&QRadioButton::clicked,[&]{compute();});
}

unsigned int CvPointCloudStatisticalOutlierRemovalModel::nPorts(PortType portType) const
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

bool CvPointCloudStatisticalOutlierRemovalModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvPointCloudStatisticalOutlierRemovalModel::dataType(PortType const portType,
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

std::shared_ptr<NodeData> CvPointCloudStatisticalOutlierRemovalModel::outData(PortIndex portIndex)
{
    if (portIndex == 0) {
        return std::make_shared<PointCloudData>(_pc);
    } else if (portIndex == 1) {
        return std::make_shared<ResultData>(_res);
    }
}
QJsonObject CvPointCloudStatisticalOutlierRemovalModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["k"] = k->text();
    saver["s"] = s->text();
    saver["f"] = f->isChecked();
    saver["t"] = t->isChecked();
    return saver;
}

void CvPointCloudStatisticalOutlierRemovalModel::load(const QJsonObject &js)
{
    k->setText(js["k"].toString());
    s->setText(js["s"].toString());
    f->setChecked(js["f"].toBool());
    t->setChecked(js["t"].toBool());
}

void CvPointCloudStatisticalOutlierRemovalModel::compute()
{
    auto d = std::dynamic_pointer_cast<PointCloudData>(_nodeData);
    if (d->getData()->empty()) {
        return;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(d->getData());
    sor.setMeanK(k->text().toInt());
    sor.setStddevMulThresh(s->text().toFloat());
    sor.setNegative(f->isChecked());
    sor.filter(*_pc);

    extern StreamerMainWindow *smw;
    smw->updateVTK(_pc);

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
}

void CvPointCloudStatisticalOutlierRemovalModel::setInData(std::shared_ptr<NodeData> nodeData,
                                                           PortIndex const portIndex)
{
    auto d = std::dynamic_pointer_cast<PointCloudData>(nodeData);
    if (!d) {
        return;
    }
    _nodeData = nodeData;
    compute();
};
