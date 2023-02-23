#include "CvModels/CvPointCloudDownsampleVoxelGridModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudDownSampleVoxelGridModel::CvPointCloudDownSampleVoxelGridModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    auto size_group = new QGroupBox("LeafSize", _box);
    auto size_lay = new QVBoxLayout(_box);
    x = new QLineEdit("0.01", _box);
    y = new QLineEdit("0.01", _box);
    z = new QLineEdit("0.01", _box);
    auto xl = new QLabel("x", _box);
    auto yl = new QLabel("y", _box);
    auto zl = new QLabel("z", _box);
    auto xt = new QHBoxLayout(_box);
    auto yt = new QHBoxLayout(_box);
    auto zt = new QHBoxLayout(_box);
    createLineEditFormCurQObj(xt, xl, x);
    createLineEditFormCurQObj(yt, yl, y);
    createLineEditFormCurQObj(zt, zl, z);
    size_lay->addLayout(xt);
    size_lay->addLayout(yt);
    size_lay->addLayout(zt);
    size_group->setLayout(size_lay);
    _layout->addWidget(size_group);
    _box->setLayout(_layout);
    _box->resize(200, 200);
    _box->installEventFilter(this);

    connect(x, &QLineEdit::editingFinished, [=] { compute(); });
    connect(y, &QLineEdit::editingFinished, [=] { compute(); });
    connect(z, &QLineEdit::editingFinished, [=] { compute(); });
}

unsigned int CvPointCloudDownSampleVoxelGridModel::nPorts(PortType portType) const
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

bool CvPointCloudDownSampleVoxelGridModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvPointCloudDownSampleVoxelGridModel::dataType(PortType const portType,
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

std::shared_ptr<NodeData> CvPointCloudDownSampleVoxelGridModel::outData(PortIndex portIndex)
{
    if (portIndex == 0) {
        return std::make_shared<PointCloudData>(_pc);
    } else if (portIndex == 1) {
        return std::make_shared<ResultData>(_res);
    }
}
QJsonObject CvPointCloudDownSampleVoxelGridModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["x"] = x->text();
    saver["y"] = x->text();
    saver["z"] = x->text();
    return saver;
}

void CvPointCloudDownSampleVoxelGridModel::load(const QJsonObject &js)
{
    x->setText(js["x"].toString());
    y->setText(js["y"].toString());
    z->setText(js["z"].toString());
}

void CvPointCloudDownSampleVoxelGridModel::compute()
{
    auto d = std::dynamic_pointer_cast<PointCloudData>(_nodeData);
    if (d->getData()->empty()) {
        return;
    }
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(d->getData());
    sor.setLeafSize(x->text().toFloat(), y->text().toFloat(), z->text().toFloat());
    sor.filter(*_pc);

    extern StreamerMainWindow *smw;
    smw->updateVTK(_pc);

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
}

void CvPointCloudDownSampleVoxelGridModel::setInData(std::shared_ptr<NodeData> nodeData,
                                                     PortIndex const portIndex)
{
    auto d = std::dynamic_pointer_cast<PointCloudData>(nodeData);
    if (!d) {
        return;
    }
    _nodeData = nodeData;
    compute();
};
