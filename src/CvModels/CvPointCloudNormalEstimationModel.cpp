#include "CvModels/CvPointCloudNormalEstimationModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudNormalEstimationModel::CvPointCloudNormalEstimationModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    r = new QLineEdit("0.03", _box);
    auto r_lay = new QHBoxLayout(_box);
    auto rlab = new QLabel("Radius", _box);
    createLineEditFormCurQObj(r_lay, rlab, r);

    //    _layout->addWidget(r_group);
    _layout->addLayout(r_lay);

    _box->setLayout(_layout);
    _box->resize(200, 200);
    _box->installEventFilter(this);
    connect(r, &QLineEdit::editingFinished, [=] { compute(); });
}

unsigned int CvPointCloudNormalEstimationModel::nPorts(PortType portType) const
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

bool CvPointCloudNormalEstimationModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _box) {
        if (_pc->empty() || _normal->empty()) {
            return false;
        }
        if (event->type() == QEvent::MouseButtonPress) {
            extern StreamerMainWindow *smw;
            smw->updateVTK(_pc, _normal);
            return true;
        }
    }
    return false;
}

NodeDataType CvPointCloudNormalEstimationModel::dataType(PortType const portType,
                                                         PortIndex const portIndex) const
{
    if (portType == PortType::In) {
        return PointCloudData().type();
    } else if (portType == PortType::Out) {
        if (portIndex == 0) {
            return NormalData().type();
        } else if (portIndex == 1) {
            return ResultData().type();
        }
    } else {
        return NodeDataType();
    }
}

std::shared_ptr<NodeData> CvPointCloudNormalEstimationModel::outData(PortIndex portIndex)
{
    if (portIndex == 0) {
        return std::make_shared<NormalData>(_normal);
    } else if (portIndex == 1) {
        return std::make_shared<ResultData>(_res);
    }
}
QJsonObject CvPointCloudNormalEstimationModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["r"] = r->text();
    return saver;
}

void CvPointCloudNormalEstimationModel::load(const QJsonObject &js)
{
    r->setText(js["r"].toString());
}

void CvPointCloudNormalEstimationModel::compute()
{
    auto d = std::dynamic_pointer_cast<PointCloudData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->getData()->empty()) {
        return;
    }
    bool f;
    auto val = r->text().toFloat(&f);
    if (!f) {
        createMessageBox(nullptr,
                         ":icons/error.png",
                         "参数错误",
                         "输入的参数必须为数字",
                         1,
                         {"返回"});
        return;
    }
    _res.clear();

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(d->getData());
    normalEstimation.setRadiusSearch(val);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*_normal);

    _pc = d->getData();
    extern StreamerMainWindow *smw;
    smw->updateVTK(_pc, _normal);
    std::cout<<_normal->size()<<std::endl;
    _res = "normal size :" + std::to_string(_normal->size());

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
}

void CvPointCloudNormalEstimationModel::setInData(std::shared_ptr<NodeData> nodeData,
                                                  PortIndex const portIndex)
{
    auto d = std::dynamic_pointer_cast<PointCloudData>(nodeData);
    if (!d) {
        return;
    }
    _nodeData = nodeData;

    compute();
};
