#include "CvModels/CvPointCloudTransformModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudTransformModel::CvPointCloudTransformModel()
    : _box(new QGroupBox())
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    _box->resize(200, 200);

    _box->installEventFilter(this);
}

unsigned int CvPointCloudTransformModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 2;
        break;

    case PortType::Out:
        result = 1;
        break;

    default:
        break;
    }
    return result;
}

bool CvPointCloudTransformModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvPointCloudTransformModel::dataType(PortType const pt, PortIndex const pi) const
{
    if (pt == PortType::In) {
        if (pi == 0) {
            return PointCloudData().type();
        } else if (pi == 1) {
            return Matrix4fData().type();
        }
    } else if (pt == PortType::Out) {
        return PointCloudData().type();
    } else {
        return NodeDataType();
    }
}

std::shared_ptr<NodeData> CvPointCloudTransformModel::outData(PortIndex)
{
    return std::make_shared<PointCloudData>(_pc);
}

void CvPointCloudTransformModel::compute()
{
    auto inpc = std::dynamic_pointer_cast<PointCloudData>(_inpc);
    auto inmat4f = std::dynamic_pointer_cast<Matrix4fData>(_inmat4f);

    Eigen::Matrix4f mat;
    mat << inmat4f->getData();

    pcl::transformPointCloud(*inpc->getData(), *_pc, mat);

    extern StreamerMainWindow *smw;
    smw->updateVTK(_pc);
    Q_EMIT dataUpdated(0);
}

void CvPointCloudTransformModel::setInData(std::shared_ptr<NodeData> node, PortIndex const portIndex)
{
    if (portIndex == 0) {
        if (!node) {
            _inpc = nullptr;
            return;
        }
        _inpc = node;
    } else if (portIndex == 1) {
        if (!node) {
            _inmat4f = nullptr;
            return;
        }
        _inmat4f = node;
    }
    if (_inpc != nullptr && _inmat4f != nullptr) {
        compute();
    }
}