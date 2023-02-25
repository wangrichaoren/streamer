#include "CvModels/CvPointCloudFPFHEstimationOMPModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudFPFHEstimationOMPModel::CvPointCloudFPFHEstimationOMPModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    auto rs_group = new QGroupBox("RadiusSearch");
    auto rs_lay = new QVBoxLayout();
    r = new QLineEdit("0.025");
    rs_lay->addWidget(r);
    rs_group->setLayout(rs_lay);

    _layout->addWidget(rs_group);
    _box->setLayout(_layout);
    _box->resize(200, 200);
    _box->installEventFilter(this);
}

unsigned int CvPointCloudFPFHEstimationOMPModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 2;
        break;

    case PortType::Out:
        result = 2;

    default:
        break;
    }
    return result;
}

bool CvPointCloudFPFHEstimationOMPModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _box) {
        if (_outPc->empty()) {
            return false;
        }
        if (event->type() == QEvent::MouseButtonPress) {
            extern StreamerMainWindow *smw;
            smw->updateVTK(_outPc);
            return true;
        }
    }
    return false;
}

NodeDataType CvPointCloudFPFHEstimationOMPModel::dataType(PortType const portType,
                                                          PortIndex const portIndex) const
{
    if (portType == PortType::In) {
        if (portIndex == 0) {
            return PointCloudData().type();
        } else if (portIndex == 1) {
            return NormalData().type();
        }
    } else if (portType == PortType::Out) {
        if (portIndex == 0) {
            return FPFHSignature33Data().type();
        } else if (portIndex == 1) {
            return ResultData().type();
        }
    } else {
        return NodeDataType();
    }
}

std::shared_ptr<NodeData> CvPointCloudFPFHEstimationOMPModel::outData(PortIndex portIndex)
{
    if (portIndex == 0) {
        return std::make_shared<FPFHSignature33Data>(_outFeature);
    } else if (portIndex == 1) {
        return std::make_shared<ResultData>(_outRes);
    }
}

QJsonObject CvPointCloudFPFHEstimationOMPModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["r"] = r->text();
    return saver;
}

void CvPointCloudFPFHEstimationOMPModel::load(const QJsonObject &js)
{
    r->setText(js["r"].toString());
}

void CvPointCloudFPFHEstimationOMPModel::compute()
{
    auto d1 = std::dynamic_pointer_cast<PointCloudData>(_inNodeData);
    auto d2 = std::dynamic_pointer_cast<NormalData>(_inNormal);

    bool f;
    auto val = r->text().toFloat(&f);
    if (!f) {
        createMessageBox(nullptr, ":icons/error.png", "错误", "输入的参数必须为数字", 1, {"返回"});
        return;
    }
    _outRes.clear();

    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fest;
    // 此处使用的半径必须要大于估计表面法线时使用的半径
    fest.setRadiusSearch(val);
    fest.setInputCloud(d1->getData());
    fest.setInputNormals(d2->getData());
    fest.compute(*_outFeature);

    if (d1->getData()->points.size() != _outFeature->points.size()) {
        createMessageBox(nullptr,
                         ":icons/error.png",
                         "计算错误",
                         "输入的点云数量和计算出来的特征数量应该一致,但是实际不一致,"
                         "请检查输入的数据是否正确,或存在算法bug!",
                         1,
                         {"返回"});
        return;
    }

    // todo 返回原来输入的点云，特征怎么显示？待定
    _outPc = d1->getData();
    // 必须有相同的数量
    _outRes = "feature size: " + to_string(_outFeature->points.size())
              + "\ninput point size: " + to_string(d1->getData()->points.size());
    extern StreamerMainWindow *smw;
    smw->updateVTK(_outPc);

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
    Q_EMIT dataUpdated(2);
}

void CvPointCloudFPFHEstimationOMPModel::setInData(std::shared_ptr<NodeData> nodeData,
                                                   PortIndex const portIndex)
{
    if (portIndex == 0) {
        auto p0 = std::dynamic_pointer_cast<PointCloudData>(nodeData);
        if (!p0) {
            _inNodeData = nullptr;
            return;
        }
        _inNodeData = nodeData;
    } else if (portIndex == 1) {
        auto p1 = std::dynamic_pointer_cast<NormalData>(nodeData);
        if (!p1) {
            _inNormal = nullptr;
            return;
        }
        _inNormal = nodeData;
    }

    if (_inNodeData != nullptr and _inNormal != nullptr) {
        compute();
    }
};