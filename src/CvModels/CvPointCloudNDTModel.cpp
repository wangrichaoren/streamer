#include "CvModels/CvPointCloudNDTModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudNDTModel::CvPointCloudNDTModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    //    TransformationEpsilon 0.01
    auto tse_group = new QGroupBox("TransformationEpsilon");
    auto tse_lay = new QVBoxLayout();
    tse = new QLineEdit("0.01");
    tse_lay->addWidget(tse);
    tse_group->setLayout(tse_lay);

    //StepSize 0.1
    auto ss_group = new QGroupBox("StepSize");
    auto ss_lay = new QVBoxLayout();
    ss = new QLineEdit("0.1");
    ss_lay->addWidget(ss);
    ss_group->setLayout(ss_lay);

    //Resolution(1.0)
    auto r_group = new QGroupBox("Resolution");
    auto r_lay = new QVBoxLayout();
    r = new QLineEdit("1.0");
    r_lay->addWidget(r);
    r_group->setLayout(r_lay);

    //    MaximumIterations(35)
    auto mi_group = new QGroupBox("MaximumIterations");
    auto mi_lay = new QVBoxLayout();
    mi = new QLineEdit("0");
    mi_lay->addWidget(mi);
    mi_group->setLayout(mi_lay);

    auto calc_btn = new QPushButton("计算");
    _layout->addWidget(tse_group);
    _layout->addWidget(ss_group);
    _layout->addWidget(r_group);
    _layout->addWidget(mi_group);
    _layout->addWidget(calc_btn);

    _box->setLayout(_layout);
    _box->resize(200, 350);
    _box->installEventFilter(this);
    connect(calc_btn, &QPushButton::clicked, [=] { compute(); });
}

unsigned int CvPointCloudNDTModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 3;
        break;

    case PortType::Out:
        result = 3;

    default:
        break;
    }
    return result;
}

bool CvPointCloudNDTModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvPointCloudNDTModel::dataType(PortType const portType, PortIndex const portIndex) const
{
    if (portType == PortType::In) {
        if (portIndex == 0) {
            // source input pc
            return PointCloudData().type();
        } else if (portIndex == 1) {
            // target pc
            return PointCloudData().type();
        } else if (portIndex == 2) {
            // init guess mat4f
            return Matrix4fData().type();
        }
    } else if (portType == PortType::Out) {
        if (portIndex == 0) {
            return PointCloudData().type();
        } else if (portIndex == 1) {
            return Matrix4fData().type();
        } else if (portIndex == 2) {
            return ResultData().type();
        }
    } else {
        return NodeDataType();
    }
}

std::shared_ptr<NodeData> CvPointCloudNDTModel::outData(PortIndex portIndex)
{
    if (portIndex == 0) {
        return std::make_shared<PointCloudData>(_outPc);
    } else if (portIndex == 1) {
        return std::make_shared<Matrix4fData>(_outMat4f);
    } else if (portIndex == 2) {
        return std::make_shared<ResultData>(_outRes);
    }
}

QJsonObject CvPointCloudNDTModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["tse"] = tse->text();
    saver["ss"] = ss->text();
    saver["r"] = r->text();
    saver["mi"] = mi->text();
    return saver;
}

void CvPointCloudNDTModel::load(const QJsonObject &js)
{
    tse->setText(js["tse"].toString());
    ss->setText(js["ss"].toString());
    r->setText(js["r"].toString());
    mi->setText(js["mi"].toString());
}

void CvPointCloudNDTModel::compute()
{
    auto d1 = std::dynamic_pointer_cast<PointCloudData>(_inNodeData);
    auto d2 = std::dynamic_pointer_cast<PointCloudData>(_targetNodeData);
    auto d3 = std::dynamic_pointer_cast<Matrix4fData>(_inMat4f);
    if (!d1 || !d2) {
        createMessageBox(nullptr,
                         ":icons/error.png",
                         "错误",
                         "至少存在两个刚性变换点云的输入",
                         1,
                         {"返回"});
        return;
    }
    bool f;
    auto tse_val = tse->text().toFloat(&f);
    auto ss_val = ss->text().toFloat(&f);
    auto r_val = r->text().toFloat(&f);
    auto mi_val = mi->text().toInt(&f);
    if (!f) {
        createMessageBox(nullptr, ":icons/error.png", "错误", "输入的参数必须为数字", 1, {"返回"});
        return;
    }

    _outRes.clear();

    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;

    ndt.setInputSource(d1->getData());

    ndt.setInputTarget(d2->getData());

    // 设置终止条件的最小变换差。
    ndt.setTransformationEpsilon(tse_val);

    // 为 More-Thuente 行搜索设置最大步长。
    ndt.setStepSize(ss_val);
    // 无损检测网格结构的//设置分辨率(体素网格协方差)。
    ndt.setResolution(r_val);
    // 设置最大注册迭代次数
    if (mi_val != 0) {
        ndt.setMaximumIterations(mi_val);
    }

    if (!d3) {
        ndt.align(*_outPc);
    } else {
        auto init_guess = d3->getData();
        ndt.align(*_outPc, init_guess);
    }

    auto tm = ndt.getFinalTransformation();
    pcl::transformPointCloud(*d1->getData(), *_outPc, tm);
    *_outPc = (*_outPc) + (*d2->getData());
    _outMat4f = ndt.getFinalTransformation();
    _outRes = "hasConverged: " + std::to_string(ndt.hasConverged())
              + "\nscore: " + std::to_string(ndt.getFitnessScore()) + "\nTransformation: \n"
              + std::to_string(tm(0, 0)) + " " + std::to_string(tm(0, 1)) + " "
              + std::to_string(tm(0, 2)) + " " + std::to_string(tm(0, 3)) + "\n"
              + std::to_string(tm(1, 0)) + " " + std::to_string(tm(1, 1)) + " "
              + std::to_string(tm(1, 2)) + " " + std::to_string(tm(1, 3)) + "\n"
              + std::to_string(tm(2, 0)) + " " + std::to_string(tm(2, 1)) + " "
              + std::to_string(tm(2, 2)) + " " + std::to_string(tm(2, 3)) + "\n"
              + std::to_string(tm(3, 0)) + " " + std::to_string(tm(3, 1)) + " "
              + std::to_string(tm(3, 2)) + " " + std::to_string(tm(3, 3)) + "\n";

    extern StreamerMainWindow *smw;
    smw->updateVTK(_outPc);

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
    Q_EMIT dataUpdated(2);
}

void CvPointCloudNDTModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const portIndex)
{
    if (portIndex == 0) {
        auto p0 = std::dynamic_pointer_cast<PointCloudData>(nodeData);
        if (!p0) {
            _inNodeData = nullptr;
            return;
        }
        _inNodeData = nodeData;

    } else if (portIndex == 1) {
        auto p1 = std::dynamic_pointer_cast<PointCloudData>(nodeData);
        if (!p1) {
            _targetNodeData = nullptr;
            return;
        }
        _targetNodeData = nodeData;
    } else if (portIndex == 2) {
        auto p2 = std::dynamic_pointer_cast<Matrix4fData>(nodeData);
        if (!p2) {
            _inMat4f = nullptr;
            return;
        }
        _inMat4f = nodeData;
    }
    if (_inNodeData != nullptr) {
        if (_targetNodeData != nullptr) {
            compute();
        }
    }
};