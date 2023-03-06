#include "CvModels/CvPointIaRansacModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudIaRansacModel::CvPointCloudIaRansacModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    loading_dialog = new LoadingDialog();

    auto calc_btn = new QPushButton("计算", _box);
    auto m_g = new QGroupBox("MinSampleDistance", _box);
    auto mcd_g = new QGroupBox("MaxCorrespondenceDistance", _box);
    auto mi_g = new QGroupBox("MaximumIterations", _box);
    auto mlay = new QVBoxLayout(_box);
    auto mcdlay = new QVBoxLayout(_box);
    auto milay = new QVBoxLayout(_box);
    m = new QLineEdit("0.05", _box);
    mcd = new QLineEdit("0.0001", _box);
    mi = new QLineEdit("0", _box);
    mlay->addWidget(m);
    mcdlay->addWidget(mcd);
    milay->addWidget(mi);
    m_g->setLayout(mlay);
    mcd_g->setLayout(mcdlay);
    mi_g->setLayout(milay);

    _layout->addWidget(m_g);
    _layout->addWidget(mcd_g);
    _layout->addWidget(mi_g);
    _layout->addWidget(calc_btn);

    _box->setLayout(_layout);
    _box->resize(200, 300);
    _box->installEventFilter(this);

    connect(calc_btn, &QPushButton::clicked, [=] { compute(); });

    connect(this, &CvPointCloudIaRansacModel::computingStarted, [=] { loading_dialog->show(); });

    connect(this, &CvPointCloudIaRansacModel::computingFinished, [=] {
        loading_dialog->hide();
        loading_dialog->close();
    });
}

unsigned int CvPointCloudIaRansacModel::nPorts(PortType portType) const
{
    unsigned int result = 0;

    switch (portType) {
    case PortType::In:
        result = 4;
        break;

    case PortType::Out:
        result = 3;

    default:
        break;
    }
    return result;
}

bool CvPointCloudIaRansacModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvPointCloudIaRansacModel::dataType(PortType const portType,
                                                 PortIndex const portIndex) const
{
    if (portType == PortType::In) {
        if (portIndex == 0) {
            return PointCloudData().type();
        } else if (portIndex == 1) {
            return FPFHSignature33Data().type();
        } else if (portIndex == 2) {
            return PointCloudData().type();
        } else if (portIndex == 3) {
            return FPFHSignature33Data().type();
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

std::shared_ptr<NodeData> CvPointCloudIaRansacModel::outData(PortIndex portIndex)
{
    if (portIndex == 0) {
        return std::make_shared<PointCloudData>(_outPc);
    } else if (portIndex == 1) {
        return std::make_shared<Matrix4fData>(_outMat4f);
    } else if (portIndex == 2) {
        return std::make_shared<ResultData>(_outRes);
    }
}

QJsonObject CvPointCloudIaRansacModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["m"] = m->text();
    saver["mcd"] = mcd->text();
    saver["mi"] = mi->text();
    return saver;
}

void CvPointCloudIaRansacModel::load(const QJsonObject &js)
{
    m->setText(js["m"].toString());
    mcd->setText(js["mcd"].toString());
    mi->setText(js["mi"].toString());
}

void CvPointCloudIaRansacModel::compute()
{
    std::cout << "start compute" << std::endl;
    auto d1 = std::dynamic_pointer_cast<PointCloudData>(_inNodeData);
    auto d2 = std::dynamic_pointer_cast<FPFHSignature33Data>(_inFeature);
    auto d3 = std::dynamic_pointer_cast<PointCloudData>(_inTargetNodeData);
    auto d4 = std::dynamic_pointer_cast<FPFHSignature33Data>(_inTargetFeature);

    bool f;
    auto m_v = m->text().toFloat(&f);
    auto mcd_v = mcd->text().toFloat(&f);
    auto mi_v = mi->text().toInt(&f);
    if (!f) {
        createMessageBox(nullptr, ":icons/error.png", "错误", "输入的参数必须为数字", 1, {"返回"});
        return;
    }
    _outRes.clear();
    Q_EMIT computingStarted();
    std::thread t([=] {
        this->calc(d1->getData(), d2->getData(), d3->getData(), d4->getData(), m_v, mcd_v, mi_v);
    });

    loading_dialog->exec();
    t.join();
    std::cout << "1" << std::endl;
    extern StreamerMainWindow *smw;
    smw->updateVTK(_outPc);
    std::cout << "2" << std::endl;

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
    Q_EMIT dataUpdated(2);
    std::cout << "3" << std::endl;
}

void CvPointCloudIaRansacModel::setInData(std::shared_ptr<NodeData> nodeData,
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
        auto p1 = std::dynamic_pointer_cast<FPFHSignature33Data>(nodeData);
        if (!p1) {
            _inFeature = nullptr;
            return;
        }
        _inFeature = nodeData;
    } else if (portIndex == 2) {
        auto p2 = std::dynamic_pointer_cast<PointCloudData>(nodeData);
        if (!p2) {
            _inTargetNodeData = nullptr;
            return;
        }
        _inTargetNodeData = nodeData;
    } else if (portIndex == 3) {
        auto p3 = std::dynamic_pointer_cast<FPFHSignature33Data>(nodeData);
        if (!p3) {
            _inTargetFeature = nullptr;
            return;
        }
        _inTargetFeature = nodeData;
    }
}

void CvPointCloudIaRansacModel::calc(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &d1,
                                     const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &d2,
                                     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &d3,
                                     const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &d4,
                                     float m_v,
                                     float mcd_v,
                                     int mi_v)
{
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>
        sac_ia_;
    sac_ia_.setInputSource(d1);
    sac_ia_.setSourceFeatures(d2);
    sac_ia_.setInputTarget(d3);
    sac_ia_.setTargetFeatures(d4);

    sac_ia_.setMinSampleDistance(m_v);
    sac_ia_.setMaxCorrespondenceDistance(mcd_v);
    if (mi_v != 0) {
        sac_ia_.setMaximumIterations(mi_v);
    }
    sac_ia_.align(*_outPc);

    auto has_converged = sac_ia_.hasConverged();
    if (has_converged) {
        auto tm = sac_ia_.getFinalTransformation();

        *_outPc = (*_outPc) + (*d3);

        _outRes = "hasConverged: true\nscore: " + to_string(sac_ia_.getFitnessScore())
                  + "\nTransformation: \n" + std::to_string(tm(0, 0)) + " "
                  + std::to_string(tm(0, 1)) + " " + std::to_string(tm(0, 2)) + " "
                  + std::to_string(tm(0, 3)) + "\n" + std::to_string(tm(1, 0)) + " "
                  + std::to_string(tm(1, 1)) + " " + std::to_string(tm(1, 2)) + " "
                  + std::to_string(tm(1, 3)) + "\n" + std::to_string(tm(2, 0)) + " "
                  + std::to_string(tm(2, 1)) + " " + std::to_string(tm(2, 2)) + " "
                  + std::to_string(tm(2, 3)) + "\n" + std::to_string(tm(3, 0)) + " "
                  + std::to_string(tm(3, 1)) + " " + std::to_string(tm(3, 2)) + " "
                  + std::to_string(tm(3, 3)) + "\n";
        auto i = sac_ia_.getFinalTransformation();
        _outMat4f << i;
    } else {
        _outRes = "hasConverged: false";
    }
    Q_EMIT computingFinished();
};