#include "CvModels/CvPointCloudSampleConsensusPrerejectiveModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudSampleConsensusPrerejectiveModel::CvPointCloudSampleConsensusPrerejectiveModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    auto nos_group = new QGroupBox("NumberOfSamples", _box);
    auto nos_lay = new QVBoxLayout(_box);
    nos = new QLineEdit("3", _box);
    nos_lay->addWidget(nos);
    nos_group->setLayout(nos_lay);

    auto cr_group = new QGroupBox("CorrespondenceRandomness", _box);
    auto cr_lay = new QVBoxLayout(_box);
    cr = new QLineEdit("5", _box);
    cr_lay->addWidget(cr);
    cr_group->setLayout(cr_lay);

    auto st_group = new QGroupBox("SimilarityThreshold", _box);
    auto st_lay = new QVBoxLayout(_box);
    st = new QLineEdit("0.9", _box);
    st_lay->addWidget(st);
    st_group->setLayout(st_lay);

    auto mcd_group = new QGroupBox("MaxCorrespondenceDistance", _box);
    auto mcd_lay = new QVBoxLayout(_box);
    mcd = new QLineEdit("0.0125", _box);
    mcd_lay->addWidget(mcd);
    mcd_group->setLayout(mcd_lay);

    auto ift_group = new QGroupBox("InlierFraction", _box);
    auto ift_lay = new QVBoxLayout(_box);
    ift = new QLineEdit("0.25", _box);
    ift_lay->addWidget(ift);
    ift_group->setLayout(ift_lay);

    auto mi_group = new QGroupBox("MaximumIterations", _box);
    auto mi_lay = new QVBoxLayout(_box);
    mi = new QLineEdit("0", _box);
    mi_lay->addWidget(mi);
    mi_group->setLayout(mi_lay);

    loading_dialog = new LoadingDialog(nullptr);

    auto calc_btn = new QPushButton("计算", _box);

    _layout->addWidget(nos_group);
    _layout->addWidget(cr_group);
    _layout->addWidget(st_group);
    _layout->addWidget(mcd_group);
    _layout->addWidget(ift_group);
    _layout->addWidget(mi_group);
    _layout->addWidget(calc_btn);

    _box->setLayout(_layout);
    _box->resize(200, 500);
    _box->installEventFilter(this);

    connect(calc_btn, &QPushButton::clicked, [=] { compute(); });

    connect(this, &CvPointCloudSampleConsensusPrerejectiveModel::computingStarted, [=] {
        loading_dialog->show();
    });

    connect(this, &CvPointCloudSampleConsensusPrerejectiveModel::computingFinished, [=] {
        loading_dialog->hide();
        loading_dialog->close();
    });
}

unsigned int CvPointCloudSampleConsensusPrerejectiveModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 5;
        break;

    case PortType::Out:
        result = 3;

    default:
        break;
    }
    return result;
}

bool CvPointCloudSampleConsensusPrerejectiveModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvPointCloudSampleConsensusPrerejectiveModel::dataType(PortType const portType,
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
        } else if (portIndex == 4) {
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

std::shared_ptr<NodeData> CvPointCloudSampleConsensusPrerejectiveModel::outData(PortIndex portIndex)
{
    if (portIndex == 0) {
        return std::make_shared<PointCloudData>(_outPc);
    } else if (portIndex == 1) {
        return std::make_shared<Matrix4fData>(_outMat4f);
    } else if (portIndex == 2) {
        return std::make_shared<ResultData>(_outRes);
    }
}

QJsonObject CvPointCloudSampleConsensusPrerejectiveModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["nos"] = nos->text();
    saver["cr"] = cr->text();
    saver["st"] = st->text();
    saver["mcd"] = mcd->text();
    saver["ift"] = ift->text();
    saver["mi"] = mi->text();
    return saver;
}

void CvPointCloudSampleConsensusPrerejectiveModel::load(const QJsonObject &js)
{
    nos->setText(js["nos"].toString());
    cr->setText(js["cr"].toString());
    st->setText(js["st"].toString());
    mcd->setText(js["mcd"].toString());
    ift->setText(js["ift"].toString());
    mi->setText(js["mi"].toString());
}

void CvPointCloudSampleConsensusPrerejectiveModel::compute()
{
    std::cout << "start compute" << std::endl;
    auto d1 = std::dynamic_pointer_cast<PointCloudData>(_inNodeData);
    auto d2 = std::dynamic_pointer_cast<FPFHSignature33Data>(_inFeature);
    auto d3 = std::dynamic_pointer_cast<PointCloudData>(_inTargetNodeData);
    auto d4 = std::dynamic_pointer_cast<FPFHSignature33Data>(_inTargetFeature);

    bool f;
    auto nos_v = nos->text().toInt(&f);
    auto cr_v = cr->text().toInt(&f);
    auto st_v = st->text().toFloat(&f);
    auto mcd_v = mcd->text().toFloat(&f);
    auto ift_v = ift->text().toFloat(&f);
    auto mi_v = mi->text().toInt(&f);
    if (!f) {
        createMessageBox(nullptr, ":icons/error.png", "错误", "输入的参数必须为数字", 1, {"返回"});
        return;
    }
    _outRes.clear();

    Q_EMIT computingStarted();

    std::thread t([=] {
        this->calc(d1->getData(),
                   d2->getData(),
                   d3->getData(),
                   d4->getData(),
                   nos_v,
                   cr_v,
                   st_v,
                   mcd_v,
                   ift_v,
                   mi_v);
    });

    loading_dialog->exec();
    t.join();

    extern StreamerMainWindow *smw;
    smw->updateVTK(_outPc);

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
    Q_EMIT dataUpdated(2);
}

void CvPointCloudSampleConsensusPrerejectiveModel::setInData(std::shared_ptr<NodeData> nodeData,
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
    } else if (portIndex == 4) {
        auto p4 = std::dynamic_pointer_cast<Matrix4fData>(nodeData);
        if (!p4) {
            _inMat4f = nullptr;
            return;
        }
        _inMat4f = nodeData;
    }

//    if ((_inNodeData != nullptr) + (_inFeature != nullptr) + (_inTargetNodeData != nullptr)
//            + (_inTargetFeature != nullptr)
//        == 4) {
//        compute();
//    }
}
void CvPointCloudSampleConsensusPrerejectiveModel::calc(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr d1,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr d2,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr d3,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr d4,
    float nos_v,
    float cr_v,
    float st_v,
    float mcd_v,
    float ift_v,
    int mi_v)
{
    pcl::SampleConsensusPrerejective<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> scp;
    scp.setInputSource(d1);
    scp.setSourceFeatures(d2);
    scp.setInputTarget(d3);
    scp.setTargetFeatures(d4);
    scp.setNumberOfSamples(nos_v); // Number of points to sample for generating/prerejecting a pose
    scp.setCorrespondenceRandomness(cr_v);   // Number of nearest features to use
    scp.setSimilarityThreshold(st_v);        // Polygonal edge length similarity threshold
    scp.setMaxCorrespondenceDistance(mcd_v); // Inlier threshold
    scp.setInlierFraction(ift_v); // Required inlier fraction for accepting a pose hypothesis
    if (mi_v != 0) {
        scp.setMaximumIterations(mi_v); // Number of RANSAC iterations
    }

    {
        pcl::ScopeTime t("Alignment");
        auto d5 = std::dynamic_pointer_cast<Matrix4fData>(_inMat4f);
        if (d5) {
            std::cout << "input port Matrix4fData exist!" << std::endl;
            scp.align(*_outPc, d5->getData());
        } else {
            std::cout << "input port Matrix4fData not exist!" << std::endl;
            scp.align(*_outPc);
        }
    }

    auto has_converged = scp.hasConverged();
    if (has_converged) {
        auto tm = scp.getFinalTransformation();
        *_outPc = (*_outPc) + (*d3);
        _outRes = "hasConverged: true\nscore: " + to_string(scp.getFitnessScore())
                  + "\nTransformation: \n" + std::to_string(tm(0, 0)) + " "
                  + std::to_string(tm(0, 1)) + " " + std::to_string(tm(0, 2)) + " "
                  + std::to_string(tm(0, 3)) + "\n" + std::to_string(tm(1, 0)) + " "
                  + std::to_string(tm(1, 1)) + " " + std::to_string(tm(1, 2)) + " "
                  + std::to_string(tm(1, 3)) + "\n" + std::to_string(tm(2, 0)) + " "
                  + std::to_string(tm(2, 1)) + " " + std::to_string(tm(2, 2)) + " "
                  + std::to_string(tm(2, 3)) + "\n" + std::to_string(tm(3, 0)) + " "
                  + std::to_string(tm(3, 1)) + " " + std::to_string(tm(3, 2)) + " "
                  + std::to_string(tm(3, 3)) + "\n";
    } else {
        _outRes = "hasConverged: false";
        return;
    }
    Q_EMIT computingFinished();
};