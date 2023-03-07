#include "CvModels/CvPointCloudICPModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudICPModel::CvPointCloudICPModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    auto mcd_group = new QGroupBox("MaxCorrespondenceDistance", _box);
    auto mcdlay = new QVBoxLayout(_box);
    mcd = new QLineEdit("10", _box);
    mcdlay->addWidget(mcd);
    mcd_group->setLayout(mcdlay);

    auto te_group = new QGroupBox("TransformationEpsilon", _box);
    auto telay = new QVBoxLayout(_box);
    te = new QLineEdit("0.00001", _box);
    telay->addWidget(te);
    te_group->setLayout(telay);

    auto ef_group = new QGroupBox("EuclideanFitnessEpsilon", _box);
    auto eflay = new QVBoxLayout(_box);
    ef = new QLineEdit("0.002", _box);
    eflay->addWidget(ef);
    ef_group->setLayout(eflay);

    auto mi_group = new QGroupBox("MaximumIterations", _box);
    auto milay = new QVBoxLayout(_box);
    mi = new QLineEdit("0", _box);
    milay->addWidget(mi);
    mi_group->setLayout(milay);

    loading_dialog = new LoadingDialog(nullptr);

    auto calc_btn = new QPushButton("计算", _box);

    _layout->addWidget(mcd_group);
    _layout->addWidget(te_group);
    _layout->addWidget(ef_group);
    _layout->addWidget(mi_group);
    _layout->addWidget(calc_btn);

    _box->setLayout(_layout);
    _box->resize(200, 500);
    _box->installEventFilter(this);

    connect(calc_btn, &QPushButton::clicked, [=] { compute(); });

    connect(this, &CvPointCloudICPModel::computingStarted, [=] { loading_dialog->show(); });

    connect(this, &CvPointCloudICPModel::computingFinished, [=] {
        loading_dialog->hide();
        loading_dialog->close();
    });
}

unsigned int CvPointCloudICPModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 2;
        break;

    case PortType::Out:
        result = 3;

    default:
        break;
    }
    return result;
}

bool CvPointCloudICPModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _box) {
        if (_showPc->empty()) {
            return false;
        }
        if (event->type() == QEvent::MouseButtonPress) {
            extern StreamerMainWindow *smw;
            smw->updateVTK(_showPc);
            return true;
        }
    }
    return false;
}

NodeDataType CvPointCloudICPModel::dataType(PortType const portType, PortIndex const portIndex) const
{
    if (portType == PortType::In) {
        if (portIndex == 0) {
            return PointCloudData().type();
        } else if (portIndex == 1) {
            return PointCloudData().type();
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

std::shared_ptr<NodeData> CvPointCloudICPModel::outData(PortIndex portIndex)
{
    if (portIndex == 0) {
        return std::make_shared<PointCloudData>(_outPc);
    } else if (portIndex == 1) {
        return std::make_shared<Matrix4fData>(_outMat4f);
    } else if (portIndex == 2) {
        return std::make_shared<ResultData>(_outRes);
    }
}

QJsonObject CvPointCloudICPModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["mcd"] = mcd->text();
    saver["te"] = te->text();
    saver["ef"] = ef->text();
    saver["mi"] = mi->text();
    return saver;
}

void CvPointCloudICPModel::load(const QJsonObject &js)
{
    mcd->setText(js["mcd"].toString());
    te->setText(js["te"].toString());
    ef->setText(js["ef"].toString());
    mi->setText(js["mi"].toString());
}

void CvPointCloudICPModel::compute()
{
    std::cout << "start compute" << std::endl;
    auto d1 = std::dynamic_pointer_cast<PointCloudData>(_inNodeData);
    auto d2 = std::dynamic_pointer_cast<PointCloudData>(_inTargetNodeData);

    bool f;
    auto mcd_v = mcd->text().toFloat(&f);
    auto te_v = te->text().toFloat(&f);
    auto ef_v = ef->text().toFloat(&f);
    auto mi_v = mi->text().toInt(&f);

    if (!f) {
        createMessageBox(nullptr, ":icons/error.png", "错误", "输入的参数必须为数字", 1, {"返回"});
        return;
    }

    _outRes.clear();

    Q_EMIT computingStarted();

    std::thread t([=] { this->calc(d1->getData(), d2->getData(), mcd_v, te_v, ef_v, mi_v); });

    loading_dialog->exec();
    t.join();

    extern StreamerMainWindow *smw;
    smw->updateVTK(_showPc);

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
    Q_EMIT dataUpdated(2);
}

void CvPointCloudICPModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const portIndex)
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
            _inTargetNodeData = nullptr;
            return;
        }
        _inTargetNodeData = nodeData;
    }
}

void CvPointCloudICPModel::calc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr d1,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr d2,
                                float mcd_v,
                                float te_v,
                                float ef_v,
                                int mi_v)
{
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(d1);
    icp.setInputTarget(d2);
    //    icp.setMaxCorrespondenceDistance(mcd_v);
    //    icp.setTransformationEpsilon(te_v);
    //    icp.setEuclideanFitnessEpsilon(ef_v);
    if (mi_v != 0) {
        icp.setMaximumIterations(mi_v);
    }

    icp.align(*_outPc);

    auto has_converged = icp.hasConverged();
    if (has_converged) {
        auto tm = icp.getFinalTransformation();
        _outMat4f << tm;
        *_showPc = (*_outPc) + (*d2);
        _outRes = "hasConverged: true\nscore: " + to_string(icp.getFitnessScore())
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
    }
    Q_EMIT
    computingFinished();
};