#include "CvModels/CvPointCloudSampleConsensusModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include "Widget/StreamerMainWindow.hpp"
#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvPointCloudSampleConsensusModel::CvPointCloudSampleConsensusModel()
    : _box(new QGroupBox())
    , _layout(new QVBoxLayout(_box))
{
    _box->setMinimumSize(200, 200);
    _box->setMaximumSize(500, 500);

    auto type_group = new QGroupBox("Type", _box);
    p = new QRadioButton("plane", _box);
    ci = new QRadioButton("circle", _box);
    sp = new QRadioButton("sphere", _box);
    cy = new QRadioButton("cylinder", _box);
    st = new QRadioButton("stick", _box);
    npp = new QRadioButton("normal_parallel_plane", _box);
    pp = new QRadioButton("perpendicular_plane", _box);
    auto type_lay = new QGridLayout(_box);
    type_lay->addWidget(p);
    type_lay->addWidget(ci);
    type_lay->addWidget(sp);
    type_lay->addWidget(cy);
    type_lay->addWidget(st);
    type_lay->addWidget(npp);
    type_lay->addWidget(pp);
    type_group->setLayout(type_lay);
    p->setChecked(true);

    auto dt_group = new QGroupBox("DistanceThreshold", _box);
    auto dt_lay = new QVBoxLayout(_box);
    th = new QLineEdit("0.01");
    dt_lay->addWidget(th);
    dt_group->setLayout(dt_lay);

    _layout->addWidget(type_group);
    _layout->addWidget(dt_group);

    _box->setLayout(_layout);
    _box->resize(200, 300);
    _box->installEventFilter(this);

    connect(p, &QRadioButton::toggled, [=] { compute(); });
    connect(ci, &QRadioButton::toggled, [=] { compute(); });
    connect(sp, &QRadioButton::toggled, [=] { compute(); });
    connect(cy, &QRadioButton::toggled, [=] { compute(); });
    connect(st, &QRadioButton::toggled, [=] { compute(); });
    connect(npp, &QRadioButton::toggled, [=] { compute(); });
    connect(pp, &QRadioButton::toggled, [=] { compute(); });
    connect(th, &QLineEdit::editingFinished, [=] { compute(); });
}

unsigned int CvPointCloudSampleConsensusModel::nPorts(PortType portType) const
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

bool CvPointCloudSampleConsensusModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvPointCloudSampleConsensusModel::dataType(PortType const portType,
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

std::shared_ptr<NodeData> CvPointCloudSampleConsensusModel::outData(PortIndex portIndex)
{
    // todo reture inliner?
    if (portIndex == 0) {
        return std::make_shared<PointCloudData>(_pc);
    } else if (portIndex == 1) {
        return std::make_shared<ResultData>(_res);
    }
}
QJsonObject CvPointCloudSampleConsensusModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["p"] = p->isChecked();
    saver["ci"] = ci->isChecked();
    saver["sp"] = sp->isChecked();
    saver["cy"] = cy->isChecked();
    saver["st"] = st->isChecked();
    saver["npp"] = npp->isChecked();
    saver["pp"] = pp->isChecked();
    saver["th"] = th->text();
    return saver;
}

void CvPointCloudSampleConsensusModel::load(const QJsonObject &js)
{
    p->setChecked(js["p"].toBool());
    ci->setChecked(js["ci"].toBool());
    sp->setChecked(js["sp"].toBool());
    cy->setChecked(js["cy"].toBool());
    st->setChecked(js["st"].toBool());
    npp->setChecked(js["npp"].toBool());
    pp->setChecked(js["pp"].toBool());
    th->setText(js["th"].toString());
}

void CvPointCloudSampleConsensusModel::compute()
{
    auto d = std::dynamic_pointer_cast<PointCloudData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->getData()->empty()) {
        return;
    }

    std::vector<int> inliers;
    bool f;
    auto val = th->text().toFloat(&f);
    if (!f) {
        return;
    }

    if (p->isChecked()) {
        // 平面
        pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(d->getData()));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
        ransac.setDistanceThreshold(val);
        ransac.computeModel();
        ransac.getInliers(inliers);

    } else if (ci->isChecked()) {
        pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGB>::Ptr model(
            new pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGB>(d->getData()));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
        ransac.setDistanceThreshold(val);
        ransac.computeModel();
        ransac.getInliers(inliers);
    } else if (sp->isChecked()) {
        pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>::Ptr model(
            new pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>(d->getData()));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
        ransac.setDistanceThreshold(val);
        ransac.computeModel();
        ransac.getInliers(inliers);
    } else if (cy->isChecked()) {
        pcl::SampleConsensusModelCylinder<pcl::PointXYZRGB, pcl::Normal>::Ptr model(
            new pcl::SampleConsensusModelCylinder<pcl::PointXYZRGB, pcl::Normal>(d->getData()));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
        ransac.setDistanceThreshold(val);
        ransac.computeModel();
        ransac.getInliers(inliers);
    } else if (st->isChecked()) {
        pcl::SampleConsensusModelStick<pcl::PointXYZRGB>::Ptr model(
            new pcl::SampleConsensusModelStick<pcl::PointXYZRGB>(d->getData()));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
        ransac.setDistanceThreshold(val);
        ransac.computeModel();
        ransac.getInliers(inliers);
    } else if (npp->isChecked()) {
        pcl::SampleConsensusModelNormalParallelPlane<pcl::PointXYZRGB, pcl::Normal>::Ptr model(
            new pcl::SampleConsensusModelNormalParallelPlane<pcl::PointXYZRGB, pcl::Normal>(
                d->getData()));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
        ransac.setDistanceThreshold(val);
        ransac.computeModel();
        ransac.getInliers(inliers);
    } else if (pp->isChecked()) {
        pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>::Ptr model(
            new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGB>(d->getData()));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model);
        ransac.setDistanceThreshold(val);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }

    pcl::copyPointCloud(*d->getData(), inliers, *_pc);

    extern StreamerMainWindow *smw;
    smw->updateVTK(_pc);

    Q_EMIT dataUpdated(0);
    Q_EMIT dataUpdated(1);
}

void CvPointCloudSampleConsensusModel::setInData(std::shared_ptr<NodeData> nodeData,
                                                 PortIndex const portIndex)
{
    auto d = std::dynamic_pointer_cast<PointCloudData>(nodeData);
    if (!d) {
        return;
    }
    _nodeData = nodeData;
    compute();
};
