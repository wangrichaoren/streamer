#include "../include/CvModels/CvDistanceTransformModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include <QRadioButton>
#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvDistanceTransformModel::CvDistanceTransformModel()
    : _box(new QGroupBox())
{
    auto bf = _box->font();
    bf.setBold(true);
    _box->setFont(bf);

    auto type_group = new QGroupBox("Type", _box);
    auto type_lay = new QGridLayout(_box);
    l1 = new QRadioButton("DIST_L1", _box);
    l2 = new QRadioButton("DIST_L2", _box);
    l3 = new QRadioButton("DIST_C", _box);
    l4 = new QRadioButton("DIST_L12", _box);
    l5 = new QRadioButton("DIST_FAIR", _box);
    l6 = new QRadioButton("DIST_WELSCH", _box);
    l7 = new QRadioButton("DIST_HUBER", _box);
    type_lay->addWidget(l1);
    type_lay->addWidget(l2);
    type_lay->addWidget(l3);
    type_lay->addWidget(l4);
    type_lay->addWidget(l5);
    type_lay->addWidget(l6);
    type_lay->addWidget(l7);
    type_group->setLayout(type_lay);
    auto mask_group = new QGroupBox("MaskSize", _box);
    auto mask_lay = new QVBoxLayout(_box);
    mask_size_combox = new QComboBox(_box);
    QStringList size;
    size << "0"
         << "3"
         << "5";
    mask_size_combox->addItems(size);

    mask_lay->addWidget(mask_size_combox);
    mask_group->setLayout(mask_lay);

    l1->setChecked(true);

    auto all_lay = new QVBoxLayout(_box);
    all_lay->addWidget(type_group);
    all_lay->addWidget(mask_group);
    _box->setLayout(all_lay);

    _box->resize(200, 200);

    connect(l1, &QRadioButton::clicked, [=] { compute(); });
    connect(l2, &QRadioButton::clicked, [=] { compute(); });
    connect(l3, &QRadioButton::clicked, [=] { compute(); });
    connect(l4, &QRadioButton::clicked, [=] { compute(); });
    connect(l5, &QRadioButton::clicked, [=] { compute(); });
    connect(l6, &QRadioButton::clicked, [=] { compute(); });
    connect(l7, &QRadioButton::clicked, [=] { compute(); });

    connect(mask_size_combox, &QComboBox::currentTextChanged, [=] { compute(); });
}

unsigned int CvDistanceTransformModel::nPorts(PortType portType) const
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
    };

    return result;
}

NodeDataType CvDistanceTransformModel::dataType(PortType const portType,
                                                PortIndex const portIndex) const
{
    switch (portType) {
    case PortType::In:
        return ImageData().type();

    case PortType::Out:
        switch (portIndex) {
        case 0:
            return ImageData().type();
        case 1:
            return ResultData().type();
        }

    case PortType::None:
        break;
    }
    // FIXME: control may reach end of non-void function [-Wreturn-type]
    return NodeDataType();
}

std::shared_ptr<NodeData> CvDistanceTransformModel::outData(PortIndex idx)
{
    if (idx == 0) {
        return std::make_shared<ImageData>(_mat);
    } else if (idx == 1) {
        return std::make_shared<ResultData>(res);
    }
}

void CvDistanceTransformModel::setInData(std::shared_ptr<NodeData> nodeData,
                                         PortIndex const portIndex)
{
    _nodeData = nodeData;

    if (_nodeData) {
        auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
        if (d->mat().empty()) {
            return;
        };

        this->compute();

    } else {
        return;
    }
}
void CvDistanceTransformModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->mat().empty()) {
        return;
    }

    bool f;
    auto size_val = mask_size_combox->currentText().toInt(&f);

    if (!f) {
        return;
    }
    res.clear();

    int distance_type = 0;
    if (l1->isChecked()) {
        distance_type = cv::DIST_L1;
    }
    if (l2->isChecked()) {
        distance_type = cv::DIST_L2;
    }
    if (l3->isChecked()) {
        distance_type = cv::DIST_C;
    }
    if (l4->isChecked()) {
        distance_type = cv::DIST_L12;
    }
    if (l5->isChecked()) {
        distance_type = cv::DIST_FAIR;
    }
    if (l6->isChecked()) {
        distance_type = cv::DIST_WELSCH;
    }
    if (l7->isChecked()) {
        distance_type = cv::DIST_HUBER;
    }
    cv::distanceTransform(d->mat(), _mat, distance_type, size_val);

//    cv::normalize(_mat, _mat, 0, 1.0, cv::NORM_MINMAX);
//    cv::threshold(_mat, _mat, 0.1, 1.0, cv::THRESH_BINARY);
//    normalize(_mat, _mat, 0, 255, cv::NORM_MINMAX);
//    _mat.convertTo(_mat, CV_8UC1);//
////    cv::imshow("test2", _mat);
//    cv::imwrite("/home/wrc/Downloads/test.png",_mat);

    Q_EMIT dataUpdated(0);
}
void CvDistanceTransformModel::load(const QJsonObject &s)
{
    l1->setChecked(s["l1"].toBool());
    l2->setChecked(s["l2"].toBool());
    l3->setChecked(s["l3"].toBool());
    l4->setChecked(s["l4"].toBool());
    l5->setChecked(s["l5"].toBool());
    l6->setChecked(s["l6"].toBool());
    l7->setChecked(s["l7"].toBool());

    mask_size_combox->setCurrentText(s["size"].toString());
}

QJsonObject CvDistanceTransformModel::save() const
{
    auto s = NodeDelegateModel::save();
    s["l1"] = l1->isChecked();
    s["l2"] = l2->isChecked();
    s["l3"] = l3->isChecked();
    s["l4"] = l4->isChecked();
    s["l5"] = l5->isChecked();
    s["l6"] = l6->isChecked();
    s["l7"] = l7->isChecked();

    s["size"] = mask_size_combox->currentText();

    return s;
}