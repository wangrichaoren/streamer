#include "../include/CvModels/CvFindContoursModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include <QComboBox>
#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvFindContoursModel::CvFindContoursModel()
    : _box(new QGroupBox())
{
    auto bf = _box->font();
    bf.setBold(true);
    _box->setFont(bf);

    auto mode_group = new QGroupBox("模式", _box);
    auto mode_vlay = new QVBoxLayout(_box);
    mode_retr_external = new QRadioButton("RETR_EXTERNAL", _box);
    mode_retr_list = new QRadioButton("RETR_LIST", _box);
    mode_retr_ccomp = new QRadioButton("RETR_CCOMP", _box);
    mode_retr_tree = new QRadioButton("RETR_TREE", _box);
    mode_vlay->addWidget(mode_retr_external);
    mode_vlay->addWidget(mode_retr_list);
    mode_vlay->addWidget(mode_retr_ccomp);
    mode_vlay->addWidget(mode_retr_tree);
    mode_group->setLayout(mode_vlay);
    mode_retr_external->setChecked(true);

    auto method_group = new QGroupBox("方法", _box);
    auto method_vlay = new QVBoxLayout(_box);
    method_chain_approx_none = new QRadioButton("CHAIN_APPROX_NONE", _box);
    method_chain_approx_simple = new QRadioButton("CHAIN_APPROX_SIMPLE", _box);
    method_chain_approx_tc89_l1 = new QRadioButton("CHAIN_APPROX_TC89_L1", _box);
    method_chain_approx_tc89_kcos = new QRadioButton("CHAIN_APPROX_TC89_KCOS", _box);
    method_vlay->addWidget(method_chain_approx_none);
    method_vlay->addWidget(method_chain_approx_simple);
    method_vlay->addWidget(method_chain_approx_tc89_l1);
    method_vlay->addWidget(method_chain_approx_tc89_kcos);
    method_group->setLayout(method_vlay);
    method_chain_approx_none->setChecked(true);

//    // 按面积提取
//    auto extract_group = new QGroupBox("提取", _box);
//    auto ex_lay = new QVBoxLayout(_box);
//    auto ex_com = new QComboBox(_box);
//    ex_lay->addWidget(ex_com);
//    extract_group->setLayout(ex_lay);
//    // todo 其他提取方式待添加
//    QStringList extractList;
//    extractList << "按面积范围提取";
//    //                << "按父子关系提取";
//    ex_com->addItems(extractList);
//    auto area_group = new QGroupBox(_box);
//    auto area_lay = new QHBoxLayout(_box);
//    area_min_ed = new QLineEdit("0", _box);
//    area_max_ed = new QLineEdit("10000", _box);
//    auto area_label = new QLabel("~", _box);
//    area_lay->addWidget(area_min_ed);
//    area_lay->addWidget(area_label);
//    area_lay->addWidget(area_max_ed);
//    area_group->setLayout(area_lay);
//    ex_lay->addWidget(area_group);

    auto all_lay = new QVBoxLayout(_box);
    all_lay->addWidget(mode_group);
    all_lay->addWidget(method_group);
//    all_lay->addWidget(extract_group);
    _box->setLayout(all_lay);

    _box->resize(200, 200);

    connect(mode_retr_external, &QRadioButton::clicked, [=] { compute(); });
    connect(mode_retr_list, &QRadioButton::clicked, [=] { compute(); });
    connect(mode_retr_ccomp, &QRadioButton::clicked, [=] { compute(); });
    connect(mode_retr_tree, &QRadioButton::clicked, [=] { compute(); });

    connect(method_chain_approx_none, &QRadioButton::clicked, [=] { compute(); });
    connect(method_chain_approx_simple, &QRadioButton::clicked, [=] { compute(); });
    connect(method_chain_approx_tc89_l1, &QRadioButton::clicked, [=] { compute(); });
    connect(method_chain_approx_tc89_kcos, &QRadioButton::clicked, [=] { compute(); });

//    connect(ex_com, &QComboBox::currentTextChanged, [=] {
//        //       std::cout<<ex_com->currentText().toStdString()<<std::endl;
//    });
}

unsigned int CvFindContoursModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 1;
        break;

    case PortType::Out:
        result = 1;

    default:
        break;
    };

    return result;
}

NodeDataType CvFindContoursModel::dataType(PortType const portType, PortIndex const portIndex) const
{
    switch (portType) {
    case PortType::In:
        return ImageData().type();

    case PortType::Out:
        return ContoursData().type();

    case PortType::None:
        break;
    }
    // FIXME: control may reach end of non-void function [-Wreturn-type]
    return NodeDataType();
}

std::shared_ptr<NodeData> CvFindContoursModel::outData(PortIndex)
{
    ContoursDataStruct dataStruct;
    dataStruct.contours = contours;
    dataStruct.hierachy = hierachy;
    ContoursData data(dataStruct);
    return std::make_shared<ContoursData>(data);
}

void CvFindContoursModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const portIndex)
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

    // dataUpdated 触发下游的节点更新
    Q_EMIT dataUpdated(0);
}
void CvFindContoursModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->mat().empty()) {
        return;
    }

    bool mode_f;
    bool method_f;

    if (mode_retr_external->isChecked())
        mode_f = CV_RETR_EXTERNAL;
    if (mode_retr_list->isChecked())
        mode_f = CV_RETR_LIST;
    if (mode_retr_ccomp->isChecked())
        mode_f = CV_RETR_CCOMP;
    if (mode_retr_tree->isChecked())
        mode_f = CV_RETR_TREE;
    if (method_chain_approx_none->isChecked())
        method_f = CV_CHAIN_APPROX_NONE;
    if (method_chain_approx_simple->isChecked())
        method_f = CV_CHAIN_APPROX_SIMPLE;
    if (method_chain_approx_tc89_l1->isChecked())
        method_f = CV_CHAIN_APPROX_TC89_L1;
    if (method_chain_approx_tc89_kcos->isChecked())
        method_f = CV_CHAIN_APPROX_TC89_KCOS;

    d->mat().copyTo(_mat);

    cv::findContours(_mat, contours, hierachy, mode_f, method_f);

    //    drawContours(_mat, contours, -1, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);

    //    for (const auto& c : contours) {
    //        double area = cv::contourArea(c);
    //        //        double arc = cv::arcLength(c, true);
    ////        std::cout << "面积: " << area << std::endl;
    //        //        cv::Point2f center;
    //        //        float radius;
    //        //        cv::minEnclosingCircle(c, center, radius);
    //        //        //        _mat.
    //        //        cv::circle(_mat, center, radius, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    //    }

    Q_EMIT dataUpdated(0);
}
void CvFindContoursModel::load(const QJsonObject &s)
{
    mode_retr_external->setChecked(s["mode_retr_external"].toBool());
    mode_retr_list->setChecked(s["mode_retr_list"].toBool());
    mode_retr_ccomp->setChecked(s["mode_retr_ccomp"].toBool());
    mode_retr_tree->setChecked(s["mode_retr_tree"].toBool());
    method_chain_approx_none->setChecked(s["method_chain_approx_none"].toBool());
    method_chain_approx_simple->setChecked(s["method_chain_approx_simple"].toBool());
    method_chain_approx_tc89_l1->setChecked(s["method_chain_approx_tc89_l1"].toBool());
    method_chain_approx_tc89_kcos->setChecked(s["method_chain_approx_tc89_kcos"].toBool());
}

QJsonObject CvFindContoursModel::save() const
{
    auto s = NodeDelegateModel::save();
    s["mode_retr_external"] = mode_retr_external->isChecked();
    s["mode_retr_list"] = mode_retr_list->isChecked();
    s["mode_retr_ccomp"] = mode_retr_ccomp->isChecked();
    s["mode_retr_tree"] = mode_retr_tree->isChecked();
    s["method_chain_approx_none"] = method_chain_approx_none->isChecked();
    s["method_chain_approx_simple"] = method_chain_approx_simple->isChecked();
    s["method_chain_approx_tc89_l1"] = method_chain_approx_tc89_l1->isChecked();
    s["method_chain_approx_tc89_kcos"] = method_chain_approx_tc89_kcos->isChecked();

    return s;
}