#include "../include/CvModels/CvFindContoursModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvFindContoursModel::CvFindContoursModel()
    : _label(new QLabel("Image Visual"))
    , _box(new QGroupBox())
{
    auto bf = _box->font();
    bf.setBold(true);
    _box->setFont(bf);

    // show label
    _label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    QFont f = _label->font();
    f.setItalic(true);
    _label->setFont(f);
    _label->setMinimumSize(200, 200);
    _label->installEventFilter(this);

    // todo
    auto mode_group = new QGroupBox("模式");
    auto mode_vlay = new QVBoxLayout();
    mode_retr_external = new QRadioButton("RETR_EXTERNAL");
    mode_retr_list = new QRadioButton("RETR_LIST");
    mode_retr_ccomp = new QRadioButton("RETR_CCOMP");
    mode_retr_tree = new QRadioButton("RETR_TREE");
    mode_vlay->addWidget(mode_retr_external);
    mode_vlay->addWidget(mode_retr_list);
    mode_vlay->addWidget(mode_retr_ccomp);
    mode_vlay->addWidget(mode_retr_tree);
    mode_group->setLayout(mode_vlay);
    mode_retr_external->setChecked(true);

    auto method_group = new QGroupBox("方法");
    auto method_vlay = new QVBoxLayout();
    method_chain_approx_none = new QRadioButton("CHAIN_APPROX_NONE");
    method_chain_approx_simple = new QRadioButton("CHAIN_APPROX_SIMPLE");
    method_chain_approx_tc89_l1 = new QRadioButton("CHAIN_APPROX_TC89_L1");
    method_chain_approx_tc89_kcos = new QRadioButton("CHAIN_APPROX_TC89_KCOS");
    method_vlay->addWidget(method_chain_approx_none);
    method_vlay->addWidget(method_chain_approx_simple);
    method_vlay->addWidget(method_chain_approx_tc89_l1);
    method_vlay->addWidget(method_chain_approx_tc89_kcos);
    method_group->setLayout(method_vlay);
    method_chain_approx_none->setChecked(true);

    auto all_lay = new QVBoxLayout();
    //    all_lay->addWidget(_label);
    all_lay->addWidget(mode_group);
    all_lay->addWidget(method_group);
    _box->setLayout(all_lay);
    // todo

    _box->resize(200, 200);

    connect(mode_retr_external, &QRadioButton::clicked, [=] { compute(); });
    connect(mode_retr_list, &QRadioButton::clicked, [=] { compute(); });
    connect(mode_retr_ccomp, &QRadioButton::clicked, [=] { compute(); });
    connect(mode_retr_tree, &QRadioButton::clicked, [=] { compute(); });

    connect(method_chain_approx_none, &QRadioButton::clicked, [=] { compute(); });
    connect(method_chain_approx_simple, &QRadioButton::clicked, [=] { compute(); });
    connect(method_chain_approx_tc89_l1, &QRadioButton::clicked, [=] { compute(); });
    connect(method_chain_approx_tc89_kcos, &QRadioButton::clicked, [=] { compute(); });
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

bool CvFindContoursModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _label) {
        int w = _label->width();
        int h = _label->height();

        if (event->type() == QEvent::Resize) {
            auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
            if (d) {
                if (_mat.empty()) {
                    return false;
                };
                auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
                _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));
            }
        } else if (event->type() == QEvent::MouseButtonPress) {
            if (_mat.empty()) {
                return false;
            }
            auto shower = new Full2DDialog(nullptr, &_mat);
            shower->setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
            shower->showNormal();
            shower->exec();
            shower->deleteLater();
        }
    }
    return false;
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
        _label->setPixmap(QPixmap());
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
    // todo ----

    int w = _label->width();
    int h = _label->height();

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

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