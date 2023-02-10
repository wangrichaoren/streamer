#include "../include/CvModels/CvFilter2dModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvFilter2dModel::CvFilter2dModel()
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

    auto pIntVld = new QIntValidator();

    dep_edit = new QLineEdit("-1");
    dep_edit->setValidator(pIntVld);
    auto dep_lay = new QHBoxLayout();
    auto dep_lab = new QLabel("深度");
    createLineEditFormCurQObj(dep_lay, dep_lab, dep_edit);

    auto all_lay = new QVBoxLayout();

    auto kernel_G = new QGroupBox("卷积核3*3");
    auto v_line_lay = new QVBoxLayout();

    auto h_lay1 = new QHBoxLayout();
    auto h_lay2 = new QHBoxLayout();
    auto h_lay3 = new QHBoxLayout();

    n00 = new QLineEdit("-1");
    n01 = new QLineEdit("-1");
    n02 = new QLineEdit("-1");
    n10 = new QLineEdit("-1");
    n11 = new QLineEdit("9");
    n12 = new QLineEdit("-1");
    n20 = new QLineEdit("-1");
    n21 = new QLineEdit("-1");
    n22 = new QLineEdit("-1");

    n00->setValidator(pIntVld);
    n01->setValidator(pIntVld);
    n02->setValidator(pIntVld);
    n10->setValidator(pIntVld);
    n11->setValidator(pIntVld);
    n12->setValidator(pIntVld);
    n20->setValidator(pIntVld);
    n21->setValidator(pIntVld);
    n22->setValidator(pIntVld);

    h_lay1->addWidget(n00);
    h_lay1->addWidget(n01);
    h_lay1->addWidget(n02);

    h_lay2->addWidget(n10);
    h_lay2->addWidget(n11);
    h_lay2->addWidget(n12);

    h_lay3->addWidget(n20);
    h_lay3->addWidget(n21);
    h_lay3->addWidget(n22);

    v_line_lay->addLayout(h_lay1);
    v_line_lay->addLayout(h_lay2);
    v_line_lay->addLayout(h_lay3);
    kernel_G->setLayout(v_line_lay);

    all_lay->addWidget(_label);
    all_lay->addLayout(dep_lay);
    all_lay->addWidget(kernel_G);
    _box->setLayout(all_lay);
    _box->resize(200, 200);

    connect(dep_edit, &QLineEdit::textChanged, [=] { compute(); });
    connect(n00, &QLineEdit::textChanged, [=] { compute(); });
    connect(n01, &QLineEdit::textChanged, [=] { compute(); });
    connect(n02, &QLineEdit::textChanged, [=] { compute(); });
    connect(n10, &QLineEdit::textChanged, [=] { compute(); });
    connect(n11, &QLineEdit::textChanged, [=] { compute(); });
    connect(n12, &QLineEdit::textChanged, [=] { compute(); });
    connect(n20, &QLineEdit::textChanged, [=] { compute(); });
    connect(n21, &QLineEdit::textChanged, [=] { compute(); });
    connect(n22, &QLineEdit::textChanged, [=] { compute(); });
}

unsigned int CvFilter2dModel::nPorts(PortType portType) const
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

bool CvFilter2dModel::eventFilter(QObject *object, QEvent *event)
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
        }
    }
    return false;
}

NodeDataType CvFilter2dModel::dataType(PortType const, PortIndex const) const
{
    return ImageData().type();
}

std::shared_ptr<NodeData> CvFilter2dModel::outData(PortIndex)
{
    // todo 不能返回原来的了
    return std::make_shared<ImageData>(_mat);
}

void CvFilter2dModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const)
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
void CvFilter2dModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->mat().empty()) {
        return;
    }

    cv::Mat kernel = (cv::Mat_<char>(3, 3) << n00->text().toInt(),
                      n01->text().toInt(),
                      n02->text().toInt(),
                      n10->text().toInt(),
                      n11->text().toInt(),
                      n12->text().toInt(),
                      n20->text().toInt(),
                      n21->text().toInt(),
                      n22->text().toInt());
    filter2D(d->mat(), _mat, dep_edit->text().toInt(), kernel);

    int w = _label->width();
    int h = _label->height();

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    Q_EMIT dataUpdated(0);
}
QJsonObject CvFilter2dModel::save() const
{
    auto s = NodeDelegateModel::save();
    s["dep"] = dep_edit->text();

    s["n00"] = n00->text();
    s["n01"] = n01->text();
    s["n02"] = n02->text();
    s["n10"] = n10->text();
    s["n11"] = n11->text();
    s["n12"] = n12->text();
    s["n20"] = n20->text();
    s["n21"] = n21->text();
    s["n22"] = n22->text();

    return s;
}
void CvFilter2dModel::load(const QJsonObject &s)
{
    dep_edit->setText(s["dep"].toString());

    n00->setText(s["n00"].toString());
    n01->setText(s["n01"].toString());
    n02->setText(s["n02"].toString());
    n10->setText(s["n10"].toString());
    n11->setText(s["n11"].toString());
    n12->setText(s["n12"].toString());
    n20->setText(s["n20"].toString());
    n21->setText(s["n21"].toString());
    n22->setText(s["n22"].toString());
}
