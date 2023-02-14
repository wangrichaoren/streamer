#include "../include/CvModels/CvBlurModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvBlurModel::CvBlurModel()
    : _box(new QGroupBox())
    , _label(new QLabel("Image Visual"))
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

    auto g = new QGroupBox("卷积核大小",_box);
    auto vl = new QVBoxLayout(_box);
    auto h_lay1 = new QHBoxLayout(_box);
    auto h_lay2 = new QHBoxLayout(_box);
    auto lab1 = new QLabel("宽",_box);
    auto lab2 = new QLabel("高",_box);
    val1 = new QLineEdit("5",_box);
    val2 = new QLineEdit("5",_box);
    createLineEditFormCurQObj(h_lay1, lab1, val1);
    createLineEditFormCurQObj(h_lay2, lab2, val2);
    vl->addLayout(h_lay1);
    vl->addLayout(h_lay2);
    g->setLayout(vl);

    auto all_lay = new QVBoxLayout(_box);
    all_lay->addWidget(_label);
    all_lay->addWidget(g);

    _box->setLayout(all_lay);
    _box->resize(200, 200);

    connect(val1, &QLineEdit::editingFinished, [=] { compute(); });
    connect(val2, &QLineEdit::editingFinished, [=] { compute(); });
}

unsigned int CvBlurModel::nPorts(PortType portType) const
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

bool CvBlurModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvBlurModel::dataType(PortType const, PortIndex const) const
{
    return ImageData().type();
}

std::shared_ptr<NodeData> CvBlurModel::outData(PortIndex)
{
    // todo 不能返回原来的了
    return std::make_shared<ImageData>(_mat);
}

void CvBlurModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const)
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
void CvBlurModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->mat().empty()) {
        return;
    }

    // todo ------------------------

    cv::blur(d->mat(), _mat, cv::Size(val1->text().toInt(), val2->text().toInt()));

    // todo ------------------------

    int w = _label->width();
    int h = _label->height();

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    Q_EMIT dataUpdated(0);
}
QJsonObject CvBlurModel::save() const
{
    auto s = NodeDelegateModel::save();
    s["w"] = val1->text();
    s["h"] = val2->text();
    return s;
}
void CvBlurModel::load(const QJsonObject &s)
{
    //    NodeDelegateModel::load(<unnamed>);
    val1->setText(s["w"].toString());
    val2->setText(s["h"].toString());
}
