#include "../include/CvModels/CvMedianBlurModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvMedianBlurModel::CvMedianBlurModel()
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

    auto hlay = new QHBoxLayout();
    val = new QLineEdit("5");
    val->setValidator(pIntVld);
    val->setPlaceholderText("中值必须为奇数");
    auto lab = new QLabel("中值");
    createLineEditFormCurQObj(hlay, lab, val);

    auto all_lay = new QVBoxLayout();
    all_lay->addWidget(_label);
    all_lay->addLayout(hlay);

    _box->setLayout(all_lay);
    _box->resize(200, 200);

    connect(val, &QLineEdit::textEdited, [=](const QString &v) { compute(); });
}

unsigned int CvMedianBlurModel::nPorts(PortType portType) const
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

bool CvMedianBlurModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvMedianBlurModel::dataType(PortType const, PortIndex const) const
{
    return ImageData().type();
}

std::shared_ptr<NodeData> CvMedianBlurModel::outData(PortIndex)
{
    // todo 不能返回原来的了
    return std::make_shared<ImageData>(_mat);
}

void CvMedianBlurModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const)
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
void CvMedianBlurModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->mat().empty()) {
        return;
    }

    //    // todo ------------------------
    auto ksize = val->text().toInt();
    if (ksize % 2 == 0) {
        qDebug("中值必须为奇数!当前为偶数，自动+1转换成奇数!");
        ksize += 1;
    }
    cv::medianBlur(d->mat(), _mat, ksize);

    // todo ------------------------

    int w = _label->width();
    int h = _label->height();

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    Q_EMIT dataUpdated(0);
}
void CvMedianBlurModel::load(const QJsonObject &s)
{
    val->setText(s["val"].toString());
}

QJsonObject CvMedianBlurModel::save() const
{
    auto s = NodeDelegateModel::save();
    s["val"] = val->text();
    return s;
}
