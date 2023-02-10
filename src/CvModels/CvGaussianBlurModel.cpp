#include "../include/CvModels/CvGaussianBlurModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvGaussianBlurModel::CvGaussianBlurModel()
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

    auto kG = new QGroupBox("卷积核");
    auto kvlay = new QVBoxLayout();
    _kredi = new QLineEdit("5");
    _kcedi = new QLineEdit("5");
    _kredi->setValidator(pIntVld);
    _kcedi->setValidator(pIntVld);
    auto rlab = new QLabel("宽");
    auto clab = new QLabel("高");
    auto rlay = new QHBoxLayout();
    auto clay = new QHBoxLayout();
    createLineEditFormCurQObj(rlay, rlab, _kredi);
    createLineEditFormCurQObj(clay, clab, _kcedi);
    kvlay->addLayout(rlay);
    kvlay->addLayout(clay);
    kG->setLayout(kvlay);

    auto sigmaG = new QGroupBox("Sigma");
    auto svlay = new QVBoxLayout();
    auto xlay = new QHBoxLayout();
    auto ylay = new QHBoxLayout();
    auto xlab = new QLabel("SigmaX");
    auto ylab = new QLabel("SigmaY");
    _sigmx = new QLineEdit("0");
    _sigmy = new QLineEdit("0");
    _sigmx->setValidator(pIntVld);
    _sigmy->setValidator(pIntVld);
    createLineEditFormCurQObj(xlay, xlab, _sigmx);
    createLineEditFormCurQObj(ylay, ylab, _sigmy);
    svlay->addLayout(xlay);
    svlay->addLayout(ylay);
    sigmaG->setLayout(svlay);

    auto all_lay = new QVBoxLayout();
    all_lay->addWidget(_label);
    all_lay->addWidget(kG);
    all_lay->addWidget(sigmaG);

    _box->setLayout(all_lay);
    _box->resize(200, 200);

    connect(_kredi, &QLineEdit::textEdited, [=](const QString &v) { compute(); });
    connect(_kcedi, &QLineEdit::textEdited, [=](const QString &v) { compute(); });
    connect(_sigmx, &QLineEdit::textEdited, [=](const QString &v) { compute(); });
    connect(_sigmy, &QLineEdit::textEdited, [=](const QString &v) { compute(); });
}

unsigned int CvGaussianBlurModel::nPorts(PortType portType) const
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

bool CvGaussianBlurModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvGaussianBlurModel::dataType(PortType const, PortIndex const) const
{
    return ImageData().type();
}

std::shared_ptr<NodeData> CvGaussianBlurModel::outData(PortIndex)
{
    // todo 不能返回原来的了
    return std::make_shared<ImageData>(_mat);
}

void CvGaussianBlurModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const)
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
void CvGaussianBlurModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->mat().empty()) {
        return;
    }

    //    // todo ------------------------
    int width = _kredi->text().toInt();
    int height = _kcedi->text().toInt();
    if (width % 2 == 0) {
        width += 1;
    }
    if (height % 2 == 0) {
        height += 1;
    }
    cv::GaussianBlur(d->mat(),
                     _mat,
                     cv::Size(width, height),
                     _sigmy->text().toInt(),
                     _sigmx->text().toInt());
    // todo ------------------------

    int w = _label->width();
    int h = _label->height();

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    Q_EMIT dataUpdated(0);
}
void CvGaussianBlurModel::load(const QJsonObject &s)
{
    //    NodeDelegateModel::load(<unnamed>);
    _kredi->setText(s["w"].toString());
    _kcedi->setText(s["h"].toString());
    _sigmx->setText(s["sigmx"].toString());
    _sigmy->setText(s["sigmy"].toString());
}

QJsonObject CvGaussianBlurModel::save() const
{
    auto s = NodeDelegateModel::save();
    s["w"] = _kredi->text();
    s["h"] = _kcedi->text();
    s["sigmx"] = _sigmx->text();
    s["sigmy"] = _sigmy->text();

    return s;
}
