#include "CvModels/CvShapeBaseMatchModel.hpp"

#include "DataTypes/ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvShapeBaseMatchModel::CvShapeBaseMatchModel()
    : _box(new QGroupBox())
    , _label(new QLabel("Image Visual", _box))
    , all_lay(new QVBoxLayout(_box))
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
    all_lay->addWidget(_label);
    // todo
    _box->setLayout(all_lay);
    _box->resize(200, 200);
}

unsigned int CvShapeBaseMatchModel::nPorts(PortType portType) const
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

bool CvShapeBaseMatchModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _label) {
        this->embeddedWidget()->setCursor(Qt::PointingHandCursor);
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

NodeDataType CvShapeBaseMatchModel::dataType(PortType const portType,
                                             PortIndex const portIndex) const
{
    return ImageData().type();
}

std::shared_ptr<NodeData> CvShapeBaseMatchModel::outData(PortIndex)
{
    return std::make_shared<ImageData>(_mat);
}

void CvShapeBaseMatchModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const)
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
void CvShapeBaseMatchModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
    if (!d) {
        return;
    }
    if (d->mat().empty()) {
        return;
    }

    // todo ----

    // todo ----

    int w = _label->width();
    int h = _label->height();

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    Q_EMIT dataUpdated(0);
}
void CvShapeBaseMatchModel::load(const QJsonObject &s) {}

QJsonObject CvShapeBaseMatchModel::save() const
{
    auto s = NodeDelegateModel::save();

    return s;
}