#include "CvBinaryModel.hpp"

#include "ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvBinaryModel::CvBinaryModel()
    : _label(new QLabel("Image Visual"))
{
    _label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);

    QFont f = _label->font();
    f.setBold(true);
    f.setItalic(true);

    _label->setFont(f);

    _label->setMinimumSize(200, 200);

    _label->installEventFilter(this);
}

unsigned int CvBinaryModel::nPorts(PortType portType) const
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

bool CvBinaryModel::eventFilter(QObject *object, QEvent *event)
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

NodeDataType CvBinaryModel::dataType(PortType const, PortIndex const) const
{
    return ImageData().type();
}

std::shared_ptr<NodeData> CvBinaryModel::outData(PortIndex)
{
    // todo 不能返回原来的了
    return std::make_shared<ImageData>(_mat);
}

void CvBinaryModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const)
{
    _nodeData = nodeData;

    if (_nodeData) {
        auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
        if (d->mat().empty()) {
            return;
        };

        int w = _label->width();
        int h = _label->height();

        // to bin
        cv::threshold(d->mat(),_mat,125,255,cv::THRESH_BINARY);

        auto pix = QPixmap::fromImage(cvMat2QImage(_mat));

        _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    } else {
        _label->setPixmap(QPixmap());
    }

    // dataUpdated 触发下游的节点更新
    Q_EMIT dataUpdated(0);
}
