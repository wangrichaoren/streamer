#include "../include/CvModels/CvImageShowModel.hpp"

#include "../include/DataTypes/ImageData.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvImageShowModel::CvImageShowModel()
    : _box(new QGroupBox())
    , _label(new QLabel("Image Visual", _box))
    , _saveBtn(new QPushButton("保存", _box))
    , _layout(new QVBoxLayout(_box))
{
    _box->setStyleSheet("QGroupBox{padding-top:15px; margin-top:-15px;padding-left:15px; "
                        "margin-left:-15px;padding-right:15px; margin-right:-15px;}");

    _label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);

    QFont f = _label->font();
    f.setBold(true);
    f.setItalic(true);

    _label->setFont(f);

    _label->setMinimumSize(200, 200);

    auto bf = _saveBtn->font();
    bf.setBold(true);
    _saveBtn->setFont(bf);

    _layout->addWidget(_label);
    _layout->addWidget(_saveBtn);

    _box->setLayout(_layout);

    _label->installEventFilter(this);
    _saveBtn->installEventFilter(this);
}

unsigned int CvImageShowModel::nPorts(PortType portType) const
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

bool CvImageShowModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _label) {
        this->embeddedWidget()->setCursor(Qt::PointingHandCursor);
        int w = _label->width();
        int h = _label->height();

        if (event->type() == QEvent::Resize) {
            auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
            if (d) {
                if (d->mat().empty()) {
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
    } else if (object == _saveBtn) {
        if (event->type() == QEvent::MouseButtonPress) {
            QString fileName = QFileDialog::getSaveFileName(nullptr,
                                                            tr("Save Image"),
                                                            QDir::homePath(),
                                                            tr("Image Files (*.png *.jpg *.bmp)"));

            if (!fileName.isEmpty()) {
                if (!fileName.endsWith("png", Qt::CaseInsensitive))
                    fileName += ".png";
                auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
                cv::imwrite(fileName.toStdString(), d->mat());
            }
        }
    }
    return false;
}

NodeDataType CvImageShowModel::dataType(PortType const, PortIndex const) const
{
    return ImageData().type();
}

std::shared_ptr<NodeData> CvImageShowModel::outData(PortIndex)
//# TODO 要是out有两个呢？
{
    return _nodeData;
}

void CvImageShowModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const)
{
    _nodeData = nodeData;

    if (_nodeData) {
        auto d = std::dynamic_pointer_cast<ImageData>(_nodeData);
        if (d->mat().empty()) {
            return;
        }
        _mat = d->mat();
        int w = _label->width();
        int h = _label->height();
//        if (_mat.channels() == 1) {
//            cv::cvtColor(_mat, _mat, cv::COLOR_GRAY2RGB);
//        } else if (_mat.channels() == 4) {
//            cv::cvtColor(_mat, _mat, cv::COLOR_RGBA2RGB);
//        }
        auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
        _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));
    } else {
        _label->setPixmap(QPixmap());
    }

    // dataUpdated 触发下游的节点更新
    Q_EMIT dataUpdated(0);
}
