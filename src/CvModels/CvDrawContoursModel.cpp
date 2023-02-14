#include "../include/CvModels/CvDrawContoursModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtWidgets/QFileDialog>

CvDrawContoursModel::CvDrawContoursModel()
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

    // todo

    // todo
    auto all_lay = new QHBoxLayout(_box);
    all_lay->addWidget(_label);
    _box->setLayout(all_lay);

    _box->resize(200, 200);
}

unsigned int CvDrawContoursModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 2;
        break;

    case PortType::Out:
        result = 1;

    default:
        break;
    };

    return result;
}

bool CvDrawContoursModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _label) {
        int w = _label->width();
        int h = _label->height();

        if (event->type() == QEvent::Resize) {
            auto d = std::dynamic_pointer_cast<ImageData>(_imageNodeData);
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

NodeDataType CvDrawContoursModel::dataType(PortType const portType, PortIndex const portIndex) const
{
    switch (portType) {
    case PortType::In:
        if (portIndex == 0) {
            return ImageData().type();
        } else if (portIndex == 1) {
            return ContoursData().type();
        }
    case PortType::Out:
        return ImageData().type();
    case PortType::None:
        break;
    }
    return NodeDataType();
}

std::shared_ptr<NodeData> CvDrawContoursModel::outData(PortIndex)
{
    return std::make_shared<ImageData>(_mat);
}

void CvDrawContoursModel::setInData(std::shared_ptr<NodeData> nodeData, PortIndex const portIndex)
{
    if (portIndex == 0) {
        if (nodeData) {
            _imageNodeData = nodeData;
            auto d = std::dynamic_pointer_cast<ImageData>(nodeData);
            if (d->mat().empty()) {
                return;
            }
            if (d->mat().channels() == 1) {
                createMessageBox(nullptr,
                                 ":icons/error.png",
                                 "不正常的输入",
                                 "绘制轮廓需要RGB三通道图像的输入，在单通道图像上绘制不生效!",
                                 1,
                                 {"确认"});
            }

            port0_ok = true;
        } else {
            port0_ok = false;
            _imageNodeData.reset();
        }
    }

    if (portIndex == 1) {
        if (nodeData) {
            _contoursNodeData = nodeData;
            port1_ok = true;
        } else {
            port1_ok = false;
            _contoursNodeData.reset();
        }
    }

    if (port0_ok + port1_ok == 2) {
        compute();
    } else {
        _label->clear();
        _mat.release();
    }

    // dataUpdated 触发下游的节点更新
    Q_EMIT dataUpdated(0);
}

void CvDrawContoursModel::compute()
{
    auto d = std::dynamic_pointer_cast<ImageData>(_imageNodeData);
    auto c = std::dynamic_pointer_cast<ContoursData>(_contoursNodeData);
    if (!d || !c) {
        return;
    }
    if (d->mat().empty() || c->getData().contours.empty() || c->getData().hierachy.size() == 0) {
        return;
    }

    // todo ----
    if (d->mat().type() == CV_8UC4) {
        cv::cvtColor(d->mat(), _mat, cv::COLOR_RGBA2RGB);
    }

    cv::drawContours(_mat, c->getData().contours, -1, cv::Scalar(0, 255, 0), 10, cv::LINE_AA);
    // todo ----

    int w = _label->width();
    int h = _label->height();

    auto pix = QPixmap::fromImage(cvMat2QImage(_mat));
    _label->setPixmap(pix.scaled(w, h, Qt::KeepAspectRatio));

    Q_EMIT dataUpdated(0);
}

void CvDrawContoursModel::load(const QJsonObject &s) {}

QJsonObject CvDrawContoursModel::save() const
{
    auto s = NodeDelegateModel::save();

    return s;
}