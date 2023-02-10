#include "../include/CvModels/CvImageLoaderModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include <QJsonArray>
#include <QtWidgets/QFileDialog>

CvImageLoaderModel::CvImageLoaderModel()
    : _label(new QLabel("Click label to load image."))
    , _path_lineedit(new QLineEdit(""))
    , _layout(new QVBoxLayout())
    , _box(new QGroupBox())
{
    // group box 无边框
    _box->setStyleSheet("QGroupBox{padding-top:15px; margin-top:-15px;padding-left:15px; "
                        "margin-left:-15px;padding-right:15px; margin-right:-15px;}");
    _label->setAlignment(Qt::AlignVCenter | Qt::AlignHCenter);

    QFont f = _label->font();
    f.setBold(true);
    f.setItalic(true);

    _label->setFont(f);

    _label->setMinimumSize(200, 200);
    _label->setMaximumSize(500, 300);

    _label->installEventFilter(this);

    QFont f2 = _path_lineedit->font();
    _path_lineedit->setFont(f2);
    _path_lineedit->setReadOnly(true);
    _layout->addWidget(_label);
    _layout->addWidget(_path_lineedit);

    _box->setLayout(_layout);
}

unsigned int CvImageLoaderModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 0;
        break;

    case PortType::Out:
        result = 1;

    default:
        break;
    }

    return result;
}

bool CvImageLoaderModel::eventFilter(QObject *object, QEvent *event)
{
    if (object == _label) {
        int w = _label->width();
        int h = _label->height();

        if (event->type() == QEvent::MouseButtonPress) {
            QString fileName = QFileDialog::getOpenFileName(nullptr,
                                                            tr("Open Image"),
                                                            QDir::homePath(),
                                                            tr("Image Files (*.png *.jpg *.bmp)"));
            if (fileName.isEmpty()) {
                return false;
            }
            _mat = cv::imread(fileName.toStdString(), -1);

            // _mat->pixmap
            _q_pix = QPixmap::fromImage(MatToQImage(_mat));

            _label->setPixmap(_q_pix.scaled(w, h, Qt::KeepAspectRatio));

            _path_lineedit->setText(fileName);

            Q_EMIT dataUpdated(0);
            return true;

        } else if (event->type() == QEvent::Resize) {
            if (!_q_pix.isNull())
                _label->setPixmap(_q_pix.scaled(w, h, Qt::KeepAspectRatio));
        }
    }

    return false;
}

NodeDataType CvImageLoaderModel::dataType(PortType const, PortIndex const) const
{
    return ImageData().type();
}

std::shared_ptr<NodeData> CvImageLoaderModel::outData(PortIndex)
{
    return std::make_shared<ImageData>(_mat);
}
QJsonObject CvImageLoaderModel::save() const
{
    auto saver = NodeDelegateModel::save();
    saver["image_path"] = _path_lineedit->text();
    return saver;
}
void CvImageLoaderModel::load(const QJsonObject &js)
{
    _path_lineedit->setText(js["image_path"].toString());
    _mat = cv::imread(js["image_path"].toString().toStdString(), -1);
    // _mat->pixmap
    _q_pix = QPixmap::fromImage(MatToQImage(_mat));

    int w = _label->width();
    int h = _label->height();

    _label->setPixmap(_q_pix.scaled(w, h, Qt::KeepAspectRatio));
}

void load(QJsonObject const &jsonDocument)
{
    //    for (QJsonValueRef nodeJson : nodesJsonArray) {
    //        NodeId restoredNodeId = static_cast<NodeId>(nodeJson.toObject()["id"].toInt());
    //        std::cout<<"aaa"<<std::endl;
    //        std::cout<<restoredNodeId<<std::endl;
    //    }

    //        qDebug(e["nodes"]);
    //    std::cout<<e["nodes"].toString().toStdString()<<std::endl;
    //    NodeDelegateModel::load("image_path");
}
