#include "CvImageLoaderModel.hpp"

#include <QtCore/QDir>
#include <QtCore/QEvent>

#include <QtWidgets/QFileDialog>

CvImageLoaderModel::CvImageLoaderModel()
    : _label(new QLabel("Click label to load image."))
    , _path_lineedit(new QLineEdit(""))
    , _layout(new QVBoxLayout())
    , _box(new QGroupBox())
{
    // group box 无边框
    _box->setStyleSheet("QGroupBox{padding-top:15px; margin-top:-15px;padding-left:15px; margin-left:-15px;padding-right:15px; margin-right:-15px;}") ;
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

            _mat = cv::imread(fileName.toStdString(), -1);

            // _mat->pixmap
            _q_pix = QPixmap::fromImage(this->MatToQImage(_mat));

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

QImage CvImageLoaderModel::MatToQImage(const cv::Mat &mat)
{
    if (mat.type() == CV_8UC1) {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for (int i = 0; i < 256; i++) {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for (int row = 0; row < mat.rows; row++) {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if (mat.type() == CV_8UC3) {
        // Copy input Mat
        const uchar *pSrc = (const uchar *) mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    } else if (mat.type() == CV_8UC4) {
        // Copy input Mat
        const uchar *pSrc = (const uchar *) mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    } else {
        //MessageInfo("ERROR: Mat could not be converted to QImage.", 1);
        //emit sig_RunInfo("ERROR: Mat could not be converted to QImage.", 1);
        //if (!globalPara.IsInlineRun) Runstateinfo("ERROR: Mat could not be converted to QImage.", 1);
        return QImage();
    }
}
