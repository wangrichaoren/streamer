#include <iostream>

#include "../include/Utils/Utils.hpp"

int createMessageBox(QWidget *parent,
                     const char *icon_file,
                     const char *window_title,
                     const char *text,
                     const int &btn_num,
                     const std::vector<const char *> &btn_texts)
{
    QMessageBox Box(parent);
    auto f = Box.font();
    f.setBold(true);
    Box.setFont(f);

    auto p = QPixmap(icon_file);
    p = p.scaled(150,
                 150,
                 Qt::AspectRatioMode::KeepAspectRatio,
                 Qt::TransformationMode::SmoothTransformation);
    Box.setIconPixmap(p);
    Box.setWindowTitle(window_title);
    Box.setText(text);

    for (int i = 0; i < btn_num; ++i) {
        Box.addButton(btn_texts[i], QMessageBox::AcceptRole);
    }

    return Box.exec();
}

QImage MatToQImage(const cv::Mat &mat)
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

QImage cvMat2QImage(const cv::Mat &mat)
{
    QImage image;
    switch (mat.type()) {
    case CV_8UC1:
        //QImage构造：数据，宽度，高度，每行多少字节，存储结构
        image = QImage((const unsigned char *) mat.data,
                       mat.cols,
                       mat.rows,
                       mat.cols,
                       QImage::Format_Grayscale8);
        break;
    case CV_8UC3:
        image = QImage((const unsigned char *) mat.data,
                       mat.cols,
                       mat.rows,
                       mat.cols * 3,
                       QImage::Format_RGB888);
        image = image.rgbSwapped(); //BRG转为RGB
        //Qt5.14增加了Format_BGR888
        //image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.cols * 3, QImage::Format_BGR888);
        break;
    case CV_8UC4:
        image = QImage((const unsigned char *) mat.data,
                       mat.cols,
                       mat.rows,
                       mat.cols * 4,
                       QImage::Format_ARGB32);
        break;
    }
    return image;
}

void createLineEditFormCurQObj(QHBoxLayout *hlay_obj, QLabel *lab_obj, QLineEdit *edit_obj)
{
    auto font = lab_obj->font();
    font.setBold(true);
    lab_obj->setFont(font);
    hlay_obj->addWidget(lab_obj);
    hlay_obj->addWidget(edit_obj);
}