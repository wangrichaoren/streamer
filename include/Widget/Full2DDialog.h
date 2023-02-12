//
// Created by wrc on 23-2-12.
//

#ifndef STREAMER_FULL2DDIALOG_H
#define STREAMER_FULL2DDIALOG_H

#include <opencv2/opencv.hpp>
#include <QDialog>
#include <QGraphicsScene>
#include "Utils/Utils.hpp"

QT_BEGIN_NAMESPACE
namespace Ui {
class Full2DDialog;
}
QT_END_NAMESPACE

class Full2DDialog : public QDialog
{
    Q_OBJECT

public:
    explicit Full2DDialog(QWidget *parent = nullptr, cv::Mat *m = nullptr);
    ~Full2DDialog() override;

    void initView();

private:
    Ui::Full2DDialog *ui;
    cv::Mat *mat;
    QGraphicsScene *scene = new QGraphicsScene(this);
};

#endif //STREAMER_FULL2DDIALOG_H
