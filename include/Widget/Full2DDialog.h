//
// Created by wrc on 23-2-12.
//

#ifndef STREAMER_FULL2DDIALOG_H
#define STREAMER_FULL2DDIALOG_H

#include "Utils/Utils.hpp"
#include "Widget/MyGraphicsView.h"
#include <opencv2/opencv.hpp>
#include <QDialog>
#include <QGraphicsScene>

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

    bool eventFilter(QObject *, QEvent *) override;

private:
    Ui::Full2DDialog *ui;
    cv::Mat *mat;
    MyGraphicsView *myGraphicsView{nullptr};
};

#endif //STREAMER_FULL2DDIALOG_H
