//
// Created by wrc on 23-2-12.
//

// You may need to build the project (run Qt uic code generator) to get "ui_Full2DDialog.h" resolved

#include "../include/Widget/Full2DDialog.h"
#include "../include/Widget/ui_Full2DDialog.h"

Full2DDialog::Full2DDialog(QWidget *parent, cv::Mat *m)
    : QDialog(parent)
    , ui(new Ui::Full2DDialog)
    , mat(m)
{
    ui->setupUi(this);
    ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    initView();
}

Full2DDialog::~Full2DDialog()
{
    delete ui;
}
void Full2DDialog::initView()
{
    if (mat->empty()) {
        return;
    }
    auto pix = QPixmap::fromImage(cvMat2QImage(*mat));
    scene->addPixmap(pix);
    ui->graphicsView->setScene(scene);
}
