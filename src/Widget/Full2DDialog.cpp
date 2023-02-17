//
// Created by wrc on 23-2-12.
//

// You may need to build the project (run Qt uic code generator) to get "ui_Full2DDialog.h" resolved

#include "Widget/Full2DDialog.h"
#include "Widget/ui_Full2DDialog.h"
#include <QVBoxLayout>

Full2DDialog::Full2DDialog(QWidget *parent, cv::Mat *m)
    : QDialog(parent)
    , ui(new Ui::Full2DDialog)
    , mat(m)
{
    ui->setupUi(this);
    ui->graphicsView->resize(this->width(), this->height());
    ui->graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    initView();
}

Full2DDialog::~Full2DDialog()
{
    delete myGraphicsView;
    delete ui;
}
void Full2DDialog::initView()
{
    if (mat->empty()) {
        return;
    }
    myGraphicsView = new MyGraphicsView(ui->graphicsView);
    myGraphicsView->graphics(*mat);
}
