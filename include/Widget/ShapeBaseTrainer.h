//
// Created by wrc on 23-2-18.
//

#ifndef STREAMER_SHAPEBASETRAINER_H
#define STREAMER_SHAPEBASETRAINER_H

#include "Line2Dup/line2Dup.hpp"
#include "Utils/Utils.hpp"
#include "Widget/MyGraphicsView.h"
#include <QDialog>
#include <QFileDialog>

QT_BEGIN_NAMESPACE
namespace Ui {
class ShapeBaseTrainerDialog;
}
QT_END_NAMESPACE

class ShapeBaseTrainerDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ShapeBaseTrainerDialog(QWidget *parent = nullptr);
    ~ShapeBaseTrainerDialog() override;

private:
    Ui::ShapeBaseTrainerDialog *ui;
    MyGraphicsView *m_view;
    MyGraphicsView *m_view2;
};

#endif //STREAMER_SHAPEBASETRAINER_H
