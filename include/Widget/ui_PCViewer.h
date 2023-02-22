/********************************************************************************
** Form generated from reading UI file 'PCViewer.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCVIEWER_H
#define UI_PCVIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCViewer
{
public:
    QGridLayout *gridLayout;
    QVTKWidget *qvtkWidget;

    void setupUi(QDialog *PCViewer)
    {
        if (PCViewer->objectName().isEmpty())
            PCViewer->setObjectName(QString::fromUtf8("PCViewer"));
        PCViewer->resize(922, 778);
        gridLayout = new QGridLayout(PCViewer);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        qvtkWidget = new QVTKWidget(PCViewer);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));

        gridLayout->addWidget(qvtkWidget, 0, 0, 1, 1);


        retranslateUi(PCViewer);

        QMetaObject::connectSlotsByName(PCViewer);
    } // setupUi

    void retranslateUi(QDialog *PCViewer)
    {
        PCViewer->setWindowTitle(QCoreApplication::translate("PCViewer", "PCViewer", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PCViewer: public Ui_PCViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCVIEWER_H
