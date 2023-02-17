/********************************************************************************
** Form generated from reading UI file 'Full2DDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FULL2DDIALOG_H
#define UI_FULL2DDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>

QT_BEGIN_NAMESPACE

class Ui_Full2DDialog
{
public:
    QGridLayout *gridLayout;
    QGraphicsView *graphicsView;

    void setupUi(QDialog *Full2DDialog)
    {
        if (Full2DDialog->objectName().isEmpty())
            Full2DDialog->setObjectName(QString::fromUtf8("Full2DDialog"));
        Full2DDialog->resize(813, 710);
        gridLayout = new QGridLayout(Full2DDialog);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        graphicsView = new QGraphicsView(Full2DDialog);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(graphicsView->sizePolicy().hasHeightForWidth());
        graphicsView->setSizePolicy(sizePolicy);

        gridLayout->addWidget(graphicsView, 0, 0, 1, 1);


        retranslateUi(Full2DDialog);

        QMetaObject::connectSlotsByName(Full2DDialog);
    } // setupUi

    void retranslateUi(QDialog *Full2DDialog)
    {
        Full2DDialog->setWindowTitle(QCoreApplication::translate("Full2DDialog", "Full2DDialog", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Full2DDialog: public Ui_Full2DDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FULL2DDIALOG_H
