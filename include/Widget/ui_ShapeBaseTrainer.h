/********************************************************************************
** Form generated from reading UI file 'ShapeBaseTrainer.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SHAPEBASETRAINER_H
#define UI_SHAPEBASETRAINER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_ShapeBaseTrainerDialog
{
public:
    QGridLayout *gridLayout;
    QGroupBox *groupBox_6;
    QGridLayout *gridLayout_7;
    QPushButton *open_pushButton;
    QPushButton *train_pushButton;
    QPushButton *exit_pushButton;
    QGraphicsView *graphicsView;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_6;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout_2;
    QLabel *label_2;
    QLabel *label_7;
    QLineEdit *uplineEdit;
    QLineEdit *downlineEdit;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_3;
    QLabel *label_5;
    QLineEdit *scale1_lineEdit;
    QLineEdit *scale2_lineEdit;
    QLabel *label_6;
    QLineEdit *scale_step_lineEdit;
    QGroupBox *groupBox_5;
    QGridLayout *gridLayout_5;
    QLineEdit *num_lineEdit;
    QLabel *label;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_4;
    QLabel *label_3;
    QLineEdit *down_r_lineEdit;
    QLineEdit *up_r_lineEdit;
    QLabel *label_4;
    QLineEdit *r_step_lineEdit;
    QGroupBox *groupBox_7;
    QGridLayout *gridLayout_8;
    QGraphicsView *graphicsView_2;

    void setupUi(QDialog *ShapeBaseTrainerDialog)
    {
        if (ShapeBaseTrainerDialog->objectName().isEmpty())
            ShapeBaseTrainerDialog->setObjectName(QString::fromUtf8("ShapeBaseTrainerDialog"));
        ShapeBaseTrainerDialog->resize(1036, 867);
        ShapeBaseTrainerDialog->setCursor(QCursor(Qt::ArrowCursor));
        gridLayout = new QGridLayout(ShapeBaseTrainerDialog);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        groupBox_6 = new QGroupBox(ShapeBaseTrainerDialog);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        gridLayout_7 = new QGridLayout(groupBox_6);
        gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
        open_pushButton = new QPushButton(groupBox_6);
        open_pushButton->setObjectName(QString::fromUtf8("open_pushButton"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(open_pushButton->sizePolicy().hasHeightForWidth());
        open_pushButton->setSizePolicy(sizePolicy);
        open_pushButton->setMinimumSize(QSize(180, 70));
        QFont font;
        font.setPointSize(15);
        font.setBold(true);
        font.setWeight(75);
        open_pushButton->setFont(font);

        gridLayout_7->addWidget(open_pushButton, 0, 0, 1, 1);

        train_pushButton = new QPushButton(groupBox_6);
        train_pushButton->setObjectName(QString::fromUtf8("train_pushButton"));
        sizePolicy.setHeightForWidth(train_pushButton->sizePolicy().hasHeightForWidth());
        train_pushButton->setSizePolicy(sizePolicy);
        train_pushButton->setMinimumSize(QSize(180, 70));
        train_pushButton->setFont(font);

        gridLayout_7->addWidget(train_pushButton, 1, 0, 1, 1);

        exit_pushButton = new QPushButton(groupBox_6);
        exit_pushButton->setObjectName(QString::fromUtf8("exit_pushButton"));
        sizePolicy.setHeightForWidth(exit_pushButton->sizePolicy().hasHeightForWidth());
        exit_pushButton->setSizePolicy(sizePolicy);
        exit_pushButton->setMinimumSize(QSize(180, 70));
        exit_pushButton->setFont(font);

        gridLayout_7->addWidget(exit_pushButton, 2, 0, 1, 1);


        gridLayout->addWidget(groupBox_6, 2, 1, 1, 1);

        graphicsView = new QGraphicsView(ShapeBaseTrainerDialog);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        sizePolicy.setHeightForWidth(graphicsView->sizePolicy().hasHeightForWidth());
        graphicsView->setSizePolicy(sizePolicy);
        graphicsView->setStyleSheet(QString::fromUtf8(""));

        gridLayout->addWidget(graphicsView, 0, 0, 2, 3);

        groupBox = new QGroupBox(ShapeBaseTrainerDialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMinimumSize(QSize(0, 0));
        QFont font1;
        font1.setBold(true);
        font1.setWeight(75);
        groupBox->setFont(font1);
        gridLayout_6 = new QGridLayout(groupBox);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        groupBox_4 = new QGroupBox(groupBox);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setEnabled(false);
        gridLayout_2 = new QGridLayout(groupBox_4);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label_2 = new QLabel(groupBox_4);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_2->addWidget(label_2, 0, 0, 1, 1);

        label_7 = new QLabel(groupBox_4);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_2->addWidget(label_7, 2, 0, 1, 1);

        uplineEdit = new QLineEdit(groupBox_4);
        uplineEdit->setObjectName(QString::fromUtf8("uplineEdit"));
        sizePolicy.setHeightForWidth(uplineEdit->sizePolicy().hasHeightForWidth());
        uplineEdit->setSizePolicy(sizePolicy);
        QFont font2;
        font2.setBold(true);
        font2.setItalic(true);
        font2.setWeight(75);
        uplineEdit->setFont(font2);

        gridLayout_2->addWidget(uplineEdit, 2, 1, 1, 2);

        downlineEdit = new QLineEdit(groupBox_4);
        downlineEdit->setObjectName(QString::fromUtf8("downlineEdit"));
        sizePolicy.setHeightForWidth(downlineEdit->sizePolicy().hasHeightForWidth());
        downlineEdit->setSizePolicy(sizePolicy);
        downlineEdit->setFont(font2);
        downlineEdit->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        gridLayout_2->addWidget(downlineEdit, 0, 1, 1, 2);


        gridLayout_6->addWidget(groupBox_4, 0, 1, 1, 1);

        groupBox_3 = new QGroupBox(groupBox);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        gridLayout_3 = new QGridLayout(groupBox_3);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        label_5 = new QLabel(groupBox_3);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_3->addWidget(label_5, 0, 0, 1, 1);

        scale1_lineEdit = new QLineEdit(groupBox_3);
        scale1_lineEdit->setObjectName(QString::fromUtf8("scale1_lineEdit"));
        sizePolicy.setHeightForWidth(scale1_lineEdit->sizePolicy().hasHeightForWidth());
        scale1_lineEdit->setSizePolicy(sizePolicy);
        scale1_lineEdit->setFont(font2);

        gridLayout_3->addWidget(scale1_lineEdit, 0, 1, 1, 1);

        scale2_lineEdit = new QLineEdit(groupBox_3);
        scale2_lineEdit->setObjectName(QString::fromUtf8("scale2_lineEdit"));
        sizePolicy.setHeightForWidth(scale2_lineEdit->sizePolicy().hasHeightForWidth());
        scale2_lineEdit->setSizePolicy(sizePolicy);
        scale2_lineEdit->setFont(font2);

        gridLayout_3->addWidget(scale2_lineEdit, 1, 1, 1, 1);

        label_6 = new QLabel(groupBox_3);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_3->addWidget(label_6, 2, 0, 1, 1);

        scale_step_lineEdit = new QLineEdit(groupBox_3);
        scale_step_lineEdit->setObjectName(QString::fromUtf8("scale_step_lineEdit"));
        sizePolicy.setHeightForWidth(scale_step_lineEdit->sizePolicy().hasHeightForWidth());
        scale_step_lineEdit->setSizePolicy(sizePolicy);
        scale_step_lineEdit->setFont(font2);

        gridLayout_3->addWidget(scale_step_lineEdit, 2, 1, 1, 1);


        gridLayout_6->addWidget(groupBox_3, 1, 1, 1, 1);

        groupBox_5 = new QGroupBox(groupBox);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        gridLayout_5 = new QGridLayout(groupBox_5);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        num_lineEdit = new QLineEdit(groupBox_5);
        num_lineEdit->setObjectName(QString::fromUtf8("num_lineEdit"));
        sizePolicy.setHeightForWidth(num_lineEdit->sizePolicy().hasHeightForWidth());
        num_lineEdit->setSizePolicy(sizePolicy);
        num_lineEdit->setFont(font2);
        num_lineEdit->setAlignment(Qt::AlignCenter);
        num_lineEdit->setCursorMoveStyle(Qt::LogicalMoveStyle);

        gridLayout_5->addWidget(num_lineEdit, 0, 1, 1, 1);

        label = new QLabel(groupBox_5);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_5->addWidget(label, 0, 0, 1, 1);


        gridLayout_6->addWidget(groupBox_5, 0, 0, 1, 1);

        groupBox_2 = new QGroupBox(groupBox);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        gridLayout_4 = new QGridLayout(groupBox_2);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_4->addWidget(label_3, 0, 0, 1, 1);

        down_r_lineEdit = new QLineEdit(groupBox_2);
        down_r_lineEdit->setObjectName(QString::fromUtf8("down_r_lineEdit"));
        sizePolicy.setHeightForWidth(down_r_lineEdit->sizePolicy().hasHeightForWidth());
        down_r_lineEdit->setSizePolicy(sizePolicy);
        down_r_lineEdit->setFont(font2);

        gridLayout_4->addWidget(down_r_lineEdit, 0, 1, 1, 1);

        up_r_lineEdit = new QLineEdit(groupBox_2);
        up_r_lineEdit->setObjectName(QString::fromUtf8("up_r_lineEdit"));
        sizePolicy.setHeightForWidth(up_r_lineEdit->sizePolicy().hasHeightForWidth());
        up_r_lineEdit->setSizePolicy(sizePolicy);
        up_r_lineEdit->setFont(font2);

        gridLayout_4->addWidget(up_r_lineEdit, 1, 1, 1, 1);

        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_4->addWidget(label_4, 2, 0, 1, 1);

        r_step_lineEdit = new QLineEdit(groupBox_2);
        r_step_lineEdit->setObjectName(QString::fromUtf8("r_step_lineEdit"));
        sizePolicy.setHeightForWidth(r_step_lineEdit->sizePolicy().hasHeightForWidth());
        r_step_lineEdit->setSizePolicy(sizePolicy);
        r_step_lineEdit->setFont(font2);

        gridLayout_4->addWidget(r_step_lineEdit, 2, 1, 1, 1);


        gridLayout_6->addWidget(groupBox_2, 1, 0, 1, 1);


        gridLayout->addWidget(groupBox, 2, 0, 1, 1);

        groupBox_7 = new QGroupBox(ShapeBaseTrainerDialog);
        groupBox_7->setObjectName(QString::fromUtf8("groupBox_7"));
        groupBox_7->setFont(font1);
        gridLayout_8 = new QGridLayout(groupBox_7);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        graphicsView_2 = new QGraphicsView(groupBox_7);
        graphicsView_2->setObjectName(QString::fromUtf8("graphicsView_2"));
        sizePolicy.setHeightForWidth(graphicsView_2->sizePolicy().hasHeightForWidth());
        graphicsView_2->setSizePolicy(sizePolicy);

        gridLayout_8->addWidget(graphicsView_2, 0, 0, 1, 1);


        gridLayout->addWidget(groupBox_7, 2, 2, 1, 1);


        retranslateUi(ShapeBaseTrainerDialog);

        QMetaObject::connectSlotsByName(ShapeBaseTrainerDialog);
    } // setupUi

    void retranslateUi(QDialog *ShapeBaseTrainerDialog)
    {
        ShapeBaseTrainerDialog->setWindowTitle(QCoreApplication::translate("ShapeBaseTrainerDialog", "ShapeBaseTrainer", nullptr));
        groupBox_6->setTitle(QString());
        open_pushButton->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "\351\200\211\346\213\251\351\242\204\350\256\255\347\273\203\345\233\276\347\211\207", nullptr));
        train_pushButton->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "\350\256\255\347\273\203", nullptr));
        exit_pushButton->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "\351\200\200\345\207\272", nullptr));
        groupBox->setTitle(QString());
        groupBox_4->setTitle(QCoreApplication::translate("ShapeBaseTrainerDialog", "\351\207\207\346\240\267", nullptr));
        label_2->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "\344\270\212\351\207\207\346\240\267", nullptr));
        label_7->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "\344\270\213\351\207\207\346\240\267", nullptr));
        uplineEdit->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "8", nullptr));
        downlineEdit->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "4", nullptr));
        groupBox_3->setTitle(QCoreApplication::translate("ShapeBaseTrainerDialog", "\347\274\251\346\224\276", nullptr));
        label_5->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "\347\274\251\346\224\276\350\214\203\345\233\264", nullptr));
        scale1_lineEdit->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "0.8", nullptr));
        scale2_lineEdit->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "1.1", nullptr));
        label_6->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "\347\274\251\346\224\276\346\255\245\351\225\277", nullptr));
        scale_step_lineEdit->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "0.1", nullptr));
        groupBox_5->setTitle(QCoreApplication::translate("ShapeBaseTrainerDialog", "\347\211\271\345\276\201", nullptr));
        num_lineEdit->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "128", nullptr));
        label->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "\347\211\271\345\276\201\346\225\260\351\207\217", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("ShapeBaseTrainerDialog", "\346\227\213\350\275\254", nullptr));
        label_3->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "\346\227\213\350\275\254\350\214\203\345\233\264", nullptr));
        down_r_lineEdit->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "0", nullptr));
        up_r_lineEdit->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "360", nullptr));
        label_4->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "\346\227\213\350\275\254\346\255\245\351\225\277", nullptr));
        r_step_lineEdit->setText(QCoreApplication::translate("ShapeBaseTrainerDialog", "1", nullptr));
        groupBox_7->setTitle(QString());
    } // retranslateUi

};

namespace Ui {
    class ShapeBaseTrainerDialog: public Ui_ShapeBaseTrainerDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SHAPEBASETRAINER_H
