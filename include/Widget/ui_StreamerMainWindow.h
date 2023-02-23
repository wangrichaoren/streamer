/********************************************************************************
** Form generated from reading UI file 'StreamerMainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_STREAMERMAINWINDOW_H
#define UI_STREAMERMAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_StreamerMainWindow
{
public:
    QAction *cz_action;
    QAction *about_action;
    QAction *new_flow_action;
    QAction *open_flow_action;
    QAction *save_flow_action;
    QAction *exit_action;
    QAction *template_marker_action;
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QFrame *frame;
    QGridLayout *gridLayout_3;
    QTabWidget *tabWidget;
    QWidget *tab;
    QGridLayout *gridLayout_2;
    QPushButton *full_pc_pushButton;
    QSpacerItem *horizontalSpacer;
    QSpacerItem *horizontalSpacer_2;
    QVTKWidget *qvtkWidget;
    QSpacerItem *verticalSpacer;
    QCheckBox *show_coords_checkBox;
    QFrame *frame_2;
    QGridLayout *gridLayout_4;
    QStatusBar *statusbar;
    QMenuBar *menuBar;
    QMenu *file_menu;
    QMenu *tool_menu;
    QMenu *help_menu;
    QToolBar *toolBar;

    void setupUi(QMainWindow *StreamerMainWindow)
    {
        if (StreamerMainWindow->objectName().isEmpty())
            StreamerMainWindow->setObjectName(QString::fromUtf8("StreamerMainWindow"));
        StreamerMainWindow->resize(1111, 886);
        StreamerMainWindow->setTabShape(QTabWidget::Rounded);
        StreamerMainWindow->setDockOptions(QMainWindow::AllowTabbedDocks|QMainWindow::AnimatedDocks);
        cz_action = new QAction(StreamerMainWindow);
        cz_action->setObjectName(QString::fromUtf8("cz_action"));
        about_action = new QAction(StreamerMainWindow);
        about_action->setObjectName(QString::fromUtf8("about_action"));
        new_flow_action = new QAction(StreamerMainWindow);
        new_flow_action->setObjectName(QString::fromUtf8("new_flow_action"));
        open_flow_action = new QAction(StreamerMainWindow);
        open_flow_action->setObjectName(QString::fromUtf8("open_flow_action"));
        save_flow_action = new QAction(StreamerMainWindow);
        save_flow_action->setObjectName(QString::fromUtf8("save_flow_action"));
        exit_action = new QAction(StreamerMainWindow);
        exit_action->setObjectName(QString::fromUtf8("exit_action"));
        template_marker_action = new QAction(StreamerMainWindow);
        template_marker_action->setObjectName(QString::fromUtf8("template_marker_action"));
        centralwidget = new QWidget(StreamerMainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        frame = new QFrame(centralwidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setFrameShape(QFrame::NoFrame);
        frame->setFrameShadow(QFrame::Plain);
        gridLayout_3 = new QGridLayout(frame);
        gridLayout_3->setSpacing(0);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setContentsMargins(3, 3, 3, 0);
        tabWidget = new QTabWidget(frame);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setMinimumSize(QSize(100, 0));
        tabWidget->setMaximumSize(QSize(200, 16777215));
        QFont font;
        font.setPointSize(10);
        font.setBold(true);
        font.setItalic(true);
        font.setWeight(75);
        font.setStrikeOut(false);
        tabWidget->setFont(font);
        tabWidget->setAutoFillBackground(true);
        tabWidget->setStyleSheet(QString::fromUtf8("QTabBar::tab{color:gray;}\n"
""));
        tabWidget->setTabPosition(QTabWidget::North);
        tabWidget->setTabShape(QTabWidget::Triangular);
        tabWidget->setIconSize(QSize(10, 10));
        tabWidget->setElideMode(Qt::ElideNone);
        tabWidget->setUsesScrollButtons(true);
        tabWidget->setDocumentMode(false);
        tabWidget->setTabsClosable(false);
        tabWidget->setTabBarAutoHide(false);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        gridLayout_2 = new QGridLayout(tab);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        full_pc_pushButton = new QPushButton(tab);
        full_pc_pushButton->setObjectName(QString::fromUtf8("full_pc_pushButton"));
        QFont font1;
        font1.setPointSize(10);
        font1.setBold(true);
        font1.setWeight(75);
        full_pc_pushButton->setFont(font1);
        full_pc_pushButton->setIconSize(QSize(10, 10));
        full_pc_pushButton->setAutoDefault(false);

        gridLayout_2->addWidget(full_pc_pushButton, 2, 1, 1, 1);

        horizontalSpacer = new QSpacerItem(20, 30, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer, 2, 2, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(20, 30, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer_2, 2, 0, 1, 1);

        qvtkWidget = new QVTKWidget(tab);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(qvtkWidget->sizePolicy().hasHeightForWidth());
        qvtkWidget->setSizePolicy(sizePolicy);
        qvtkWidget->setMinimumSize(QSize(100, 100));
        qvtkWidget->setMaximumSize(QSize(178, 178));
        QFont font2;
        font2.setPointSize(10);
        font2.setBold(true);
        font2.setItalic(true);
        font2.setWeight(75);
        font2.setStyleStrategy(QFont::PreferAntialias);
        qvtkWidget->setFont(font2);
        qvtkWidget->setCursor(QCursor(Qt::CrossCursor));

        gridLayout_2->addWidget(qvtkWidget, 0, 0, 1, 3);

        verticalSpacer = new QSpacerItem(20, 528, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_2->addItem(verticalSpacer, 3, 1, 1, 1);

        show_coords_checkBox = new QCheckBox(tab);
        show_coords_checkBox->setObjectName(QString::fromUtf8("show_coords_checkBox"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(show_coords_checkBox->sizePolicy().hasHeightForWidth());
        show_coords_checkBox->setSizePolicy(sizePolicy1);
        QFont font3;
        font3.setPointSize(11);
        font3.setBold(true);
        font3.setWeight(75);
        show_coords_checkBox->setFont(font3);
        show_coords_checkBox->setIconSize(QSize(20, 20));
        show_coords_checkBox->setCheckable(true);
        show_coords_checkBox->setChecked(false);

        gridLayout_2->addWidget(show_coords_checkBox, 1, 1, 1, 1);

        tabWidget->addTab(tab, QString());

        gridLayout_3->addWidget(tabWidget, 0, 1, 1, 1);

        frame_2 = new QFrame(frame);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(frame_2->sizePolicy().hasHeightForWidth());
        frame_2->setSizePolicy(sizePolicy2);
        frame_2->setFrameShape(QFrame::NoFrame);
        frame_2->setFrameShadow(QFrame::Plain);
        frame_2->setLineWidth(0);
        gridLayout_4 = new QGridLayout(frame_2);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        gridLayout_4->setHorizontalSpacing(6);
        gridLayout_4->setVerticalSpacing(0);
        gridLayout_4->setContentsMargins(0, 0, 3, 0);

        gridLayout_3->addWidget(frame_2, 0, 0, 1, 1);


        gridLayout->addWidget(frame, 0, 0, 1, 1);

        StreamerMainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(StreamerMainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        StreamerMainWindow->setStatusBar(statusbar);
        menuBar = new QMenuBar(StreamerMainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1111, 28));
        file_menu = new QMenu(menuBar);
        file_menu->setObjectName(QString::fromUtf8("file_menu"));
        tool_menu = new QMenu(menuBar);
        tool_menu->setObjectName(QString::fromUtf8("tool_menu"));
        help_menu = new QMenu(menuBar);
        help_menu->setObjectName(QString::fromUtf8("help_menu"));
        StreamerMainWindow->setMenuBar(menuBar);
        toolBar = new QToolBar(StreamerMainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        StreamerMainWindow->addToolBar(Qt::TopToolBarArea, toolBar);

        menuBar->addAction(file_menu->menuAction());
        menuBar->addAction(tool_menu->menuAction());
        menuBar->addAction(help_menu->menuAction());
        file_menu->addAction(new_flow_action);
        file_menu->addAction(open_flow_action);
        file_menu->addAction(save_flow_action);
        file_menu->addAction(exit_action);
        tool_menu->addAction(template_marker_action);
        help_menu->addAction(cz_action);
        help_menu->addAction(about_action);

        retranslateUi(StreamerMainWindow);

        tabWidget->setCurrentIndex(0);
        full_pc_pushButton->setDefault(false);


        QMetaObject::connectSlotsByName(StreamerMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *StreamerMainWindow)
    {
        StreamerMainWindow->setWindowTitle(QCoreApplication::translate("StreamerMainWindow", "Streamer", nullptr));
        cz_action->setText(QCoreApplication::translate("StreamerMainWindow", "\346\223\215\344\275\234\350\257\264\346\230\216", nullptr));
        about_action->setText(QCoreApplication::translate("StreamerMainWindow", "\345\205\263\344\272\216", nullptr));
        new_flow_action->setText(QCoreApplication::translate("StreamerMainWindow", "\346\226\260\345\273\272", nullptr));
        open_flow_action->setText(QCoreApplication::translate("StreamerMainWindow", "\346\211\223\345\274\200", nullptr));
        save_flow_action->setText(QCoreApplication::translate("StreamerMainWindow", "\344\277\235\345\255\230", nullptr));
        exit_action->setText(QCoreApplication::translate("StreamerMainWindow", "\351\200\200\345\207\272", nullptr));
        template_marker_action->setText(QCoreApplication::translate("StreamerMainWindow", "\346\250\241\346\235\277\346\213\276\345\217\226\345\231\250", nullptr));
        full_pc_pushButton->setText(QCoreApplication::translate("StreamerMainWindow", "\346\237\245\347\234\213", nullptr));
        show_coords_checkBox->setText(QCoreApplication::translate("StreamerMainWindow", "Coords", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("StreamerMainWindow", "3DViewer", nullptr));
        file_menu->setTitle(QCoreApplication::translate("StreamerMainWindow", "\346\226\207\344\273\266", nullptr));
        tool_menu->setTitle(QCoreApplication::translate("StreamerMainWindow", "\345\267\245\345\205\267", nullptr));
        help_menu->setTitle(QCoreApplication::translate("StreamerMainWindow", "\345\270\256\345\212\251", nullptr));
        toolBar->setWindowTitle(QCoreApplication::translate("StreamerMainWindow", "toolBar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class StreamerMainWindow: public Ui_StreamerMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_STREAMERMAINWINDOW_H
