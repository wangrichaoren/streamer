//
// Created by wrc on 23-2-10.
//

#include <QtNodes/ConnectionStyle>
#include <QtNodes/DataFlowGraphModel>
#include <QtNodes/DataFlowGraphicsScene>
#include <QtNodes/GraphicsView>
#include <QtNodes/GraphicsViewStyle>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtNodes/NodeStyle>

#include <memory>
#include <QtGui/QScreen>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>

#include "../include/Utils/Utils.hpp"

#include "../include/GeneralModels/AdditionModel.hpp"
#include "../include/GeneralModels/DivisionModel.hpp"
#include "../include/GeneralModels/ImageLoaderModel.hpp"
#include "../include/GeneralModels/ImageShowModel.hpp"
#include "../include/GeneralModels/MultiplicationModel.hpp"
#include "../include/GeneralModels/NumberDisplayDataModel.hpp"
#include "../include/GeneralModels/NumberSourceDataModel.hpp"
#include "../include/GeneralModels/SubtractionModel.hpp"

#include "../include/CvModels/CvBinaryModel.hpp"
#include "../include/CvModels/CvBlurModel.hpp"
#include "../include/CvModels/CvDrawContoursModel.hpp"
#include "../include/CvModels/CvEdgeModel.hpp"
#include "../include/CvModels/CvFilter2dModel.hpp"
#include "../include/CvModels/CvFindContoursModel.hpp"
#include "../include/CvModels/CvGaussianBlurModel.hpp"
#include "../include/CvModels/CvImageLoaderModel.hpp"
#include "../include/CvModels/CvImageShowModel.hpp"
#include "../include/CvModels/CvMedianBlurModel.hpp"
#include "../include/CvModels/CvMorphModel.hpp"
#include "../include/CvModels/CvRGB2GrayModel.hpp"

using QtNodes::ConnectionStyle;
using QtNodes::DataFlowGraphicsScene;
using QtNodes::DataFlowGraphModel;
using QtNodes::GraphicsView;
using QtNodes::GraphicsViewStyle;
using QtNodes::NodeDelegateModelRegistry;
using QtNodes::NodeStyle;

static std::shared_ptr<NodeDelegateModelRegistry> registerDataModels()
{
    auto ret = std::make_shared<NodeDelegateModelRegistry>();

    // 通用模型
    ret->registerModel<NumberSourceDataModel>("通用");
    ret->registerModel<NumberDisplayDataModel>("通用");
    ret->registerModel<AdditionModel>("通用");
    ret->registerModel<SubtractionModel>("通用");
    ret->registerModel<MultiplicationModel>("通用");
    ret->registerModel<DivisionModel>("通用");

    // 2d模型
    ret->registerModel<CvImageLoaderModel>("2D");
    ret->registerModel<CvImageShowModel>("2D");
    ret->registerModel<CvRGB2GrayModel>("2D");
    ret->registerModel<CvBinaryModel>("2D");
    ret->registerModel<CvMorphModel>("2D");
    ret->registerModel<CvFilter2dModel>("2D");
    ret->registerModel<CvBlurModel>("2D");
    ret->registerModel<CvMedianBlurModel>("2D");
    ret->registerModel<CvGaussianBlurModel>("2D");
    ret->registerModel<CvEdgeModel>("2D");
    ret->registerModel<CvFindContoursModel>("2D");
    ret->registerModel<CvDrawContoursModel>("2D");

    // todo 3d模型

    return ret;
}

static void setStyle()
{
    GraphicsViewStyle::setStyle(
        R"(
      {
        "GraphicsViewStyle": {
          "BackgroundColor": [255, 255, 240],
          "FineGridColor": [245, 245, 230],
          "CoarseGridColor": [235, 235, 220]
        }
      }
      )");

    NodeStyle::setNodeStyle(
        R"(
      {
        "NodeStyle": {
          "NormalBoundaryColor": "darkgray",
          "SelectedBoundaryColor": "dark",
          "GradientColor0": "mintcream",
          "GradientColor1": "mintcream",
          "GradientColor2": "mintcream",
          "GradientColor3": "mintcream",
          "ShadowColor": [150, 150, 150],
          "FontColor": [10, 10, 10],
          "FontColorFaded": [100, 100, 100],
          "ConnectionPointColor": "white",
          "PenWidth": 2.0,
          "HoveredPenWidth": 2.5,
          "ConnectionPointDiameter": 10.0,
          "Opacity": 1.0
        }
      }
      )");

    ConnectionStyle::setConnectionStyle(
        R"(
  {
    "ConnectionStyle": {
      "ConstructionColor": "gray",
      "NormalColor": "black",
      "SelectedColor": "gray",
      "SelectedHaloColor": "deepskyblue",
      "HoveredColor": "deepskyblue",

      "LineWidth": 3.0,
      "ConstructionLineWidth": 2.0,
      "PointDiameter": 10.0,

      "UseDataDefinedColors": true
    }
  }
  )");
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    setStyle();

    std::shared_ptr<NodeDelegateModelRegistry> registry = registerDataModels();

    QWidget mainWidget;

    // 垂直布局 v_layout_all 添加menu toolbar h_center_layout log
    // 水平布局 h_layout_center 添加choose show option
    auto *v_layout_all = new QVBoxLayout(&mainWidget);

    // 菜单栏
    auto menuBar = new QMenuBar(&mainWidget);
    QMenu *menu = menuBar->addMenu("文件");
    QMenu *help_menu = menuBar->addMenu("帮助");
    v_layout_all->addWidget(menuBar);

    // 工具栏
    auto tool_group = new QGroupBox(&mainWidget);
    auto *h_layout_tool = new QHBoxLayout(tool_group);
    int btn_w = 100;

    auto new_btn = new QPushButton(tool_group); //新建
    auto new_icon = QIcon(":icons/new.png");
    new_btn->setIcon(new_icon);
    new_btn->setMaximumWidth(btn_w);
    new_btn->setToolTip("新建图文件");

    auto openfile_btn = new QPushButton(tool_group); //打开
    auto openfile_icon = QIcon(":icons/open.png");
    openfile_btn->setIcon(openfile_icon);
    openfile_btn->setMaximumWidth(btn_w);
    openfile_btn->setToolTip("打开图文件");

    auto save_btn = new QPushButton(tool_group); //保存
    auto save_icon = QIcon(":icons/save.png");
    save_btn->setIcon(save_icon);
    save_btn->setMaximumWidth(btn_w);
    save_btn->setToolTip("保存图文件(快捷键ctrl+s)");

    auto lock_btn = new QPushButton(tool_group); //锁
    auto lock_icon = QIcon(":icons/lock.png");
    auto unlock_icon = QIcon(":icons/unlock.png");
    bool lock_status = false;
    lock_btn->setIcon(unlock_icon);
    lock_btn->setMaximumWidth(btn_w);
    lock_btn->setToolTip("上锁/解锁");

    auto vline0 = new QFrame(tool_group); // 分割线
    vline0->setFrameShape(QFrame::VLine);
    vline0->setStyleSheet("QFrame{color:gray;border:1px dotted ;}");

    auto vline1 = new QFrame(tool_group); // 分割线
    vline1->setFrameShape(QFrame::VLine);
    vline1->setStyleSheet("QFrame{color:gray;border:1px dotted ;}");

    auto back_btn = new QPushButton(tool_group); //撤回
    auto back_icon = QIcon(":icons/withdraw.png");
    back_btn->setIcon(back_icon);
    back_btn->setMaximumWidth(btn_w);
    back_btn->setToolTip("撤回(快捷键ctrl+z)");

    auto up_btn = new QPushButton(tool_group); //跳转到上一个
    auto up_icon = QIcon(":icons/left.png");
    up_btn->setIcon(up_icon);
    up_btn->setMaximumWidth(btn_w);
    up_btn->setToolTip("跳转至上一个图元");

    auto down_btn = new QPushButton(tool_group); //跳转到下一个
    auto down_icon = QIcon(":icons/right.png");
    down_btn->setIcon(down_icon);
    down_btn->setMaximumWidth(btn_w);
    down_btn->setToolTip("跳转至下一个图元");

    auto del_btn = new QPushButton(tool_group); //删除
    auto del_icon = QIcon(":icons/del.png");
    del_btn->setIcon(del_icon);
    del_btn->setMaximumWidth(btn_w);
    del_btn->setToolTip("删除当前图元(快捷键delete)");

    auto full_btn = new QPushButton(tool_group); //全景
    auto full_icon = QIcon(":icons/fullscreen.png");
    full_btn->setIcon(full_icon);
    full_btn->setMaximumWidth(btn_w);
    full_btn->setToolTip("自适应图大小");

    auto start_btn = new QPushButton(tool_group); //运行
    auto start_icon = QIcon(":icons/start.png");
    start_btn->setIcon(start_icon);
    start_btn->setMaximumWidth(btn_w);
    start_btn->setToolTip("开始");

    auto stop_btn = new QPushButton(tool_group); //停止
    auto stop_icon = QIcon(":icons/stop.png");
    stop_btn->setIcon(stop_icon);
    stop_btn->setMaximumWidth(btn_w);
    stop_btn->setToolTip("停止");

    auto vline2 = new QFrame(tool_group); // 分割线2
    vline2->setFrameShape(QFrame::VLine);
    vline2->setStyleSheet("QFrame{color:gray;border:1px dotted ;}");

    auto vline3 = new QFrame(tool_group); // 分割线2
    vline3->setFrameShape(QFrame::VLine);
    vline3->setStyleSheet("QFrame{color:gray;border:1px dotted ;}");

    auto vline4 = new QFrame(tool_group); // 分割线2
    vline4->setFrameShape(QFrame::VLine);
    vline4->setStyleSheet("QFrame{color:gray;border:1px dotted ;}");

    // 按钮添加到h layout上
    h_layout_tool->addWidget(vline0);
    h_layout_tool->addWidget(new_btn);
    h_layout_tool->addWidget(openfile_btn);
    h_layout_tool->addWidget(save_btn);
    h_layout_tool->addWidget(vline1);

    h_layout_tool->addWidget(up_btn);
    h_layout_tool->addWidget(down_btn);
    h_layout_tool->addWidget(back_btn);
    h_layout_tool->addWidget(del_btn);
    h_layout_tool->addWidget(full_btn);

    h_layout_tool->addWidget(vline2);
    h_layout_tool->addWidget(start_btn);
    h_layout_tool->addWidget(stop_btn);
    h_layout_tool->addWidget(vline3);
    h_layout_tool->addWidget(lock_btn);
    h_layout_tool->addWidget(vline4);

    //    h_layout_tool->setContentsMargins(0, 0, 0, 0);
    h_layout_tool->setSpacing(20);
    h_layout_tool->addStretch();

    tool_group->setLayout(h_layout_tool);
    v_layout_all->addWidget(tool_group);

    auto newBuildAction = menu->addAction("新建流图");
    auto loadAction = menu->addAction("打开流图");
    auto saveAction = menu->addAction("保存流图");
    auto exitAction = menu->addAction("退出");

    auto helpAction = help_menu->addAction("操作说明");
    auto copyrightAction = help_menu->addAction("版权说明");

    auto data_flow_graphics_model = DataFlowGraphModel(registry);

    auto scene = new DataFlowGraphicsScene(data_flow_graphics_model, &mainWidget);

    auto view = new GraphicsView(scene);

    auto save_act = new QAction(scene);
    save_act->setShortcutContext(Qt::ShortcutContext::WidgetShortcut);
    save_act->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_S));
    view->addAction(save_act);
    QObject::connect(save_act, &QAction::triggered, [=] { saveAction->triggered(); });

    v_layout_all->addWidget(view);
    v_layout_all->setContentsMargins(0, 0, 0, 0);
    v_layout_all->setSpacing(0);

    QObject::connect(lock_btn, &QPushButton::clicked, [&] {
        lock_status = !lock_status;
        if (lock_status) {
            lock_btn->setIcon(lock_icon);
            view->setEnabled(false);
            menuBar->setEnabled(false);
            lock_btn->setStyleSheet("QPushButton {color:white;background-color:rgb(100, 0, 0);}");
        } else {
            lock_btn->setIcon(unlock_icon);
            view->setEnabled(true);
            menuBar->setEnabled(true);
            lock_btn->setStyleSheet("");
        }
    });

    QObject::connect(saveAction, &QAction::triggered, scene, &DataFlowGraphicsScene::save);

    QObject::connect(new_btn, &QPushButton::clicked, [&] { newBuildAction->triggered(); });

    QObject::connect(openfile_btn, &QPushButton::clicked, [&] { loadAction->triggered(); });

    QObject::connect(save_btn, &QPushButton::clicked, [&] { saveAction->triggered(); });

    // todo 撤回按钮
    QObject::connect(back_btn, &QPushButton::clicked, [&] {
        createMessageBox(&mainWidget, ":/icons/error.png", "未开发", "功能未开发...", 1, {"返回"});
    });

    // todo 上一步按钮
    QObject::connect(up_btn, &QPushButton::clicked, [&] {
        createMessageBox(&mainWidget, ":/icons/error.png", "未开发", "功能未开发...", 1, {"返回"});
    });

    // todo 下一步按钮
    QObject::connect(down_btn, &QPushButton::clicked, [&] {
        createMessageBox(&mainWidget, ":/icons/error.png", "未开发", "功能未开发...", 1, {"返回"});
    });

    // todo 删除按钮
    QObject::connect(del_btn, &QPushButton::clicked, [&] {
        createMessageBox(&mainWidget, ":/icons/error.png", "未开发", "功能未开发...", 1, {"返回"});
    });

    // todo 全屏按钮
    QObject::connect(full_btn, &QPushButton::clicked, [&] {
        createMessageBox(&mainWidget, ":/icons/error.png", "未开发", "功能未开发...", 1, {"返回"});
    });

    // todo 开始按钮
    QObject::connect(start_btn, &QPushButton::clicked, [&] {
        createMessageBox(&mainWidget, ":/icons/error.png", "未开发", "功能未开发...", 1, {"返回"});
    });

    // todo 停止按钮
    QObject::connect(stop_btn, &QPushButton::clicked, [&] {
        createMessageBox(&mainWidget, ":/icons/error.png", "未开发", "功能未开发...", 1, {"返回"});
    });

    QObject::connect(loadAction, &QAction::triggered, scene, &DataFlowGraphicsScene::load);

    QObject::connect(newBuildAction, &QAction::triggered, [&] {
        auto nodes = data_flow_graphics_model.allNodeIds();
        if (!nodes.empty()) {
            auto f = createMessageBox(
                &mainWidget,
                ":/icons/inquire.png",
                "是否保存当前流图",
                "当前流图中,可能存在未保存的图文件,新建可能会导致当前文件丢失,是否保存流图?",
                3,
                {"取消", "否", "是"});
            if (f == 0) {
                return;
            } else if (f == 2) {
                saveAction->triggered(true);
            }
            for (auto node_id : nodes) {
                data_flow_graphics_model.deleteNode(node_id);
            }
        } else {
            return;
        }
    });

    QObject::connect(exitAction, &QAction::triggered, [&] { mainWidget.close(); });

    // filter menu
    QObject::connect(scene, &DataFlowGraphicsScene::sceneLoaded, view, &GraphicsView::centerScene);

    // 帮助说明
    QObject::connect(helpAction, &QAction::triggered, [&] {
        createMessageBox(&mainWidget, ":icons/cool.png", "操作说明", "稍后补充......", 1, {"返回"});
    });

    // 版权说明
    QObject::connect(copyrightAction, &QAction::triggered, [&] {
        createMessageBox(
            &mainWidget,
            ":/icons/shy.png",
            "版权说明",
            "本软件免费使用,开发初衷皆为致力于打造简单/高效/"
            "复用率高的生产环境,欢迎使用!\n\nDmitry Pinaev et al, Qt Nodes, (2022), GitHub "
            "repository, https://github.com/paceholder/nodeeditor\n\nAuthor: Wrc development. ",
            1,
            {"返回"});
    });

    mainWidget.setWindowTitle("Streamer");

    mainWidget.setWindowIcon(QIcon(":icons/winicon.svg"));
    mainWidget.setWindowState(Qt::WindowMaximized);
    mainWidget.show();

    return QApplication::exec();
}
