#include <QtNodes/ConnectionStyle>
#include <QtNodes/DataFlowGraphModel>
#include <QtNodes/DataFlowGraphicsScene>
#include <QtNodes/GraphicsView>
#include <QtNodes/NodeData>
#include <QtNodes/NodeDelegateModelRegistry>

#include <memory>
#include <QMessageBox>
#include <QtGui/QScreen>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QVBoxLayout>

#include <QtGui/QScreen>

#include "AdditionModel.hpp"
#include "DivisionModel.hpp"
#include "MultiplicationModel.hpp"
#include "NumberDisplayDataModel.hpp"
#include "NumberSourceDataModel.hpp"
#include "SubtractionModel.hpp"

using QtNodes::ConnectionStyle;
using QtNodes::DataFlowGraphicsScene;
using QtNodes::DataFlowGraphModel;
using QtNodes::GraphicsView;
using QtNodes::NodeDelegateModelRegistry;

static std::shared_ptr<NodeDelegateModelRegistry> registerDataModels() {
    auto ret = std::make_shared<NodeDelegateModelRegistry>();
    ret->registerModel<NumberSourceDataModel>("数据源");

    ret->registerModel<NumberDisplayDataModel>("结果");

    ret->registerModel<AdditionModel>("运算");

    ret->registerModel<SubtractionModel>("运算");

    ret->registerModel<MultiplicationModel>("运算");

    ret->registerModel<DivisionModel>("运算");

    return ret;
}

static void setStyle() {
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

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    setStyle();

    std::shared_ptr<NodeDelegateModelRegistry> registry = registerDataModels();

    QWidget mainWidget;

    auto menuBar = new QMenuBar();
    QMenu *menu = menuBar->addMenu("文件");
    QMenu *about_menu = menuBar->addMenu("关于");

    auto saveAction = menu->addAction("保存流图");
    auto loadAction = menu->addAction("加载流图");

    auto helpAction = about_menu->addAction("帮助");
    auto copyrightAction = about_menu->addAction("版权说明");

    auto *l = new QVBoxLayout(&mainWidget);

    DataFlowGraphModel dataFlowGraphModel(registry);

    l->addWidget(menuBar);
    auto scene = new DataFlowGraphicsScene(dataFlowGraphModel, &mainWidget);

    auto view = new GraphicsView(scene);
    l->addWidget(view);
    l->setContentsMargins(0, 0, 0, 0);
    l->setSpacing(0);

    QObject::connect(saveAction, &QAction::triggered, scene, &DataFlowGraphicsScene::save);

    QObject::connect(loadAction, &QAction::triggered, scene, &DataFlowGraphicsScene::load);

    QObject::connect(scene, &DataFlowGraphicsScene::sceneLoaded, view, &GraphicsView::centerScene);

    QObject::connect(helpAction, &QAction::triggered, [&] {
        QMessageBox::information(&mainWidget,
                                 "关于",
                                 "鼠标右键控件即可生成弹出工具菜单,将对应数据流向连接即可输出结果.",
                                 "返回");
    });

    QObject::connect(copyrightAction, &QAction::triggered, [&] {
        QMessageBox::information(&mainWidget, "版权说明", "本软件为免费资源,欢迎使用!", "返回");
    });

    mainWidget.setWindowTitle("Vision Tool");
    mainWidget.setWindowState(Qt::WindowMaximized);
    mainWidget.show();

    return QApplication::exec();
}
