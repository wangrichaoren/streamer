//
// Created by wrc on 23-2-22.
//

// You may need to build the project (run Qt uic code generator) to get "ui_StreamerMainWindow.h" resolved

#include "Widget/StreamerMainWindow.hpp"

#include "Widget/ui_StreamerMainWindow.h"
#include <utility>

StreamerMainWindow::StreamerMainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::StreamerMainWindow)
{
    ui->setupUi(this);

    setupStyle();

    auto reg = registerDataModels();

    // setup center window
    data_flow_graphics_model = new DataFlowGraphModel(reg);
    scene = new DataFlowGraphicsScene(*data_flow_graphics_model, this);
    view = new GraphicsView(scene);
    ui->frame_2->layout()->addWidget(view);

    // center window mask
    view_mask = new QWidget(this);
    QPalette pal(view_mask->palette());
    view_mask->setStyleSheet("background-color:rgba(0, 0, 0, 60%);");
    view_mask->setAutoFillBackground(true);
    view_mask->setPalette(pal);
    view_mask->hide();

    // setup vtk widget
    initialVtkWidget();

    // tool bar add btn
    ui->toolBar->layout()->setSpacing(10); // 设置tool bar间按钮的间隙，20px
    new_flow_btn = new QPushButton(this);
    new_flow_btn->setIcon(QIcon(":icons/new.png"));
    new_flow_btn->setToolTip("新建Flow");
    open_flow_btn = new QPushButton(this);
    open_flow_btn->setIcon(QIcon(":icons/open.png"));
    open_flow_btn->setToolTip("打开Flow");
    save_flow_btn = new QPushButton(this);
    save_flow_btn->setIcon(QIcon(":icons/save.png"));
    save_flow_btn->setToolTip("保存Flow");
    lock_btn = new QPushButton(this);
    lock_btn->setIcon(QIcon(":icons/lock.png"));
    lock_btn->setToolTip("Streamer上锁");

    ui->show_coords_checkBox->setIcon(QIcon(":icons/xyz.png"));
    ui->show_coords_checkBox->setToolTip("显示坐标");

    pcl_viewer_btn = new QPushButton(this);
    pcl_viewer_btn->setIcon(QIcon(":icons/pc_undis.png"));
    pcl_viewer_btn->setToolTip("3D侧边烂显示");

    ui->toolBar->setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonIconOnly);
    ui->toolBar->addWidget(new_flow_btn);   // 新建
    ui->toolBar->addWidget(open_flow_btn);  // 打开
    ui->toolBar->addWidget(save_flow_btn);  // 保存
    ui->toolBar->addWidget(lock_btn);       // 图层上锁
    ui->toolBar->addWidget(pcl_viewer_btn); // 隐藏3d显示栏

    ui->frame_2->installEventFilter(this);

    // check box connect
    connect(ui->show_coords_checkBox, &QCheckBox::clicked, [=](bool f) {
        if (f) {
            pcl_viewer->addCoordinateSystem(0.5);
            ui->qvtkWidget->update();
        } else {
            pcl_viewer->removeAllCoordinateSystems();
            ui->qvtkWidget->update();
        }
    });

    // btn connect
    connect(pcl_viewer_btn, &QPushButton::clicked, [=] {
        _pc_display_state = !_pc_display_state;
        if (_pc_display_state) {
            ui->tabWidget->hide();
            pcl_viewer_btn->setIcon(QIcon(":icons/pc_dis.png"));
        } else {
            ui->tabWidget->show();
            pcl_viewer_btn->setIcon(QIcon(":icons/pc_undis.png"));
        }
    });

    connect(lock_btn, &QPushButton::clicked, [=] {
        _lock_state = !_lock_state;
        if (_lock_state) {
            lock_btn->setIcon(QIcon(":icons/unlock.png"));
            ui->frame->setEnabled(false);
            ui->menuBar->setEnabled(false);
            new_flow_btn->setEnabled(false);
            open_flow_btn->setEnabled(false);
            save_flow_btn->setEnabled(false);
            pcl_viewer_btn->setEnabled(false);
            if (!_pc_display_state) {
                pcl_viewer_btn->clicked(true);
            }
            lock_btn->setStyleSheet("QPushButton {color:white;background-color:rgb(100, 0, 0);}");
            view_mask->resize(ui->frame_2->width() - 3, ui->frame_2->height());
            view_mask->move(ui->frame_2->x() + 9, // 9 是控件间的spacing
                            ui->frame_2->y() + ui->toolBar->height() + ui->menuBar->height() + 9);
            view_mask->show();

        } else {
            lock_btn->setIcon(QIcon(":icons/lock.png"));
            ui->frame->setEnabled(true);
            ui->menuBar->setEnabled(true);
            new_flow_btn->setEnabled(true);
            open_flow_btn->setEnabled(true);
            save_flow_btn->setEnabled(true);
            pcl_viewer_btn->setEnabled(true);
            lock_btn->setStyleSheet("");
            view_mask->hide();
        }
    });

    connect(new_flow_btn, &QPushButton::clicked, [=] { ui->new_flow_action->triggered(true); });

    connect(open_flow_btn, &QPushButton::clicked, [=] { ui->open_flow_action->triggered(true); });

    connect(save_flow_btn, &QPushButton::clicked, [=] { ui->save_flow_action->triggered(true); });

    // actions connect
    connect(ui->template_marker_action, &QAction::triggered, [=] {
        auto shape_base_marker = new ShapeBaseTrainerDialog(this);
        shape_base_marker->setWindowFlags(Qt::Window);
        shape_base_marker->setWindowState(Qt::WindowMaximized);
        this->hide();
        shape_base_marker->show();
        shape_base_marker->exec();
        shape_base_marker->deleteLater();
        this->show();
    });

    // new build flow file act
    connect(ui->new_flow_action, &QAction::triggered, [=] {
        auto nodes = data_flow_graphics_model->allNodeIds();
        if (!nodes.empty()) {
            auto f = createMessageBox(
                this,
                ":/icons/inquire.png",
                "是否保存当前流图",
                "当前流图中,可能存在未保存的图文件,新建可能会导致当前文件丢失,是否保存流图?",
                3,
                {"取消", "否", "是"});
            if (f == 0) {
                return;
            } else if (f == 2) {
                ui->save_flow_action->triggered(true);
            }
            for (auto node_id : nodes) {
                data_flow_graphics_model->deleteNode(node_id);
            }
        } else {
            return;
        }
    });

    // open exist flow file act
    connect(ui->open_flow_action, &QAction::triggered, scene, &DataFlowGraphicsScene::load);

    // save flow file act
    connect(ui->save_flow_action, &QAction::triggered, scene, &DataFlowGraphicsScene::save);

    connect(ui->cz_action, &QAction::triggered, [=] {
        createMessageBox(this, ":icons/cool.png", "操作说明", "待补充...", 1, {"返回"});
        return;
    });

    connect(ui->about_action, &QAction::triggered, [=] {
        createMessageBox(
            this,
            ":icons/shy.png",
            "版权说明",
            "本软件免费使用,开发初衷皆为致力于打造简单/高效/"
            "复用率高的生产环境,欢迎使用!\n\nDmitry Pinaev et al, Qt Nodes, (2022), GitHub "
            "repository, https://github.com/paceholder/nodeeditor\n\nAuthor: Wrc development. ",
            1,
            {"返回"});
        return;
    });

    connect(scene, &DataFlowGraphicsScene::sceneLoaded, view, &GraphicsView::centerScene);

    connect(this,
            &StreamerMainWindow::updateVTK,
            [=](const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
                pcl::PointCloud<pcl::Normal>::Ptr normal) { VtkRender(std::move(pc), normal); });

    connect(ui->full_pc_pushButton, &QPushButton::clicked, [=] {
        if (m_pc->empty()) {
            return;
        }
        pcl::visualization::Camera camera{};
        pcl_viewer->getCameraParameters(camera);
        auto pc_viewer = new PCViewer(this, camera, m_pc, m_normal,ui->show_coords_checkBox->isChecked());
        pc_viewer->setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
        pc_viewer->showNormal();
        pc_viewer->exec();
        pc_viewer->deleteLater();
    });

    connect(ui->exit_action, &QAction::triggered, [=] { this->close(); });

    // 默认轴显示
    ui->show_coords_checkBox->setChecked(true);
    ui->show_coords_checkBox->clicked(true);
    this->setWindowIcon(QIcon(":icons/winicon.svg"));
    this->setWindowState(Qt::WindowMaximized);
}

StreamerMainWindow::~StreamerMainWindow()
{
    delete pcl_viewer;
    delete data_flow_graphics_model;
    delete ui;
}

bool StreamerMainWindow::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->frame_2) {
        if (event->type() == QEvent::Resize) {
            view_mask->resize(ui->frame_2->width() - 3, ui->frame_2->height());
            view_mask->move(ui->frame_2->x() + 9, // 9 是控件间的spacing
                            ui->frame_2->y() + ui->toolBar->height() + ui->menuBar->height() + 9);
            return true;
        }
    }
    return false;
}

void StreamerMainWindow::VtkRender(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
                                   pcl::PointCloud<pcl::Normal>::Ptr normal)
{
    pcl_viewer->removeAllPointClouds();
    // check xyz or xyzrgb datatype
    auto vec_rgb = pc->points.data()->getRGBVector3i();
    if ((vec_rgb[0] + vec_rgb[1] + vec_rgb[2]) == 0) {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> rgb(pc, "y");
        pcl_viewer->addPointCloud(pc, rgb, "cloud");
    } else {
        pcl_viewer->addPointCloud(pc, "cloud");
    }

    if (normal != nullptr) {
        pcl_viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(pc, normal, 3, 0.2, "normal");
    }

    pcl_viewer->resetCamera();
    ui->qvtkWidget->update();
    m_pc = std::move(pc);
    m_normal = std::move(normal);
};

void StreamerMainWindow::initialVtkWidget()
{
    pcl_viewer = new pcl::visualization::PCLVisualizer("pcl_viewer", false);
    ui->qvtkWidget->SetRenderWindow(pcl_viewer->getRenderWindow());
    pcl_viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
}
