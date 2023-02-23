//
// Created by wrc on 23-2-23.
//

// You may need to build the project (run Qt uic code generator) to get "ui_PCViewer.h" resolved

#include "Widget/PCViewer.h"

#include "Widget/ui_PCViewer.h"
#include <utility>

PCViewer::PCViewer(QWidget *parent, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, bool is_show_coord)
    : QDialog(parent)
    , ui(new Ui::PCViewer)
    , _pc(std::move(pc))
    , is_show_coord(is_show_coord)
{
    ui->setupUi(this);

    initialVtkWidget();
}

PCViewer::~PCViewer()
{
    delete ui;
}

void PCViewer::initialVtkWidget()
{
    pcl::visualization::PCLVisualizer::Ptr pcl_viewer(
        new pcl::visualization::PCLVisualizer("pcl_viewer", false));
    ui->qvtkWidget->SetRenderWindow(pcl_viewer->getRenderWindow());
    pcl_viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    pcl_viewer->removeAllPointClouds();
    auto vec_rgb = _pc->points.data()->getRGBVector3i();
    if ((vec_rgb[0] + vec_rgb[1] + vec_rgb[2]) == 0) {
        std::cout << "not rgb" << std::endl;
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> rgb(_pc, "y");
        //           pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> rgb(_pc);
        pcl_viewer->addPointCloud(_pc, rgb, "cloud");
    } else {
        std::cout << "is rgb" << std::endl;
        pcl_viewer->addPointCloud(_pc, "cloud");
    }
    if (is_show_coord) {
        pcl_viewer->addCoordinateSystem(0.5);
    }
    pcl_viewer->addPointCloud(_pc, "cloud");
//    pcl_viewer->resetCamera();
    ui->qvtkWidget->update();
}
