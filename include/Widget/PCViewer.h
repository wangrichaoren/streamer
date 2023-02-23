//
// Created by wrc on 23-2-23.
//

#ifndef STREAMER_PCVIEWER_H
#define STREAMER_PCVIEWER_H

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <QDialog>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>


QT_BEGIN_NAMESPACE
namespace Ui {
class PCViewer;
}
QT_END_NAMESPACE

class PCViewer : public QDialog
{
    Q_OBJECT

public:
    explicit PCViewer(QWidget *parent = nullptr,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = nullptr,bool is_show_coord=true);
    ~PCViewer() override;

    void initialVtkWidget();

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pc;
    Ui::PCViewer *ui;
    bool is_show_coord;
};

#endif //STREAMER_PCVIEWER_H
