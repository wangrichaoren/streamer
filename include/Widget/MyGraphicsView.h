//
// Created by wrc on 23-2-16.
//

#ifndef STREAMER_MYGRAPHICSVIEW_H
#define STREAMER_MYGRAPHICSVIEW_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QDragEnterEvent>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QGraphicsView>
#include <QLabel>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QPointF>
#include <QRectF>
#include <QTimer>
#include <QWidget>
#include <QtConcurrent/QtConcurrent>
#include <QtGui>
#include <QGraphicsSceneMouseEvent>
#include "Utils/Utils.hpp"


using namespace std;

#define EDGE_WIDTH 2 //边框宽度
#define EDGPADDING 5

// 枚举的roi拖拽方向+角点
enum EnumDirection {
    dir_mid,
    dir_top,
    dir_bottom,
    dir_left,
    dir_right,
    dir_none,
    dir_top_left,
    dir_top_right,
    dir_bottom_left,
    dir_bottom_right
};

class MyGraphicsView : public QGraphicsItem
{
    //    Q_OBJECT
public:
    MyGraphicsView(QGraphicsView *instance);

    ~MyGraphicsView();

    bool hasRoiRect();

    cv::Mat getRoiRectToMat();

    cv::Mat getScenceToMat();

    void setImage(QPixmap *pixmap);

    void init();

    void graphics(QImage &image, string path);

    void graphics(cv::Mat &mat);


    QRectF boundingRect() const;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *);

    void wheelEvent(QGraphicsSceneWheelEvent *event);

    //    EmDirection2 region(const QPointF &point);
    void moveRect(const QPointF &mousePoint);

    void moveRectLine(QGraphicsSceneMouseEvent *, EnumDirection);

    void scaleRect(QGraphicsSceneMouseEvent *, EnumDirection);

    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);

    void ResetItemPos();

    void mousePressEvent(QGraphicsSceneMouseEvent *event);

    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

    qreal getScaleValue() const;

    void setQGraphicsViewWH(int nwidth, int nheight);

private:
    QGraphicsView *instance;
    qreal m_scaleValue;
    qreal m_scaleDafault;
    QPixmap m_pix;
    QPointF m_startPos;
    QRectF roirect;
    QGraphicsScene *graphicsScene;
    EnumDirection direction;
    string img_path;
};

#endif //STREAMER_MYGRAPHICSVIEW_H
