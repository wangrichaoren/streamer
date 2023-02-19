#include "Widget/MyGraphicsView.h"

MyGraphicsView::MyGraphicsView(QGraphicsView *instance, bool is_mark_model)
    : instance{instance}
    , is_mark_model{is_mark_model}
{
    graphicsScene = new QGraphicsScene(instance); //要用QGraphicsView就必须要有QGraphicsScene搭配着用
    init();
}

void MyGraphicsView::init()
{
    instance->setMouseTracking(true);
    instance->setFocusPolicy(Qt::StrongFocus);
    setAcceptDrops(true);
    roirect = QRectF(0, 0, 0, 0);
    m_scaleValue = 0;
    m_scaleDafault = 0;
    direction = dir_none;
}

QRectF MyGraphicsView::boundingRect() const
{
    return QRectF(-m_pix.width() / 2, -m_pix.height() / 2, m_pix.width(), m_pix.height());
}

void MyGraphicsView::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    // 不能频繁的update，会导致按钮阻塞
    painter->drawPixmap(-m_pix.width() / 2, -m_pix.height() / 2, m_pix);
    painter->drawRect(roirect);
    QPen pen;
    pen.setColor(Qt::blue);
    pen.setWidth(EDGE_WIDTH);

    painter->setPen(pen);
    painter->drawRect(roirect);
}

void MyGraphicsView::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    auto mousep = event->pos();

    if (event->button() == Qt::LeftButton) {
        m_startPos = event->pos(); //鼠标左击时，获取当前鼠标在图片中的坐标，

        if (roirect.topLeft().x() + EDGPADDING <= mousep.x()
            && mousep.x() <= roirect.topRight().x() - EDGPADDING
            && mousep.y() <= roirect.topLeft().y() + EDGPADDING
            && mousep.y() >= roirect.topLeft().y() - EDGPADDING) {
            // 上边框
            //            cout<<"上边框"<<endl;
            direction = dir_top;
            instance->setCursor(Qt::SizeVerCursor);
        } else if (mousep.x() >= roirect.bottomLeft().x() + EDGPADDING
                   && mousep.x() <= roirect.bottomRight().x() - EDGPADDING
                   && mousep.y() >= roirect.bottomLeft().y() - EDGPADDING
                   && mousep.y() <= roirect.bottomLeft().y() + EDGPADDING) {
            // 下边框
            //            cout<<"下边框"<<endl;
            direction = dir_bottom;
            instance->setCursor(Qt::SizeVerCursor);
        } else if (mousep.x() <= roirect.topLeft().x() + EDGPADDING
                   && mousep.x() >= roirect.topLeft().x() - EDGPADDING
                   && mousep.y() >= roirect.topLeft().y() + EDGPADDING
                   && mousep.y() <= roirect.bottomLeft().y() - EDGPADDING) {
            // 左边框
            //            cout<<"左边框"<<endl;
            direction = dir_left;
            instance->setCursor(Qt::SizeHorCursor);
        } else if (mousep.x() <= roirect.topRight().x() + EDGPADDING
                   && mousep.x() >= roirect.topRight().x() - EDGPADDING
                   && mousep.y() >= roirect.topRight().y() + EDGPADDING
                   && mousep.y() <= roirect.bottomRight().y() - EDGPADDING) {
            // 右边框
            //            cout<<"右边框"<<endl;
            direction = dir_right;
            instance->setCursor(Qt::SizeHorCursor);
        } else if (QRectF(roirect.topLeft().x() + EDGPADDING,
                          roirect.topLeft().y() + EDGPADDING,
                          roirect.width() - 2 * EDGPADDING,
                          roirect.height() - 2 * EDGPADDING)
                       .contains(mousep)) {
            // roi中间
            //            cout<<"roi中间"<<endl;
            direction = dir_mid;
            instance->setCursor(Qt::ClosedHandCursor);
        } else if (mousep.x() >= roirect.topLeft().x() - EDGPADDING
                   && mousep.x() <= roirect.topLeft().x() + EDGPADDING
                   && mousep.y() >= roirect.topLeft().y() - EDGPADDING
                   && mousep.y() <= roirect.topLeft().y() + EDGPADDING) {
            // 左上角
            direction = dir_top_left;
            instance->setCursor(Qt::CrossCursor);
        } else if (mousep.x() >= roirect.bottomRight().x() - EDGPADDING
                   && mousep.x() <= roirect.bottomRight().x() + EDGPADDING
                   && mousep.y() >= roirect.bottomRight().y() - EDGPADDING
                   && mousep.y() <= roirect.bottomRight().y() + EDGPADDING) {
            // 右下角
            direction = dir_bottom_right;
            instance->setCursor(Qt::CrossCursor);
        } else if (mousep.x() >= roirect.topRight().x() - EDGPADDING
                   && mousep.x() <= roirect.topRight().x() + EDGPADDING
                   && mousep.y() >= roirect.topRight().y() - EDGPADDING
                   && mousep.y() <= roirect.topRight().y() + EDGPADDING) {
            // 右上角
            direction = dir_top_right;
            instance->setCursor(Qt::CrossCursor);
        } else if (mousep.x() >= roirect.bottomLeft().x() - EDGPADDING
                   && mousep.x() <= roirect.bottomLeft().x() + EDGPADDING
                   && mousep.y() >= roirect.bottomLeft().y() - EDGPADDING
                   && mousep.y() <= roirect.bottomLeft().y() + EDGPADDING) {
            // 左下角
            direction = dir_bottom_left;
            instance->setCursor(Qt::CrossCursor);
        } else {
            // roi外面
            //            cout<<"roi外面"<<endl;
            direction = dir_none;
            instance->setCursor(Qt::ArrowCursor);
        }

    } else if (event->button() == Qt::RightButton) {
        ResetItemPos(); //右击鼠标重置大小
    }
}

void MyGraphicsView::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if (!is_mark_model) {
        QPointF point = (event->pos() - m_startPos) * m_scaleValue;
        moveBy(point.x(), point.y());
        return;
    }
    // todo ？需要鼠标点击后在移动才能进入 move事件函数？不合理啊？
    if ((direction == dir_top) | (direction == dir_bottom) | (direction == dir_left)
        | (direction == dir_right)) {
        moveRectLine(event, direction);
    } else if ((direction == dir_top_left) | (direction == dir_top_right)
               | (direction == dir_bottom_left) | (direction == dir_bottom_right)) {
        scaleRect(event, direction);
    } else if (direction == dir_mid) {
        moveRect(event->pos());
    } else if (direction == dir_none) {
        QPointF point = (event->pos() - m_startPos) * m_scaleValue;
        moveBy(point.x(), point.y());
    }

    update();
}

void MyGraphicsView::mouseReleaseEvent(QGraphicsSceneMouseEvent *)
{
    instance->setCursor(Qt::ArrowCursor);
}

void MyGraphicsView::wheelEvent(QGraphicsSceneWheelEvent *event) //鼠标滚轮事件
{
    if ((event->delta() > 0) && (m_scaleValue >= 50)) //最大放大到原始图像的50倍
    {
        return;
    } else if ((event->delta() < 0)
               && (m_scaleValue <= m_scaleDafault)) //图像缩小到自适应大小之后就不继续缩小
    {
        ResetItemPos(); //重置图片大小和位置，使之自适应控件窗口大小
    } else {
        qreal qrealOriginScale = m_scaleValue;
        if (event->delta() > 0) //鼠标滚轮向前滚动
        {
            m_scaleValue *= 1.1; //每次放大10%
        } else {
            m_scaleValue *= 0.9; //每次缩小10%
        }
        setScale(m_scaleValue);
        if (event->delta() > 0) {
            moveBy(-event->pos().x() * qrealOriginScale * 0.1,
                   -event->pos().y() * qrealOriginScale
                       * 0.1); //使图片缩放的效果看起来像是以鼠标所在点为中心进行缩放的
        } else {
            moveBy(event->pos().x() * qrealOriginScale * 0.1,
                   event->pos().y() * qrealOriginScale
                       * 0.1); //使图片缩放的效果看起来像是以鼠标所在点为中心进行缩放的
        }
    }
}

void MyGraphicsView::setQGraphicsViewWH(
    int nwidth,
    int nheight) //将主界面的控件QGraphicsView的width和height传进本类中，并根据图像的长宽和控件的长宽的比例来使图片缩放到适合控件的大小
{
    int nImgWidth = m_pix.width();
    int nImgHeight = m_pix.height();
    qreal temp1 = nwidth * 1.0 / nImgWidth;
    qreal temp2 = nheight * 1.0 / nImgHeight;
    if (temp1 > temp2) {
        m_scaleDafault = temp2;
    } else {
        m_scaleDafault = temp1;
    }
    setScale(m_scaleDafault);
    m_scaleValue = m_scaleDafault;
}

void MyGraphicsView::ResetItemPos() //重置图片位置
{
    m_scaleValue = m_scaleDafault; //缩放比例回到一开始的自适应比例
    setScale(m_scaleDafault);      //缩放到一开始的自适应大小
    setPos(0, 0);
}

qreal MyGraphicsView::getScaleValue() const
{
    return m_scaleValue;
}

void MyGraphicsView::setImage(QPixmap *pixmap)
{
    m_pix = *pixmap;
}

void MyGraphicsView::graphics(QImage &image, string path)
{
    img_path = path;
    roirect = QRectF(0, 0, 0, 0);
    QPixmap ConvertPixmap = QPixmap::fromImage(
        image); //The QPixmap class is an off-screen image representation that can be used as a paint device
    setImage(&ConvertPixmap);
    int nwith = instance->width();    //获取界面控件Graphics View的宽度
    int nheight = instance->height(); //获取界面控件Graphics View的高度
    setQGraphicsViewWH(nwith, nheight); //将界面控件Graphics View的width和height传进类m_Image中
    graphicsScene->addItem(this);       //将QGraphicsItem类对象放进QGraphicsScene中
    instance->setSceneRect(QRectF(
        -(nwith / 2),
        -(nheight / 2),
        nwith,
        nheight)); //使视窗的大小固定在原始大小，不会随图片的放大而放大（默认状态下图片放大的时候视窗两边会自动出现滚动条，并且视窗内的视野会变大），防止图片放大后重新缩小的时候视窗太大而不方便观察图片
    instance->setScene(
        graphicsScene); //Sets the current scene to scene. If scene is already being viewed, this function does nothing.
    instance->setFocus(); //将界面的焦点设置到当前Graphics View控件
}

void MyGraphicsView::graphics(cv::Mat &mat)
{
    //    graphicsScene->removeItem(this);
    graphicsScene->removeItem(this->topLevelItem());
    auto q_img = cvMat2QImage(mat);

    QPixmap ConvertPixmap = QPixmap::fromImage(q_img);

    setImage(&ConvertPixmap);
    int nwith = instance->width();    //获取界面控件Graphics View的宽度
    int nheight = instance->height(); //获取界面控件Graphics View的高度
    setQGraphicsViewWH(nwith, nheight); //将界面控件Graphics View的width和height传进类m_Image中
    graphicsScene->addItem(this);       //将QGraphicsItem类对象放进QGraphicsScene中
    instance->setSceneRect(QRectF(
        -(nwith / 2),
        -(nheight / 2),
        nwith,
        nheight)); //使视窗的大小固定在原始大小，不会随图片的放大而放大（默认状态下图片放大的时候视窗两边会自动出现滚动条，并且视窗内的视野会变大），防止图片放大后重新缩小的时候视窗太大而不方便观察图片
    instance->setScene(
        graphicsScene); //Sets the current scene to scene. If scene is already being viewed, this function does nothing.
    instance->setFocus(); //将界面的焦点设置到当前Graphics View控件
}

MyGraphicsView::~MyGraphicsView()
{
    std::cout << "xxx graphicsScene" << std::endl;
}

void MyGraphicsView::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
    if (!is_mark_model) {
        return;
    }
    QGraphicsItem::mouseDoubleClickEvent(event);
    auto p = event->pos();
    if (roirect.contains(p)) {
        qDebug("鼠标在roi内，删除roi");
        roirect = QRectF(0, 0, 0, 0);
    } else {
        qDebug("鼠标不在roi内，重新生成roi");
        roirect = QRectF(p.x(), p.y(), 100, 100);
    }
    update();
}

void MyGraphicsView::moveRect(const QPointF &mousePoint)
{
    auto diffx = mousePoint.x() - m_startPos.x();
    auto diffy = mousePoint.y() - m_startPos.y();
    ;
    QRectF ret(roirect.x() + diffx, roirect.y() + diffy, roirect.width(), roirect.height());
    if (this->shape().contains(ret)) {
        roirect = ret;
        m_startPos = mousePoint;
    }
}

void MyGraphicsView::moveRectLine(QGraphicsSceneMouseEvent *event, EnumDirection dir)
{
    QRectF tmp_rect;
    auto mousep = event->pos();

    if (dir == dir_top) {
        tmp_rect.setBottomRight(roirect.bottomRight());
        tmp_rect.setTopLeft(QPointF(roirect.x(), mousep.y()));
    } else if (dir == dir_bottom) {
        tmp_rect.setTopLeft(roirect.topLeft());
        tmp_rect.setBottomRight(QPointF(roirect.bottomRight().x(), mousep.y()));
    } else if (dir == dir_left) {
        tmp_rect.setTopLeft(QPointF(mousep.x(), roirect.y()));
        tmp_rect.setBottomRight(roirect.bottomRight());
    } else if (dir == dir_right) {
        tmp_rect.setTopLeft(roirect.topLeft());
        tmp_rect.setBottomRight(QPointF(mousep.x(), roirect.bottomRight().y()));
    }
    if (this->shape().contains(mousep)) {
        roirect = tmp_rect;
        m_startPos = mousep;
    }
}

void MyGraphicsView::scaleRect(QGraphicsSceneMouseEvent *event, EnumDirection dir)
{
    QRectF tmp_rect;
    auto mousep = event->pos();

    if (dir == dir_top_left) {
        tmp_rect.setTopLeft(QPointF(mousep.x(), mousep.y()));
        tmp_rect.setBottomRight(roirect.bottomRight());
    } else if (dir == dir_bottom_right) {
        tmp_rect.setTopLeft(roirect.topLeft());
        tmp_rect.setBottomRight(QPointF(mousep.x(), mousep.y()));
    } else if (dir == dir_top_right) {
        tmp_rect.setBottomLeft(roirect.bottomLeft());
        tmp_rect.setTopRight(QPointF(mousep.x(), mousep.y()));
    } else if (dir == dir_bottom_left) {
        tmp_rect.setBottomLeft(QPointF(mousep.x(), mousep.y()));
        tmp_rect.setTopRight(roirect.topRight());
    }

    if (this->shape().contains(mousep)) {
        roirect = tmp_rect;
        m_startPos = mousep;
    }
}

cv::Mat MyGraphicsView::getRoiRectToMat()
{
    if (!hasRoiRect()) {
        throw runtime_error("请先框选roi，再执行训练!");
    }
    // show的时候是以图像中心为原点，要强制转一下!
    /**
     * 算法思路：
     */
    QPointF trans_point(-m_pix.width() / 2, -m_pix.height() / 2);
    auto diffx = roirect.center().x() - trans_point.x();
    auto diffy = roirect.center().y() - trans_point.y();
    auto topleft = QPointF(diffx - roirect.width() / 2, diffy - roirect.height() / 2);
    //    auto botright = QPointF(diffx + roirect.width() / 2, diffy + roirect.height() / 2);
    // 直接从原图获取
    auto mat = cv::imread(img_path);
    auto cvimg = mat(cv::Range(topleft.y(), topleft.y() + roirect.height()),
                     cv::Range(topleft.x(), topleft.x() + roirect.width()));

    // bug 从Qimgae转换成Mat的时候产生的图片有条纹...直接截取原图
    //    QRectF tmprect = QRectF(topleft, botright);
    //    auto tp = m_pix.toImage().copy(tmprect.toRect());
    //
    //    auto cvimg = QImage2Mat(tp);

    return cvimg;
}

bool MyGraphicsView::hasRoiRect()
{
    if (roirect.isEmpty()) {
        return false;
    }
    return true;
}

cv::Mat MyGraphicsView::getScenceToMat()
{
    return cv::imread(img_path);
}
void MyGraphicsView::clearView()
{
    graphicsScene->removeItem(this->topLevelItem());
    //    std::cout << "has roi: " << hasRoiRect() << std::endl;

    roirect = QRectF(0, 0, 0, 0);
}
