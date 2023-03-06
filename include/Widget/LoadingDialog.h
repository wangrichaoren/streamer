//
// Created by wrc on 23-2-27.
//

#ifndef STREAMER_LOADINGDIALOG_H
#define STREAMER_LOADINGDIALOG_H
#include <QDialog>
#include <QGraphicsDropShadowEffect>
#include <QLabel>
#include <QMovie>
#include <QPainter>
#include <QPushButton>
#include <QThread>
//#define USER_CANCEL -1
class LoadingDialog
    : public QDialog
{
    Q_OBJECT
public:
    explicit LoadingDialog(QWidget *parent = nullptr);
    ~LoadingDialog();
    //设置提示文本
    void setTipsText(QString strTipsText);
    //设置是否显示取消等待按钮
//    void setCanCancel(bool bCanCancel);
    //移动到指定窗口中间显示
    void moveToCenter(QWidget *pParent);


protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void initUi();
Q_SIGNALS:
    void cancelWaiting();
public slots:
    void cancelBtnClicked();

    void loadingShow();

private:
    QFrame *m_pCenterFrame;
    QLabel *m_pMovieLabel;
    QMovie *m_pLoadingMovie;
    QLabel *m_pTipsLabel;
};
#endif //STREAMER_LOADINGDIALOG_H
