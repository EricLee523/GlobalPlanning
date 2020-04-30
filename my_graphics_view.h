//
// Created by lzy on 19-9-20.
//
#ifndef GLOBALPLANNING_MYGRAPHICSVIEW_H
#define GLOBALPLANNING_MYGRAPHICSVIEW_H
#include <QObject>
#include <QGraphicsView>
#include <QWidget>
#include <QColor>
#include <QDebug>
#include <QPoint>
#include <QMouseEvent>
#include <QPainterPath>
#include <QPainter>
#include <QPicture>
#include <QImage>
#include <QPixmap>
#include <QRect>
#include <QGraphicsPixmapItem>
#include "global_planner.h"

QT_BEGIN_NAMESPACE
class QWheelEvent;
QT_END_NAMESPACE

class MyGraphicsView : public QGraphicsView
{
Q_OBJECT
public:
    explicit MyGraphicsView(QWidget *parent = 0);
    ~MyGraphicsView();
    /**
     * @brief 将mat类型的图像转换成QImage型的图像
     * @param image1
     */
    void ShowImage(cv::Mat image1);
    /**
     * @brief 画出规划好的路线
     */
    void DrawChoiceWay();


    GlobalPlanner *ptr_globalplanner_;
    Mat image_;//保留原始解析后的图像信息
    Mat image_show_;//在该图像上添加规划路径信息
    double scale_ = 90000;//缩放比例
    bool start_btn_click_state_ = false;
    bool end_btn_click_state_ = false;

private:
    /**
     * @brief 加载地图并初始化图像信息
     */
    void initImage();

    /**
     * @brief 创建地图在界面中显示
     */
    void ConstructQtMap();



    QImage qimage_;
    double edge_width_ = 60;
    QGraphicsScene *scene;
    bool mouse_translate_flag_;
    QPoint last_mouse_pos_;  // 鼠标最后按下的位置

protected:

    void mousePressEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

};


#endif //GLOBALPLANNING_MYGRAPHICSVIEW_H