//
// Created by lzy on 19-9-20.
//
#include "my_graphics_view.h"
#include <QWheelEvent>
#include <QPoint>
#include <QDebug>
#define VIEW_WIDTH  viewport()->rect().width()
#define VIEW_HEIGHT viewport()->rect().height()

MyGraphicsView::MyGraphicsView(QWidget *parent)
        : QGraphicsView(parent)
{
    scene = new QGraphicsScene;
    initImage();
}

MyGraphicsView::~MyGraphicsView()
{
    delete ptr_globalplanner_;
    delete scene;

}
/**
 * @brief 加载地图并初始化图像信息
 */
void MyGraphicsView::ShowImage(cv::Mat image1){

    qimage_ = QImage((unsigned char*)image1.data, image1.cols, image1.rows, image1.step, QImage::Format_RGB888);
    scene->addPixmap(QPixmap::fromImage(qimage_));
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);//去掉滚动条
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setScene(scene);
}
/**
 * @brief 加载地图并初始化图像信息
 */
void MyGraphicsView:: initImage()
{
    ptr_globalplanner_ = new GlobalPlanner();
    ptr_globalplanner_->Init();
    ConstructQtMap();
    ShowImage(image_show_);
}


void MyGraphicsView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        QGraphicsPixmapItem *pixmapItem = new QGraphicsPixmapItem();
        QPointF map_pos= pixmapItem->mapFromScene(mapToScene(event->pos()));
        //cout<<"zuobiao:"<<map_pos.x()<<","<<map_pos.y()<<endl;
        mouse_translate_flag_ = true;
        last_mouse_pos_ = event->pos();
        if(start_btn_click_state_)//选择起点
        {
            QRect currentPosRec(map_pos.x() - 4, map_pos.y() - 4, 8, 8);//以鼠标的坐标点为中心新建一个矩形类
            QPoint point;
            double left = ptr_globalplanner_->left_;
            double top = ptr_globalplanner_->top_;
            for(int i = 0; i < ptr_globalplanner_->my_way_vec_.size();++i)
            {
                if(ptr_globalplanner_->my_way_vec_[i].road_type == 3)
                    continue;
                node start_node = ptr_globalplanner_->my_way_vec_[i].referLine_offset[0];//偏移出来用来做全局规划的第一个点
                int x = edge_width_ / 2 + (int) ((start_node.lng - left) * scale_);
                int y = edge_width_ / 2 + (int) ((start_node.lat - top) * scale_);
                y = image_.rows - 1 - y;
                point.setX(x);
                point.setY(y);
                if(point.x()> currentPosRec.left() && point.x() <= currentPosRec.right() && point.y() >= currentPosRec.top() && point.y() <= currentPosRec.bottom())
                {
                    circle(image_show_, cvPoint(point.x(), point.y()), 4, Scalar(0, 255, 0), -1);
                    for(int j = 0; j < ptr_globalplanner_->map_data_.size(); ++j)
                    {
                        if(ptr_globalplanner_->map_data_[j].lat == start_node.lat && ptr_globalplanner_->map_data_[j].lng == start_node.lng )
                        {
                            ptr_globalplanner_->start_node_id_ = ptr_globalplanner_->map_data_[j].id;
                            cout<<ptr_globalplanner_->start_node_id_<<endl;
                            break;
                        }
                    }
                    break;
                }

            }

            start_btn_click_state_ = false;
            ShowImage(image_show_);
        }
        if(end_btn_click_state_ && !start_btn_click_state_)//选择终点
        {
            QRect currentPosRec(map_pos.x() - 4, map_pos.y() - 4, 8, 8);//以鼠标的坐标点为中心新建一个矩形类
            QPoint point;
            double left = ptr_globalplanner_->left_;
            double top = ptr_globalplanner_->top_;
            for (int i = 0; i < ptr_globalplanner_->my_way_vec_.size(); ++i)
            {
                if(ptr_globalplanner_->my_way_vec_[i].road_type == 3)
                    continue;

                int num = ptr_globalplanner_->my_way_vec_[i].referLine_offset.size();
                node end_node = ptr_globalplanner_->my_way_vec_[i].referLine_offset[num - 1];//偏移出来用来做全局规划的最后一个点
                int x = edge_width_ / 2 + (int) ((end_node.lng - left) * scale_);
                int y = edge_width_ / 2 + (int) ((end_node.lat - top) * scale_);
                y = image_.rows - 1 - y;
                point.setX(x);
                point.setY(y);
                if (point.x() > currentPosRec.left() && point.x() <= currentPosRec.right() && point.y() >= currentPosRec.top() && point.y() <= currentPosRec.bottom())
                {
                    circle(image_show_, cvPoint(point.x(), point.y()), 4, Scalar(255, 0, 0), -1);
                    for(int j = 0; j < ptr_globalplanner_->map_data_.size(); ++j)
                    {
                        if(ptr_globalplanner_->map_data_[j].lat == end_node.lat && ptr_globalplanner_->map_data_[j].lng == end_node.lng )
                        {
                            ptr_globalplanner_->goal_node_id_ = ptr_globalplanner_->map_data_[j].id;
                            cout<<ptr_globalplanner_->goal_node_id_<<endl;
                            break;
                        }
                    }

                    break;
                }

            }

            end_btn_click_state_ = false;
            ShowImage(image_show_);
        }
        delete pixmapItem;
    }
}



void MyGraphicsView:: mouseDoubleClickEvent(QMouseEvent *event)
{

}

/**
 * @brief 创建地图在界面中显示
 */
void MyGraphicsView::ConstructQtMap() {
    int image_width =(int) ((ptr_globalplanner_->right_ - ptr_globalplanner_->left_) * scale_) + edge_width_;//以scale倍的比例建图
    int image_height = (int) ((ptr_globalplanner_->bottom_ - ptr_globalplanner_->top_) * scale_) + edge_width_;
    image_ = Mat(image_height, image_width, CV_8UC3, Scalar(134,134,134));
    double left = ptr_globalplanner_->left_;
    double top = ptr_globalplanner_->top_;

    vector<Way>::iterator way_itor = ptr_globalplanner_->my_way_vec_.begin();
    while (way_itor != ptr_globalplanner_->my_way_vec_.end()) {
        if (way_itor->road_type != 3) {
            int num = way_itor->referLine_offset.size();
            uchar color_r, color_g, color_b;
            if (way_itor->id < 9999)//正向道路
            {
                color_r = 128;
                color_g = 0;
                color_b = 128;
            } else {
                color_r = 0;
                color_g = 0;
                color_b = 255;
            }

            CvPoint pre_pt;
            pre_pt.x = edge_width_ / 2 + (int) ((way_itor->referLine_offset[0].lng - left) * scale_);
            pre_pt.y = edge_width_ / 2 + (int) ((way_itor->referLine_offset[0].lat - top) * scale_);
            pre_pt.y = image_.rows - 1 - pre_pt.y;
            circle(image_, pre_pt, 1, Scalar(255, 255, 255), -1);
            CvPoint pt;
            pt.x = edge_width_ / 2 + (int) ((way_itor->referLine_offset[num - 1].lng - left) * scale_);
            pt.y = edge_width_ / 2 + (int) ((way_itor->referLine_offset[num - 1].lat - top) * scale_);
            pt.y = image_.rows - 1 - pt.y;
            circle(image_, pt, 1, Scalar(255, 255, 255), -1);
            line(image_, pre_pt, pt, Scalar(color_r, color_g, color_b));
        }


        way_itor++;
    }
    image_show_= image_.clone();//复制

}
/**
 * @brief 画出规划好的路线
 */
void MyGraphicsView::DrawChoiceWay()
{
    double left = ptr_globalplanner_->left_;
    double top = ptr_globalplanner_->top_;
    int tmp_index;
//    tmp_index = ptr_globalplanner_->glo_path_index_[0];
//    AstarNode first_node = ptr_globalplanner_->map_data_[tmp_index];
//    CvPoint point1;
//    point1.x = edge_width_ / 2 + (int) ((first_node.lng - left) * scale_);
//    point1.y = edge_width_ / 2 + (int) ((first_node.lat - top) * scale_);
//    point1.y = image_show_.rows - 1 - point1.y;
//    circle(image_show_, point1, 5, Scalar(255,225 ,255));//白色
//    putText(image_show_, "1", point1, 2, 0.7, Scalar(0, 255, 255));//颜色设置为黄色，字大小为0.7
    AstarNode tmp_node;
    for(int i = 1; i < ptr_globalplanner_->glo_path_index_.size(); ++i)
    {
        CvPoint prept;
        tmp_index = ptr_globalplanner_->glo_path_index_[i-1];
        tmp_node = ptr_globalplanner_->map_data_[tmp_index];
        prept.x = edge_width_ / 2 + (int)((tmp_node.lng - left) * scale_);
        prept.y = edge_width_ / 2 + (int)((tmp_node.lat - top) * scale_);
        prept.y = image_show_.rows - 1 - prept.y;
        CvPoint pt;
        tmp_index = ptr_globalplanner_->glo_path_index_[i];
        tmp_node = ptr_globalplanner_->map_data_[tmp_index];
        pt.x = edge_width_ / 2 + (int)((tmp_node.lng - left) * scale_);
        pt.y = edge_width_ / 2 + (int)((tmp_node.lat - top) * scale_);
        pt.y = image_show_.rows - 1 - pt.y;
        line(image_show_, prept, pt, Scalar(0, 255, 255), 2);//绘制的道路颜色设置为黄色
    }
    ShowImage(image_show_);
}

void MyGraphicsView::wheelEvent(QWheelEvent *event)
{
    int wheelDeltaValue = event->delta();
    // 向上滚动，放大;
    if (wheelDeltaValue > 0)
    {
        this->scale(1.2, 1.2);
    }else// 向下滚动，缩小;
    {
        this->scale(1.0 / 1.2, 1.0 / 1.2);
    }

}

void MyGraphicsView::mouseMoveEvent(QMouseEvent *event)
{
    if (mouse_translate_flag_){
        QPointF mouseDelta = mapToScene(event->pos()) - mapToScene(last_mouse_pos_);
        // view 根据鼠标下的点作为锚点来定位 scene
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        QPoint newCenter(VIEW_WIDTH / 2 - mouseDelta.x(),  VIEW_HEIGHT / 2 - mouseDelta.y());
        centerOn(mapToScene(newCenter));
        // scene 在 view 的中心点作为锚点
        setTransformationAnchor(QGraphicsView::AnchorViewCenter);
    }

    last_mouse_pos_ = event->pos();
}

void MyGraphicsView::mouseReleaseEvent(QMouseEvent *event)
{
    mouse_translate_flag_ = false;
}
