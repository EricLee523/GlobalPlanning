//
// Created by lzy on 19-9-20.
//
#include "global_planner_dlg.h"
#include "ui_global_planner_dlg.h"

GlobalPlannerDlg::GlobalPlannerDlg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GlobalPlannerDlg)
{
    ui->setupUi(this);
    InitButtons();

    ptr_my_graphic_view_ = ui->graphicsView;
}
/**
 * @brief 初始化按键的信号和槽、以及按键是否可用
 */
void GlobalPlannerDlg::InitButtons()
{
    QObject::connect(ui->START_POS_BTN,SIGNAL(clicked()),this,SLOT(start_pos_btn_click()));
    QObject::connect(ui->END_POS_BTN,SIGNAL(clicked()),this,SLOT(end_pos_btn_click()));
    QObject::connect(ui->START_BTN,SIGNAL(clicked()),this,SLOT(start_btn_click()));
    QObject::connect(ui->RESET_BTN,SIGNAL(clicked()),this,SLOT(reset_btn_click()));


    ui->START_POS_BTN->setDisabled(false);
    ui->START_BTN->setDisabled(true);
    ui->END_POS_BTN->setDisabled(true);
}

GlobalPlannerDlg::~GlobalPlannerDlg()
{
    delete ui;
    delete ptr_my_graphic_view_;
}
void GlobalPlannerDlg::start_pos_btn_click()
{
    ptr_my_graphic_view_->start_btn_click_state_ = true;
    ui->END_POS_BTN->setDisabled(false);
}
void GlobalPlannerDlg::end_pos_btn_click()
{
    ptr_my_graphic_view_->end_btn_click_state_ = true;

    ui->START_POS_BTN->setDisabled(true);
    ui->START_BTN->setDisabled(false);
}
void GlobalPlannerDlg::start_btn_click()
{
    bool result = ptr_my_graphic_view_->ptr_globalplanner_->searchPath();
    if (result){
        ui->STATUS_BOX->insertPlainText(tr("全局规划成功\n"));
    }else{
        ui->STATUS_BOX->insertPlainText(tr("全局规划失败\n"));
    }
    ptr_my_graphic_view_->ptr_globalplanner_->GetMdf();
    ptr_my_graphic_view_->DrawChoiceWay();
    ptr_my_graphic_view_->ptr_globalplanner_->GetGlobalPath();

    ui->END_POS_BTN->setDisabled(true);

}

void GlobalPlannerDlg::reset_btn_click()
{
    ui->STATUS_BOX->setText(tr(""));//将状态显示栏清空
    ptr_my_graphic_view_->ptr_globalplanner_->Reset();//重置相关变量
    ptr_my_graphic_view_->image_show_ = ptr_my_graphic_view_->image_.clone();//重新初始化图像
    ptr_my_graphic_view_->ShowImage(ptr_my_graphic_view_->image_show_);


    ui->START_POS_BTN->setDisabled(false);
    ui->START_BTN->setDisabled(true);
    ui->END_POS_BTN->setDisabled(true);
}
