//
// Created by lzy on 19-9-20.
//
#ifndef GLOBAL_PLANNER_DLG_H
#define GLOBAL_PLANNER_DLG_H

#include <QDialog>
#include "my_graphics_view.h"

namespace Ui {
class GlobalPlannerDlg;
}

class GlobalPlannerDlg : public QDialog
{
    Q_OBJECT

public:
    explicit GlobalPlannerDlg(QWidget *parent = 0);
    ~GlobalPlannerDlg();

private:
    /**
     * @brief 初始化按键的信号和槽、以及按键是否可用
     */
    void InitButtons();

private:
    Ui::GlobalPlannerDlg *ui;
    MyGraphicsView *ptr_my_graphic_view_;

private slots:
    void start_pos_btn_click();
    void end_pos_btn_click();
    void start_btn_click();
    void reset_btn_click();
};

#endif // GLOBAL_PLANNER_DLG_H
