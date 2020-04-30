#include "global_planner_dlg.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    GlobalPlannerDlg w;
    w.show();

    return a.exec();
}

