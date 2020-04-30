/********************************************************************************
** Form generated from reading UI file 'global_planner_dlg.ui'
**
** Created by: Qt User Interface Compiler version 5.9.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GLOBAL_PLANNER_DLG_H
#define UI_GLOBAL_PLANNER_DLG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextBrowser>
#include <my_graphics_view.h>

QT_BEGIN_NAMESPACE

class Ui_GlobalPlannerDlg
{
public:
    QGroupBox *groupBox;
    MyGraphicsView *graphicsView;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout;
    QPushButton *START_POS_BTN;
    QPushButton *END_POS_BTN;
    QPushButton *START_BTN;
    QPushButton *RESET_BTN;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_2;
    QLabel *STATUS_LABLE;
    QTextBrowser *STATUS_BOX;
    QPushButton *START_DECISION_BTN;

    void setupUi(QDialog *GlobalPlannerDlg)
    {
        if (GlobalPlannerDlg->objectName().isEmpty())
            GlobalPlannerDlg->setObjectName(QStringLiteral("GlobalPlannerDlg"));
        GlobalPlannerDlg->resize(1056, 661);
        QFont font;
        font.setFamily(QStringLiteral("Sans Serif"));
        GlobalPlannerDlg->setFont(font);
        groupBox = new QGroupBox(GlobalPlannerDlg);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 10, 851, 641));
        QFont font1;
        font1.setFamily(QStringLiteral("Sans Serif"));
        font1.setPointSize(12);
        groupBox->setFont(font1);
        groupBox->setLayoutDirection(Qt::LeftToRight);
        groupBox->setStyleSheet(QLatin1String("QGroupBox{\n"
"border-width:2px;\n"
"border-style:solid;\n"
"border-radius: 5px;\n"
"border-color:gray;\n"
"margin-top:0.1ex;\n"
"}\n"
"QGroupBox::title{\n"
"subcontrol-origin:margin;\n"
"subcontrol-position:top left;\n"
"left:10px;\n"
"margin-left:0px;\n"
"padding:0 1px;\n"
"}"));
        groupBox->setAlignment(Qt::AlignCenter);
        groupBox->setFlat(false);
        graphicsView = new MyGraphicsView(groupBox);
        graphicsView->setObjectName(QStringLiteral("graphicsView"));
        graphicsView->setGeometry(QRect(10, 30, 831, 591));
        groupBox_2 = new QGroupBox(GlobalPlannerDlg);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(890, 10, 131, 186));
        QFont font2;
        font2.setFamily(QStringLiteral("Sans Serif"));
        font2.setPointSize(10);
        groupBox_2->setFont(font2);
        groupBox_2->setAlignment(Qt::AlignBottom|Qt::AlignHCenter);
        gridLayout = new QGridLayout(groupBox_2);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        START_POS_BTN = new QPushButton(groupBox_2);
        START_POS_BTN->setObjectName(QStringLiteral("START_POS_BTN"));

        gridLayout->addWidget(START_POS_BTN, 0, 0, 1, 1);

        END_POS_BTN = new QPushButton(groupBox_2);
        END_POS_BTN->setObjectName(QStringLiteral("END_POS_BTN"));

        gridLayout->addWidget(END_POS_BTN, 1, 0, 1, 1);

        START_BTN = new QPushButton(groupBox_2);
        START_BTN->setObjectName(QStringLiteral("START_BTN"));

        gridLayout->addWidget(START_BTN, 2, 0, 1, 1);

        RESET_BTN = new QPushButton(groupBox_2);
        RESET_BTN->setObjectName(QStringLiteral("RESET_BTN"));

        gridLayout->addWidget(RESET_BTN, 3, 0, 1, 1);

        groupBox_3 = new QGroupBox(GlobalPlannerDlg);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(870, 210, 171, 181));
        groupBox_3->setFont(font2);
        groupBox_3->setAlignment(Qt::AlignBottom|Qt::AlignHCenter);
        gridLayout_2 = new QGridLayout(groupBox_3);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        STATUS_LABLE = new QLabel(groupBox_3);
        STATUS_LABLE->setObjectName(QStringLiteral("STATUS_LABLE"));
        STATUS_LABLE->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(STATUS_LABLE, 0, 0, 1, 1);

        STATUS_BOX = new QTextBrowser(groupBox_3);
        STATUS_BOX->setObjectName(QStringLiteral("STATUS_BOX"));

        gridLayout_2->addWidget(STATUS_BOX, 1, 0, 1, 1);

        START_DECISION_BTN = new QPushButton(GlobalPlannerDlg);
        START_DECISION_BTN->setObjectName(QStringLiteral("START_DECISION_BTN"));
        START_DECISION_BTN->setGeometry(QRect(900, 480, 91, 26));

        retranslateUi(GlobalPlannerDlg);

        QMetaObject::connectSlotsByName(GlobalPlannerDlg);
    } // setupUi

    void retranslateUi(QDialog *GlobalPlannerDlg)
    {
        GlobalPlannerDlg->setWindowTitle(QApplication::translate("GlobalPlannerDlg", "GlobalPlannerDlg", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("GlobalPlannerDlg", "\345\205\250\345\261\200\350\247\204\345\210\222", Q_NULLPTR));
        groupBox_2->setTitle(QString());
        START_POS_BTN->setText(QApplication::translate("GlobalPlannerDlg", "\351\200\211\346\213\251\350\265\267\347\202\271", Q_NULLPTR));
        END_POS_BTN->setText(QApplication::translate("GlobalPlannerDlg", "\351\200\211\346\213\251\347\273\210\347\202\271", Q_NULLPTR));
        START_BTN->setText(QApplication::translate("GlobalPlannerDlg", "\345\274\200\345\247\213", Q_NULLPTR));
        RESET_BTN->setText(QApplication::translate("GlobalPlannerDlg", "\351\207\215\347\275\256", Q_NULLPTR));
        groupBox_3->setTitle(QString());
        STATUS_LABLE->setText(QApplication::translate("GlobalPlannerDlg", "\347\212\266\346\200\201\346\230\276\347\244\272", Q_NULLPTR));
        START_DECISION_BTN->setText(QApplication::translate("GlobalPlannerDlg", "\345\220\257\345\212\250\345\206\263\347\255\226\347\250\213\345\272\217", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class GlobalPlannerDlg: public Ui_GlobalPlannerDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GLOBAL_PLANNER_DLG_H
