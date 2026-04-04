/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.18
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QPushButton *btnInit;
    QPushButton *btnRunStep;
    QTabWidget *tabWidget;
    QWidget *tab_Dashboard;
    QFormLayout *formLayout;
    QLabel *label_1;
    QLabel *lblFSMState;
    QLabel *label;
    QLabel *lblPackVoltage;
    QLabel *label_3;
    QLabel *lblMaxTemp;
    QLabel *label_5;
    QLabel *lblSOC;
    QLabel *label_8;
    QLabel *lblBalancing;
    QLabel *label_7;
    QLabel *lblFaultStatus;
    QWidget *tab_Simulations;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_HW;
    QHBoxLayout *horizontalLayout_3;
    QCheckBox *chkIMDFault;
    QCheckBox *chkNFAULT;
    QCheckBox *chkCommLoss;
    QGroupBox *groupBox_Cells;
    QGridLayout *gridLayout;
    QCheckBox *chkOV;
    QCheckBox *chkUV;
    QCheckBox *chkOT;
    QCheckBox *chkUT;
    QCheckBox *chkImbalance;
    QSpacerItem *verticalSpacer;
    QWidget *tab_DriverView;
    QVBoxLayout *verticalLayout_Driver;
    QPushButton *btnToggleCharge;
    QGroupBox *grpDriverStats;
    QFormLayout *formLayout_Driver;
    QLabel *label_DV_Voltage;
    QLabel *lblDV_PackVoltage;
    QLabel *label_DV_SOC;
    QLabel *lblDV_SOC;
    QLabel *label_DV_MaxCell;
    QLabel *lblDV_MaxCellVolt;
    QLabel *label_DV_Current;
    QLabel *lblDV_PackCurrent;
    QGroupBox *grpChargerStatus;
    QFormLayout *formLayout_Charger;
    QLabel *label_CHG_V;
    QLabel *lblCHG_Voltage;
    QLabel *label_CHG_I;
    QLabel *lblCHG_Current;
    QLabel *label_CHG_T;
    QLabel *lblCHG_Temp;
    QLabel *label_CHG_Input;
    QLabel *lblCHG_InputV;
    QLabel *label_CHG_Fault;
    QLabel *lblCHG_Faults;
    QSpacerItem *verticalSpacer_Driver;
    QTextEdit *txtLog;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(650, 650);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        btnInit = new QPushButton(centralwidget);
        btnInit->setObjectName(QString::fromUtf8("btnInit"));

        horizontalLayout->addWidget(btnInit);

        btnRunStep = new QPushButton(centralwidget);
        btnRunStep->setObjectName(QString::fromUtf8("btnRunStep"));

        horizontalLayout->addWidget(btnRunStep);


        verticalLayout->addLayout(horizontalLayout);

        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab_Dashboard = new QWidget();
        tab_Dashboard->setObjectName(QString::fromUtf8("tab_Dashboard"));
        formLayout = new QFormLayout(tab_Dashboard);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        label_1 = new QLabel(tab_Dashboard);
        label_1->setObjectName(QString::fromUtf8("label_1"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_1);

        lblFSMState = new QLabel(tab_Dashboard);
        lblFSMState->setObjectName(QString::fromUtf8("lblFSMState"));

        formLayout->setWidget(0, QFormLayout::FieldRole, lblFSMState);

        label = new QLabel(tab_Dashboard);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label);

        lblPackVoltage = new QLabel(tab_Dashboard);
        lblPackVoltage->setObjectName(QString::fromUtf8("lblPackVoltage"));

        formLayout->setWidget(1, QFormLayout::FieldRole, lblPackVoltage);

        label_3 = new QLabel(tab_Dashboard);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_3);

        lblMaxTemp = new QLabel(tab_Dashboard);
        lblMaxTemp->setObjectName(QString::fromUtf8("lblMaxTemp"));

        formLayout->setWidget(2, QFormLayout::FieldRole, lblMaxTemp);

        label_5 = new QLabel(tab_Dashboard);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_5);

        lblSOC = new QLabel(tab_Dashboard);
        lblSOC->setObjectName(QString::fromUtf8("lblSOC"));

        formLayout->setWidget(3, QFormLayout::FieldRole, lblSOC);

        label_8 = new QLabel(tab_Dashboard);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        formLayout->setWidget(4, QFormLayout::LabelRole, label_8);

        lblBalancing = new QLabel(tab_Dashboard);
        lblBalancing->setObjectName(QString::fromUtf8("lblBalancing"));

        formLayout->setWidget(4, QFormLayout::FieldRole, lblBalancing);

        label_7 = new QLabel(tab_Dashboard);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        formLayout->setWidget(5, QFormLayout::LabelRole, label_7);

        lblFaultStatus = new QLabel(tab_Dashboard);
        lblFaultStatus->setObjectName(QString::fromUtf8("lblFaultStatus"));

        formLayout->setWidget(5, QFormLayout::FieldRole, lblFaultStatus);

        tabWidget->addTab(tab_Dashboard, QString());
        tab_Simulations = new QWidget();
        tab_Simulations->setObjectName(QString::fromUtf8("tab_Simulations"));
        verticalLayout_2 = new QVBoxLayout(tab_Simulations);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        groupBox_HW = new QGroupBox(tab_Simulations);
        groupBox_HW->setObjectName(QString::fromUtf8("groupBox_HW"));
        horizontalLayout_3 = new QHBoxLayout(groupBox_HW);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        chkIMDFault = new QCheckBox(groupBox_HW);
        chkIMDFault->setObjectName(QString::fromUtf8("chkIMDFault"));

        horizontalLayout_3->addWidget(chkIMDFault);

        chkNFAULT = new QCheckBox(groupBox_HW);
        chkNFAULT->setObjectName(QString::fromUtf8("chkNFAULT"));

        horizontalLayout_3->addWidget(chkNFAULT);

        chkCommLoss = new QCheckBox(groupBox_HW);
        chkCommLoss->setObjectName(QString::fromUtf8("chkCommLoss"));

        horizontalLayout_3->addWidget(chkCommLoss);


        verticalLayout_2->addWidget(groupBox_HW);

        groupBox_Cells = new QGroupBox(tab_Simulations);
        groupBox_Cells->setObjectName(QString::fromUtf8("groupBox_Cells"));
        gridLayout = new QGridLayout(groupBox_Cells);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        chkOV = new QCheckBox(groupBox_Cells);
        chkOV->setObjectName(QString::fromUtf8("chkOV"));

        gridLayout->addWidget(chkOV, 0, 0, 1, 1);

        chkUV = new QCheckBox(groupBox_Cells);
        chkUV->setObjectName(QString::fromUtf8("chkUV"));

        gridLayout->addWidget(chkUV, 0, 1, 1, 1);

        chkOT = new QCheckBox(groupBox_Cells);
        chkOT->setObjectName(QString::fromUtf8("chkOT"));

        gridLayout->addWidget(chkOT, 1, 0, 1, 1);

        chkUT = new QCheckBox(groupBox_Cells);
        chkUT->setObjectName(QString::fromUtf8("chkUT"));

        gridLayout->addWidget(chkUT, 1, 1, 1, 1);

        chkImbalance = new QCheckBox(groupBox_Cells);
        chkImbalance->setObjectName(QString::fromUtf8("chkImbalance"));

        gridLayout->addWidget(chkImbalance, 2, 0, 1, 1);


        verticalLayout_2->addWidget(groupBox_Cells);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        tabWidget->addTab(tab_Simulations, QString());
        tab_DriverView = new QWidget();
        tab_DriverView->setObjectName(QString::fromUtf8("tab_DriverView"));
        verticalLayout_Driver = new QVBoxLayout(tab_DriverView);
        verticalLayout_Driver->setObjectName(QString::fromUtf8("verticalLayout_Driver"));
        btnToggleCharge = new QPushButton(tab_DriverView);
        btnToggleCharge->setObjectName(QString::fromUtf8("btnToggleCharge"));
        btnToggleCharge->setMinimumSize(QSize(0, 40));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        btnToggleCharge->setFont(font);

        verticalLayout_Driver->addWidget(btnToggleCharge);

        grpDriverStats = new QGroupBox(tab_DriverView);
        grpDriverStats->setObjectName(QString::fromUtf8("grpDriverStats"));
        QFont font1;
        font1.setPointSize(10);
        font1.setBold(true);
        font1.setWeight(75);
        grpDriverStats->setFont(font1);
        formLayout_Driver = new QFormLayout(grpDriverStats);
        formLayout_Driver->setObjectName(QString::fromUtf8("formLayout_Driver"));
        label_DV_Voltage = new QLabel(grpDriverStats);
        label_DV_Voltage->setObjectName(QString::fromUtf8("label_DV_Voltage"));

        formLayout_Driver->setWidget(0, QFormLayout::LabelRole, label_DV_Voltage);

        lblDV_PackVoltage = new QLabel(grpDriverStats);
        lblDV_PackVoltage->setObjectName(QString::fromUtf8("lblDV_PackVoltage"));
        QFont font2;
        font2.setPointSize(14);
        lblDV_PackVoltage->setFont(font2);

        formLayout_Driver->setWidget(0, QFormLayout::FieldRole, lblDV_PackVoltage);

        label_DV_SOC = new QLabel(grpDriverStats);
        label_DV_SOC->setObjectName(QString::fromUtf8("label_DV_SOC"));

        formLayout_Driver->setWidget(1, QFormLayout::LabelRole, label_DV_SOC);

        lblDV_SOC = new QLabel(grpDriverStats);
        lblDV_SOC->setObjectName(QString::fromUtf8("lblDV_SOC"));
        QFont font3;
        font3.setPointSize(14);
        font3.setBold(true);
        font3.setWeight(75);
        lblDV_SOC->setFont(font3);

        formLayout_Driver->setWidget(1, QFormLayout::FieldRole, lblDV_SOC);

        label_DV_MaxCell = new QLabel(grpDriverStats);
        label_DV_MaxCell->setObjectName(QString::fromUtf8("label_DV_MaxCell"));

        formLayout_Driver->setWidget(2, QFormLayout::LabelRole, label_DV_MaxCell);

        lblDV_MaxCellVolt = new QLabel(grpDriverStats);
        lblDV_MaxCellVolt->setObjectName(QString::fromUtf8("lblDV_MaxCellVolt"));
        lblDV_MaxCellVolt->setFont(font2);

        formLayout_Driver->setWidget(2, QFormLayout::FieldRole, lblDV_MaxCellVolt);

        label_DV_Current = new QLabel(grpDriverStats);
        label_DV_Current->setObjectName(QString::fromUtf8("label_DV_Current"));

        formLayout_Driver->setWidget(3, QFormLayout::LabelRole, label_DV_Current);

        lblDV_PackCurrent = new QLabel(grpDriverStats);
        lblDV_PackCurrent->setObjectName(QString::fromUtf8("lblDV_PackCurrent"));
        lblDV_PackCurrent->setFont(font2);

        formLayout_Driver->setWidget(3, QFormLayout::FieldRole, lblDV_PackCurrent);


        verticalLayout_Driver->addWidget(grpDriverStats);

        grpChargerStatus = new QGroupBox(tab_DriverView);
        grpChargerStatus->setObjectName(QString::fromUtf8("grpChargerStatus"));
        grpChargerStatus->setFont(font1);
        formLayout_Charger = new QFormLayout(grpChargerStatus);
        formLayout_Charger->setObjectName(QString::fromUtf8("formLayout_Charger"));
        label_CHG_V = new QLabel(grpChargerStatus);
        label_CHG_V->setObjectName(QString::fromUtf8("label_CHG_V"));

        formLayout_Charger->setWidget(0, QFormLayout::LabelRole, label_CHG_V);

        lblCHG_Voltage = new QLabel(grpChargerStatus);
        lblCHG_Voltage->setObjectName(QString::fromUtf8("lblCHG_Voltage"));
        lblCHG_Voltage->setFont(font2);

        formLayout_Charger->setWidget(0, QFormLayout::FieldRole, lblCHG_Voltage);

        label_CHG_I = new QLabel(grpChargerStatus);
        label_CHG_I->setObjectName(QString::fromUtf8("label_CHG_I"));

        formLayout_Charger->setWidget(1, QFormLayout::LabelRole, label_CHG_I);

        lblCHG_Current = new QLabel(grpChargerStatus);
        lblCHG_Current->setObjectName(QString::fromUtf8("lblCHG_Current"));
        lblCHG_Current->setFont(font2);

        formLayout_Charger->setWidget(1, QFormLayout::FieldRole, lblCHG_Current);

        label_CHG_T = new QLabel(grpChargerStatus);
        label_CHG_T->setObjectName(QString::fromUtf8("label_CHG_T"));

        formLayout_Charger->setWidget(2, QFormLayout::LabelRole, label_CHG_T);

        lblCHG_Temp = new QLabel(grpChargerStatus);
        lblCHG_Temp->setObjectName(QString::fromUtf8("lblCHG_Temp"));
        QFont font4;
        font4.setPointSize(12);
        lblCHG_Temp->setFont(font4);

        formLayout_Charger->setWidget(2, QFormLayout::FieldRole, lblCHG_Temp);

        label_CHG_Input = new QLabel(grpChargerStatus);
        label_CHG_Input->setObjectName(QString::fromUtf8("label_CHG_Input"));

        formLayout_Charger->setWidget(3, QFormLayout::LabelRole, label_CHG_Input);

        lblCHG_InputV = new QLabel(grpChargerStatus);
        lblCHG_InputV->setObjectName(QString::fromUtf8("lblCHG_InputV"));

        formLayout_Charger->setWidget(3, QFormLayout::FieldRole, lblCHG_InputV);

        label_CHG_Fault = new QLabel(grpChargerStatus);
        label_CHG_Fault->setObjectName(QString::fromUtf8("label_CHG_Fault"));

        formLayout_Charger->setWidget(4, QFormLayout::LabelRole, label_CHG_Fault);

        lblCHG_Faults = new QLabel(grpChargerStatus);
        lblCHG_Faults->setObjectName(QString::fromUtf8("lblCHG_Faults"));
        lblCHG_Faults->setFont(font1);

        formLayout_Charger->setWidget(4, QFormLayout::FieldRole, lblCHG_Faults);


        verticalLayout_Driver->addWidget(grpChargerStatus);

        verticalSpacer_Driver = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_Driver->addItem(verticalSpacer_Driver);

        tabWidget->addTab(tab_DriverView, QString());

        verticalLayout->addWidget(tabWidget);

        txtLog = new QTextEdit(centralwidget);
        txtLog->setObjectName(QString::fromUtf8("txtLog"));
        txtLog->setReadOnly(true);

        verticalLayout->addWidget(txtLog);

        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "BMS Controller Dashboard", nullptr));
        btnInit->setText(QCoreApplication::translate("MainWindow", "Initialize BMS (Serial)", nullptr));
        btnRunStep->setText(QCoreApplication::translate("MainWindow", "Manual FSM Step", nullptr));
        label_1->setText(QCoreApplication::translate("MainWindow", "FSM State Code:", nullptr));
        lblFSMState->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Est. Pack Voltage:", nullptr));
        lblPackVoltage->setText(QCoreApplication::translate("MainWindow", "0 mV", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Max Cell Temp:", nullptr));
        lblMaxTemp->setText(QCoreApplication::translate("MainWindow", "0.0 \302\260C", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "State of Charge:", nullptr));
        lblSOC->setText(QCoreApplication::translate("MainWindow", "0 %", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "Cell Balancing:", nullptr));
        lblBalancing->setText(QCoreApplication::translate("MainWindow", "INACTIVE", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "System Fault Status:", nullptr));
        lblFaultStatus->setText(QCoreApplication::translate("MainWindow", "UNKNOWN", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_Dashboard), QCoreApplication::translate("MainWindow", "Dashboard", nullptr));
        groupBox_HW->setTitle(QCoreApplication::translate("MainWindow", "Hardware & Communications", nullptr));
        chkIMDFault->setText(QCoreApplication::translate("MainWindow", "IMD Fault (Pin 5 LOW)", nullptr));
        chkNFAULT->setText(QCoreApplication::translate("MainWindow", "NFAULT Trigger (Pin 19 LOW)", nullptr));
        chkCommLoss->setText(QCoreApplication::translate("MainWindow", "Stack Comm Loss", nullptr));
        groupBox_Cells->setTitle(QCoreApplication::translate("MainWindow", "Measurement Faults", nullptr));
        chkOV->setText(QCoreApplication::translate("MainWindow", "Over-Voltage (>4.2V)", nullptr));
        chkUV->setText(QCoreApplication::translate("MainWindow", "Under-Voltage (<3.0V)", nullptr));
        chkOT->setText(QCoreApplication::translate("MainWindow", "Over-Temp (>55\302\260C)", nullptr));
        chkUT->setText(QCoreApplication::translate("MainWindow", "Under-Temp (<-1\302\260C)", nullptr));
        chkImbalance->setText(QCoreApplication::translate("MainWindow", "Cell Imbalance (Delta > 20mV)", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_Simulations), QCoreApplication::translate("MainWindow", "Simulations & Faults", nullptr));
        btnToggleCharge->setText(QCoreApplication::translate("MainWindow", "Start Charging", nullptr));
        grpDriverStats->setTitle(QCoreApplication::translate("MainWindow", "Internal Battery Stats", nullptr));
        label_DV_Voltage->setText(QCoreApplication::translate("MainWindow", "Total Pack Voltage:", nullptr));
        lblDV_PackVoltage->setText(QCoreApplication::translate("MainWindow", "0.0 V", nullptr));
        label_DV_SOC->setText(QCoreApplication::translate("MainWindow", "State of Charge (SOC):", nullptr));
        lblDV_SOC->setStyleSheet(QCoreApplication::translate("MainWindow", "color: #2E8B57;", nullptr));
        lblDV_SOC->setText(QCoreApplication::translate("MainWindow", "0 %", nullptr));
        label_DV_MaxCell->setText(QCoreApplication::translate("MainWindow", "Highest Cell Voltage:", nullptr));
        lblDV_MaxCellVolt->setText(QCoreApplication::translate("MainWindow", "0 mV", nullptr));
        label_DV_Current->setText(QCoreApplication::translate("MainWindow", "Pack Current:", nullptr));
        lblDV_PackCurrent->setText(QCoreApplication::translate("MainWindow", "0.0 A", nullptr));
        grpChargerStatus->setTitle(QCoreApplication::translate("MainWindow", "Elcon Charger Interface (CAN 3865)", nullptr));
        label_CHG_V->setText(QCoreApplication::translate("MainWindow", "Charger Output Voltage:", nullptr));
        lblCHG_Voltage->setText(QCoreApplication::translate("MainWindow", "0.0 V", nullptr));
        label_CHG_I->setText(QCoreApplication::translate("MainWindow", "Charger Output Current:", nullptr));
        lblCHG_Current->setText(QCoreApplication::translate("MainWindow", "0.0 A", nullptr));
        label_CHG_T->setText(QCoreApplication::translate("MainWindow", "Charger Temperature:", nullptr));
        lblCHG_Temp->setText(QCoreApplication::translate("MainWindow", "-- \302\260C", nullptr));
        label_CHG_Input->setText(QCoreApplication::translate("MainWindow", "AC Input Voltage:", nullptr));
        lblCHG_InputV->setText(QCoreApplication::translate("MainWindow", "0 V", nullptr));
        label_CHG_Fault->setText(QCoreApplication::translate("MainWindow", "Charger Fault Status:", nullptr));
        lblCHG_Faults->setText(QCoreApplication::translate("MainWindow", "NO DATA", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_DriverView), QCoreApplication::translate("MainWindow", "Charging", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
