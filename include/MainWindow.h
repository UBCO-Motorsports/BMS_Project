#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btnInit_clicked();
    void readSerialData(); // New slot to read USB data
    void on_btnToggleCharge_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort *serial;
    QString serialBuffer;
    bool m_manualChargeRequest = false;
};

#endif // MAINWINDOW_H