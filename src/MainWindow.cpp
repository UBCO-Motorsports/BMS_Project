#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QSerialPortInfo>
#include <QRegularExpression>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    serial(new QSerialPort(this))
{
    ui->setupUi(this);

    // Connect the UI buttons
    connect(ui->btnInit, &QPushButton::clicked, this, &MainWindow::on_btnInit_clicked);
    connect(ui->btnToggleCharge, &QPushButton::clicked, this, &MainWindow::on_btnToggleCharge_clicked);

    // Hardcoded COM Port - Ensure this matches your Teensy
    serial->setPortName("COM10");
    serial->setBaudRate(115200);

    if (serial->open(QIODevice::ReadWrite)) {
        ui->txtLog->append("Successfully connected to Teensy on COM10");
        connect(serial, &QSerialPort::readyRead, this, &MainWindow::readSerialData);
    } else {
        ui->txtLog->append("ERROR: Could not open COM10.");
    }
}

MainWindow::~MainWindow() {
    if (serial->isOpen()) {
        serial->close();
    }
    delete ui;
}

void MainWindow::on_btnInit_clicked() {
    if (serial->isOpen()) {
        ui->txtLog->append("Note: Press the physical RESET button on the Teensy to re-init.");
    }
}

void MainWindow::on_btnToggleCharge_clicked() {
    if (!serial->isOpen()) return;

    m_manualChargeRequest = !m_manualChargeRequest;

    if (m_manualChargeRequest) {
        serial->write("CMD:CHARGE_ON\n");
        ui->btnToggleCharge->setText("Stop Charging");
        ui->btnToggleCharge->setStyleSheet("background-color: red; color: white;");
        ui->txtLog->append("GUI: Sending START CHARGE command");
    } else {
        serial->write("CMD:CHARGE_OFF\n");
        ui->btnToggleCharge->setText("Start Charging");
        ui->btnToggleCharge->setStyleSheet("");
        ui->txtLog->append("GUI: Sending STOP CHARGE command");
    }
}

void MainWindow::readSerialData() {
    // Read all available data from the serial port and append it to our buffer 
    serialBuffer += serial->readAll();
    
    // Process the buffer line by line 
    while (serialBuffer.contains('\n')) {
        int newlineIndex = serialBuffer.indexOf('\n');
        QString line = serialBuffer.left(newlineIndex).trimmed();
        serialBuffer.remove(0, newlineIndex + 1);
        
        // Output the raw line to the GUI log for real-time debugging 
        ui->txtLog->append(line); 

        // --- COMBINED STATE OF CHARGE PARSING ---
        // This handles both the main dashboard and the driver view simultaneously 
        if (line.startsWith("State of Charge")) {
            QRegularExpression re(":\\s*(\\d+)");
            QRegularExpressionMatch match = re.match(line);
            if (match.hasMatch()) {
                QString socString = match.captured(1);
                ui->lblSOC->setText(socString + " %");
                ui->lblDV_SOC->setText(socString + " %");
                
                // Set color-coding for the Driver View SOC 
                int soc = socString.toInt();
                if (soc <= 20) ui->lblDV_SOC->setStyleSheet("color: red; font-weight: bold;");
                else if (soc <= 50) ui->lblDV_SOC->setStyleSheet("color: orange; font-weight: bold;");
                else ui->lblDV_SOC->setStyleSheet("color: #2E8B57; font-weight: bold;"); // Sea Green
            }
        }

        // --- ELCON CHARGER DATA PARSING (CAN-3865) ---
        
        // Output Voltage to Battery (BYTE1-2 of Message 2) 
        else if (line.startsWith("Charger Voltage")) {
            QRegularExpression re(":\\s*(\\d+)");
            if (auto m = re.match(line); m.hasMatch()) {
                // Scaling: 0.1V/bit. Assumes Teensy prints in mV for GUI consistency [cite: 10, 13]
                double volts = m.captured(1).toDouble() / 1000.0;
                ui->lblCHG_Voltage->setText(QString::number(volts, 'f', 2) + " V");
            }
        }
        
        // Output Current to Battery (BYTE3-4 of Message 2) 
        else if (line.startsWith("Charger Current")) {
            QRegularExpression re(":\\s*(\\d+)");
            if (auto m = re.match(line); m.hasMatch()) {
                // Scaling: 0.1A/bit. Assumes Teensy prints in mA for GUI consistency [cite: 10, 13]
                double amps = m.captured(1).toDouble() / 1000.0;
                ui->lblCHG_Current->setText(QString::number(amps, 'f', 2) + " A");
            }
        }
        
        // Charger Internal Temperature (BYTE6 of Message 2) 
        else if (line.startsWith("Charger Temp")) {
            QRegularExpression re(":\\s*([\\d\\-]+)");
            if (auto m = re.match(line); m.hasMatch()) {
                ui->lblCHG_Temp->setText(m.captured(1) + " °C");
            }
        }
        
        // AC Input Voltage (BYTE7 of Message 2) 
        else if (line.startsWith("Charger Input V")) {
            QRegularExpression re(":\\s*(\\d+)");
            if (auto m = re.match(line); m.hasMatch()) {
                ui->lblCHG_InputV->setText(m.captured(1) + " VAC");
            }
        }
        
        // Charger Status Flags (BYTE5 of Message 2) 
        else if (line.startsWith("Charger Faults")) {
            QString status = line.section(':', 1).trimmed();
            ui->lblCHG_Faults->setText(status);
            // Red alert if anything other than "OK" is received 
            if (status != "OK" && !status.isEmpty()) {
                ui->lblCHG_Faults->setStyleSheet("color: red; font-weight: bold;");
            } else {
                ui->lblCHG_Faults->setStyleSheet("color: green; font-weight: bold;");
            }
        }

        // --- STANDARD BMS DATA PARSING ---

        // Total Pack Voltage 
        else if (line.startsWith("Total Pack Voltage")) {
            QRegularExpression re(":\\s*(\\d+)\\s*mV");
            QRegularExpressionMatch match = re.match(line);
            if (match.hasMatch()) {
                QString mV = match.captured(1);
                ui->lblPackVoltage->setText(mV + " mV");
                // Update Driver View in Volts 
                double volts = mV.toDouble() / 1000.0;
                ui->lblDV_PackVoltage->setText(QString::number(volts, 'f', 1) + " V");
            }
        }
        
        // Cell Temperature 
        else if (line.startsWith("Cell Temp")) {
            QRegularExpression re("max\\s*([\\d\\.\\-]+)");
            QRegularExpressionMatch match = re.match(line);
            if (match.hasMatch()) {
                ui->lblMaxTemp->setText(match.captured(1) + " °C");
            }
        }
        
        // System-Wide Fault Status 
        else if (line.startsWith("Overall Fault")) {
            if (line.contains("YES")) {
                ui->lblFaultStatus->setText("FAULT ACTIVE");
                ui->lblFaultStatus->setStyleSheet("color: red; font-weight: bold;");
            } else {
                ui->lblFaultStatus->setText("OK");
                ui->lblFaultStatus->setStyleSheet("color: green; font-weight: bold;");
            }
        }
        
        // Balancing Status 
        else if (line.startsWith("Balancing active")) {
            bool active = line.contains("YES");
            ui->lblBalancing->setText(active ? "ACTIVE" : "INACTIVE");
            ui->lblBalancing->setStyleSheet(active ? "color: blue; font-weight: bold;" : "");
        }

        // Max Cell Voltage (Detailed View) 
        else if (line.startsWith("Max Cell Volt")) {
            QRegularExpression re(":\\s*(\\d+)\\s*mV");
            QRegularExpressionMatch match = re.match(line);
            if (match.hasMatch()) {
                ui->lblDV_MaxCellVolt->setText(match.captured(1) + " mV");
            }
        }

        // Pack Current (Detailed View with style logic) 
        else if (line.startsWith("Pack Current")) {
            QRegularExpression re(":\\s*(\\-?\\d+)\\s*mA");
            QRegularExpressionMatch match = re.match(line);
            if (match.hasMatch()) {
                double amps = match.captured(1).toDouble() / 1000.0;
                ui->lblDV_PackCurrent->setText(QString::number(amps, 'f', 2) + " A");
                
                // Color logic: Blue for charging (negative), Red for high discharge 
                if (amps < 0) ui->lblDV_PackCurrent->setStyleSheet("color: blue; font-weight: bold;");
                else if (amps > 50) ui->lblDV_PackCurrent->setStyleSheet("color: red; font-weight: bold;");
                else ui->lblDV_PackCurrent->setStyleSheet("color: black;");
            }
        }
    }
}