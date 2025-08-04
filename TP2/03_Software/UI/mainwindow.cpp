#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSerialPortInfo>
#include <QDebug>

// Compute CRC16 (polynomial 0xA001, initial 0xFFFF) for STM32 frames
static quint16 computeCRC16(const QByteArray &data) {
    quint16 crc = 0xFFFF;
    for (quint8 b : data) {
        crc ^= static_cast<quint16>(b);
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serial(new QSerialPort(this))
    , pollTimer(new QTimer(this))
    , mode(CommMode::RS232)
{
    ui->setupUi(this);

    // Rango 0–1024 en sliders
    ui->sldPWM1->setRange(0, 1024);
    ui->sldPWM2->setRange(0, 1024);

    // Puertos y baudrates
    for (const QSerialPortInfo &p : QSerialPortInfo::availablePorts())
        ui->cbPort->addItem(p.portName());
    ui->cbBaudRate->addItems({"9600","19200","38400","57600","115200"});

    // Conexiones
    connect(ui->cbMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::on_cbMode_currentIndexChanged);
    connect(ui->cbBaudRate, &QComboBox::currentTextChanged,
            this, &MainWindow::onBaudRateChanged);
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::readSerialData);
    connect(serial, &QSerialPort::errorOccurred, this, &MainWindow::handleError);
    connect(pollTimer, &QTimer::timeout, this, &MainWindow::onPollTimeout);

    on_cbMode_currentIndexChanged(0);
}

MainWindow::~MainWindow() {
    if (serial->isOpen())
        serial->close();
    delete ui;
}

void MainWindow::on_cbMode_currentIndexChanged(int idx) {
    mode = (idx == 0 ? CommMode::RS232 : CommMode::RS485);
    bool ctrl = (mode == CommMode::RS485);
    ui->chkOut1->setEnabled(ctrl);
    ui->chkOut2->setEnabled(ctrl);
    ui->chkOut3->setEnabled(ctrl);
    ui->sldPWM1->setEnabled(ctrl);
    ui->sldPWM2->setEnabled(ctrl);
}

void MainWindow::onBaudRateChanged(const QString &baud) {
    int b = baud.toInt();
    if (serial->isOpen()) {
        if (!serial->setBaudRate(b))
            ui->statusbar->showMessage("Error cambiando baudrate", 3000);
        else
            ui->statusbar->showMessage("Baudrate=" + baud, 2000);
    }
}

void MainWindow::on_btnConnect_clicked() {
    if (serial->isOpen()) {
        pollTimer->stop();
        serial->close();
        ui->btnConnect->setText("Conectar");
        ui->statusbar->showMessage("Puerto cerrado", 2000);
    } else {
        serial->setPortName(ui->cbPort->currentText());
        serial->setDataBits(QSerialPort::Data8);
        serial->setParity(QSerialPort::NoParity);
        serial->setStopBits(QSerialPort::OneStop);
        serial->setFlowControl(QSerialPort::NoFlowControl);
        serial->setBaudRate(ui->cbBaudRate->currentText().toInt());

        if (!serial->open(QIODevice::ReadWrite)) {
            ui->statusbar->showMessage("No se pudo abrir puerto", 3000);
            return;
        }
        serial->setDataTerminalReady(true);
        ui->btnConnect->setText("Desconectar");
        ui->statusbar->showMessage("Conectado a " + serial->portName(), 2000);

        if (mode == CommMode::RS485) {
            sendControlFrame();
            pollTimer->start(1000);
        }
    }
}

void MainWindow::handleError(QSerialPort::SerialPortError err) {
    if (err == QSerialPort::ResourceError) {
        ui->statusbar->showMessage("¡Error crítico! " + serial->errorString(), 5000);
        serial->close();
        pollTimer->stop();
        ui->btnConnect->setText("Conectar");
    } else {
        ui->statusbar->showMessage(serial->errorString(), 3000);
    }
}

void MainWindow::readSerialData() {
    recvBuffer.append(serial->readAll());
    // La Blue Pill envía tramas de 13 bytes: 1 header + 11 datos + 2 CRC16
    while (recvBuffer.size() >= 13) {
        if (static_cast<quint8>(recvBuffer[0]) != 0x02) {
            recvBuffer.remove(0, 1);
            continue;
        }
        QByteArray frame = recvBuffer.left(13);
        quint16 crc_recv = static_cast<quint8>(frame[11])
                         | (static_cast<quint8>(frame[12]) << 8);
        if (computeCRC16(frame.left(11)) == crc_recv) {
            processFrame(frame);
            recvBuffer.remove(0, 13);
        } else {
            recvBuffer.remove(0, 1);
            ui->statusbar->showMessage("CRC Error", 2000);
        }
    }
}

void MainWindow::processFrame(const QByteArray &f) {
    // Extracción según protocolo STM32 (13 bytes)
    quint16 a0 = static_cast<quint8>(f[1]) | (static_cast<quint8>(f[2]) << 8);
    quint16 a1 = static_cast<quint8>(f[3]) | (static_cast<quint8>(f[4]) << 8);
    quint16 a2 = static_cast<quint8>(f[5]) | (static_cast<quint8>(f[6]) << 8);
    quint8  inp = static_cast<quint8>(f[7]);
    qint8   t   = static_cast<qint8>(f[8]);
    quint16 p   = (static_cast<quint8>(f[9]) << 8) | static_cast<quint8>(f[10]);

    ui->lcdADC0->display(a0);
    ui->lcdADC1->display(a1);
    ui->lcdADC2->display(a2);
    ui->lcdTemp->display(t);
    ui->lcdPress->display(p);

    auto setLed = [&](QLabel *L, bool on) {
        L->setStyleSheet(on ? "background-color:green;" : "background-color:red;");
    };
    setLed(ui->ledInput1, inp & 0x01);
    setLed(ui->ledInput2, inp & 0x02);
    setLed(ui->ledInput3, inp & 0x04);

    // Mostrar raw hex
    QString hex;
    for (auto b : f)
        hex += QString("%1 ").arg(static_cast<quint8>(b), 2, 16, QLatin1Char('0'));
    ui->txtRawData->append(hex.trimmed().toUpper());
}

void MainWindow::sendControlFrame() {
    if (mode != CommMode::RS485 || !serial->isOpen())
        return;

    quint16 pwm1 = ui->sldPWM1->value();
    quint16 pwm2 = ui->sldPWM2->value();
    quint8 mask = (ui->chkOut1->isChecked() ? 0x01 : 0x00)
                | (ui->chkOut2->isChecked() ? 0x02 : 0x00)
                | (ui->chkOut3->isChecked() ? 0x04 : 0x00);

    QByteArray f;
    f.append(char(0x02));               // Header
    f.append(char(mask));               // Control bits
    f.append(char(pwm1 & 0xFF));        // PWM1 LSB
    f.append(char((pwm1 >> 8) & 0xFF)); // PWM1 MSB
    f.append(char(pwm2 & 0xFF));        // PWM2 LSB
    f.append(char((pwm2 >> 8) & 0xFF)); // PWM2 MSB

    // Append CRC16 (LSB first, MSB next) → trama total 8 bytes
    quint16 crc = computeCRC16(f);
    f.append(char(crc & 0xFF));
    f.append(char((crc >> 8) & 0xFF));

    serial->write(f);
}

// Slots conectados a UI
void MainWindow::onPollTimeout()               { sendControlFrame(); }
void MainWindow::on_chkOut1_toggled(bool)      { sendControlFrame(); }
void MainWindow::on_chkOut2_toggled(bool)      { sendControlFrame(); }
void MainWindow::on_chkOut3_toggled(bool)      { sendControlFrame(); }
void MainWindow::on_sldPWM1_valueChanged(int v){ ui->lblValPWM1->setText(QString::number(v)); sendControlFrame(); }
void MainWindow::on_sldPWM2_valueChanged(int v){ ui->lblValPWM2->setText(QString::number(v)); sendControlFrame(); }
