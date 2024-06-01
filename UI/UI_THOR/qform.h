
#ifndef QFORM_H
#define QFORM_H

#include <QMainWindow>
#include <debugger.h>
#include <maps.h>
#include "settingsdialog.h"

#include <QMessageBox>
#include <QtSerialPort/QSerialPort>
#include <QtNetwork/QUdpSocket>
#include <QLabel>
#include <QInputDialog>
#include <QTimer>

#include <QNetworkDatagram>
#include <QFile>
#include <QTextStream>
#include <QTime>
#include <QDate>
#include <QDir>
#include <QtCharts>
#include <QSplineSeries>
#include <QChartView>
#include <QGridLayout>
#include <QList>
#include <QDialog>
#include <QElapsedTimer>
#include <QPalette>
#include <QColor>


QT_BEGIN_NAMESPACE
namespace Ui { class QForm; }
QT_END_NAMESPACE

class QForm : public QMainWindow

{
    Q_OBJECT

public:
    QForm(QWidget *parent = nullptr);
    ~QForm();

private slots:
    void on_pushButtonOPENUDP_clicked();

    void openSerialPorts();

    void closeSerialPorts();

    void dataRecived();

    void decodeData(uint8_t *datosRx, uint8_t source);

    void sendDataSerial();

    void timeOut();

    void sendDataUDP();

    void OnUdpRxData();

    void on_pushButtonALIVE_clicked();

    //Muestreo de datos ADC
    void createChartADC();
    void addPointChartADC0(uint16_t point);
    void addPointChartADC1(uint16_t point);
    void addPointChartADC2(uint16_t point);

    //Muestreo de datos Velocidad Motores
    void createChartVelocidadMotores();
    void addPointChartVelMotor0(int32_t point);
    void addPointChartVelMotor1(int32_t point);

    void on_radioButtonUpdateRealTime_clicked();

    void on_pushButtonStartStop_clicked();

    void on_pushButtonReset_clicked();

private:
    Ui::QForm *ui;
    QSerialPort *serial;

    debugger *viewDebugger;
    Maps *viewMaps;
    SettingsDialog *settingPorts;

    QLabel *estadoSerial;
    QTimer  *timer1;

    QUdpSocket *UdpSocket1;
    QHostAddress RemoteAddress;
    quint16 RemotePort;
    QHostAddress clientAddress;
    int puertoremoto;

    typedef enum{
        START,
        HEADER_1,
        HEADER_2,
        HEADER_3,
        NBYTES,
        TOKEN,
        PAYLOAD
    }_eProtocolo;

    _eProtocolo estadoProtocolo,estadoProtocoloUdp;

    bool ARRANQUE = true;

    typedef enum{
        UDP=0,
        SERIE=1,
        ACK=0x0D,
        SETLEDS=0x10,
        GETALIVE=0xF0,
        GETFIRMWARE=0xF1,
        UNKNOWCMD=0xFF,
        GETSWITCHES=0x12,
        GETANALOGSENSORS=0xA0,
        SETMOTORTEST=0xA1,
        SETSERVOANGLE=0xA2,
        SERVOMOVESTOP=0x0A,
        GETDISTANCE=0xA3,
        GETSPEED=0xA4,
        SETSERVOLIMITS=0xA5,
        SETBLACKCOLORDETECTED=0xA6,
        SETWHITECOLORDETECTED=0xA7,
        STATUSCAMINOS = 0xA8,
        STATUSROBOT = 0x05,
        NROCAMINO=0x11,
        STARTSTOP = 0xA9,
        UPDATEREALTIME = 0x13,
        RESETMODE = 0x04,
        OTHERS
    }_eCmd;

    typedef enum{
        GIROINICIAL = 0,
        BUSCARCIRCULO = 1,
        BUSCARPARED = 2,
        BUSCARPUERTA = 3,
        SALIRDELCIRCULO = 4,
        BUSCARLINEA_INTOEX = 5,
        LINEAENCONTRADA = 6,
        CONTARBLANCOS = 7,
        CAMINOCOMPLETADO = 8,
        BUSCANDOINTERSECCION = 9,
        INTERSECCIONPARED_INTOEX = 10,
        INTERSECCIONPARED_EXTOIN = 11,
        BUSCARLINEA_EXTOIN = 12,
        FESTEJOFINAL = 13
    }_eModoPromocion;

    typedef struct{
        uint8_t timeOut;
        uint8_t cheksum;
        uint8_t payLoad[256];
        uint8_t nBytes;
        uint8_t index;
    }_sDatos ;

    _sDatos rxData, rxDataUdp;

    typedef union {
        double  d32;
        float f32;
        int i32;
        unsigned int ui32;
        unsigned short ui16[2];
        short i16[2];
        uint8_t ui8[4];
        char chr[4];
        unsigned char uchr[4];
        int8_t  i8[4];
    }_udat;

    _udat myWord;

    int contadorAlive=0;

    uint8_t nroCaminos;


    // Grafico del ADC
    QSplineSeries *adc0Spline;
    QSplineSeries *adc1Spline;
    QSplineSeries *adc2Spline;

    QChart *adcChart;
    QChartView *adcChartView;
    QGridLayout *adcLayout;

    QList<QPointF> adc0Datos;
    QList<QPointF> adc1Datos;
    QList<QPointF> adc2Datos;

    // Grafico del Velocidad Motores
    QSplineSeries *velMotor0Spline;
    QSplineSeries *velMotor1Spline;

    QChart *velMotorChart;
    QChartView *velMotorChartView;
    QGridLayout *velMotorLayout;

    QList<QPointF> velMotor0Datos;
    QList<QPointF> velMotor1Datos;

};

#endif // QFORM_H
