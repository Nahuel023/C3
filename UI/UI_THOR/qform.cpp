
#include "qform.h"
#include "ui_qform.h"


QForm::QForm(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::QForm)
{
    ui->setupUi(this);

    serial=new QSerialPort(this);
    settingPorts=new SettingsDialog(this);
    estadoSerial = new QLabel(this);
    estadoSerial->setText("Desconectado ......");
    ui->statusbar->addWidget(estadoSerial);
    ui->actionDisconnect_Device->setEnabled(false);
    timer1=new QTimer(this);
    UdpSocket1 = new QUdpSocket(this);


    viewDebugger = new debugger(this);
    viewMaps = new Maps(this);


    connect(ui->actionScan_Ports, &QAction::triggered, settingPorts,&SettingsDialog::show);
    connect(ui->actionConnect_Device, &QAction::triggered,this,&QForm::openSerialPorts);
    connect(ui->actionDisconnect_Device, &QAction::triggered, this, &QForm::closeSerialPorts);
    connect(ui->pushButtonSENDCMD,&QPushButton::clicked, this, &QForm::sendDataSerial);
    connect(serial,&QSerialPort::readyRead,this,&QForm::dataRecived);
    connect(timer1,&QTimer::timeout,this,&QForm::timeOut);
    connect(UdpSocket1,&QUdpSocket::readyRead,this,&QForm::OnUdpRxData);
    connect(ui->pushButtonSENDUDP,&QPushButton::clicked,this,&QForm::sendDataUDP);

    connect(ui->actionQuit,&QAction::triggered,this,&QForm::close);
    connect(ui->actionDebugger, &QAction::triggered, viewDebugger,&debugger::show);
    connect(ui->actionMaps, &QAction::triggered, viewMaps,&Maps::show);

    ui->comboBox_CMD->addItem("ALIVE", 0xF0);
    ui->comboBox_CMD->addItem("STARTSTOP", 0xA9);
    ui->comboBox_CMD->addItem("STATUSCAMINOS", 0xA8);
    ui->comboBox_CMD->addItem("NROCAMINO", 0x11);
    ui->comboBox_CMD->addItem("FIRMWARE", 0xF1);
    ui->comboBox_CMD->addItem("SENSORES", 0xA0);
    ui->comboBox_CMD->addItem("MOTORES", 0xA1);
    ui->comboBox_CMD->addItem("SERVO", 0xA2);
    ui->comboBox_CMD->addItem("DISTANCIA", 0xA3);
    ui->comboBox_CMD->addItem("VELOCIDAD", 0xA4);
    ui->comboBox_CMD->addItem("SWITCHS", 0x12);
    ui->comboBox_CMD->addItem("LEDS", 0x10);
    ui->comboBox_CMD->addItem("UPDATEREALTIME", 0x13);
    ui->comboBox_CMD->addItem("RESETMODE", 0x04);
    ui->comboBox_CMD->addItem("STATUSROBOT", 0x05);

    estadoProtocolo=START;
    rxData.timeOut=0;
    ui->pushButtonSENDUDP->setEnabled(false);
    ui->pushButtonALIVE->setEnabled(false);
    ui->pushButtonSENDCMD->setEnabled(false);
    timer1->start(100);

    //Graficos
    createChartADC();
    createChartVelocidadMotores();

}

QForm::~QForm()
{
    delete ui;
}

void QForm::openSerialPorts(){
    const SettingsDialog::Settings p = settingPorts->settings();
    serial->setPortName(p.name);
    serial->setBaudRate(p.baudRate);
    serial->setDataBits(p.dataBits);
    serial->setParity(p.parity);
    serial->setStopBits(p.stopBits);
    serial->setFlowControl(p.flowControl);
    serial->open(QSerialPort::ReadWrite);
    if (serial->isOpen()){
        ui->pushButtonALIVE->setEnabled(true);
        ui->pushButtonSENDCMD->setEnabled(true);
        ui->actionDisconnect_Device->setEnabled(true);
        estadoSerial->setStyleSheet("QLabel { color : blue; }");
        estadoSerial->setText(tr("Connected to %1 : %2, %3, %4, %5, %6, %7")
                                  .arg(p.name)
                                  .arg(p.stringBaudRate)
                                  .arg(p.stringDataBits)
                                  .arg(p.stringParity)
                                  .arg(p.stringStopBits)
                                  .arg(p.stringFlowControl)
                                  .arg(p.fabricante)
                              );
    } else {
        QMessageBox::critical(this, tr("Error"), serial->errorString());
    }
}

void QForm::closeSerialPorts(){
    ui->actionDisconnect_Device->setEnabled(false);
    estadoSerial->setText("Desconectado ......");
    if (serial->isOpen())
        serial->close();
}

void QForm::dataRecived(){
    unsigned char *incomingBuffer;
    int count;

    count = serial->bytesAvailable();

    if(count<=0)
        return;

    incomingBuffer = new unsigned char[count];

    serial->read((char *)incomingBuffer,count);

    QString str="";

    for(int i=0; i<=count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg((char)incomingBuffer[i]);
        else
            str = str +"{" + QString("%1").arg(incomingBuffer[i],2,16,QChar('0')) + "}";
    }
    ui->textEdit_RAW->append("MBED-->SERIAL-->PC (" + str + ")");

    //Cada vez que se recibe un dato reinicio el timeOut
    rxData.timeOut=6;

    for(int i=0;i<count; i++){
        switch (estadoProtocolo) {
        case START:
            if (incomingBuffer[i]=='U'){
                estadoProtocolo=HEADER_1;
            }
            break;
        case HEADER_1:
            if (incomingBuffer[i]=='N')
                estadoProtocolo=HEADER_2;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case HEADER_2:
            if (incomingBuffer[i]=='E')
                estadoProtocolo=HEADER_3;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case HEADER_3:
            if (incomingBuffer[i]=='R')
                estadoProtocolo=NBYTES;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case NBYTES:
            rxData.nBytes=incomingBuffer[i];
            estadoProtocolo=TOKEN;
            break;
        case TOKEN:
            if (incomingBuffer[i]==':'){
                estadoProtocolo=PAYLOAD;
                rxData.cheksum='U'^'N'^'E'^'R'^ rxData.nBytes^':';
                rxData.payLoad[0]=rxData.nBytes;
                rxData.index=1;
            }
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case PAYLOAD:
            if (rxData.nBytes>1){
                rxData.payLoad[rxData.index++]=incomingBuffer[i];
                rxData.cheksum^=incomingBuffer[i];
            }
            rxData.nBytes--;
            if(rxData.nBytes==0){
                estadoProtocolo=START;
                if(rxData.cheksum==incomingBuffer[i]){
                    decodeData(&rxData.payLoad[0], SERIE);
                }else{
                    ui->textEdit_RAW->append("Chk Calculado ** " +QString().number(rxData.cheksum,16) + " **" );
                    ui->textEdit_RAW->append("Chk recibido ** " +QString().number(incomingBuffer[i],16) + " **" );

                }
            }
            break;
        default:
            estadoProtocolo=START;
            break;
        }
    }
    delete [] incomingBuffer;

}

void QForm::decodeData(uint8_t *datosRx, uint8_t source){
    int32_t length = sizeof(*datosRx)/sizeof(datosRx[0]);
    QString str, strOut;
    _udat w;
    for(int i = 1; i<length; i++){
        if(isalnum(datosRx[i]))
            str = str + QString("%1").arg(char(datosRx[i]));
        else
            str = str +QString("%1").arg(datosRx[i],2,16,QChar('0'));
    }
    ui->textEdit_RAW->append("*(MBED-S->PC)->decodeData (" + str + ")");

    switch (datosRx[1]) {
    case GETANALOGSENSORS://     ANALOGSENSORS=0xA0,
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "LEFT IR: " + str;
        ui->textEdit_PROCCES->append(strOut);
        ui->label_LIR->setText(str);
        addPointChartADC0(w.ui16[0]);

        w.ui8[0] = datosRx[4];
        w.ui8[1] = datosRx[5];
        str = QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "CENTER IR: " + str;
        ui->textEdit_PROCCES->append(strOut);
        ui->label_CIR->setText(str);
        addPointChartADC1(w.ui16[0]);

        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];
        str =QString("%1").arg(w.ui16[0], 5, 10, QChar('0'));
        strOut = "RIGHT IR: " + str;
        ui->label_RIR->setText(str);
        ui->textEdit_PROCCES->append(strOut);
        addPointChartADC2(w.ui16[0]);

        break;
    case SETMOTORTEST://     MOTORTEST=0xA1,
        if(datosRx[2]==0x0D)
            str= "Test Motores ACK";
        ui->textEdit_PROCCES->append(str);
        break;
    case SETSERVOANGLE://     SERVOANGLE=0xA2,
        if(datosRx[2]==0x0D)
            str= "Servo moviendose. Esperando posición Final!!!";
                else{
                if(datosRx[2]==0x0A)
                    str= "Servo en posición Final!!!";
            }
        ui->textEdit_PROCCES->append(str);
        break;
    case GETDISTANCE://     GETDISTANCE=0xA3,
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];
        str = QString().number(w.ui32/58);
        ui->label_DISTANCE->setText(str+ "cm");
        ui->textEdit_PROCCES->append("DISTANCIA: "+QString().number(w.ui32/58)+ "cm");
        break;
    case GETSPEED://     GETSPEED=0xA4,
        str = "VM1: ";
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];
        strOut = QString("%1").arg(w.i32, 4, 10, QChar('0'));
        ui->label_LENC->setText(strOut);
        str = str + QString("%1").arg(w.i32, 4, 10, QChar('0')) + " - VM2: ";
        addPointChartVelMotor0(w.i32);

        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];
        w.ui8[2] = datosRx[8];
        w.ui8[3] = datosRx[9];
        strOut = QString("%1").arg(w.i32, 4, 10, QChar('0'));
        ui->label_RENC->setText(strOut);
        str = str + QString("%1").arg(w.i32, 4, 10, QChar('0'));
        ui->textEdit_PROCCES->append(str);
        addPointChartVelMotor1(w.i32);

        break;
    case GETSWITCHES: //GETSWITCHES=0xA5
        str = "SW3: ";
        if(datosRx[2] & 0x08)
            str = str + "HIGH";
        else
            str = str + "LOW";
        str = str + " - SW2: ";
        if(datosRx[2] & 0x04)
            str = str + "HIGH";
        else
            str = str + "LOW";
        str = str + " - SW1: ";
        if(datosRx[2] & 0x02)
            str = str + "HIGH";
        else
            str = str + "LOW";
        str = str + " - SW0: ";
        if(datosRx[2] & 0x01)
            str = str + "HIGH";
        else
            str = str + "LOW";
        ui->textEdit_PROCCES->append(str);
        break;

    case GETALIVE://     GETALIVE=0xF0,
        if(datosRx[2]==ACK){
            contadorAlive++;
            if(source)
                str="ALIVE BLUEPILL VIA *SERIE* RECIBIDO!!!";
            else{
                contadorAlive++;
                str="ALIVE BLUEPILL VIA *UDP* RECIBIDO N°: " + QString().number(contadorAlive,10);
            }
        }else{
            str= "ALIVE BLUEPILL VIA *SERIE*  NO ACK!!!";
        }
        ui->textEdit_PROCCES->append(str);
        break;
    case GETFIRMWARE://     GETFIRMWARE=0xF1
        str = "FIRMWARE:";
        for(uint8_t a=0;a<(datosRx[0]-1);a++){
            str += (QChar)datosRx[2+a];
        }
        ui->textEdit_PROCCES->append(str);

        break;
    case STATUSCAMINOS:
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];
        str = QString().number(w.ui32);
        ui->textEdit_PROCCES->append("CAMINO 1: "+QString().number(w.ui32));
        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];
        w.ui8[2] = datosRx[8];
        w.ui8[3] = datosRx[9];
        str = QString().number(w.ui32);
        ui->textEdit_PROCCES->append("CAMINO 2: "+QString().number(w.ui32));
        w.ui8[0] = datosRx[10];
        w.ui8[1] = datosRx[11];
        w.ui8[2] = datosRx[12];
        w.ui8[3] = datosRx[13];
        str = QString().number(w.ui32);
        ui->textEdit_PROCCES->append("CAMINO 3: "+QString().number(w.ui32));
        w.ui8[0] = datosRx[14];
        w.ui8[1] = datosRx[15];
        w.ui8[2] = datosRx[16];
        w.ui8[3] = datosRx[17];
        str = QString().number(w.ui32);
        ui->textEdit_PROCCES->append("CAMINO 4: "+QString().number(w.ui32));
        break;
    case NROCAMINO:
        ui->textEdit_PROCCES->append("CONTADOR DE CAMINO: " + QString().number(datosRx[2]));

        break;
    case STATUSROBOT:
        switch (datosRx[2]) {
        case GIROINICIAL:
            ui->labelStatus->setText("GIRO INICIAL");
            break;
        case BUSCARCIRCULO:
            ui->labelStatus->setText("BUSCAR CIRCULO");
            break;
        case BUSCARPARED:
            ui->labelStatus->setText("BUSCAR PARED");
            break;
        case BUSCARPUERTA:
            ui->labelStatus->setText("BUSCAR PUERTA");
            break;
        case SALIRDELCIRCULO:
            ui->labelStatus->setText("SALIR DEL CIRCULO");
            break;
        case LINEAENCONTRADA:
            ui->labelStatus->setText("LINEA ENCONTRADA");
            break;
        case CONTARBLANCOS:
            ui->labelStatus->setText("CONTAR BLANCOS");
            break;
        case CAMINOCOMPLETADO:
            ui->labelStatus->setText("CAMINO COMPLETADO");
            break;
        case BUSCANDOINTERSECCION:
            ui->labelStatus->setText("BUSCANDO INTERSECCION");
            break;
        case INTERSECCIONPARED_INTOEX:
            ui->labelStatus->setText("INTERSECCION PARED INTOEX");
            break;
        case INTERSECCIONPARED_EXTOIN:
            ui->labelStatus->setText("INTERSECCION PARED EXTOIN");
            break;
        case BUSCARLINEA_EXTOIN:
            ui->labelStatus->setText("BUSCAR LINEA EXTOIN");
            break;
        case FESTEJOFINAL:
            ui->labelStatus->setText("FESTEJO FINAL");
            break;
        default:
            ui->labelStatus->setText("MODO NO SETTING");
            break;
        }
        break;

    default:
        str = str + "Comando DESCONOCIDO!!!!";
        ui->textEdit_PROCCES->append(str);
    }
}

void QForm::sendDataSerial(){
    uint8_t cmdId;
    _udat   w;
    bool ok;

    unsigned char dato[256];
    unsigned char indice=0, chk=0;

    QString str="";

    dato[indice++]='U';
    dato[indice++]='N';
    dato[indice++]='E';
    dato[indice++]='R';
    dato[indice++]=0x00;
    dato[indice++]=':';
    cmdId = ui->comboBox_CMD->currentData().toInt();
    switch (cmdId) {
    case SETMOTORTEST://MOTORTEST=0xA1,
        dato[indice++] =SETMOTORTEST;
        w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor1:", 0, -100, 100, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor2:", 0, -100, 100, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        dato[NBYTES]= 0x0A;
        break;
    case SETSERVOANGLE://SERVOANGLE=0xA2,
        dato[indice++] =SETSERVOANGLE;
        w.i32 = QInputDialog::getInt(this, "SERVO", "Angulo:", 0, -90, 90, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.i8[0];
        dato[NBYTES]= 0x03;
        break;
    case RESETMODE:
    case STARTSTOP:
    case NROCAMINO:
    case UPDATEREALTIME:
    case STATUSCAMINOS:
    case STATUSROBOT:
    case GETALIVE:
    case GETDISTANCE://GETDISTANCE=0xA3,
    case GETSPEED://GETSPEED=0xA4,
    case GETSWITCHES://GETSWITCHES=0xA5
    case GETFIRMWARE:// GETFIRMWARE=0xF1
    case GETANALOGSENSORS://ANALOGSENSORS=0xA0,
    case SETLEDS:
        dato[indice++]=cmdId;
        //falta implementar el envío del valor de seteo
        dato[NBYTES]=0x02;
        break;
    default:
        return;
    }
    for(int a=0 ;a<indice;a++)
        chk^=dato[a];
    dato[indice]=chk;

    if(serial->isWritable()){
        serial->write(reinterpret_cast<char *>(dato),dato[NBYTES]+PAYLOAD);
    }

    for(int i=0; i<=indice; i++){
        if(isalnum(dato[i]))
            str = str + QString("%1").arg(char(dato[i]));
        else
            str = str +"{" + QString("%1").arg(dato[i],2,16,QChar('0')) + "}";
    }

    uint16_t valor=dato[NBYTES]+PAYLOAD;
    ui->textEdit_RAW->append("INDICE ** " +QString().number(indice,10) + " **" );
    ui->textEdit_RAW->append("NUMERO DE DATOS ** " +QString().number(valor,10) + " **" );
    ui->textEdit_RAW->append("CHECKSUM ** " +QString().number(chk,16) + " **" );
    ui->textEdit_RAW->append("PC--SERIAL-->MBED ( " + str + " )");

}

void QForm::timeOut(){
    if(rxData.timeOut){
        rxData.timeOut--;
        if(!rxData.timeOut){
            estadoProtocolo=START;
        }
    }
}

void QForm::OnUdpRxData(){
    qint64          count=0;
    unsigned char   *incomingBuffer;

    while(UdpSocket1->hasPendingDatagrams()){
        count = UdpSocket1->pendingDatagramSize();
        incomingBuffer = new unsigned char[count];
        UdpSocket1->readDatagram( reinterpret_cast<char *>(incomingBuffer), count, &RemoteAddress, &RemotePort);
    }
    if (count<=0)
        return;

    QString str="";
    for(int i=0; i<=count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg(char(incomingBuffer[i]));
        else
            str = str +"{" + QString("%1").arg(incomingBuffer[i],2,16,QChar('0')) + "}";
    }
    ui->textEdit_RAW->append("MBED-->UDP-->PC (" + str + ")");
    QString adress=RemoteAddress.toString();
    ui->textEdit_RAW->append(" adr " + adress);
    ui->lineEdit_IP_REMOTA->setText(RemoteAddress.toString().right((RemoteAddress.toString().length())-7));
    ui->lineEdit_DEVICEPORT->setText(QString().number(RemotePort,10));

    for(int i=0;i<count; i++){
        switch (estadoProtocoloUdp) {
        case START:
            if (incomingBuffer[i]=='U'){
                estadoProtocoloUdp=HEADER_1;
                rxDataUdp.cheksum=0;
            }
            break;
        case HEADER_1:
            if (incomingBuffer[i]=='N')
                estadoProtocoloUdp=HEADER_2;
            else{
                i--;
                estadoProtocoloUdp=START;
            }
            break;
        case HEADER_2:
            if (incomingBuffer[i]=='E')
                estadoProtocoloUdp=HEADER_3;
            else{
                i--;
                estadoProtocoloUdp=START;
            }
            break;
        case HEADER_3:
            if (incomingBuffer[i]=='R')
                estadoProtocoloUdp=NBYTES;
            else{
                i--;
                estadoProtocoloUdp=START;
            }
            break;
        case NBYTES:
            rxDataUdp.nBytes=incomingBuffer[i];
            estadoProtocoloUdp=TOKEN;
            break;
        case TOKEN:
            if (incomingBuffer[i]==':'){
                estadoProtocoloUdp=PAYLOAD;
                rxDataUdp.cheksum='U'^'N'^'E'^'R'^ rxDataUdp.nBytes^':';
                rxDataUdp.payLoad[0]=rxDataUdp.nBytes;
                rxDataUdp.index=1;
            }
            else{
                i--;
                estadoProtocoloUdp=START;
            }
            break;
        case PAYLOAD:
            if (rxDataUdp.nBytes>1){
                rxDataUdp.payLoad[rxDataUdp.index++]=incomingBuffer[i];
                rxDataUdp.cheksum^=incomingBuffer[i];
            }
            rxDataUdp.nBytes--;
            if(rxDataUdp.nBytes==0){
                estadoProtocoloUdp=START;
                if(rxDataUdp.cheksum==incomingBuffer[i]){
                    decodeData(&rxDataUdp.payLoad[0],UDP);
                }else{
                    ui->textEdit_RAW->append(" CHK DISTINTO!!!!! ");
                }
            }
            break;

        default:
            estadoProtocoloUdp=START;
            break;
        }
    }
    delete [] incomingBuffer;

}

void QForm::sendDataUDP(){
    uint8_t cmdId;
    _udat w;
    unsigned char dato[256];
    unsigned char indice=0, chk=0;
    QString str;
    int puerto=0;
    bool ok;

    dato[indice++]='U';
    dato[indice++]='N';
    dato[indice++]='E';
    dato[indice++]='R';
    dato[indice++]=0x00;
    dato[indice++]=':';
    cmdId = ui->comboBox_CMD->currentData().toInt();
    switch (cmdId) {
    case SETMOTORTEST://MOTORTEST=0xA1,
        dato[indice++] =SETMOTORTEST;
        w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor1:", 0, -100, 100, 1, &ok);
        if(!ok)
            return;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        w.i32 = QInputDialog::getInt(this, "Velocidad", "Motor2:", 0, -100, 100, 1, &ok);
        if(!ok)
            break;
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        dato[NBYTES]= 0x0A;
        break;
    case SETSERVOANGLE://SERVOANGLE=0xA2,
        dato[indice++] =SETSERVOANGLE;
        w.i32 = QInputDialog::getInt(this, "SERVO", "Angulo:", 0, -90, 90, 1, &ok);
        if(!ok)
            return;
        dato[indice++] = w.i8[0];
        dato[NBYTES]= 0x03;
        break;
    case RESETMODE:
    case STARTSTOP:
    case NROCAMINO:
    case UPDATEREALTIME:
    case STATUSCAMINOS:
    case STATUSROBOT:
    case GETALIVE:
    case GETDISTANCE://GETDISTANCE=0xA3,
    case GETSPEED://GETSPEED=0xA4,
    case GETSWITCHES://GETSWITCHES=0xA5
    case GETFIRMWARE:// GETFIRMWARE=0xF1
    case GETANALOGSENSORS://ANALOGSENSORS=0xA0,
    case SETLEDS:
        dato[indice++]=cmdId;
        dato[NBYTES]=0x02;
        break;
    default:
        ;

    }

    puerto=ui->lineEdit_DEVICEPORT->text().toInt();
    puertoremoto=puerto;
    for(int a=0 ;a<indice;a++)
        chk^=dato[a];
    dato[indice]=chk;
    if(clientAddress.isNull())
        clientAddress.setAddress(ui->lineEdit_IP_REMOTA->text());
    if(puertoremoto==0)
        puertoremoto=puerto;
    if(UdpSocket1->isOpen()){
        UdpSocket1->writeDatagram(reinterpret_cast<const char *>(dato), (dato[4]+7), clientAddress, puertoremoto);
    }

    for(int i=0; i<=indice; i++){
        if(isalnum(dato[i]))
            str = str + QString("%1").arg(char(dato[i]));
        else
            str = str +"{" + QString("%1").arg(dato[i],2,16,QChar('0')) + "}";
    }
    str=str + clientAddress.toString() + "  " +  QString().number(puertoremoto,10);
    ui->textEdit_RAW->append("PC--UDP-->MBED ( " + str + " )");
}

void QForm::on_pushButtonOPENUDP_clicked()
{
    int Port;
    bool ok;

    if(UdpSocket1->isOpen()){
        UdpSocket1->close();
        ui->pushButtonOPENUDP->setText("OPEN UDP");
        return;
    }

    Port=ui->lineEdit_LOCALPORT->text().toInt(&ok,10);
    if(!ok || Port<=0 || Port>65535){
        QMessageBox::information(this, tr("SERVER PORT"),tr("ERRRO. Number PORT."));
        return;
    }

    try{
        UdpSocket1->abort();
        UdpSocket1->bind(Port);
        UdpSocket1->open(QUdpSocket::ReadWrite);
    }catch(...){
        QMessageBox::information(this, tr("SERVER PORT"),tr("Can't OPEN Port."));
        return;
    }
    ui->pushButtonOPENUDP->setText("CLOSE UDP");
    ui->pushButtonSENDUDP->setEnabled(true);
    if(UdpSocket1->isOpen()){
        if(clientAddress.isNull())
            clientAddress.setAddress(ui->lineEdit_IP_REMOTA->text());
        if(puertoremoto==0)
            puertoremoto=ui->lineEdit_DEVICEPORT->text().toInt();
        UdpSocket1->writeDatagram("r", 1, clientAddress, puertoremoto);
    }
}

void QForm::on_pushButtonALIVE_clicked()
{
    ui->comboBox_CMD->setCurrentIndex(0);
    sendDataSerial();
}

//Control de graficas
void QForm::createChartADC()
{
    adcChart = new QChart();

    adcChart->setTitle("Valores del ADC");
    adcChart->legend()->setVisible(true);
    adcChart->setAnimationOptions(QChart::AnimationOption::NoAnimation);

    adcChartView = new QChartView(adcChart);

    adcChartView->setRenderHint(QPainter::Antialiasing);

    adcLayout = new QGridLayout();

    adcLayout->addWidget(adcChartView, 0, 0);

    ui->widgetADC->setLayout(adcLayout);

    adc0Spline = new QSplineSeries();
    adc1Spline = new QSplineSeries();
    adc2Spline = new QSplineSeries();

    for (int i = 0 ; i <= 30 ; i++)
    {
        adc0Datos.append(QPointF(i, 0));
        adc1Datos.append(QPointF(i, 0));
        adc2Datos.append(QPointF(i, 0));
    }

    adc0Spline->append(adc0Datos);
    adc1Spline->append(adc1Datos);
    adc2Spline->append(adc2Datos);

    adc0Spline->setName("IR0");
    adc1Spline->setName("IR1");
    adc2Spline->setName("IR2");

    adcChart->addSeries(adc0Spline);
    adcChart->addSeries(adc1Spline);
    adcChart->addSeries(adc2Spline);

    adcChart->createDefaultAxes();
    adcChart->axes(Qt::Vertical).first()->setRange(0, 65535);
    adcChart->axes(Qt::Horizontal).first()->setRange(0, 30);
}

void QForm::addPointChartADC0(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        adc0Datos.replace(i, QPointF(i, adc0Datos.value(i + 1).ry()));
    }

    adc0Datos.removeLast();
    adc0Datos.append(QPointF(30, point * 1.0));

    adc0Spline->clear();
    adc0Spline->append(adc0Datos);
}

void QForm::addPointChartADC1(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        adc1Datos.replace(i, QPointF(i, adc1Datos.value(i + 1).ry()));
    }

    adc1Datos.removeLast();
    adc1Datos.append(QPointF(30, point * 1.0));

    adc1Spline->clear();
    adc1Spline->append(adc1Datos);
}

void QForm::addPointChartADC2(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        adc2Datos.replace(i, QPointF(i, adc2Datos.value(i + 1).ry()));
    }

    adc2Datos.removeLast();
    adc2Datos.append(QPointF(30, point * 1.0));

    adc2Spline->clear();
    adc2Spline->append(adc2Datos);
}

void QForm::createChartVelocidadMotores()
{
    velMotorChart = new QChart();

    velMotorChart->setTitle("Velocidad de Motores");
    velMotorChart->legend()->setVisible(true);
    velMotorChart->setAnimationOptions(QChart::AnimationOption::NoAnimation);

    velMotorChartView = new QChartView(velMotorChart);

    velMotorChartView->setRenderHint(QPainter::Antialiasing);

    velMotorLayout = new QGridLayout();

    velMotorLayout->addWidget(velMotorChartView, 0, 0);

    ui->widgetVelMotor->setLayout(velMotorLayout);

    velMotor0Spline = new QSplineSeries();
    velMotor1Spline = new QSplineSeries();

    for (int i = 0 ; i <= 30 ; i++)
    {
        velMotor0Datos.append(QPointF(i, 0));
        velMotor1Datos.append(QPointF(i, 0));
    }

    velMotor0Spline->append(velMotor0Datos);
    velMotor1Spline->append(velMotor1Datos);

    velMotor0Spline->setName("MOTOR DERECHO");
    velMotor1Spline->setName("MOTOR IZQUIERDO");

    velMotorChart->addSeries(velMotor0Spline);
    velMotorChart->addSeries(velMotor1Spline);

    velMotorChart->createDefaultAxes();
    velMotorChart->axes(Qt::Vertical).first()->setRange(-50, 50);
    velMotorChart->axes(Qt::Horizontal).first()->setRange(0, 10);
}

void QForm::addPointChartVelMotor0(int32_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        velMotor0Datos.replace(i, QPointF(i, velMotor0Datos.value(i + 1).ry()));
    }

    velMotor0Datos.removeLast();
    velMotor0Datos.append(QPointF(30, point * 1.0));

    velMotor0Spline->clear();
    velMotor0Spline->append(velMotor0Datos);
}

void QForm::addPointChartVelMotor1(int32_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        velMotor1Datos.replace(i, QPointF(i, velMotor1Datos.value(i + 1).ry()));
    }

    velMotor1Datos.removeLast();
    velMotor1Datos.append(QPointF(30, point * 1.0));

    velMotor1Spline->clear();
    velMotor1Spline->append(velMotor1Datos);
}

void QForm::on_radioButtonUpdateRealTime_clicked()
{
    ui->comboBox_CMD->setCurrentIndex(12);
    sendDataSerial();
    sendDataUDP();
}


void QForm::on_pushButtonStartStop_clicked()
{
    if(ARRANQUE){
        ARRANQUE = false;
        ui->pushButtonStartStop->setText("STOP");
        ui->comboBox_CMD->setCurrentIndex(1);
        sendDataSerial();
        sendDataUDP();

    }else{
        ARRANQUE = true;
        ui->pushButtonStartStop->setText("START");
        ui->comboBox_CMD->setCurrentIndex(1);
        sendDataSerial();
        sendDataUDP();
    }
}


void QForm::on_pushButtonReset_clicked() //RESET
{
    ui->comboBox_CMD->setCurrentIndex(13);
    sendDataSerial();
    sendDataUDP();
}

