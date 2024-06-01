    /*! \mainpage Final cumputacion 3
 * \date 10/09/2023
 * \author Medina Nahuel
 * \section Ejemplo comunicación USART
 * [Complete aqui con su descripcion]
 *
 * \section desarrollos Observaciones generales
 * [Complete aqui con sus observaciones]
 *
 * \section changelog Registro de cambios
 *
 * |   Fecha    | Descripcion                                    |
 * |:----------:|:-----------------------------------------------|
 * | 10/09/2023 | Creacion del documento                         |
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
#include "util.h"
#include "myDelay.h"
#include "debounce.h"
#include "config.h"
#include "wifi.h"
/* END Includes --------------------------------------------------------------*/


/* typedef -------------------------------------------------------------------*/
enum e_estadosPulsador {
    UP,
    DOWN,
    FALLING,
    RISING
};

/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/
#define RXBUFSIZE  256
#define TXBUFSIZE  256
#define DEBOUNCE    20
#define HEARBEATIME 100
#define GENERALTIME 10
#define NUMBUTTONS  4
#define DISTANCEINTERVAL    300

#define     FORWARD             1
#define     BACKWARD            2
#define     STOP                0

#define     SERIE               0
#define     WIFI                1

#define     ROTACIONDER         1
#define     ROTACIONIZQ         0            


#define ANTIREBOTE              40
#define RESETFLAGS      flags.bytes 
#define ISCOMAND        flags.bits.bit0
#define ENTRADACIRCULO     flags.bits.bit1
#define FLAGREALTIME     flags.bits.bit2
#define MEDIRDISTANCIA  flags.bits.bit3
#define IS100MS         flags.bits.bit4
#define IS10MS          flags.bits.bit5
#define ECHOPULSE       flags.bits.bit6
#define ECHORISE        flags.bits.bit7

#define RESETFLAGSUTIL                  flagsUtil.bytes
#define COUNTGIROON                     flagsUtil.bits.bit0
#define ROTACIONFINALIZADA              flagsUtil.bits.bit1
#define SENSORLIBRE                     flagsUtil.bits.bit2
#define MARCAINICIAL                    flagsUtil.bits.bit3
#define MARCAFINAL                      flagsUtil.bits.bit4
#define CONTADORHABILITADO              flagsUtil.bits.bit5
#define ISXSEGUNDOS                     flagsUtil.bits.bit6
#define SEGUIRLINEAHABILITADO           flagsUtil.bits.bit7


/* END define ----------------------------------------------------------------*/

/* hardware configuration ----------------------------------------------------*/

DigitalOut LED(PC_13);//!< Led de Hearbeta

BusOut leds(PB_6,PB_7,PB_14,PB_15);//!< leds de la placa

InterruptIn pulsadores(PB_4);//!< Botonnes de la placa

RawSerial PC(PA_9,PA_10);//!< Configuración del puerto serie, la velocidad (115200) tiene que ser la misma en QT

DigitalOut trigger(PB_13);//!< Salida para el trigger del sensor Ultrasonico 

InterruptIn echo(PB_12); //!<pin de eco del sensor Ultrasonico definido como interrupción 

PwmOut  servo(PA_8);//!< Pin del Servo debe ser PWM para poder modular el ancho del pulso

AnalogIn irLeft(PA_2); //!<Sensor infrarrojo para detección de linea 

AnalogIn irCenter(PA_1);//!<Sensor infrarrojo para detección de linea 

AnalogIn irRight(PA_0);//!<Sensor infrarrojo para detección de linea 

InterruptIn speedLeft(PB_9);//!<Sensor de Horquilla para medir velocidad

InterruptIn speedRight(PB_8);//!<Sensor de Horquilla para medir velocidad

BusOut  dirMLeft(PB_15,PB_14);//!< Pines para determinara la dirección de giro del motor

BusOut  dirMRight(PB_7,PB_6);//!< Pines para determinara la dirección de giro del motor

PwmOut  speedMLeft(PB_1);//!< Pin de habilitación del giro del motor, se usa para controlar velocidad del mismo

PwmOut  speedMRight(PB_0);//!< Pin de habilitación del giro del motor, se usa para controlar velocidad del mismo

/* END hardware configuration ------------------------------------------------*/


/* END Function prototypes ---------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/

const char firmware[] = "EX100923v01\n";

volatile _sRx dataRx;

uint32_t countLeftValue=0, countRightValue=0;
uint32_t countLeftValueGiro=0, countRightValueGiro=0;

uint32_t speedleftValue, speedRightValue, tiempoObjCernado = 0;

_sTx dataTx;

wifiData myWifiData;

_eModoSeguirLinea modoSeguirLinea = detectarLinea;
_eModoSeguirLinea auxSensor;
_eControlMotores ControlMotores = PARAR;


_eRobotMode robotMode = IDLE;
_eModoPromocion promotionRoutineMode = GIROINICIAL;
_eRobotPosicion robotPosicion = CIRCULO;

uint8_t Entradas = 0;

volatile uint8_t buffRx[RXBUFSIZE];

uint8_t buffTx[TXBUFSIZE];

uint8_t globalIndex, index2;

_uFlag  flags, flagsUtil, flagPosicion;

_sButton myButton;

_delay_t  generalTime;
_delay_t  waitMode;

Timer myTimer;
Timer TimeHCSR04;
Timeout TimeTriggerPulse; 
Ticker Ticker10ms;

uint32_t distancia_us;
uint8_t timeOutDistancia;
uint8_t time100ms;
_sSensor irSensor[3];

_uWord myWord;

_sTx wifiTx;

_sRx wifiRx;

uint16_t contGrados = 0;
uint16_t gradosAux = 35;
uint16_t distanciaEnCm = 100;
uint16_t distanciaEnCmAUX = 100;

uint8_t wifiBuffRx[RXBUFSIZE];

uint8_t wifiBuffTx[TXBUFSIZE];

uint8_t flagIr = 0;
uint8_t flagIrBlanco = 0;

uint8_t flagPrimerObstaculo = 0;

uint8_t flagCentro = 1;
uint8_t flagCamino = 0;
uint8_t flagCaminoExterior = 0;

static int32_t timeSpeed=0;

int pidOutput = 0;

uint16_t baseSpeed = 5000;
uint16_t leftSpeed = 0;
uint16_t rightSpeed = 0;
uint16_t erro = 0;
uint16_t errorAnterior = 0;
uint16_t posicion = 0;
uint16_t setPointDistancia = 15;
int erroDistancia = 0; 


int16_t anguloServo = 0;

uint16_t numeroHorquilla = 5;

uint8_t timeOutScanSensor = 3;
uint8_t angleScanSensor = 0;
uint16_t mejorAngleScanSensor = 0;
uint16_t mejorDistancia = 1000;

uint16_t timeOutXSegundos = 0;

uint8_t countPath = 0;

uint32_t completedRoads[4] = {0};



_sHearBeat controlDeModos;


/* END Global variables ------------------------------------------------------*/

/**
 * @brief Instanciación de la clase Wifi, le paso como parametros el buffer de recepción, el indice de 
 * escritura para el buffer de recepción y el tamaño del buffer de recepción
 */
Wifi myWifi(wifiBuffRx, &wifiRx.indexW, RXBUFSIZE);

/* Function prototypes user code ----------------------------------------------*/


void ControlDeMotores(_eControlMotores direccion,uint16_t mDerecho, uint16_t mIzquierdo){

    switch (direccion)
    {
    case ADELANTE:
        dirMLeft.write(FORWARD);
        speedMLeft.pulsewidth_us(mIzquierdo*250);
        dirMRight.write(FORWARD);
        speedMRight.pulsewidth_us(mDerecho*250);
        break;
    case ATRAS:
        dirMLeft.write(BACKWARD);
        speedMLeft.pulsewidth_us(mIzquierdo*250);
        dirMRight.write(BACKWARD);
        speedMRight.pulsewidth_us(mDerecho*250);
        break;
    case PARAR:
        dirMLeft.write(STOP);
        speedMLeft.pulsewidth_us(0);
        dirMRight.write(STOP);
        speedMRight.pulsewidth_us(0);
        break;
    case DERECHA_SOBRE_EJE:
        dirMLeft.write(FORWARD);
        speedMLeft.pulsewidth_us(mIzquierdo*250);
        dirMRight.write(BACKWARD);
        speedMRight.pulsewidth_us(mDerecho*250);
        break;
    case IZQUIERDA_SOBRE_EJE:
        dirMLeft.write(BACKWARD);
        speedMLeft.pulsewidth_us(mIzquierdo*250);
        dirMRight.write(FORWARD);
        speedMRight.pulsewidth_us(mDerecho*250);
        break;
    }
}

void CalcuModoDelSeguirLINEA(){
   
    flagIrBlanco = 0;
    flagIr = 0;

    if(irSensor[1].currentValue <7000 ){
        flagIrBlanco++;
    }else if(irSensor[1].currentValue > 25000){
        flagIr=+2;
        irSensor[1].currentValue = 25000;
    }
    if(irSensor[2].currentValue < 7000){
        flagIrBlanco++;
    }else if(irSensor[2].currentValue > 25000){
        flagIr=+4;
        irSensor[2].currentValue = 25000;
    }

    if(irSensor[0].currentValue < 7000 ){
        flagIrBlanco++;
    }else if(irSensor[0].currentValue > 25000 ){
        flagIr=+1;
        irSensor[0].currentValue = 25000;
    }
    
}

void ControlDeMotoresSeguirLinea(){

    switch (flagIr){
        case 2:
            modoSeguirLinea = adelante;      
            
            break;
        case 1:
            modoSeguirLinea = derecha;
            auxSensor = modoSeguirLinea;
            break;
        case 3:
            modoSeguirLinea = derechaYCentro;
            auxSensor = modoSeguirLinea;
            
            break;
        
        case 4:
            modoSeguirLinea = izquierda;
            auxSensor = modoSeguirLinea;

            break;
        case 6:
            modoSeguirLinea = izquierdaYcentro; 
                         
            auxSensor = modoSeguirLinea;
            break;
        case 0:
            /*if(flagIrBlanco == 3){
                modoSeguirLinea = auxSensor;
              
            }*/
            if(auxSensor == izquierda ){
                modoSeguirLinea = buscarLineaIzquierda;
                auxSensor = modoSeguirLinea;
            }else if (auxSensor == derecha ){
                modoSeguirLinea = buscarLineaDerecha;
                auxSensor = modoSeguirLinea;
            }
        

            break;
    }
}

void SeguirLinea(){

switch (modoSeguirLinea)
    {
    case detectarLinea:
        CalcuModoDelSeguirLINEA();
        ControlDeMotoresSeguirLinea();
       
        erro = 15000 - (irSensor[1].currentValue - ((irSensor[0].currentValue + irSensor[2].currentValue)/2));
   
        pidOutput = ((erro)  / 20);

        if(flagIr > 4){
        pidOutput = pidOutput * (-1);
        }

        errorAnterior = erro;
        leftSpeed = ((baseSpeed + pidOutput)*100)/25000;
        rightSpeed = ((baseSpeed - pidOutput)*100)/25000;

        break;
    case adelante:
        ControlDeMotores(ADELANTE,rightSpeed,leftSpeed);
        modoSeguirLinea = detectarLinea;
        break;
    case derecha:
        ControlDeMotores(ADELANTE,rightSpeed,leftSpeed);
        modoSeguirLinea = detectarLinea;
        break;
    case izquierda:
        ControlDeMotores(ADELANTE,rightSpeed,leftSpeed);
        modoSeguirLinea = detectarLinea;
        break;
    case parar:
        ControlDeMotores(PARAR,0,0);
        break;
    case buscarLineaDerecha:
        ControlDeMotores(DERECHA_SOBRE_EJE,25,25);
        modoSeguirLinea = detectarLinea;
        break;
    case buscarLineaIzquierda:
        ControlDeMotores(IZQUIERDA_SOBRE_EJE,25,25);
        modoSeguirLinea = detectarLinea;
        break;
    case rotarAuto:
        ControlDeMotores(DERECHA_SOBRE_EJE,25,25);
        modoSeguirLinea = detectarLinea;
        break;
    case derechaYCentro:
        ControlDeMotores(ADELANTE,rightSpeed,leftSpeed);
        modoSeguirLinea = detectarLinea;
        break;
    case izquierdaYcentro:
        ControlDeMotores(ADELANTE,rightSpeed,leftSpeed);
        modoSeguirLinea = detectarLinea;    
        break;

    default:
        break;
    }


}

void ECHOInt(){
    if(ECHORISE){ //si el flanco es ascendente
        TimeHCSR04.reset();
        TimeHCSR04.start();
        ECHORISE = 0;
    } else { //si el flanco es descendente
        distancia_us = TimeHCSR04.read_us();    //distancia del objeto en microsegundos 
        TimeHCSR04.stop();
        distanciaEnCm = (distancia_us/58);
        ECHOPULSE = 0;                          //indica que vino el flanco descendente
    }
}

void OnTimeTriggerPulse(){
    trigger = 0;
    ECHORISE = 1;
}

void Do100ms(){

    IS100MS = 0;

    distanciaEnCm = (distancia_us/58);

    timeOutDistancia--;
    if(!timeOutDistancia){
        if(ECHOPULSE){ //si nunca vino el flanco descendente hay un error 
            ECHOPULSE = 0;
            TimeHCSR04.stop();
            distancia_us = 8700;//distancia no alcanzable
        }
        timeOutDistancia = 2;
        trigger = 1;                                           //dispara trigger (señal)
        TimeTriggerPulse.attach_us(&OnTimeTriggerPulse, 10);   //espero 10us para frenar pulso de trigger
        ECHOPULSE = 1;
    }

    if (ISXSEGUNDOS)
    {
        timeOutXSegundos--;
        if (!timeOutXSegundos)
        {
            ISXSEGUNDOS = 0;
        }
    }
}

void On10ms(){
    time100ms--;
    if(!time100ms){
        IS100MS = 1;
        time100ms = 10;    
    }

    IS10MS = 1;
}

void ControlServo(uint16_t grados){
    servo.pulsewidth_us((grados * ((2600 - 400) / 180)) + 400);

}

void decodeHeader(_sRx *dataRx)
{
    uint8_t auxIndex=dataRx->indexW;
    while(dataRx->indexR != auxIndex){
        switch(dataRx->header)
        {
            case HEADER_U:
                if(dataRx->buff[dataRx->indexR] == 'U'){
                    dataRx->header = HEADER_N;
                    dataRx->timeOut = 5;
                }
            break;
            case HEADER_N:
                if(dataRx->buff[dataRx->indexR] == 'N'){
                    dataRx->header = HEADER_E;
                }else{
                    if(dataRx->buff[dataRx->indexR] != 'U'){
                        dataRx->header = HEADER_U;
                        dataRx->indexR--;
                    }
                }
            break;
            case HEADER_E:
                if(dataRx->buff[dataRx->indexR] == 'E'){
                    dataRx->header = HEADER_R;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case HEADER_R:
                if(dataRx->buff[dataRx->indexR] == 'R'){
                    dataRx->header = NBYTES;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case NBYTES:
                dataRx->nBytes=dataRx->buff[dataRx->indexR];
                dataRx->header = TOKEN;
            break;
            case TOKEN:
                if(dataRx->buff[dataRx->indexR] == ':'){
                    dataRx->header = PAYLOAD;
                    dataRx->indexData = dataRx->indexR+1;
                    dataRx->indexData &= dataRx->mask;
                    dataRx->chk = 0;
                    dataRx->chk ^= ('U' ^'N' ^'E' ^'R' ^dataRx->nBytes ^':') ;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case PAYLOAD:
                dataRx->nBytes--;
                if(dataRx->nBytes>0){
                   dataRx->chk ^= dataRx->buff[dataRx->indexR];
                }else{
                    dataRx->header = HEADER_U;
                    if(dataRx->buff[dataRx->indexR] == dataRx->chk)
                        dataRx->isComannd = true;
                }
            break;
            default:
                dataRx->header = HEADER_U;
            break;
        }
        dataRx->indexR++;
        dataRx->indexR &= dataRx->mask;
    }
}

//Decodifica el comando recibido en la transmisión y ejecuita las tareas asociadas a dicho comando

void decodeCommand(_sRx *dataRx, _sTx *dataTx)
{
    int32_t motorSpeed, auxSpeed;
    //int8_t angleSource;
    //uint32_t servoPrevio=miServo.currentValue;

    switch(dataRx->buff[dataRx->indexData]){
        case ALIVE:
            putHeaderOnTx(dataTx, ALIVE, 2);
            putByteOnTx(dataTx, ACK );
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case FIRMWARE:
            putHeaderOnTx(dataTx, FIRMWARE, 12);
            putStrOntx(dataTx, firmware);
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case BUTTONSTATUS:
            putHeaderOnTx(dataTx, BUTTONSTATUS, 2);
            putByteOnTx(dataTx, ((~((uint8_t)pulsadores.read()))&0x0F));
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case ANALOGSENSORS:
            myWord.ui16[0] =  irSensor[0].currentValue;
            putHeaderOnTx(dataTx, ANALOGSENSORS, 7);
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            myWord.ui16[0] =  irSensor[1].currentValue;
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            myWord.ui16[0] =  irSensor[2].currentValue;
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] ); 
            putByteOnTx(dataTx, dataTx->chk);       
        break;
        case SETBLACKCOLOR:
        break;
        case SETWHITECOLOR:
        break;
        case MOTORTEST:
            putHeaderOnTx(dataTx, MOTORTEST, 2);
            putByteOnTx(dataTx, ACK );
            putByteOnTx(dataTx, dataTx->chk);
            myWord.ui8[0]=getByteFromRx(dataRx,1,0);
            myWord.ui8[1]=getByteFromRx(dataRx,1,0);
            myWord.ui8[2]=getByteFromRx(dataRx,1,0);
            myWord.ui8[3]=getByteFromRx(dataRx,1,0);
            motorSpeed = myWord.i32;
            if(motorSpeed>=0)
                dirMLeft.write(FORWARD);
            else
                dirMLeft.write(BACKWARD);
            auxSpeed=(abs(motorSpeed))*250;
            speedMLeft.pulsewidth_us(auxSpeed);
            myWord.ui8[0]=getByteFromRx(dataRx,1,0);
            myWord.ui8[1]=getByteFromRx(dataRx,1,0);
            myWord.ui8[2]=getByteFromRx(dataRx,1,0);
            myWord.ui8[3]=getByteFromRx(dataRx,1,0);
            motorSpeed = myWord.i32;
            if(motorSpeed>=0)
                dirMRight.write(FORWARD);
            else
                dirMRight.write(BACKWARD);
            auxSpeed=(abs(motorSpeed))*250;
            speedMRight.pulsewidth_us (auxSpeed);
        break;
        case SERVOANGLE:
            putHeaderOnTx(dataTx, SERVOANGLE, 2);
            putByteOnTx(dataTx, ACK );
            putByteOnTx(dataTx, dataTx->chk);
            anguloServo = getByteFromRx(dataRx,1,0);
            ControlServo(anguloServo);
        break;
        case CONFIGSERVO:
        break;
        case GETDISTANCE:
            myWord.ui32 = distancia_us;
            putHeaderOnTx(dataTx, GETDISTANCE, 5);
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] ); 
            putByteOnTx(dataTx, dataTx->chk);  
        break;
        case GETSPEED:
            myWord.ui32 = speedleftValue;
            putHeaderOnTx(dataTx, GETSPEED, 9);
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] ); 
            myWord.ui32 = speedRightValue;
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] ); 
            putByteOnTx(dataTx, dataTx->chk);           
        break;
        
        case NROCAMINO:
            putHeaderOnTx(dataTx, NROCAMINO, 2);
            putByteOnTx(dataTx, countPath);
            putByteOnTx(dataTx, dataTx->chk);
            break;
        case STATUSCAMINOS:
            putHeaderOnTx(dataTx, STATUSCAMINOS, 17);
            myWord.ui32 =  completedRoads[0];
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] );

            myWord.ui32 =  completedRoads[1];
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] );

            myWord.ui32 =  completedRoads[2];
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] );

            myWord.ui32 =  completedRoads[3];
            putByteOnTx(dataTx, myWord.ui8[0] );
            putByteOnTx(dataTx, myWord.ui8[1] );
            putByteOnTx(dataTx, myWord.ui8[2] );
            putByteOnTx(dataTx, myWord.ui8[3] );

            putByteOnTx(dataTx, dataTx->chk); 

            break;
        
        case STATUSROBOT:
            putHeaderOnTx(dataTx, STATUSROBOT, 2);
            putByteOnTx(dataTx, promotionRoutineMode);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case STARTSTOP:
            if (robotMode == IDLE)
            {
                robotMode = MODEPROMOCION;
            }else{
                robotMode = IDLE;
            }   
        break;

        case UPDATEREALTIME:
            if (FLAGREALTIME)
            {
                FLAGREALTIME = 0;
            }else{
                FLAGREALTIME = 1;
            }
        break;

        case RESETMODE:
            robotMode = IDLE;
            promotionRoutineMode = GIROINICIAL;
            robotPosicion = CIRCULO;
            Entradas = 0;
        break;

        default:
            putHeaderOnTx(dataTx, (_eCmd)dataRx->buff[dataRx->indexData], 2);
            putByteOnTx(dataTx,UNKNOWN );
            putByteOnTx(dataTx, dataTx->chk);
        break;
        
    }


}

void speedTask(){
    //static int32_t timeSpeed=0;
    #define INTERVAL 1000
    if ((myTimer.read_ms()-timeSpeed)>=INTERVAL){
            timeSpeed=myTimer.read_ms();       
            speedleftValue = countLeftValue;
            countLeftValue=0;
            speedRightValue=countRightValue;
            countRightValue=0;
    } 
}


void irSensorsTask(){
    static int32_t timeSensors=0;
    static uint8_t index=0;
    #define INTERVALO 10
    if ((myTimer.read_ms()-timeSensors)>=INTERVALO){
            timeSensors=myTimer.read_ms(); 
            switch(index){
                case 0:
                    irSensor[index].currentValue=irRight.read_u16();
                break;
                case 1:
                    irSensor[index].currentValue=irCenter.read_u16();
                break;
                case 2:
                    irSensor[index].currentValue=irLeft.read_u16();
                break;
                default:
                    index=255;
            }
            index++;
            index &=(-(index<=2));
    } 

}

void speedCountLeft(void){
    countLeftValue++;
    if (COUNTGIROON)
    {
        countLeftValueGiro++;
    } 
}

void speedCountRight(void){
    countRightValue++;
    if (COUNTGIROON)
    {
        countRightValueGiro++;
    } 
}


/**********************************AUTO CONNECT WIF*********************/

void autoConnectWifi(){
    #ifdef AUTOCONNECTWIFI
        memcpy(&myWifiData.cwmode, dataCwmode, sizeof(myWifiData.cwmode));
        memcpy(&myWifiData.cwdhcp,dataCwdhcp, sizeof(myWifiData.cwdhcp) );
        memcpy(&myWifiData.cwjap,dataCwjap, sizeof(myWifiData.cwjap) );
        memcpy(&myWifiData.cipmux,dataCipmux, sizeof(myWifiData.cipmux) );
        memcpy(&myWifiData.cipstart,dataCipstart, sizeof(myWifiData.cipstart) );
        memcpy(&myWifiData.cipmode,dataCipmode, sizeof(myWifiData.cipmode) );
        memcpy(&myWifiData.cipsend,dataCipsend, sizeof(myWifiData.cipsend) );
        myWifi.configWifi(&myWifiData);
    #endif
}

void aliveAutoTask(_delay_t *aliveAutoTime)
{
    if(myWifi.isWifiReady()){
        if(delayRead(aliveAutoTime))
        {
            putHeaderOnTx(&wifiTx, ALIVE, 2);
            putByteOnTx(&wifiTx, ACK );
            putByteOnTx(&wifiTx, wifiTx.chk);
        }
    }
}

void hearbeatTask(_delay_t *timeHearbeat, uint32_t mask)
{
    static uint8_t sec=0;
    if(delayRead(timeHearbeat)){
        LED = (~mask & (1<<sec));
        sec++;
        if(controlDeModos.flagBotonPresionado){
            sec &= -(sec<10); 
        }else{
            sec &= -(sec<30);
        }
        
    }
}

void serialTask(_sRx *dataRx, _sTx *dataTx, uint8_t source)
{
    if(dataRx->isComannd){
        dataRx->isComannd=false;
        decodeCommand(dataRx,dataTx);
    }

    if(delayRead(&generalTime)){
        if(dataRx->header){
            dataRx->timeOut--;
        if(!dataRx->timeOut)
            dataRx->header = HEADER_U;
        }
    }

    if(dataRx->indexR!=dataRx->indexW){
        decodeHeader(dataRx);
       /* CODIGO A EFECTOS DE EVALUAR SI FUNCIONA LA RECEPCIÓN , SE DEBE DESCOMENTAR 
       Y COMENTAR LA LINEA decodeHeader(dataRx); 
       while (dataRx->indexR!=dataRx->indexW){
            dataTx->buff[dataTx->indexW++]=dataRx->buff[dataRx->indexR++];
            dataTx->indexW &= dataTx->mask;
            dataRx->indexR &= dataRx->mask;
        } */
    }
        

    if(dataTx->indexR!=dataTx->indexW){
        if(source){
             myWifi.writeWifiData(&dataTx->buff[dataTx->indexR++],1); 
             dataTx->indexR &=dataTx->mask; 
        }else{
            if(PC.writeable()){
                PC.putc(dataTx->buff[dataTx->indexR++]);
                dataTx->indexR &=dataTx->mask;
            }
        }
    }

}

void onRxData()
{
    while(PC.readable()){
        dataRx.buff[dataRx.indexW++]=PC.getc();
        dataRx.indexW &= dataRx.mask;
    }
}

uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength)
{
    dataTx->chk = 0;
    dataTx->buff[dataTx->indexW++]='U';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='N';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='E';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='R';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=frameLength+1;
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=':';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=ID;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= (frameLength+1);
    dataTx->chk ^= ('U' ^'N' ^'E' ^'R' ^ID ^':') ;
    return  dataTx->chk;
}

uint8_t putByteOnTx(_sTx *dataTx, uint8_t byte)
{
    dataTx->buff[dataTx->indexW++]=byte;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= byte;
    return dataTx->chk;
}

uint8_t putStrOntx(_sTx *dataTx, const char *str)
{
    globalIndex=0;
    while(str[globalIndex]){
        dataTx->buff[dataTx->indexW++]=str[globalIndex];
        dataTx->indexW &= dataTx->mask;
        dataTx->chk ^= str[globalIndex++];
    }
    return dataTx->chk ;
}

uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos){
    uint8_t getByte;
    dataRx->indexData += iniPos;
    dataRx->indexData &=dataRx->mask;
    getByte = dataRx->buff[dataRx->indexData];
    dataRx->indexData += finalPos;
    dataRx->indexData &=dataRx->mask;
    return getByte;
}

//Fuciones de resolucion
bool axisRotation(uint16_t angle, bool sentido){
    #define DISTANCIAENTREJES 415
    #define PERIMETRORUEDA    210

    uint32_t auxCountGiro;
    static uint32_t distanciaFinal;
    static uint32_t distanciaRecorrida;

    if (!COUNTGIROON)
    {
        COUNTGIROON = 1;
        distanciaFinal = (abs((DISTANCIAENTREJES*angle)/360));
        return false;
    }
    
    auxCountGiro = countRightValueGiro*18;
    distanciaRecorrida = (abs((PERIMETRORUEDA*auxCountGiro)/360));
    
    if (distanciaRecorrida >= distanciaFinal)
    {
        ControlDeMotores(PARAR, 0, 0);
        auxCountGiro = 0;
        distanciaRecorrida = 0;
        distanciaFinal = 0;
        countRightValueGiro = 0;
        COUNTGIROON = 0;
        
        return true;

    }else{

        if (sentido)
        {
            ControlDeMotores(DERECHA_SOBRE_EJE,30,30);
        }else{
            ControlDeMotores(IZQUIERDA_SOBRE_EJE,30,30);
        }

        return false;
    }
}

void wallFollow(){

    int16_t velocidadIzquierda;
    int16_t velocidadDerecha;

    if (distanciaEnCm > 30)
    {
        ControlDeMotores(PARAR, 0,0);
        SENSORLIBRE = 1;
        
    }else{
 
        int8_t Kp = 8;

        // 2. Calcular el error
        int8_t error = 10 - distanciaEnCm;
 
        // 3. Aplicar el control proporcional
        int16_t correccion = (Kp * error)/10;

        // 4. Ajustar la velocidad de las ruedas
        if (robotPosicion == CUADRADOEXTERNO)
        {
            velocidadIzquierda = 15 + correccion;
            velocidadDerecha = 15 - correccion;
        }else{
            velocidadIzquierda = 15 - correccion;
            velocidadDerecha = 15 + correccion;
        }
        
        if (velocidadIzquierda > 40)
        {
            velocidadIzquierda = 40;
        }else if (velocidadDerecha > 40)
        {
            velocidadDerecha = 40;
        }

        if (velocidadIzquierda < 0)
        {
            velocidadIzquierda = 0;
        }

        if (velocidadDerecha < 0)
        {
            velocidadDerecha = 0;
        }
        
        // 6. Actualizar la velocidad de las ruedas
        ControlDeMotores(ADELANTE, (uint8_t)velocidadDerecha, (uint8_t)velocidadIzquierda);
    }
    
}

void loadNewPath(){
    static uint32_t  initialValueTime, finalValueTime, TimeValue;
    
    if (MARCAFINAL)
    {
        finalValueTime=myTimer.read_us();
        if (finalValueTime>=initialValueTime){
            TimeValue = finalValueTime - initialValueTime;
        }else{
            TimeValue = 0xFFFFFFFF; //Tiempo no alcanzado
        }
        completedRoads[countPath] = TimeValue;
        MARCAFINAL = 0;
        countPath = 0;
        
    }else{
        initialValueTime=myTimer.read_us();
    }
  
}

bool allWhite(){
    return (irSensor[0].currentValue < 9000 && irSensor[1].currentValue < 9000 && irSensor[2].currentValue < 9000);
}

bool allBlack(){
    return (irSensor[0].currentValue > 20000 && irSensor[1].currentValue > 20000 && irSensor[2].currentValue > 20000);
}

bool somethingBlack(){
    return (irSensor[0].currentValue > 20000 || irSensor[1].currentValue > 20000 || irSensor[2].currentValue > 20000 
            || (irSensor[0].currentValue > 20000 && irSensor[1].currentValue > 20000) 
            || (irSensor[1].currentValue > 20000 && irSensor[2].currentValue > 20000));
}

void completedRoadsTask(){
    putHeaderOnTx(&dataTx, STATUSCAMINOS, 17);
    myWord.ui32 =  completedRoads[0];
    putByteOnTx(&dataTx, myWord.ui8[0] );
    putByteOnTx(&dataTx, myWord.ui8[1] );
    putByteOnTx(&dataTx, myWord.ui8[2] );
    putByteOnTx(&dataTx, myWord.ui8[3] );

    myWord.ui32 =  completedRoads[1];
    putByteOnTx(&dataTx, myWord.ui8[0] );
    putByteOnTx(&dataTx, myWord.ui8[1] );
    putByteOnTx(&dataTx, myWord.ui8[2] );
    putByteOnTx(&dataTx, myWord.ui8[3] );

    myWord.ui32 =  completedRoads[2];
    putByteOnTx(&dataTx, myWord.ui8[0] );
    putByteOnTx(&dataTx, myWord.ui8[1] );
    putByteOnTx(&dataTx, myWord.ui8[2] );
    putByteOnTx(&dataTx, myWord.ui8[3] );

    myWord.ui32 =  completedRoads[3];
    putByteOnTx(&dataTx, myWord.ui8[0] );
    putByteOnTx(&dataTx, myWord.ui8[1] );
    putByteOnTx(&dataTx, myWord.ui8[2] );
    putByteOnTx(&dataTx, myWord.ui8[3] );

    putByteOnTx(&dataTx, dataTx.chk);

    putHeaderOnTx(&dataTx, NROCAMINO, 2);
    putByteOnTx(&dataTx, countPath);
    putByteOnTx(&dataTx, dataTx.chk);
    

}

void updateDataQT(){
    static int32_t timeUpdateQT=0;
    static uint8_t index=0;
    #define INTERVALUPDATE 100
    if ((myTimer.read_ms()-timeUpdateQT)>=INTERVALUPDATE){
            timeUpdateQT=myTimer.read_ms(); 
            switch(index){
                case 0:
                    myWord.ui16[0] =  irSensor[0].currentValue;
                    putHeaderOnTx(&wifiTx, ANALOGSENSORS, 7);
                    putByteOnTx(&wifiTx, myWord.ui8[0] );
                    putByteOnTx(&wifiTx, myWord.ui8[1] );
                    myWord.ui16[0] =  irSensor[1].currentValue;
                    putByteOnTx(&wifiTx, myWord.ui8[0] );
                    putByteOnTx(&wifiTx, myWord.ui8[1] );
                    myWord.ui16[0] =  irSensor[2].currentValue;
                    putByteOnTx(&wifiTx, myWord.ui8[0] );
                    putByteOnTx(&wifiTx, myWord.ui8[1] ); 
                    putByteOnTx(&wifiTx, wifiTx.chk);
                break;
                case 1:
                    myWord.ui32 = distancia_us;
                    putHeaderOnTx(&wifiTx, GETDISTANCE, 5);
                    putByteOnTx(&wifiTx, myWord.ui8[0] );
                    putByteOnTx(&wifiTx, myWord.ui8[1] );
                    putByteOnTx(&wifiTx, myWord.ui8[2] );
                    putByteOnTx(&wifiTx, myWord.ui8[3] ); 
                    putByteOnTx(&wifiTx, wifiTx.chk);
                break;
                case 2:
                    myWord.ui32 = speedleftValue;
                    putHeaderOnTx(&wifiTx, GETSPEED, 9);
                    putByteOnTx(&wifiTx, myWord.ui8[0] );
                    putByteOnTx(&wifiTx, myWord.ui8[1] );
                    putByteOnTx(&wifiTx, myWord.ui8[2] );
                    putByteOnTx(&wifiTx, myWord.ui8[3] ); 
                    myWord.ui32 = speedRightValue;
                    putByteOnTx(&wifiTx, myWord.ui8[0] );
                    putByteOnTx(&wifiTx, myWord.ui8[1] );
                    putByteOnTx(&wifiTx, myWord.ui8[2] );
                    putByteOnTx(&wifiTx, myWord.ui8[3] ); 
                    putByteOnTx(&wifiTx, wifiTx.chk); 
                break;
                case 3:
                    putHeaderOnTx(&wifiTx, STATUSROBOT, 2);
                    putByteOnTx(&wifiTx, promotionRoutineMode);
                    putByteOnTx(&wifiTx, wifiTx.chk);
                break;
                default:
                    index=255;
            }
            index++;
            index &=(-(index<=3));
    } 
}

//Manejo de estados y resolucion de la consigna

void robotWorkout(){
    switch (robotMode)
    {
    case MODEPROMOCION:
        promotionRoutine();
        break;
    
    case MODESEGUIRLINEA:
        SeguirLinea();
        break;
    
    default:
        //IDLE
        ControlDeMotores(PARAR,0,0);
        break;
    }
}

void promotionRoutine(){

    switch (promotionRoutineMode)
    {
    case GIROINICIAL:
        if (axisRotation(360,ROTACIONDER))
        {
            promotionRoutineMode = BUSCARCIRCULO;
            robotPosicion = CIRCULO;
        }
        break;
    
    case BUSCARCIRCULO:
        if (somethingBlack())
        {
            if (axisRotation(90,ROTACIONIZQ))
            {
                ControlDeMotores(PARAR,0,0);
                ControlServo(0); //Preparo el sensor para el proximo estado
                promotionRoutineMode = BUSCARPARED;
                robotPosicion = CIRCULO;
                ISXSEGUNDOS = 1;          //activo la espera
                timeOutXSegundos = 100; //Time out contador de 10 segundos de vuelta por el ciculo
            } 
        }else{
            ControlDeMotores(ADELANTE,15,15);
        }
        
        break;

    case BUSCARPARED:
        //Duracion de vuelta 10 segundos - datos QT
            if ((distanciaEnCm <= (mejorDistancia+2)) && !ISXSEGUNDOS)
            {
                ControlDeMotores(PARAR,0,0);
                ControlServo(90); //Preparo el sensor para el proximo estado
                promotionRoutineMode = BUSCARPUERTA;
                robotPosicion = CIRCULO;
                ISXSEGUNDOS = 1;          //activo la espera
                timeOutXSegundos = 5; //Time out contador le doy tiempo al servo para acomodarse
            }else{
                SeguirLinea();
                if (distanciaEnCm < mejorDistancia) {
                    mejorDistancia = distanciaEnCm;
                }
            }   
        break;

    case BUSCARPUERTA:
        if (!ISXSEGUNDOS)
        {
            if ((distanciaEnCm > 50))
                { 
                    ControlServo(90); //Preparo el sensor para el proximo estado
                    ControlDeMotores(PARAR,0,0);  
                    ROTACIONFINALIZADA = 0;
                    promotionRoutineMode = SALIRDELCIRCULO;
                    robotPosicion = CIRCULO;

                }else{
                    ControlDeMotores(PARAR,0,0);
                    ControlServo(0); //Preparo el sensor para el proximo estado
                    promotionRoutineMode = BUSCARPARED;
                    robotPosicion = CIRCULO;
                    ISXSEGUNDOS = 1;          //activo la espera
                    timeOutXSegundos = 5; //Time out contador se entiende que ya estamos en el circulo
                }
        }    
        break;

    case SALIRDELCIRCULO:
        if ((distanciaEnCm > 3 && distanciaEnCm < 8) && ROTACIONFINALIZADA)
        {
            ControlDeMotores(PARAR,0,0);
            ControlServo(0); //Preparo el sensor para el proximo estado
            ROTACIONFINALIZADA = 0;
            promotionRoutineMode = BUSCARLINEA_INTOEX;
            robotPosicion = CUADRADO;
        }else{
            if (!ROTACIONFINALIZADA)
            {
                if (axisRotation(70,ROTACIONDER))
                {
                    ROTACIONFINALIZADA = 1;
                    ControlDeMotores(ADELANTE,15,15);
                }
                
            }
        }
        break;

    case BUSCARLINEA_INTOEX:

        if (somethingBlack())
        {
            SeguirLinea();
            promotionRoutineMode = LINEAENCONTRADA;
            robotPosicion = CUADRADO;
        }else{
            if (!ROTACIONFINALIZADA)
            {
                if (axisRotation(90,ROTACIONIZQ))
                {
                        ROTACIONFINALIZADA = 1;
                }
                    
            }
        }
    
        break;
        
    case LINEAENCONTRADA:
        if (allBlack()){
            promotionRoutineMode = CONTARBLANCOS;
            //MARCAINICIAL = 0;
            SENSORLIBRE = 0;
        }else{
            SeguirLinea();
        }
        break;

    case CONTARBLANCOS:
        if (!SENSORLIBRE)
        {
            wallFollow();
        }else{
            SeguirLinea();
            
            if (robotPosicion == PISTAEXTERIOR)
            {
                promotionRoutineMode = CAMINOCOMPLETADO;
                ControlServo(90); //Preparo el sensor para el proximo estado
                ISXSEGUNDOS = 1;          //activo la espera
                timeOutXSegundos = 5; //Time out contador se entiende que ya estamos en el circulo

            }else{
                promotionRoutineMode = LINEAENCONTRADA;
                robotPosicion = PISTAEXTERIOR;
            }   
        }

        if (allBlack())
        {
            if (MARCAINICIAL)
            {
                MARCAFINAL = 1;
                
                //loadNewPath();
                //completedRoadsTask();
                //MARCAINICIAL = 0;
                //SENSORLIBRE = 0

            }

            CONTADORHABILITADO = 1;
            
        }else if (somethingBlack() && MARCAINICIAL)
        {
            CONTADORHABILITADO = 1;

        }else if (allWhite())
        {
            if (CONTADORHABILITADO)
            {
                CONTADORHABILITADO = 0;
                MARCAINICIAL = 1;
                countPath++;
                completedRoads[0] = countPath;
            }
            
        }

        
        break;

    case CAMINOCOMPLETADO:

        if (ENTRADACIRCULO)
        {
            promotionRoutineMode = VOLVERALCIRCULO;
            robotPosicion = CUADRADO;
            ROTACIONFINALIZADA = 0;
            ControlServo(90); //Preparo el sensor para el proximo estado
            ISXSEGUNDOS = 1;          //activo la espera
            timeOutXSegundos = 5;     //Time out contador
        }else{
            if (!ISXSEGUNDOS)
            {
                //SeguirLinea();
                promotionRoutineMode = BUSCANDOINTERSECCION;
                robotPosicion = PISTAEXTERIOR;
                ISXSEGUNDOS = 1;          //activo la espera
                timeOutXSegundos = 5;     //Time out contador 
            }else{
                ControlDeMotores(PARAR,0,0); 
            }
        }
        
        break;
    
    case BUSCANDOINTERSECCION:
        if (allWhite() && (distanciaEnCm > 3 && distanciaEnCm < 6) && axisRotation(0,ROTACIONDER)) //Reset de rotacion, anteriormente salimos en false
        {
            promotionRoutineMode = INTERSECCIONPARED_INTOEX;
            robotPosicion = CUADRADOEXTERNO;
            ROTACIONFINALIZADA = 0;
            SEGUIRLINEAHABILITADO = 0;
        }else{
            if (!ISXSEGUNDOS)
            {
                ControlDeMotores(ADELANTE,15,15);
            }else{
                SeguirLinea();
            }   
        }
    break;

    case INTERSECCIONPARED_INTOEX:

        if (ROTACIONFINALIZADA)
        {
            if(delayRead(&waitMode)){
                SEGUIRLINEAHABILITADO = 1;
            }

            if (SEGUIRLINEAHABILITADO)
            {
                SeguirLinea();
                if ((distanciaEnCm > 3 && distanciaEnCm < 12) && (!ISXSEGUNDOS)){
                    promotionRoutineMode = INTERSECCIONPARED_EXTOIN;
                    robotPosicion = CUADRADOEXTERNO;
                }
            }else{
                ControlDeMotores(PARAR, 0, 0);
            }

        }else{
            if (!ROTACIONFINALIZADA)
            {
                if (axisRotation(80,ROTACIONDER))
                {
                    ROTACIONFINALIZADA = 1;
                    ControlServo(170); //Preparo el sensor para el proximo estado
                    ISXSEGUNDOS = 1;          //activo la espera
                    timeOutXSegundos = 90; //Time out contador se entiende que ya estamos en el circulo
                }
            }
        }

        break;

    case INTERSECCIONPARED_EXTOIN:

        if (irSensor[0].currentValue > 20000 && irSensor[1].currentValue > 20000)
        {
            promotionRoutineMode = BUSCARLINEA_EXTOIN;
            robotPosicion = CUADRADOEXTERNO;
            ISXSEGUNDOS = 1;          //activo la espera
            timeOutXSegundos = 3; //Time out contador se entiende que ya estamos en el circulo
            ROTACIONFINALIZADA = 0;
            ControlDeMotores(ADELANTE,15,15);
        }else{
            wallFollow();
        }

    break;

    case BUSCARLINEA_EXTOIN:
        if (!ISXSEGUNDOS)
        {
           if (axisRotation(70,ROTACIONDER))
            {
                promotionRoutineMode = LINEAENCONTRADA;
                robotPosicion = CUADRADOEXTERNO;
                ENTRADACIRCULO = 1;
            }               
        }
        break;

    case VOLVERALCIRCULO:
        if ((distanciaEnCm > 20 && distanciaEnCm < 30) && (!ISXSEGUNDOS))
        {
            promotionRoutineMode = CIRCULOENCONTRADO;
            robotPosicion = CIRCULO;
            ControlServo(0); //Preparo el sensor para el proximo estado
        }else{
            SeguirLinea();
        }
        
        break;

    case CIRCULOENCONTRADO:
        if (!ROTACIONFINALIZADA)
            {
                if (axisRotation(110,ROTACIONDER))
                {
                    promotionRoutineMode = BUSCARPARED;
                    robotPosicion = CIRCULO;
                    ISXSEGUNDOS = 1;          //activo la espera
                    timeOutXSegundos = 100; //Time out contador de 10 segundos de vuelta por el ciculo
                    ROTACIONFINALIZADA = 1;
                    ENTRADACIRCULO = 0;
                    ControlDeMotores(PARAR,0,0);
                }  
            }
        
        break;
        
    default:
        //FESTEJOFINAL
        ControlDeMotores(ADELANTE,15,15);
        break;
    }
}

/* END Function prototypes user code ------------------------------------------*/


int main(){   
    
    dataRx.buff = (uint8_t *)buffRx;
    dataRx.indexR = 0;
    dataRx.indexW = 0;
    dataRx.header = HEADER_U;
    dataRx.mask = RXBUFSIZE - 1;

    dataTx.buff = buffTx;
    dataTx.indexR = 0;
    dataTx.indexW = 0;
    dataTx.mask = TXBUFSIZE -1;

    wifiRx.buff = wifiBuffRx;
    wifiRx.indexR = 0;
    wifiRx.indexW = 0;
    wifiRx.header = HEADER_U;
    wifiRx.mask = RXBUFSIZE - 1;

    wifiTx.buff = wifiBuffTx;
    wifiTx.indexR = 0;
    wifiTx.indexW = 0;
    wifiTx.mask = TXBUFSIZE -1;

    RESETFLAGS = 0;
    RESETFLAGSUTIL = 0;

    time100ms = 10;

/* Local variables -----------------------------------------------------------*/
    _delay_t    hearbeatTime;
    _delay_t    debounceTime;
    _delay_t    medicionTime;
    _delay_t    aliveAutoTime;

/* END Local variables -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/
    PC.baud(115200);
    myTimer.start();
    speedMLeft.period_ms(25);
    speedMRight.period_ms(25);
    servo.period_ms(20);
    trigger.write(false);
   /********** attach de interrupciones ********/
    Ticker10ms.attach_us(&On10ms, 10000);

    PC.attach(&onRxData, SerialBase::IrqType::RxIrq);

    speedLeft.rise(&speedCountLeft);

    speedRight.rise(&speedCountRight);

    //ECHO del sensor HC-SR04
    echo.rise(&ECHOInt);
    echo.fall(&ECHOInt);

    //ULTRASONICO
    ECHORISE = 1;
    timeOutDistancia = 2;
     /********** FIN - attach de interrupciones ********/
    speedMLeft.pulsewidth_us(0);
    speedMRight.pulsewidth_us(0);
    
    delayConfig(&hearbeatTime, HEARBEATIME);
    delayConfig(&debounceTime, DEBOUNCE);
    delayConfig(&generalTime,GENERALTIME);
    delayConfig(&medicionTime,DISTANCEINTERVAL);
    delayConfig(&aliveAutoTime, ALIVEAUTOINTERVAL);
    delayConfig(&waitMode, 1000);

    //delayConfig(&timeGlobal, GENERALTIME );

    myWifi.initTask();
    autoConnectWifi();
   
    startButon(&myButton);
    StartHeratBeat(&controlDeModos);
    ControlServo(90);

    while(1){
        myWifi.taskWifi();
        hearbeatTask(&hearbeatTime,controlDeModos.mask);
        serialTask((_sRx *)&dataRx,&dataTx, SERIE);
        serialTask(&wifiRx,&wifiTx, WIFI);
        buttonTask(&debounceTime,&myButton, pulsadores.read(),&controlDeModos);
        speedTask();
        irSensorsTask();
        aliveAutoTask(&aliveAutoTime); 
        
        if (FLAGREALTIME)
        {
            updateDataQT();
        }
        
        if(IS100MS){
            Do100ms();
        }

        if (IS10MS)
        {
            IS10MS = 0;
            robotWorkout();
        }
        
    }

/* END User code -------------------------------------------------------------*/
}

