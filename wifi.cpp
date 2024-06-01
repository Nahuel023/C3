/*=============================================================================
 * Copyright (c) 2021, Alejandro Rougier <alejandro.rougier@uner.edu.ar>
 * 				
 * All rights reserved.
 * License: Free
 * Date: 2021/11/12
 * Version: v1.0
 *===========================================================================*/

/*==================[ Inclusions ]============================================*/

#include "wifi.h"
/*==================[ Local MAcros ]============================================*/
#define STARTUPTIME     10000
#define TIMETOCHECK     8000
#define RESETTIME       500

static uint16_t delayrespuesta=10;
#define DELAYRESPONSE  delayrespuesta

/*==================[ Local variables ]============================================*/
uint8_t *buffRxw;                    //!< Puntero local al bufer circular de recepción
uint8_t *indexRxWrite;              //!< Puntero local al indice de escritura del bufer circular de recepción
static uint32_t maxBufferLength;    //!< Variable local tamaño del bufer circular de recepción
static wifiData *dataConfigwifi;    //!< Puntero local a los datos de configuración
static bool configActive=false;     //!< Flag de configuración activa
static bool startUpActive=true;     //!< Flag de inicio de chequeo del ESP
static uint8_t numTimeSend, numTimeRecive;
/*==================[ Local Prototypes ]============================================*/
/**
 * @brief Función que se  llama cuando ocurre la IRQ_Rx
 * 
 */
static void onDataRx();

/*==================[ Local typedef ]============================================*/

/**
 * @brief Estructura para manejar la trasnmisión, recepción de datos del ESP
 * 
 */
typedef struct{
    uint8_t estado;           //!< Indica cual es el estado de la transmisión durante la configuración 
    uint8_t indexWriteRx;    //!< Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     //!< Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    //!< Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     //!< Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[256];   //!< Buffer circular de recepción
    uint8_t bufferTx[256];   //!< Buffer circular de transmisión
}_sDatoConfig ;

static _sDatoConfig esp8266Data;

/**
 * @brief Enumeración para la MEF de configuración del ESP
 * 
 */
typedef enum{
			CWMODE_DEF,
			CWDHCP_DEF,
			CWJAP_DEF,
            CIPMUX,
			CIFSR,
			CIPSTART,
			CIPMODE,
			CIPSEND,
			AWAITINGRESPONSE,
			INCOMMINGRESPONSE,
            READYTOTRASMIT,
			AUTOMATIC,
            ALIVE=0xF0
} _eEstadoESP;

static _eEstadoESP espState=CWMODE_DEF;

/**
 * @brief Enumeración de la MEF de las tareas comunes de la clase Wifi
 * 
 */
typedef enum{
        RESETWIFI,
        STARTUP,
        STANBY,
        CONFIG,
        READY
}_eStateTask;

static _eStateTask wifiTaskState;

/*==================[ other Variables ]============================================*/
Timer timerWifi;

uint32_t timeWifi=0;

uint32_t timestartUp=0;

uint32_t timerReset=0;

DigitalOut chipEnableESP(PA_3);

RawSerial wifiCom(PB_10,PB_11,115200);

/*==================[ Public Methods ]============================================*/

Wifi::Wifi(uint8_t *buff, uint8_t *indexWRx, uint32_t lengthBuff)
{
    buffRxw=buff;
    indexRxWrite=indexWRx;
    maxBufferLength=lengthBuff;
    esp8266Data.indexReadRx=esp8266Data.indexReadTx=esp8266Data.indexWriteRx=esp8266Data.indexWriteTx=0;
    wifiTaskState=RESETWIFI;
    numTimeSend=numTimeRecive=0;
}

Wifi::~Wifi()
{

}


void Wifi::configWifi(wifiData *dataconfig){
    configActive=true;
    dataConfigwifi=dataconfig;
    esp8266Data.estado=READYTOTRASMIT;
    wifiReady=false;
}

void Wifi::writeWifiData(uint8_t *buff, uint8_t nBytes){
    for(uint8_t i=0; i<nBytes; i++)
        esp8266Data.bufferTx[esp8266Data.indexWriteTx++]=buff[i];
}


void Wifi::taskWifi(){
    
    switch (wifiTaskState)
    {
    case RESETWIFI:
    if((timerWifi.read_us()-timerReset)>=RESETTIME){
        timerReset=timerWifi.read_us();
        if(chipEnableESP.read()){
            chipEnableESP.write(false);   
        }else{
            chipEnableESP.write(true);
            espState=CWMODE_DEF;
            wifiTaskState=STARTUP;
            numTimeSend=numTimeRecive=0;
            timeWifi=timerWifi.read_ms();
            timestartUp=timerWifi.read_ms();
        }
    }   
    break;
    case STARTUP:
        if((timerWifi.read_ms()-timeWifi)>=STARTUPTIME){
            startUpActive=false;
            wifiTaskState=STANBY;
        }
        if((timerWifi.read_ms()-timestartUp)>=TIMETOCHECK){
            if(wifiResponse("GOT IP\0", false)){
                espState=CIPMUX; 
                startUpActive=false;
                wifiTaskState=STANBY;
            }
        }
       
    break;
    case STANBY:
        if(configActive)
            wifiTaskState=CONFIG;
        break;
    case CONFIG:
        if(esp8266Data.indexReadTx!=esp8266Data.indexWriteTx)
            wifiSend();

        if(esp8266Data.estado==AWAITINGRESPONSE){
            if((timerWifi.read_ms()-timeWifi)>=DELAYRESPONSE){
                timeWifi=timerWifi.read_ms();
                configWifiMef(dataConfigwifi);
            }
        }else{
            configWifiMef(dataConfigwifi);
        }
        break;
    case READY:
        if(esp8266Data.indexReadTx!=esp8266Data.indexWriteTx)
            wifiSend();
        break;
    default:
        break;
    }
}

uint8_t Wifi::isWifiReady(){
    return wifiReady;
}

void Wifi::initTask(){
    chipEnableESP.write(true);
    wifiCom.attach (&onDataRx, RawSerial::RxIrq);
    timerWifi.start();
    timestartUp=timerWifi.read_ms();
    timerReset=timerWifi.read_us();
}

void Wifi::resetWifi(){
    wifiTaskState=RESETWIFI;
}
/*==================[ Private c Methods ]============================================*/

void Wifi::wifiSend(){
    if(wifiCom.writeable())
        wifiCom.putc(esp8266Data.bufferTx[esp8266Data.indexReadTx++]);
}

void Wifi::configWifiMef(wifiData *parameters){
    
    switch (espState)
    {
    case CWMODE_DEF:
        if(esp8266Data.estado==READYTOTRASMIT){
            for(uint8_t i=0; i < (sizeof(parameters->cwmode));i++){
                esp8266Data.bufferTx[esp8266Data.indexWriteTx++]=parameters->cwmode[i];
                if(parameters->cwmode[i]=='\n')
                    break;
            }
            esp8266Data.estado=AWAITINGRESPONSE;
            numTimeSend++;
        }else{
            if (wifiResponse("OK\0",false)){
                numTimeRecive=numTimeSend=0;
                esp8266Data.estado=READYTOTRASMIT;
                espState=CWDHCP_DEF;
            }else{
                numTimeRecive++;
            }
        }
        break;
    case CWDHCP_DEF:
        if(esp8266Data.estado==READYTOTRASMIT){
                for(uint8_t i=0; i < (sizeof(parameters->cwdhcp));i++){
                    esp8266Data.bufferTx[esp8266Data.indexWriteTx++]=parameters->cwdhcp[i];
                    if(parameters->cwdhcp[i]=='\n')
                        break;
                }
                esp8266Data.estado=AWAITINGRESPONSE;
                 numTimeSend++;
        }else{
            if (wifiResponse("OK\0",false)){
                 numTimeRecive=numTimeSend=0;
                esp8266Data.estado=READYTOTRASMIT;
                espState=CWJAP_DEF;
            }else{
                numTimeRecive++;
            }
            
        }
    break;
    case CWJAP_DEF:
        if(esp8266Data.estado==READYTOTRASMIT){
                for(uint8_t i=0; i < (sizeof(parameters->cwjap));i++){
                    esp8266Data.bufferTx[esp8266Data.indexWriteTx++]=parameters->cwjap[i];
                    if(parameters->cwjap[i]=='\n')
                        break;
                }
                esp8266Data.estado=AWAITINGRESPONSE;
                DELAYRESPONSE=5000;
                numTimeSend++;
        }else{
            if (wifiResponse("OK\0",false)){
                numTimeRecive=numTimeSend=0;
                esp8266Data.estado=READYTOTRASMIT;
                espState=CIPMUX;
                DELAYRESPONSE=20;
            }else{
                numTimeRecive++;
            }
        }
        break;
    case CIPMUX:
        if(esp8266Data.estado==READYTOTRASMIT){
                for(uint8_t i=0; i < (sizeof(parameters->cipmux));i++){
                    esp8266Data.bufferTx[esp8266Data.indexWriteTx++]=parameters->cipmux[i];
                    if(parameters->cipmux[i]=='\n')
                        break;
                }
                esp8266Data.estado=AWAITINGRESPONSE;
                numTimeSend++;
        }else{
            if (wifiResponse("OK\0",false)){
                numTimeRecive=numTimeSend=0;
                esp8266Data.estado=READYTOTRASMIT;
                espState=CIPSTART;
            }else{
                numTimeRecive++;
            }
        }
        break;
    case CIPSTART:
        if(esp8266Data.estado==READYTOTRASMIT){
                for(uint8_t i=0; i < (sizeof(parameters->cipstart));i++){
                    esp8266Data.bufferTx[esp8266Data.indexWriteTx++]=parameters->cipstart[i];
                    if(parameters->cipstart[i]=='\n')
                        break;
                }
                esp8266Data.estado=AWAITINGRESPONSE;
                DELAYRESPONSE=1500;
                numTimeSend++;
        }else{
            if (wifiResponse("OK\0",false)){
                numTimeRecive=numTimeSend=0;
                DELAYRESPONSE=15;
                esp8266Data.estado=READYTOTRASMIT;
                espState=CIPMODE;
            }else{
                numTimeRecive++;
            }
        }
        break;
    case CIPMODE:
        if(esp8266Data.estado==READYTOTRASMIT){
                for(uint8_t i=0; i < (sizeof(parameters->cipmode));i++){
                    esp8266Data.bufferTx[esp8266Data.indexWriteTx++]=parameters->cipmode[i];
                    if(parameters->cipmode[i]=='\n')
                        break;
                }
                esp8266Data.estado=AWAITINGRESPONSE;
                 numTimeSend++;
        }else{
            if (wifiResponse("OK\0",false)){
                numTimeRecive=numTimeSend=0;
                esp8266Data.estado=READYTOTRASMIT;
                espState=CIPSEND;
            }else{
                numTimeRecive++;
            }
        }

        break;
    case CIPSEND:
        if(esp8266Data.estado==READYTOTRASMIT){
                for(uint8_t i=0; i < (sizeof(parameters->cipsend));i++){
                    esp8266Data.bufferTx[esp8266Data.indexWriteTx++]=parameters->cipsend[i];
                    if(parameters->cipsend[i]=='\n')
                        break;
                }
                esp8266Data.estado=AWAITINGRESPONSE;
                 numTimeSend++;
        }else{
            if ( (wifiResponse("OK\0",false)) || (wifiResponse(">\0",false))){ 
                numTimeRecive=numTimeSend=0;
                esp8266Data.estado=READYTOTRASMIT;
                espState=AUTOMATIC;
            }else{
                numTimeRecive++;
            }
        }
        break;
    case AUTOMATIC:
        wifiTaskState=READY;
        configActive=false;
        wifiReady=true;
        break;
    default:
        break;
    }
    if(numTimeSend>3){
       resetWifi();
    }
    if(numTimeRecive>3){
      numTimeRecive=0;
      esp8266Data.estado=READYTOTRASMIT;
    }
}


bool Wifi::wifiResponse(const char *subcadena, unsigned char stopSearch){
    uint8_t indexSub=0, respuesta=false;
    while(esp8266Data.indexReadRx!=esp8266Data.indexWriteRx){
        while(esp8266Data.bufferRx[esp8266Data.indexReadRx]==subcadena[indexSub]){
            indexSub++;
            if(subcadena[indexSub]=='\0'){
                    respuesta=true;
            }
        }
        if(respuesta==true && stopSearch==true){
            break;
        }else{
            esp8266Data.indexReadRx++;
        }
    }
    return respuesta;        
}

/*==================[ others Methods ]============================================*/
static void onDataRx(){
    while (wifiCom.readable())
    {
        if(configActive || startUpActive){
            esp8266Data.bufferRx[esp8266Data.indexWriteRx++]=wifiCom.getc();
        }
        else{
            buffRxw[*indexRxWrite]=wifiCom.getc();
            *indexRxWrite=*indexRxWrite+1;
            if (*indexRxWrite > maxBufferLength)
                *indexRxWrite=0;
        }
    }
}
