/*! \mainpage Archivo para <gregar variables/definiciones/constantes etc a un programa>
 * \date 01/01/2023
 * \author Nombre
 * \section genDesc Descripcion general
 * [Complete aqui con su descripcion]
 *
 * \section desarrollos Observaciones generales
 * [Complete aqui con sus observaciones]
 *
 * \section changelog Registro de cambios
 *
 * |   Fecha    | Descripcion                                    |
 * |:----------:|:-----------------------------------------------|
 * |10/09/2023 | Creacion del documento                         |
 *
 */


#ifndef UTIL_H_
#define UTIL_H_

/* Includes ------------------------------------------------------------------*/
#include "myDelay.h"
#include <stdlib.h>
/* END Includes --------------------------------------------------------------*/


/* typedef -------------------------------------------------------------------*/
/**
 * @brief Mapa de bits para declarar banderas
 * 
 */
typedef union{
    struct{
        uint8_t bit7 : 1;
        uint8_t bit6 : 1;
        uint8_t bit5 : 1;
        uint8_t bit4 : 1;
        uint8_t bit3 : 1;
        uint8_t bit2 : 1;
        uint8_t bit1 : 1;
        uint8_t bit0 : 1;
    }bits;
    uint8_t bytes;
}_uFlag;

/**
 * 
 * @brief Unión ara la descomposición/composición de números mayores a 1 byte
 * 
 */
typedef union{
    uint32_t    ui32;
    int32_t     i32;
    uint16_t    ui16[2];
    int16_t     i16[2];
    uint8_t     ui8[4];
    int8_t      i8[4];
}_uWord;

/**
 * @brief estructura para la recepción de datos por puerto serie
 * 
 */
typedef struct{
    uint8_t *buff;      /*!< Puntero para el buffer de recepción*/
    uint8_t indexR;     /*!< indice de lectura del buffer circular*/
    uint8_t indexW;     /*!< indice de escritura del buffer circular*/
    uint8_t indexData;  /*!< indice para identificar la posición del dato*/
    uint8_t mask;       /*!< máscara para controlar el tamaño del buffer*/
    uint8_t chk;        /*!< variable para calcular el checksum*/
    uint8_t nBytes;     /*!< variable para almacenar el número de bytes recibidos*/
    uint8_t header;     /*!< variable para mantener el estado dela MEF del protocolo*/
    uint8_t timeOut;    /*!< variable para resetear la MEF si no llegan más caracteres luego de cierto tiempo*/
    uint8_t isComannd;
}_sRx;

/**
 * @brief Estructura para la transmisión de datos por el puerto serie
 * 
 */
typedef struct{
    uint8_t *buff;      /*!< Puntero para el buffer de transmisión*/
    uint8_t indexR;     /*!<indice de lectura del buffer circular*/
    uint8_t indexW;     /*!<indice de escritura del buffer circular*/
    uint8_t mask;       /*!<máscara para controlar el tamaño del buffer*/
    uint8_t chk;        /*!< variable para calcular el checksum*/
}_sTx;


/**
 * @brief estructura para el manejo de sensores
 * 
 */
typedef struct{
    uint16_t    currentValue;
    uint16_t    currentValueTotal;
    uint16_t    maxValue;
    uint16_t    minValue;
    uint16_t    blackValue;
    uint16_t    whiteValue;
}_sSensor;


/**
 * @brief Enumeración para la maquina de estados
 * que se encarga de decodificar el protocolo
 * de comunicación
 *  
 */
typedef enum{
    HEADER_U,
    HEADER_N,
    HEADER_E,
    HEADER_R,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eDecode;


/**
 * @brief Enumeración de los comandos del protocolo
 * 
 */
typedef enum{
    ALIVE = 0xF0,
    FIRMWARE= 0xF1,
    BUTTONSTATUS = 0x12,
    ANALOGSENSORS = 0xA0,
    SETBLACKCOLOR = 0xA6,
    SETWHITECOLOR = 0xA7,
    MOTORTEST = 0xA1,
    SERVOANGLE = 0xA2,
    CONFIGSERVO = 0xA5,
    SERVOFINISHMOVE = 0x0A,
    GETDISTANCE = 0xA3,
    GETSPEED = 0xA4,
    ACK = 0x0D,
    UNKNOWN = 0xFF,
    STATUSCAMINOS = 0xA8,
    NROCAMINO = 0x11,
    STARTSTOP = 0xA9,
    UPDATEREALTIME = 0x13,
    RESETMODE = 0x04,
    STATUSROBOT = 0x05,
    CONTADOR = 88
}_eCmd;


typedef enum{
    detectarLinea = 0,
    adelante = 1,
    derechaYCentro = 2,
    izquierdaYcentro= 3,
    parar = 4,
    buscarLineaDerecha = 5,
    buscarLineaIzquierda = 6,
    rotarAuto = 7,
    derecha = 8,
    izquierda = 9
}_eModoSeguirLinea;


typedef enum{
    ADELANTE = 0,
    ATRAS = 1,
    DERECHA_SOBRE_EJE = 3,
    IZQUIERDA_SOBRE_EJE = 4,
    PARAR = 5
}_eControlMotores;

typedef enum{
    MODEPROMOCION = 1,
    MODESEGUIRLINEA = 2,
    IDLE = 3
}_eRobotMode;

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
    VOLVERALCIRCULO = 13,
    CIRCULOENCONTRADO = 14,
    FESTEJOFINAL = 15
}_eModoPromocion;

typedef enum{
    CIRCULO = 1,                      
    CUADRADO = 2,                  
    PISTAEXTERIOR = 3,
    CUADRADOEXTERNO = 4
}_eRobotPosicion;

                 


/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* END define ----------------------------------------------------------------*/

/* hardware configuration ----------------------------------------------------*/


/* END hardware configuration ------------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
/**
 * @brief Hearbeat, indica el funcionamiento del sistema
 * 
 * @param timeHearbeat Variable para el intervalo de tiempo
 * @param mask Secuencia de encendido/apagado del led de Hearbeat
 */
void hearbeatTask(_delay_t *timeHearbeat, uint32_t mask);

/**
 * @brief Ejecuta las tareas del puerto serie Decodificación/trasnmisión
 * 
 * @param dataRx Estructura de datos de la recepción
 * @param dataTx Estructura de datos de la trasnmisión
 * @param source Identifica la fuente desde donde se enviaron los datos
 */
void serialTask(_sRx *dataRx, _sTx *dataTx, uint8_t source);

/**
 * @brief Rutina para medir la velocidad
 * 
 */
void speedTask();

/**
 * @brief Rutina para hacer la medición de los sensores IR
 * 
 */
void irSensorsTask();


/**
 * @brief Recepción de datos por el puerto serie
 * 
 */
void onRxData();

/**
 * @brief Pone el encabezado del protocolo, el ID y la cantidad de bytes a enviar
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param ID Identificación del comando que se envía
 * @param frameLength Longitud de la trama del comando
 * @return uint8_t devuelve el Checksum de los datos agregados al buffer de trasnmisión
 */
uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength);

/**
 * @brief Agrega un byte al buffer de transmisión
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param byte El elemento que se quiere agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisión
 */
uint8_t putByteOnTx(_sTx    *dataTx, uint8_t byte);

/**
 * @brief Agrega un String al buffer de transmisión
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param str String a agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisión
 */
uint8_t putStrOntx(_sTx *dataTx, const char *str);

uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos);

/**
 * @brief Decodifica la trama recibida
 * 
 * @param dataRx Estructura para la recepción de datos
 */
void decodeHeader(_sRx *dataRx);

/**
 * @brief Decodifica el comando recibido en la transmisión y ejecuita las tareas asociadas a dicho comando
 * 
 * @param dataRx Estructura para la recepción de datos
 * @param dataTx Estructura para la trasnmisión de datos
 */
void decodeCommand(_sRx *dataRx, _sTx *dataTx);


/**
 * @brief Función del Sensor de Horquilla Izquierdo
 * cuenta los pulsos del sensor para medir velocidad luego
 */
void speedCountLeft(void);

/**
 * @brief Función del Sensor de Horquilla Derecho
 * cuenta los pulsos del sensor para medir velocidad luego
 */
void speedCountRight(void);


/**
 * @brief Función para realizar la autoconexión de los datos de Wifi
 * 
 */
void autoConnectWifi();

/**
 * @brief envía de manera automática el alive
 * 
 */
void aliveAutoTask(_delay_t *aliveAutoTime);

/**
 * @brief Decodifica el comando recibido en la transmisión y ejecuita las tareas asociadas a dicho comando
 * 
 * @param direccion Direccion asignada
 * @param mDerecho Velocidad motor derecho
 * @param mIzquierdo Velocidad motor izquierdo
 */
void ControlDeMotores(_eControlMotores direccion,uint16_t mDerecho, uint16_t mIzquierdo);

/**
 * @brief Calcula el modo de seguir linea seteando los valores IR
 * 
 */
void CalcuModoDelSeguirLINEA();

/**
 * @brief Setea el Control P
 * 
 */
void ControlDeMotoresSeguirLinea();

/**
 * @brief Manager principal del control de seguir linea
 * 
 */
void SeguirLinea();

/**
 * @brief Funcion para la interrupcion del ECHO
 * 
 */
void ECHOInt();

/**
 * @brief Pulse Trigger
 * 
 */
void OnTimeTriggerPulse();

/**
 * @brief Funcion que ejecuta acciones cada 100 ms
 * 
 */
void Do100ms();

/**
 * @brief Funcion de interrupcion 10 ms
 * 
 */
void On10ms();

/**
 * @brief Control de servo
 * 
 * @param grados Grados de giro
 */
void ControlServo(uint16_t grados);

/**
 * @brief Gestion de estados generales del robot
 * 
 */
void robotWorkout();

/**
 * @brief Rutina de resolucion de la consigna
 * 
 */
void promotionRoutine();

/**
 * @brief Rotacion por conteo de horquilla
 * 
 * @param angle Angulo de rotacion sobre el eje del robot
 * @param bool  Sentido de rotacion
 */
bool axisRotation(uint16_t angle, bool sentido);

/**
 * @brief Seguidor de pared proporcional
 * 
 */
void wallFollow();

/**
 * @brief Todos los sensores en blanco
 * 
 */
bool allWhite();

/**
 * @brief Todos los sensores en negro
 * 
 */
bool allBlack();

/**
 * @brief Uno o dos sensores en negro
 * 
 */
bool somethingBlack();

/**
 * @brief Camino completado 
 * 
 */
void completedRoadsTask();

/**
 * @brief Carga del nuevo camino
 * 
 */
void loadNewPath();

/**
 * @brief Envio de datos de control a QT
 * 
 */
void updateDataQT();
/* END Function prototypes ---------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/

/* END Global variables ------------------------------------------------------*/

#endif