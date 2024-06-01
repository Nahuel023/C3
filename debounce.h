/*! \mainpage Debounce para los botones
 * \date 10/09/2023
 * \author Nombre
 * \section genDesc Descripcion general
 * El archivo contiene los elementos para realizar el filtrado de los botones
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

#ifndef DEBOUNCE_H_
#define DEBOUNCE_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "myDelay.h"
#include "util.h"
/* END Includes --------------------------------------------------------------*/


/* typedef -------------------------------------------------------------------*/
/**
 * @brief DEfine los estados de la MEF del botón
 *        
 */
typedef enum{
    BUTTON_DOWN,
    BUTTON_UP,
    BUTTON_RISING,
    BUTTON_FALLING
}_eState;

/**
 * @brief DEfine el estado del botón. Pensado para configuración PULL_UP
 * 
 */
typedef enum{
    PRESSED,
    NOT_PRESSED,
    NO_EVENT
}_eEventInput;

/**
 * @brief Banderas para detectar los flancos con el Botón
 * 
 */
typedef enum{
    FALLINGFLAG,
    RISINGFLAG,
    NOFLAG,
}_eBtnFlag;

/**
 * @brief Estructura para el manejo de los botones
 * 
 */
typedef struct
{
    uint8_t         mask = 0;
    uint32_t        tiempo = 0;
    _eState         currentState;   //!< Estado del boton en la MEF de debounce
    _eEventInput    stateInput;     //!< Valor de la entrada digital del micro
    _eBtnFlag       flagDetected;   //!< Flag para detección de flanco 
    int32_t         timePressed;    //!< Toma el instante en que se presionó el botón 
    int32_t         timeDiff;       //!< Toma la diferencia entre el momento en que se presionó y el que se soltó
 }_sButton;

 typedef struct
{
    uint32_t            mask = 0x2AAAAAAA;
    uint8_t             estadoPrograma = 1;
    uint8_t             estadoHearBeat = 1;
    const uint32_t      MASK_SECUENCIA_MODO_OFF[5] = {0x01, 0x05, 0x15, 0x55};
    const uint32_t      MASK_SECUENCIA_MODO_ON[5] = {0x7D, 0x1F5, 0x7D5, 0x1F55};
    const uint32_t      MASK_SECUENCIA_BOTON_PRESIONADO[5] = {0x01, 0x05, 0x15, 0x55};
    uint8_t             flagHearBeatBotonPres = 0;
    uint8_t             flagCambiarHearBeat = 0;
    uint8_t             flagBotonPresionado = 0;
    uint8_t             flagModoActivo = 0;
    uint8_t             flagModoCambiado = 0;
 }_sHearBeat;


/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* END define ----------------------------------------------------------------*/

/* hardware configuration ----------------------------------------------------*/


/* END hardware configuration ------------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/
 void StartHeratBeat(_sHearBeat *controlDeModos);
/**
 * @brief Inicializa los botones
 * 
 * @param myButton Estructura de los botones
 * @param numButton Cantidad de botones
 */
void startButon(_sButton *myButton);

/**
 * @brief MAquina de Estado de los botones
 * 
 * @param timedebounce Estructura de datos para el debounce
 * @param myButton Estruvtura de datos de los botones
 * @param statePulsadores Estado del array de botones
 */
void buttonTask(_delay_t *timedebounce, _sButton *myButton, uint8_t statePulsadores,_sHearBeat *controlDeModos);
/* END Function prototypes ---------------------------------------------------*/
void ButtonDown(_sButton *myButton,_sHearBeat *controlDeModos);
/* Global variables ----------------------------------------------------------*/
void ButtonRising(_sButton *myButton,_sHearBeat *controlDeModos);

void ControlHearbeat(_sHearBeat *controlDeModos);

/* END Global variables ------------------------------------------------------*/

#endif