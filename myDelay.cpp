/*! \mainpage Ejercicio Titulo
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
 * | 01/01/2023 | Creacion del documento                         |
 *
 */



/* Includes ------------------------------------------------------------------*/
#include "myDelay.h"
/* END Includes --------------------------------------------------------------*/


/* typedef -------------------------------------------------------------------*/

/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* END define ----------------------------------------------------------------*/

/* hardware configuration ----------------------------------------------------*/


/* END hardware configuration ------------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/

/* END Function prototypes ---------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/
extern Timer myTimer;
/* END Global variables ------------------------------------------------------*/

/* Function prototypes user code ----------------------------------------------*/


void delayConfig(_delay_t *delay, uint16_t  interval){
    delay->interval =interval;
    delay->isRunnig = false;
}

uint8_t delayRead(_delay_t *delay){
    uint8_t timeReach=false;
    if(!delay->isRunnig){
        delay->isRunnig = true;
        delay->startTime = myTimer.read_ms();
    }else{
        if((myTimer.read_ms()-delay->startTime)>=delay->interval){
            timeReach = true;
            delay->isRunnig = false;
        }
    }
    return timeReach;
}
void delayWrite(_delay_t *delay, uint16_t interval){
    delay->interval = interval;
    delay->isRunnig = false;
}

/* END Function prototypes user code ------------------------------------------*/