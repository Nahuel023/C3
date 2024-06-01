/*! \mainpage Archivo para generar un delay NON_BLOCKING
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

#ifndef INC_MYDELAY_H_
#define INC_MYDELAY_H_


/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
/* END Includes --------------------------------------------------------------*/


/* typedef -------------------------------------------------------------------*/

/**
 * @brief Estructura para manejar el tiempo
 * 
 */
typedef struct{
    int32_t    startTime;   /*!< Almacena el tiempo leido del timer*/
    uint16_t    interval;   /*!< intervalo de comparación para saber si ya transcurrio el tiempo leido*/
    uint8_t     isRunnig;   /*!< Indica si el delay está activo o no*/
}_delay_t;
/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/

/* END define ----------------------------------------------------------------*/

/* hardware configuration ----------------------------------------------------*/


/* END hardware configuration ------------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Configura el entervalo del Delay
 * 
 * @param delay     Estructura para manejar el tiempo
 * @param interval  intervalo de comparación para saber si ya transcurrio el tiempo leido
 */
void delayConfig(_delay_t *delay, uint16_t  interval);

/**
 * @brief Lee el Delay para determinar si se ha cumplido el tiempo establecido
 * 
 * @param delay     Estructura para manejar el tiempo
 * @return uint8_t  Returna True o False de acuerdo a si se cumplio el tiempo o no
 */
uint8_t delayRead(_delay_t *delay);

/**
 * @brief PErmite modificar el intervalo establecido para un Delay y lo resetea
 * 
 * @param delay     Estructura para manejar el tiempo
 * @param interval  intervalo de comparación para saber si ya transcurrio el tiempo leido
 */
void delayWrite(_delay_t *delay, uint16_t interval);
/* END Function prototypes ---------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/

/* END Global variables ------------------------------------------------------*/
#endif