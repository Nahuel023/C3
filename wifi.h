/*=============================================================================
 * Copyright (c) 2021, Alejandro Rougier <alejandro.rougier@uner.edu.ar>
 * 				
 * All rights reserved.
 * License: Free
 * Date: 2021/11/12
 * Version: v1.0
 *===========================================================================*/

/*==================[ Inclusions ]============================================*/

#ifndef WIFI_H
#define WIFI_H

#include "mbed.h"

/*==================[ Global Variables ]============================================*/
#pragma pack(1)
    typedef struct 
    {
        uint8_t cwmode[17];     //!< AT+CWMODE_DEF=<mode> 1,2,3 - Ap, softStation - Ambos modos
        uint8_t cwdhcp[19];     //!< AT+CWDHCP_DEF=<<mode>,<en>
        uint8_t cwjap[100];     //!< AT+CWJAP_DEF=<ssid>,<pwd>[,<bssid>] 
        uint8_t cipmux[13];     //!< AT+CIPMUX=<mode>
        uint8_t cipstart[58];   //!< AT+CIPSTART=<type>,<remote IP>,<remote port>[(<UDP local port>),(<UDP mode>)] Single connection (AT+CIPMUX=0): 
        uint8_t cipmode[14];    //!< AT+CIPMODE=<mode> 0,1	-modo normal	- 	modo transparente(Wifi pasThrue)
        uint8_t cipsend[12];    //!< AT+CIPSEND - 	modo transparente(Wifi pasThrue)
    } wifiData;
#pragma pack(0)

/*==================[ Class Definitions ]============================================*/
class Wifi
{
    public:
        /**
         * @brief Construct a new Wifi object
         * 
         * @param buff          Puntero al buffer circular de recepción 
         * @param indexWRx      Puntero al indice de escritura del buffer circular de recepción
         * @param lengthBuff    Tamaño del buffer 
         */
         Wifi(uint8_t *buff, uint8_t *indexWRx, uint32_t lengthBuff);
        /**
         * @brief Destroy the Wifi object
         * 
         */
        ~Wifi();
        /**
         * @brief Configura el ESP8266 para su funcionamiento
         * 
         * @param puntero a wifiData : se pasan los parámetros de configuración mediante la estructura wifiData
         */
        void configWifi(wifiData *);
        /**
         * @brief  Escribe los datos para enviar por wifi en el buffer de transmisión
         * 
         * @param buff      Puntero al buffer que contiene los datos para ser enviados por wifi
         * @param nBytes    Cantidad de datos que se quieren enviar
         */
        void writeWifiData(uint8_t *buff, uint8_t nBytes);
        /**
         * @brief Tareas períodicas que ejecuta la clase
         * 
         */
        void taskWifi();
        /**
         * @brief Consulta si el dispositivo Wifi está listo 
         * 
         * @return true 
         * @return false 
         */
        uint8_t isWifiReady();
        /**
         * @brief Iniciliza las tareas comunes
         * 
         */
        void initTask();
        /**
         * @brief Resetea el Wifi para comenzar nuevamente a cargar los datos de conexion
         * 
         */
        void resetWifi();
    private:
        uint8_t wifiReady=false;
        /**
         * @brief Envía los datos a travéz del ESP
         * 
         */
        void wifiSend();
        /**
         * @brief   MEF para configurar el Wifi
         * 
         * @param puntero a wifiData : se pasan los parámetros de configuración mediante la estructura wifiData
         */
        void configWifiMef(wifiData *);   
        /**
         * @brief   Busca la cadena dentro del buffer de recepción
         * 
         *  @param puntero a la cadena constante de char que se quiere buscar
         * 
         * @return true 
         * @return false 
         */
        bool wifiResponse(const char *, unsigned char );
};
#endif