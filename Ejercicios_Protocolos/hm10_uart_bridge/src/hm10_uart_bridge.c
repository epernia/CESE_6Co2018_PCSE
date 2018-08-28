/* Copyright 2018, Eric Pernia y Lucas Dórdolo
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
/*==================[inlcusiones]============================================*/

#include "sapi.h"        // <= Biblioteca sAPI
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define  MAXI 100
/*==================[definiciones y macros]==================================*/

#define UART_PC        UART_USB
#define UART_BLUETOOTH UART_232

/*==================[definiciones de datos internos]=========================*/
// MPU9250 Address
MPU9250_address_t addr = MPU9250_ADDRESS_0; // If MPU9250 AD0 pin is connected to GND
/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

bool_t hm10bleTest( int32_t uart );
void hm10blePrintATCommands( int32_t uart );
void hm10blePrintFloat(float numero, char *etiqueta, uint8_t longitud);
/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void )
{
   // ---------- CONFIGURACIONES ------------------------------
	uint8_t medicion = 1;
   // Inicializar y configurar la plataforma
   boardConfig();

   // Inicializar UART_USB para conectar a la PC
   uartConfig( UART_PC, 9600 );
   uartWriteString( UART_PC, "UART_PC configurada.\r\n" );


   // Inicializar la IMU
   uartWriteString( UART_PC,"Inicializando IMU MPU9250...\r\n" );
   int8_t status;
   status = mpu9250Init( addr );

   // Inicializar UART_232 para conectar al modulo bluetooth
   uartConfig( UART_BLUETOOTH, 9600 );
   uartWriteString( UART_PC, "UART_BLUETOOTH para modulo Bluetooth configurada.\r\n" );
   
   uint8_t data = 0;

   uartWriteString( UART_PC, "Testeto si el modulo esta conectado enviando: AT\r\n" );
   if( hm10bleTest( UART_BLUETOOTH ) ){
      uartWriteString( UART_PC, "Modulo conectado correctamente.\r\n" );
   }  

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {

      // Si leo un dato de una UART lo envio a al otra (bridge)
      if( uartReadByte( UART_PC, &data ) ) {
         uartWriteByte( UART_BLUETOOTH, data );
      }
      if( uartReadByte( UART_BLUETOOTH, &data ) ) {
         if( data == 'h' ) {
            gpioWrite( LEDB, ON );
         }
         if( data == 'l' ) {
            gpioWrite( LEDB, OFF );
         }
         uartWriteByte( UART_PC, data );
      }
      
      // Si presiono TEC1 imprime la lista de comandos AT
      if( !gpioRead( TEC1 ) ) {
         hm10blePrintATCommands( UART_BLUETOOTH );
      }
      
      // Si presiono TEC3 enciende el led de la pantalla de la app
      if( !gpioRead( TEC3 ) ) {
         uartWriteString( UART_BLUETOOTH, "LED_ON\r\n" );
         delay(50);
      }
      // Si presiono TEC4 apaga el led de la pantalla de la app
      if( !gpioRead( TEC4 ) ) {
         uartWriteString( UART_BLUETOOTH, "LED_OFF\r\n" );
         delay(50);
      }

      //Leer el sensor y guardar en estructura de control
      // Imprimo un float cada 100ms (periodo de 1Seg)
      switch(medicion){
		  case 1:
		      mpu9250Read();
		      hm10blePrintFloat(mpu9250GetAccelX_mss(),"A_X:", 4);
			  break;
		  case 2:
		      hm10blePrintFloat(mpu9250GetAccelY_mss(),"A_Y:", 4);
			  break;
		  case 3:
		      hm10blePrintFloat(mpu9250GetAccelZ_mss(),"A_Z:", 4);
			  break;
		  case 4:
		      hm10blePrintFloat(mpu9250GetGyroX_rads(),"G_X:", 4);
			  break;
		  case 5:
		      hm10blePrintFloat(mpu9250GetGyroX_rads(),"G_Y:", 4);
			  break;
		  case 6:
		      hm10blePrintFloat(mpu9250GetGyroX_rads(),"G_Z:", 4);
			  break;
		  case 7:
		      hm10blePrintFloat(mpu9250GetMagX_uT(),"M_X:", 4);
			  break;
		  case 8:
		      hm10blePrintFloat(mpu9250GetMagY_uT(),"M_Y:", 4);
			  break;
		  case 9:
		      hm10blePrintFloat(mpu9250GetMagZ_uT(),"M_Z:", 4);
			  break;
		  case 10:
		      hm10blePrintFloat(mpu9250GetTemperature_C(),"T:", 2);
		      medicion = 0;
			  break;
      }
	  medicion++;
      delay(100);
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

bool_t hm10bleTest( int32_t uart )
{
   uartWriteString( uart, "AT\r\n" );
   return waitForReceiveStringOrTimeoutBlocking( uart, 
                                                 "OK\r\n", strlen("OK\r\n"),
                                                 50 );
}

void hm10blePrintATCommands( int32_t uart )
{
   delay(500);
   uartWriteString( uart, "AT+HELP\r\n" );
}

void hm10blePrintFloat(float numero, char *etiqueta, uint8_t longitud){

    char buf[MAXI];//Buffer para escritura
    char flot[10]; //Buffer para el float

    buf[0] = 0;
    strncat( buf, etiqueta, longitud ); // Incluyo la etiqueta
    gcvt(numero, 8, flot);				// Convierto el número en string
    strncat( buf, flot, 8 );			// Agrego el número al buffer
    strncat( buf, "\r\n", 2 );			//Finalizo el string
    uartWriteString( UART_BLUETOOTH, buf );	//Imprimo por bluetooth
    uartWriteString( UART_PC, buf );		//Imprimo por serie (Debug)
    buf[0] = 0;
}
/*==================[fin del archivo]========================================*/
