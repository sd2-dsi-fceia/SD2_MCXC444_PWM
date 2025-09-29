/* Copyright 2017, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2025, Guido Cicconi
 * All rights reserved.
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
 *
 */

/*==================[inclusions]=============================================*/

#include "SD2_board.h"

/*==================[macros and definitions]=================================*/

typedef enum
{
	EST_MEF_LED_RGB_INICIALIZACION = 0,
	EST_MEF_LED_RGB_ETAPA_1,
	EST_MEF_LED_RGB_ETAPA_2,
	EST_MEF_LED_RGB_ETAPA_3,
}estMefLedRGB_enum;

#define MEF_LED_RGB_TIME_STEP_MS 20
#define MEF_LED_RGB_MAX_DUTY 50

/*==================[internal data declaration]==============================*/

static uint8_t timer = 0;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void mefLedRGB(void)
{
    static estMefLedRGB_enum estMefLed = EST_MEF_LED_RGB_INICIALIZACION;
    static uint8_t dutyRojo = 0, dutyVerde = 0, dutyAzul = 0;

    switch (estMefLed)
    {
        case EST_MEF_LED_RGB_INICIALIZACION:
            board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_ON);
            board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_ON);
            board_setLed(BOARD_LED_ID_AZUL, BOARD_LED_MSG_ON);
            dutyRojo = 0;
            dutyVerde = 0;
            dutyAzul = MEF_LED_RGB_MAX_DUTY;
        	board_setLedBrightness(BOARD_LED_ID_ROJO, dutyRojo);
        	board_setLedBrightness(BOARD_LED_ID_VERDE, dutyVerde);
        	board_setLedBrightness(BOARD_LED_ID_AZUL, dutyAzul);
        	timer = MEF_LED_RGB_TIME_STEP_MS;
            estMefLed = EST_MEF_LED_RGB_ETAPA_1;
            break;

        case EST_MEF_LED_RGB_ETAPA_1:
        	if(!timer)
        	{
        		timer = MEF_LED_RGB_TIME_STEP_MS;
                dutyRojo++;
                dutyAzul--;
            	board_setLedBrightness(BOARD_LED_ID_ROJO, dutyRojo);
            	board_setLedBrightness(BOARD_LED_ID_AZUL, dutyAzul);
        	}
        	if(dutyRojo == MEF_LED_RGB_MAX_DUTY && dutyAzul == 0)
        	{
        		timer = MEF_LED_RGB_TIME_STEP_MS;
        		estMefLed = EST_MEF_LED_RGB_ETAPA_2;
        	}
        	break;

        case EST_MEF_LED_RGB_ETAPA_2:
        	if(!timer)
        	{
        		timer = MEF_LED_RGB_TIME_STEP_MS;
                dutyRojo--;
                dutyVerde++;
            	board_setLedBrightness(BOARD_LED_ID_ROJO, dutyRojo);
            	board_setLedBrightness(BOARD_LED_ID_VERDE, dutyVerde);
        	}
        	if(dutyRojo == 0 && dutyVerde == MEF_LED_RGB_MAX_DUTY)
        	{
        		timer = MEF_LED_RGB_TIME_STEP_MS;
        		estMefLed = EST_MEF_LED_RGB_ETAPA_3;
        	}
        	break;

        case EST_MEF_LED_RGB_ETAPA_3:
        	if(!timer)
        	{
        		timer = MEF_LED_RGB_TIME_STEP_MS;
                dutyVerde--;
                dutyAzul++;
            	board_setLedBrightness(BOARD_LED_ID_VERDE, dutyVerde);
            	board_setLedBrightness(BOARD_LED_ID_AZUL, dutyAzul);
        	}
        	if(dutyVerde == 0 && dutyAzul == MEF_LED_RGB_MAX_DUTY)
        	{
        		timer = MEF_LED_RGB_TIME_STEP_MS;
        		estMefLed = EST_MEF_LED_RGB_ETAPA_1;
        	}
            break;
    }
}

void mefLedRGB_periodicTask1ms(void)
{
	if (timer)
	{
		timer--;
	}
}

/*==================[end of file]============================================*/
