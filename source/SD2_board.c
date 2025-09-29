/* Copyright 2017, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2017, Diego Alegrechi
 * Copyright 2017, Gustavo Muro
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
#include <SD2_board.h>
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_tpm.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

static const board_gpioInfo_type board_gpioLeds[] =
{
	{PORTE, GPIOE, 31},     /* LED ROJO */
	{PORTD, GPIOD, 5},      /* LED VERDE */
	{PORTE, GPIOE, 29},     /* LED AZUL */

};

static const board_gpioInfo_type board_gpioSw[] =
{
    {PORTC, GPIOC, 3},     /* SW2 */
    {PORTA, GPIOA, 4},     /* SW3 */
};

static uint8_t dutyCyclePWM[BOARD_LED_ID_TOTAL];
static bool ledStatus[BOARD_LED_ID_TOTAL];

static const tpm_chnl_t tpm_chnl_leds[BOARD_LED_ID_TOTAL] =
{
	kTPM_Chnl_4,
	kTPM_Chnl_5,
	kTPM_Chnl_2,
};

static const port_mux_t pinMuxLeds[BOARD_LED_ID_TOTAL] =
{
	kPORT_MuxAlt3,
	kPORT_MuxAlt4,
	kPORT_MuxAlt3,
};

/*==================[internal functions declaration]=========================*/

static void config_TPM_PWM(void);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void config_TPM_PWM(void)
{
	tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam;

    int i;

    /* Configure tpm params with frequency 24kHZ */
	tpmParam.level = kTPM_LowTrue;
	tpmParam.dutyCyclePercent = 0;

	/* MCGIRCLK */
	CLOCK_SetTpmClock(3);

	TPM_GetDefaultConfig(&tpmInfo);

	/* Initialize TPM module */
	TPM_Init(TPM0, &tpmInfo);

	for (i = 0 ; i < BOARD_LED_ID_TOTAL ; i++)
	{
		tpmParam.chnlNumber = tpm_chnl_leds[i];
		TPM_SetupPwm(TPM0, &tpmParam, 1U, kTPM_CenterAlignedPwm, 24000U, CLOCK_GetFreq(kCLOCK_McgIrc48MClk));

		PORT_SetPinMux(board_gpioLeds[i].port, board_gpioLeds[i].pin, pinMuxLeds[i]);

		dutyCyclePWM[i] = 100;
		ledStatus[i] = false;
		TPM_UpdatePwmDutycycle(TPM0, tpm_chnl_leds[i], kTPM_CenterAlignedPwm, 0);
	}

	TPM_StartTimer(TPM0, kTPM_SystemClock);
}

/*==================[external functions definition]==========================*/

void board_init(void)
{
	int32_t i;
	const gpio_pin_config_t gpio_sw_config = {
		.pinDirection = kGPIO_DigitalInput,
		.outputLogic = 0U
	};

	const port_pin_config_t port_sw_config = {
		/* Internal pull-up resistor is enabled */
		.pullSelect = kPORT_PullUp,
		/* Fast slew rate is configured */
		.slewRate = kPORT_FastSlewRate,
		/* Passive filter is disabled */
		.passiveFilterEnable = kPORT_PassiveFilterDisable,
		/* Low drive strength is configured */
		.driveStrength = kPORT_LowDriveStrength,
		/* Pin is configured as PTC3 */
		.mux = kPORT_MuxAsGpio,
	};

	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortD);
	CLOCK_EnableClock(kCLOCK_PortE);

	/* inicialización de SWs */
	for (i = 0 ; i < BOARD_SW_ID_TOTAL ; i++)
	{
		PORT_SetPinConfig(board_gpioSw[i].port, board_gpioSw[i].pin, &port_sw_config);
		GPIO_PinInit(board_gpioSw[i].gpio, board_gpioSw[i].pin, &gpio_sw_config);
	}

	config_TPM_PWM();
}

void board_setLed(board_ledId_enum id, board_ledMsg_enum msg)
{
    switch (msg)
    {
        case BOARD_LED_MSG_OFF:
        	ledStatus[id] = false;
        	TPM_UpdatePwmDutycycle(TPM0, tpm_chnl_leds[id], kTPM_CenterAlignedPwm, 0);
            break;

        case BOARD_LED_MSG_ON:
        	ledStatus[id] = true;
        	TPM_UpdatePwmDutycycle(TPM0, tpm_chnl_leds[id], kTPM_CenterAlignedPwm, dutyCyclePWM[id]);
            break;

        case BOARD_LED_MSG_TOGGLE:
        	ledStatus[id] = !ledStatus[id];
        	TPM_UpdatePwmDutycycle(TPM0, tpm_chnl_leds[id], kTPM_CenterAlignedPwm, dutyCyclePWM[id] * ledStatus[id]);
            break;

        default:
            break;
    }
}

void board_setLedBrightness(board_ledId_enum id, uint8_t brightness)
{
	if (brightness > 100)
		brightness = 100;

	dutyCyclePWM[id] = brightness;
	if (ledStatus[id])
		TPM_UpdatePwmDutycycle(TPM0, tpm_chnl_leds[id], kTPM_CenterAlignedPwm, dutyCyclePWM[id]);
}

bool board_getSw(board_swId_enum id)
{
    return !GPIO_PinRead(board_gpioSw[id].gpio, board_gpioSw[id].pin);
}

/*==================[end of file]============================================*/
