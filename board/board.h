/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_


/*
 * Board identifier.
 */
#define BOARD_LOAD_REV_A
#define BOARD_NAME              "cam_ctrl Rev. A"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            0

#define STM32_HSECLK            16000000
/*#define STM32_HSE_BYPASS*/


/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F10X_MD

/*
 * GPIO
 */
// Misc

/*
 * IO pins assignments.
 */
#define GPIOA_CAM_VCC_EN        8
#define GPIOA_IR_LED           15
#define GPIOB_LED	            0
#define GPIOB_CAM_TRIG_CTRL     9
#define GPIOC_CAM_POWER_BUT    13


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything input with pull-up except:
 * PA8  - Push Pull output 10MHz     (VCC_CAM_EN).
 * PA9  - Alternate Open Drain output(USART1_TX).
 * PA10 - Digital input              (USART1_RX).
 * PA11 - Digital input with PullUp  (CAN_RX).
 * PA12 - Alternate Push Pull output (CAN_TX).
 * PA15 - Push Pull output 10MHz.    (IR_LED).
 */
#define VAL_GPIOACRL            0x11111111      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x188B84F1      /* PA15...PA8 */
#define VAL_GPIOAODR            0x00000000

/*
 * Port B setup.
 * Everything input with pull-up except:
 * PB0  - Push Pull output 10MHz     (STATUS_LED).
 * PB9  - Push Pull output 10MHz     (CAM_TRIG_CTRL).
 */
#define VAL_GPIOBCRL            0x88888881      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x88888818      /* PB15...PB8 */
#define VAL_GPIOBODR            0x00000000
/*
 * Port C setup.
 * Everything input with pull-up except:
 * PC13 - Push Pull output 10MHz     (CAM_POWER_CTRL).
 */
#define VAL_GPIOCCRL            0x88888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x88188888      /* PC15...PC8 */
#define VAL_GPIOCODR            0x00000000

/*
 * Port D setup.
 * Everything input with pull-up except:
 * PD0  - Digital input (XTAL).
 * PD1  - Digital input (XTAL).
 */
#define VAL_GPIODCRL            0x11111144      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x11111111      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup (not populated).
 * Everything input with pull-up except:
 */
#define VAL_GPIOECRL            0x11111111      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x11111111      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
