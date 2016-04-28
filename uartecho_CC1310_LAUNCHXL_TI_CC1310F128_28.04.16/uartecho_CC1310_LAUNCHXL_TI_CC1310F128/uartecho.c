  /*
   * Copyright (c) 2016, Texas Instruments Incorporated
   * All rights reserved.
   *
   * Redistribution and use in source and binary forms, with or without
   * modification, are permitted provided that the following conditions
   * are met:
   *
   * *  Redistributions of source code must retain the above copyright
   *    notice, this list of conditions and the following disclaimer.
   *
   * *  Redistributions in binary form must reproduce the above copyright
   *    notice, this list of conditions and the following disclaimer in the
   *    documentation and/or other materials provided with the distribution.
   *
   * *  Neither the name of Texas Instruments Incorporated nor the names of
   *    its contributors may be used to endorse or promote products derived
   *    from this software without specific prior written permission.
   *
   * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
   * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
   * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
   * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   */

  //
   //  ======== uartecho.c ========
    //

  // XDCtools Header files //
  #include <xdc/std.h>
  #include <xdc/runtime/System.h>
  #include <xdc/cfg/global.h>

  // BIOS Header files //
  #include <ti/sysbios/BIOS.h>
  #include <ti/sysbios/knl/Task.h>
  #include <ti/sysbios/knl/Clock.h>


  // TI-RTOS Header files //
  #include <ti/drivers/PIN.h>
  #include <ti/drivers/UART.h>
  #include <ti/drivers/I2C.h>
  #include <ti/drivers/SPI.h>

  // Example/Board Header files
  #include "Board.h"


  #include <stdlib.h>
  #include <stdbool.h>
  #include <stdint.h>

  #define TASKSTACKSIZE     768

  Task_Struct task0Struct;
  Char task0Stack[TASKSTACKSIZE];

  // Global memory storage for a PIN_Config table
  static PIN_State ledPinState;
  static SPI_Handle spiHandle;
  static SPI_Params spiParams;

  //
  // Application LED pin configuration table:
  //   - All LEDs board LEDs are off.
   //
  PIN_Config ledPinTable[] = {
      Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
      Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_GENERALPURPOSE1   | PIN_GPIO_OUTPUT_EN 	| PIN_GPIO_HIGH	 | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_CP2103UARTRESET   | PIN_GPIO_OUTPUT_EN	| PIN_GPIO_HIGH  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	Board_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
	 Board_SPI0_MOSI | PIN_PULLDOWN,
	 Board_SPI0_MISO | PIN_PULLDOWN,
	 Board_SPI0_CLK | PIN_PULLDOWN,



      PIN_TERMINATE
  };

   //  ======== echoFxn ========
   //  Task for this function is created statically. See the project's .cfg file.
  //
  Void taskFxn(UArg arg0, UArg arg1)
  {
    	PIN_Handle ledPinHandle;

    	ledPinHandle = PIN_open(&ledPinState, ledPinTable);

    	PIN_setOutputValue(ledPinHandle, Board_LED0, 1);

    	PIN_setOutputValue(ledPinHandle,Board_SPI_FLASH_CS,Board_FLASH_CS_ON);


    	/*  Configure SPI as master, 1 mHz bit rate*/
    	    SPI_Params_init(&spiParams);
    	    spiParams.bitRate = 1000000;
    	    spiParams.mode         = SPI_MASTER;
    	    spiParams.transferMode = SPI_MODE_BLOCKING;

    	    /* Attempt to open SPI. */
    	    spiHandle = SPI_open(Board_SPI0, &spiParams);

    	    if (spiHandle == NULL)
    	       {
    	    	System_printf("spi is not open \n");
    	    	  System_flush();

    	       }
    	    else{
    	    	System_printf("spi is open \n");
    	    	  System_flush();

    	    }

    }

   //  ======== main ========
   //
  int main(void)
  {
      PIN_Handle ledPinHandle;
      Task_Params taskParams;

      // Call board init functions
      Board_initGeneral();
      Board_initSPI();


      // Construct BIOS objects
      Task_Params_init(&taskParams);
      taskParams.stackSize = TASKSTACKSIZE;
      taskParams.stack = &task0Stack;
      Task_construct(&task0Struct, (Task_FuncPtr)taskFxn, &taskParams, NULL);


      // Open LED pins
      ledPinHandle = PIN_open(&ledPinState, ledPinTable);

      if(!ledPinHandle) {
    	  System_printf("Error initializing board LED pins\n");
    	  System_flush();
      }
      else{
    	  System_printf("Perfect! initializing board LED pins\n");
    	  System_flush();
      }

     // PIN_setOutputValue(ledPinHandle, Board_LED0, 1);
      PIN_setOutputValue(ledPinHandle, Board_LED1, 1);
      PIN_setOutputValue(ledPinHandle, Board_LED2, 1);
      PIN_setOutputValue(ledPinHandle, Board_CP2103UARTRESET, 1);
     // PIN_setOutputValue(ledPinHandle, Board_GENERALPURPOSE1, 1);

      // This example has logging and many other debug capabilities enabled
      System_printf("This example does not attempt to minimize code or data "
                    "footprint\n");
      System_flush();

      System_printf("Starting the UART Echo example\nSystem provider is set to "
                    "SysMin. Halt the target to view any SysMin contents in "
                    "ROV.\n");
      // SysMin will only print to the console when you call flush or exit
      System_flush();

      // Start BIOS
      BIOS_start();

      return (0);
  }
