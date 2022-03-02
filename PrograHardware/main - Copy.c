/*
 *
 * Santiago Enrique Fernández Matheu 18171
 *
 *
 * timer_spi_master.c
 *
 * Basado en los ejemplos spi_master.c y timers.c de TivaWare
 * Modificado por Luis Alberto Rivera
 */


//*********
//
// Copyright (c) 2010-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*********

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*********
//
//! \addtogroup ssi_examples_list
//! <h1>SPI Master (spi_master)</h1> --- MODIFICADO
//!
//! This example shows how to configure the SSI0 as SPI Master.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - SSI0 peripheral
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0Clk - PA2
//! - SSI0Fss - PA3
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - Timer0IntHandler.
//
//*********

//*********
// Definiciones para configuración del SPI
//*********
#define NUM_SPI_DATA    1  // Número de palabras que se envían cada vez
#define SPI_FREC  4000000  // Frecuencia para el reloj del SPI
#define SPI_ANCHO      16  // Número de bits que se envían cada vez, entre 4 y 16
#define frecuenciamuestreo 1000

uint16_t dato;  // Para lo que se envía por SPI.
uint16_t i=0;


// Se definen todas las varibles que se utilizaran para el programa
//float kP=4.20;
//float kI=252.40/frecuenciamuestreo;
//float kD=0.017*frecuenciamuestreo;



float Ref, y, u_k;

float e_k_1 = 0, e_k_2 = 0;
float u_k_1 = 0, u_k_2 = 0;

uint16_t u_kint;
//float u_k;

// Definición de constantes para ecuaciones de diferencias para cada método
//Tustin
//const float b0=38.43, b1=-71.46, b2=33.22;
//const float a1=0.1426, a2=-0.8574;

//Backward euler
//const float b0=21.34, b1=-39.79, b2=18.55;
//const float a1=1.04, a2=-0.037;

//zoh
const float b0=501.9, b1=-1001, b2=499.1;
const float a1=1.0, a2=-4.83e-12;

//zpm
//const float b0=20.71, b1=-38.52, b2=17.91;
//const float a1=1.0, a2=-4.83e-12;

//backard euler luis
//const float b0=16.62, b1=-30.91, b2=14.37;
//const float a1=1.042, a2=-0.04225;



//*********
// The interrupt handler for the timer interrupt.
//*********
void
Timer0IntHandler(void)
{

    // Notar que ahora necesitamos dos espacios para las conversiones.
    uint32_t pui32ADC0Value[2];


    uint32_t pui32DataTx[NUM_SPI_DATA]; // la función put pide tipo uint32_t
    uint8_t ui32Index;

    // Clear the timer interrupt. Necesario para lanzar la próxima interrupción.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    // Trigger the ADC conversion.
    ADCProcessorTrigger(ADC0_BASE, 2);  // Notar el cambio de "secuencia" (2 en lugar de 3).

      // Wait for conversion to be completed.
      while(!ADCIntStatus(ADC0_BASE, 2, false))  // Notar el cambio de "secuencia".
      {
      }

      // Clear the ADC interrupt flag.
      ADCIntClear(ADC0_BASE, 2);  // Notar el cambio de "secuencia".


      ADCSequenceDataGet(ADC0_BASE, 2, pui32ADC0Value);  // Notar el cambio de "secuencia".


/*
     Ref = pui32ADC0Value[1]*3.3/4095.0;  // Convertir a voltios
       y = pui32ADC0Value[0]*3.3/4095.0;  // Convertir a voltios

      u_k = control(Ref-y);
      u_k=u_k*(4095.0/3.3);

      if (u_k> 4095) // Se acota la salida para que se encuentre entre 0-4095
         u_k = 4095;
      else if (u_k < 0)
          u_k = 0;
*/
      u_k = 4095;
      u_kint=(uint16_t)(u_k);

    dato=0b0111000000000000|u_kint; // Enviamos el dato al DAC

    pui32DataTx[0] = (uint32_t)(dato);

    //Recontruccion
    //uint16_t u_kint;
   // u_kint = pui32ADC0Value[0];

 // dato=0b0111000000000000|u_kint; // Enviamos el dato al DAC

 // pui32DataTx[0] = (uint32_t)(dato);

////////////////////////////////////////////////

    // Send data
    for(ui32Index = 0; ui32Index < NUM_SPI_DATA; ui32Index++)
    {
        // Send the data using the "blocking" put function.  This function
        // will wait until there is room in the send FIFO before returning.
        // This allows you to assure that all the data you send makes it into
        // the send FIFO.
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
    }

    // Wait until SSI0 is done transferring all the data in the transmit FIFO.
    while(SSIBusy(SSI0_BASE))
    {
    }


    //UARTprintf("A0: %04d, Ref: %d.%02d,   A1: %04d, y: %d.%02d, u: %04d, y: %d.%02d\n",
      //         pui32ADC0Value[0], (uint8_t)Ref, (uint8_t)(100*(Ref-(uint8_t)Ref)),
        //       pui32ADC0Value[1], (uint8_t)y, (uint8_t)(100*(y-(uint8_t)y)),
          //     u_kint, (uint8_t)u_kV, (uint8_t)(100*(u_kV-(uint8_t)u_kV)));

}


//*********
// Configure SSI0 in master Freescale (SPI) mode.
// Se configura el timer y su interrupción. El SPI se configura para enviar 16
// bits. El envío se hace en la rutina de interrupción del timer.
//*********



/*void
InitConsole(void)
{
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);

}*/




int
main(void)

+{
    ///////////////////////////////////////////////////////////////
        uint32_t pui32residual[NUM_SPI_DATA];
        uint16_t freq_muestreo = 1000;    // En Hz



     SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                    SYSCTL_XTAL_16MHZ); // 80 MHz


     //InitConsole();

     // The ADC0 peripheral must be enabled for use.
     SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

     // For this example ADC0 is used with AIN0 and AIN1.
     // The actual port and pins used may be different on your part, consult
     // the data sheet for more information.  GPIO port E needs to be enabled
     // so these pins can be used.
     // TODO: change this to whichever GPIO port you are using.
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

     // Select the analog ADC function for these pins.
     // Consult the data sheet to see which functions are allocated per pin.
     // TODO: change this to select the port/pin you are using.
     GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);  // Configura el pin PE3
     GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);  // Configura el pin PE2

     // Se configura la secuencia 2, que permitiría hasta cuatro muestras (aunque
     // se usarán dos).
     ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);

     // Step 0 en la secuencia 2: Canal 0 (ADC_CTL_CH0) en modo single-ended (por defecto).
     ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);

     // Step 1 en la secuencia 2: Canal 1 (ADC_CTL_CH1) en modo single-ended (por defecto),
     // y configura la bandera de interrupción (ADC_CTL_IE) para "setearse"
     // cuando se tenga esta muestra. También se indica que esta es la última
     // conversión en la secuencia 2 (ADC_CTL_END).
     // Para más detalles del módulo ADC, consultar el datasheet.
     ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);

     // Since sample sequence 2 is now configured, it must be enabled.
     ADCSequenceEnable(ADC0_BASE, 2);  // Notar el cambio de "secuencia".

     // Clear the interrupt status flag.  This is done to make sure the
     // interrupt flag is cleared before we sample.
     ADCIntClear(ADC0_BASE, 2);  // Notar el cambio de "secuencia".

     // Enable the peripherals used by this example.
     SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

     // Enable processor interrupts.
     IntMasterEnable();

     // Configure the two 32-bit periodic timers.
     TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

     // El tercer argumento determina la frecuencia. El reloj se puede obtener
     // con SysCtlClockGet().
     // La frecuencia está dada por SysCtlClockGet()/(valor del 3er argumento).
     // Ejemplos: Si se pone SysCtlClockGet(), la frecuencia será de 1 Hz.
     //           Si se pone SysCtlClockGet()/1000, la frecuencia será de 1 kHz
     //TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
     TimerLoadSet(TIMER0_BASE, TIMER_A, (uint32_t)(SysCtlClockGet()/freq_muestreo));

     // Setup the interrupt for the timer timeout.
     IntEnable(INT_TIMER0A);
     TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

     // Enable the timers.
     TimerEnable(TIMER0_BASE, TIMER_A);

     // Las conversiones se hacen al darse la interrupción del timer, para que
     // el muestreo sea preciso. Luego de las configuraciones, el programa se
     // queda en un ciclo infinito haciendo nada.


    // The SSI0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    // For this example SSI0 is used with PortA[5:2].  The actual port and pins
    // used may be different on your part, consult the data sheet for more
    // information.  GPIO port A needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    // TODO: change this to select the port/pin you are using.
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);

    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 4MHz SSI frequency, and 16-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);

    // Enable the SSI0 module.
    SSIEnable(SSI0_BASE);

    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32residual[0]))
    {
    }


    while(1)
    {
    }
}
