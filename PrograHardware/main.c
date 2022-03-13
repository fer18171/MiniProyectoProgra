/*
 *
 * Santiago Enrique Fernández Matheu 18171
 * Mini Proyecto
 * Diseño e Innovación en Ingeniería 1
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
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "inc/hw_sysctl.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
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
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"

#include <math.h>

// Se agregan las nuevas librerias para i2c y para el mpu6050
#include "hw_mpu6050.h"
#include "i2cm_drv.h"
#include "mpu6050.h"


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


//****Variables para mpu6050****

// A boolean that is set when a MPU6050 command has completed.
volatile bool g_bMPU6050Done;

// I2C master instance
tI2CMInstance g_sI2CMSimpleInst;

//Device frequency
int clockFreq;
//*******************************


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

     Ref = pui32ADC0Value[1]*3.3/4095.0;  // Convertir a voltios
       y = pui32ADC0Value[0]*3.3/4095.0;  // Convertir a voltios

      u_k = control(Ref-y);
      u_k=u_k*(4095.0/3.3);

      if (u_k> 4095) // Se acota la salida para que se encuentre entre 0-4095
         u_k = 4095;
      else if (u_k < 0)
          u_k = 0;

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

}


//*********
// Configure SSI0 in master Freescale (SPI) mode.
// Se configura el timer y su interrupción. El SPI se configura para enviar 16
// bits. El envío se hace en la rutina de interrupción del timer.
//*********



void
InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
        UARTStdioConfig(0, 115200, 16000000);

}

void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.
    // I2C data transfer rate set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());

}

void delayMS(int ms) {
    //ROM_SysCtlDelay( (ROM_SysCtlClockGet()/(3*1000))*ms ) ;  // more accurate
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;  // less accurate
}

//
// The function that is provided by this example as a callback when MPU6050
// transactions have completed.
//
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // See if an error occurred.
    //
    if (ui8Status != I2CM_STATUS_SUCCESS)
    {
        //
        // An error occurred, so handle it here if required.
        //
    }
    //
    // Indicate that the MPU6050 transaction has completed.
    //
    g_bMPU6050Done = true;
}

//
// The interrupt handler for the I2C module.
//
void I2CMSimpleIntHandler(void)
{
    //
    // Call the I2C master driver interrupt handler.
    //
    I2CMIntHandler(&g_sI2CMSimpleInst);
}

//
// The MPU6050 example.
//
void MPU6050(void)
{
    uint16_t fAccel[3], fGyro[3];
    tMPU6050 sMPU6050;
    uint16_t x = 0, y = 0, z = 0;
    uint8_t xH = 0, yH = 0, zH = 0, xL = 0, yL = 0, zL = 0;

    //
    // Initialize the MPU6050. This code assumes that the I2C master instance
    // has already been initialized.
    //
    g_bMPU6050Done = false;
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    //
    // Configure the MPU6050 for +/- 4 g accelerometer range.
    //
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG, ~MPU6050_ACCEL_CONFIG_AFS_SEL_M,
        MPU6050_ACCEL_CONFIG_AFS_SEL_4G, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }


    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00, 0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2, 0x00, 0x00, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    //
    // Loop forever reading data from the MPU6050. Typically, this process
    // would be done in the background, but for the purposes of this example,
    // it is shown in an infinite loop.
    //

    while (1)
    {
        //
        // Request another reading from the MPU6050.
        //
        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }
        //
        // Get the new accelerometer and gyroscope readings.
        //
        MPU6050DataAccelGetRaw(&sMPU6050, &fAccel[0], &fAccel[1],
            &fAccel[2]);
        MPU6050DataGyroGetRaw(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);
        //
        // Do something with the new accelerometer and gyroscope readings.
        //

       /*x = fAccel[0];
        y = fAccel[1];
        z = fAccel[2];*/
        x = 778;
        xL = x&255;
        xH = x>>8;
        UARTCharPut(UART0_BASE, 10);
        UARTCharPut(UART0_BASE, 3);
        UARTCharPut(UART0_BASE, 3);
        UARTCharPut(UART0_BASE, xL);
        UARTCharPut(UART0_BASE, xH);
        //x = (atan2(fAccel[0], sqrt (fAccel[1] * fAccel[1] + fAccel[2] * fAccel[2]))*180.0)/3.14;

        //y = (atan2(fAccel[1], sqrt (fAccel[0] * fAccel[0] + fAccel[2] * fAccel[2]))*180.0)/3.14;

        //UARTprintf("Ang. X: %d | Ang. Y: %d | Ang. Z: %d\n", (int)x, (int)y, (int)z);
        //UARTprintf("1");
        //UARTprintf("Accel0: %f |Accel1: %f |Accel2: %f |Gyro0: %f |Gyro1: %f |Gyro2: %f |\n", fAccel[0], fAccel[1], fAccel[2], fGyro[0], fGyro[1], fGyro[2]);

        /*UARTCharPut(UART0_BASE,10);
        UARTCharPut(UART0_BASE,(int)x);
        UARTCharPut(UART0_BASE,(int)y);
        UARTCharPut(UART0_BASE,(int)z);*/

        //delayMS(100);
    }
}

void Key(uint8_t a,uint8_t b)
    {
        UARTCharPut(UART0_BASE, a);
        UARTCharPut(UART0_BASE, b);
        UARTCharPut(UART0_BASE, b);
            }

int
main(void)

{
    ///////////////////////////////////////////////////////////////
        uint32_t pui32residual[NUM_SPI_DATA];
        uint16_t freq_muestreo = 1000;    // En Hz



     SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                    SYSCTL_XTAL_16MHZ); // 80 MHz


     InitConsole();

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
    InitI2C0();
    MPU6050();

    return(0);
}
