// Hardware init and low-level helpers
#include "rtos.h"
#include "tm4c123gh6pm.h"

// Initialize Hardware
// Clock, GPIO, UART and wide timer initialization.
void initHw()
{
    //Initializing clock for 16MHz XTAL, PLL enabled, system clock of 40MHz
    SYSCTL_RCC_R = 0x078E3AD0;                //Set BYPASS bit, Clear USESYS bit in RCC
    SYSCTL_RCC_R = 0x078E1D40;                //Select Crystal value ->XTAL and OSCSRC, Clear PWDN in RCC
    SYSCTL_RCC_R = 0x024E1D40;                //Select SYSDIV in RCC and set USESYS bit in RCC
    while (SYSCTL_RIS_R & 0x00000040 != 0);   //Blocking function to wait for PLL to lock by polling PLLLRIS in Raw Interrupt Status (RIS) register
    SYSCTL_RCC_R = 0x024E1540;                //Enable use of PLL by clearing BYPASS bit in RCC

    //Initializing all GPIO registers to use APB
    SYSCTL_GPIOHBCTL_R = 0;

    //Initializing GPIO Port A, Port E, and Port F Peripherals
    SYSCTL_RCGC2_R |= 0x31;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= 0x04;  // Bit 2 is output, other pins are inputs
    GPIO_PORTF_DR2R_R |= 0x04; // Set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x04;  // Enable LED

    GPIO_PORTE_DIR_R |= 0x1E;  // Bits 1,2,3,4 are outputs, other pins are inputs
    GPIO_PORTE_DR2R_R |= 0x1E; // Set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R |= 0x1E;  // Enable LEDs

    GPIO_PORTA_DIR_R |= 0;  // Bits 2,3,4,5,6 are inputs
    GPIO_PORTA_DR2R_R |= 0x7C; // Set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= 0x7C;  // Enable pushbuttons
    GPIO_PORTA_PUR_R = 0x7C;  // Enable internal pull-up for push buttons

    //Configure UART0 Pins
    SYSCTL_RCGCUART_R |= 0x1;                //Enable UART0
    GPIO_PORTA_DEN_R |= 0x3;                 //Enabling pins 0 and 1 of Port A
    GPIO_PORTA_AFSEL_R |= 0x3;               //Enable the alternate function of Port A on pins 0 and 1
    GPIO_PORTA_PCTL_R |= 0x11;

    //Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                        //Disable the UART0 to allow safe programming
    UART0_CC_R = 0x0;                       //Use System Clock of 40MHz
    UART0_IBRD_R = 21;                      //r = 40MHz / (N*115.2KHz), set floor(r) = 21, where N=16
    UART0_FBRD_R = 45;                      //round(fract(r)*64) = 45
    UART0_LCRH_R = 0x70;                    //Configure for 8 data bits (8N1) with 16 level FIFO enabled
    UART0_CTL_R = 0x00000301;               //Enable the TX, RX, and UART0 Module

    //Initializing WTimer5A to count
        SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;                                             // Turn-on timer
        WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                                                        // Turn-off counter before reconfiguring
        WTIMER5_CFG_R = 4;                                                                       // Configure as 32-bit counter (A only)
        WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;             // Configure for edge time mode, count up
        WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;                                                   // Measure time from positive edge to positive edge
        WTIMER5_TAV_R = 0;                                                                       // Zero counter for first period
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
// Tight assembly loop calibrated for ~40 cycles/us.
void waitMicrosecond(uint32_t us)
{
                                              // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// readPbs returns bitmask 0-31 where each bit corresponds to a pushbutton pressed.
// Bit positions: PB0 -> bit0, PB1 -> bit1, ... PB4 -> bit4.
uint8_t readPbs()
{
    //Function to get the total of the Pushbuttons. Check the conditions, add, and return the final value
    uint16_t Pushbutton_total = 0;

            if(!Pushbutton_0)
                Pushbutton_total = Pushbutton_total+1;
            if(!Pushbutton_1)
                Pushbutton_total = Pushbutton_total+2;
            if(!Pushbutton_2)
                Pushbutton_total = Pushbutton_total+4;
            if(!Pushbutton_3)
                Pushbutton_total = Pushbutton_total+8;
            if(!Pushbutton_4)
                Pushbutton_total = Pushbutton_total+16;
            return Pushbutton_total;
}

// Set system stack pointer to value in R0. This wrapper moves R0->SP and
// Subtracts 8 to account for earlier pushes
void setsp(uint32_t x)
{
    __asm(" MOV SP, R0");                                                   //Move the R0 value into the SP
    __asm(" SUB SP, #8");                                                   //Decrement by 8 to account for the 2 pushes
}

// Return current SP in R0
uint32_t getsp()
{
    __asm(" MOV R0, SP");                                                   //Move the SP value into R0 and return it
}

// Returns R0
uint32_t getR0()
{                                                                           //Empty function to return R0
}

// Return contents of R1 in R0 (helper for SVC arg retrieval)
uint32_t getR1()
{
    __asm(" MOV R0, R1");                                                  //Move R1 value into R0
}

// Return contents of R2 in R0 (helper for SVC arg retrieval)
uint32_t getR2()
{
    __asm(" MOV R0, R2");                                                   //Move R2 value into R0
}
