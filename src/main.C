#include "rtos.h"
#include "tm4c123gh6pm.h"

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    rtosInit();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;

    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore("KeyPressed",1);
    keyReleased = createSemaphore("KeyReleased",0);
    flashReq = createSemaphore("flashReq",5);
    resource = createSemaphore("resource",1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 4);
    ok &= createThread(flash4Hz, "Flash4Hz", 0);
    ok &= createThread(oneshot, "OneShot", -4);
    ok &= createThread(readKeys, "ReadKeys", 4);
    ok &= createThread(debounce, "Debounce", 4);
    ok &= createThread(important, "Important", -8);
    ok &= createThread(uncooperative, "Uncoop", 2);
    ok &= createThread(shell, "Shell", 0);

    // Start up RTOS
    if (ok)
    {
        putsUart0("\r\n*******************************************************\r\n");
        putsUart0("\r\nYour Name");
        putsUart0("\r\nMy Project");
        putsUart0("\r\nRTOS\r\n");
        putsUart0("\r\n*******************************************************\r\n");

        rtosStart(); // never returns
    }
    else
        RED_LED = 1;

    return 0;
}
