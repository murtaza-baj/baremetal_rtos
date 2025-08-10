// Tasks, shell, UART helpers, parsing, pidof, reset
#include "rtos.h"
#include "tm4c123gh6pm.h"
#include <string.h>
#include <stdlib.h>

// pidof looks up a task by name and prints its pid. It lowercases task names
// First to do case-insensitive compare with shell input token.
void pidof()
{
    char pidof_string[20];
    uint32_t i,j;
    for(i=0;i<MAX_TASKS;i++)                                                    //Check all the tasks to find the one whose PID is needed
    {
        for(j=0;j<=16;j++)
            tcb[i].name[j] = lower(tcb[i].name[j]);                             //Convert all the characters of the task's name in lower case so that it can be compared with our input string
        if (strcmp (tcb[i].name, &input_string[position[2]])== 0)               //Compare with the input string
        {
            ltoa(tcb[i].pid,pidof_string);                                      //Convert PID number into a string
            putsUart0(pidof_string);                                            //Print the PID number
        }
    }
}

// Trigger a system reset using the AIRCR register.
void reset()
{
    NVIC_APINT_R = 0x05FA0004;                                                  //Reset
}

void ipcs()
{
    __asm   (" SVC #105");
}

void ps()
{
    __asm   (" SVC #200");
}

// Blocking function that writes a serial character when UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);          //Polls the TXFF bit to check if the Transmit FIFO is full. If it is full, then we can transmit the data
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

// ASCII-only lowercase converter for single char.
char lower(c)
{
    if (c >= 65 && c <= 90)                    //Check if the character is upper case
    {
        c = c + 32;                            //Add 32 to get lower case

    }
    return c;                                 //return the lower case character
}

// Blocking function that returns with serial data once the buffer is not empty
// This function yields while waiting so the scheduler can run other tasks.
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE) {yield();};         //If RXFE bit is set, it means the UART Receive FIFO is empty
    return UART0_DR_R & 0xFF;                             //Getting the lower 8 data bits by Anding with FF
}

// getstring reads user input from UART, handles backspace and CR, and lowercases chars.
void getstring()
{
    for (i = 0; i < max_chars; i++)
        {input_string[i] = 0;}
    count = 0;
    for (i = 0; i <= max_chars; i++)                  //max_chars=80 because screen can only fit max 80 characters at a time
    {
        c=getcUart0();
        if (c == 0x8 && count == 0)                  //0x8 = Backspace
            continue;
        else if (c == 0x8 && count != 0)
        {
            count--;                                //Decrement count to write over the previous character
            input_string[count] = 0;                //Delete the previous character
            continue;
        }
        else if (c == 0xD)                          //0xD = Carriage Return
        {
            input_string[count] = 0;                //Add null to the input_string at the end
            break;
        }
        else if (c < 0x20)                         //For c<20, all are unprintable characters, so we just ignore them
            continue;
        else if (c>= 0x20)                         //c>20 are printablle characters
        {
            input_string[count] = lower (c);      //Add the character in the input string
            count++;                              //Increment count
        }
    }
}

// Parse_string tokenizes input_string in-place, storing token starts in position[]
// and token types in type[] ('C' for letters, 'N' for numbers). Delimiters become NUL.
void parse_string()
{
    arg_count = 0;
    for (i = 0; i <=5; i++)
    {
        position[i] = 0;
        type[i] = 0;
    }
    for (i=0; i<count; i++)
        {
        if (i == 0)                                      //For the first character
        {
            if ((input_string[i] >= 65 && input_string[i] <= 90) || (input_string[i] >= 97 && input_string[i] <= 122))              //Condition to check if character is in the fields of A-Z or a-z
            {
                position[arg_count] = i;                //Save the position of the character in the array
                type[arg_count] = 'C';                  //Save the type of the character in the array
                arg_count++;                            //Increment arg_count to get the total number of interesting things at the end
            }
            else if (input_string[i] >= 48 && input_string[i] <= 57)                //Condition to check if the character is in the numeric field 0-9
            {
                position[arg_count] = i;               //Save the position of the character in the array
                type[arg_count] = 'N';                 //Save the type of the character in the array
                arg_count++;                           //Increment arg_count to get the total number of interesting things at the end
            }
        }
        else if (i > 0)                               //Condition for all the other characters except the first one
        {
            if (!((input_string[i-1] >= 65 && input_string[i-1] <= 90) || (input_string[i-1] >= 97 && input_string[i-1] <= 122) || (input_string[i-1] >= 48 && input_string[i-1] <= 57)))                   //Condition to check if the previous character was a delimiter
                {
                if ((input_string[i] >= 65 && input_string[i] <= 90) || (input_string[i] >= 97 && input_string[i] <= 122) || input_string[i] == 38)         //Condition to check if character is in the fields of A-Z or a-z
                    {
                        position[arg_count] = i;     //Save the position of the character in the array
                        type[arg_count] = 'C';        //Save the type of the character in the array
                        arg_count++;                 //Increment arg_count to get the total number of interesting things at the end
                    }
                else if (input_string[i] >= 48 && input_string[i] <= 57)           //Condition to check if the character is in the numeric field 0-9
                    {
                        position[arg_count] = i;     //Save the position of the character in the array
                        type[arg_count] = 'N';       //Save the type of the character in the array
                        arg_count++;                //Increment arg_count to get the total number of interesting things at the end
                    }
                }
        }
        if (!((input_string[i] >= 65 && input_string[i] <= 90) || (input_string[i] >= 97 && input_string[i] <= 122) || (input_string[i] >= 48 && input_string[i] <= 57) || input_string[i] == 38))     //Condition to check if character is a delimiter
            input_string[i] = 0;                   //Convert all delimiters into nulls
        }
}

// isCommand compares verb string to token at position[0]
bool isCommand(char *verb2[])
{
    if (strcmp (verb2, &input_string[position[0]])== 0)                //Comparing the string_verb with the input verb
    {
              return true;                                             //Setting status to true and returning it if both conditions are satisfied
    }
    return false;
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // Represent some lengthy operation
    waitMicrosecond(990);
    // Give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            tcb[2].processCounter = 0;
            createThread(flash4Hz, "Flash4Hz", 0);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

void shell()
{
    while (true)
    {
        putsUart0("\r\n");
        getstring();                                                                            //Get the input string
        putsUart0(input_string);                                                                //Print it
        parse_string();                                                                         //Parse the string to get useful information

        for (i=0; i<arg_count; i++)
        {
            putsUart0("\n\r");
            putsUart0(&input_string[position[i]]);                                              //Print the parsed string
            putcUart0(13);
        }

        if (isCommand("pi"))
        {
            if (strcmp ("on", &input_string[position[1]])== 0)
                priority_inheritance = ON;
            else if (strcmp ("off", &input_string[position[1]])== 0)
                priority_inheritance = OFF;
        }

        if (isCommand("sched"))
        {
            if (strcmp ("priority", &input_string[position[1]])== 0)
                scheduler = ON;
            else if (strcmp ("rr", &input_string[position[1]])== 0)
                scheduler = OFF;
        }
        if (isCommand("rtos"))
        {
            if (strcmp ("preempt", &input_string[position[1]])== 0)
                preempt = ON;
            else if (strcmp ("coop", &input_string[position[1]])== 0)
                preempt = OFF;
        }

        if (isCommand("reboot"))
            reset();

        if (isCommand("pid"))
        {
            if (strcmp ("of", &input_string[position[1]])== 0)
                pidof();
        }
        if (isCommand("kill"))
        {
            for(i=0;i<MAX_TASKS;i++)
            {
                if (tcb[i].pid == atoi(&input_string[position[1]]))
                    destroyThread(tcb[i].pid);
            }
        }

        if (isCommand("ps"))
            ps();

        if (isCommand("ipcs"))
            ipcs();
        if((strcmp ("&", &input_string[position[1]])== 0))
        {
            for(i=0;i<MAX_TASKS;i++)
            {
                if (!(tcb[i].processCounter > 0))                                       //Condition so we can only create the process once using <process_name> &
                {
                    if ((strcmp (tcb[i].name, &input_string[position[0]])== 0))
                    {
                        tcb[i].processCounter++;
                        createThread(directory[i], tcb[i].name, 0);
                    }
                }
            }
        }
    }
}
