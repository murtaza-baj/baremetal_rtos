#include "rtos.h"
#include "tm4c123gh6pm.h"
#include <string.h>
#include <stdlib.h>

// Global variables used by the shell and scheduler (definitions)
uint8_t max_chars = 80, count = 0, c, arg_count=0, position[5], i, min_args, j;
char input_string[82], type[5], str1[20];
char* string_verb;
volatile uint32_t timer_count = 0;
uint32_t l=0, m=0, directory[10];
volatile uint64_t time_all_tasks1 = 0, time_all_tasks2 = 0;
volatile bool scheduler = ON, priority_inheritance = ON, preempt = OFF;                               //All three ON by default.

// semaphore storage (definitions)
struct semaphore semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

struct semaphore *semaphore_pointer;

volatile uint8_t taskCurrent = 0;   // Index of last dispatched task
uint8_t taskCount = 0;     // Total number of valid tasks

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

tcb[MAX_TASKS];

uint32_t stack_variable;

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// Initialize kernel data structures and SysTick for 1ms tick.
void rtosInit()
{
    uint8_t i;
    // No tasks running
    taskCount = 0;
    // Clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
    //Configure SYSTICK
        NVIC_ST_CTRL_R |= 0X7;                                                      //Initializing the SYSTICK Timer's interrupt
        NVIC_ST_RELOAD_R |= SYSTICK_RELOAD_1MS;                                     //Reload value for the interrupt to go off every 1ms
}

// The scheduler chooses the next runnable task index using a circular scan.
// It applies an 8-level 'skip' bias so higher-priority tasks are selected
// More frequently (skip==0 runs every scan, larger skip values run less often).
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        if (ok && scheduler)
        {
            if(tcb[task].currentPriority == -8 || tcb[task].currentPriority == -7)
                tcb[task].skip = 0;                                                             //Bias-The number of times a task can be skipped before it has to run
            else if(tcb[task].currentPriority == -6 || tcb[task].currentPriority == -5)
                tcb[task].skip = 1;
            else if(tcb[task].currentPriority == -4 || tcb[task].currentPriority == -3)
                tcb[task].skip = 2;
            else if(tcb[task].currentPriority == -2 || tcb[task].currentPriority == -1)
                tcb[task].skip = 3;
            else if(tcb[task].currentPriority == 0 || tcb[task].currentPriority == 1)
                tcb[task].skip = 4;
            else if(tcb[task].currentPriority == 2 || tcb[task].currentPriority == 3)
                tcb[task].skip = 5;
            else if(tcb[task].currentPriority == 4 || tcb[task].currentPriority == 5)
                tcb[task].skip = 6;
            else if(tcb[task].currentPriority == 6 || tcb[task].currentPriority == 7)
                tcb[task].skip = 7;
            if(tcb[task].skip_count != tcb[task].skip)
            {
                // Not yet time for this task (skip_count increments each scan)
                tcb[task].skip_count++;
                ok = false;
            }
            else if(tcb[task].skip_count == tcb[task].skip)
            {
                // Reset skip count and allow this task to run on this pass
                tcb[task].skip_count = 0;
            }
        }
    }
    return task;
}

// rtosStart performs initial context switch to the first selected thread
// It saves system SP, sets thread SP and then calls the thread function.
void rtosStart()
{
    _fn fn;
    taskCurrent = rtosScheduler();                                      //Calls the scheduler to schedule the first task
    stack_variable = getsp();                                           //Gets the system stack pointer and saves it
    setsp(tcb[taskCurrent].sp);                                         //Sets the stack to the thread stack
    tcb[taskCurrent].state = STATE_READY;
    fn = tcb[taskCurrent].pid;                                          //Initialising the pid value of the function
    preempt = ON;
    WTIMER5_TAV_R = 0;
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                                     // Turn-on timer for counting the tcb[i].time
    (*fn)();
}

// createThread registers a function as a thread in the TCB table.
// It returns true on success. Note: processCounter logic allows launching
// A stored 'directory' function via shell using "<name> &".
bool createThread(_fn fn, char name[], int priority)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // Add task if room in task list
    if (!(tcb[i].processCounter > 1))                                     //Limiting condition to check that the task has only been created once by the <process_name> & command
    {
    if (taskCount < MAX_TASKS)
    {
        // Make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // Find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            directory[i] = tcb[i].pid;                                  //Directory for the <process_name> & command
            tcb[i].sp = &stack[i][255];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            strcpy(tcb[i].name,name);                                   //Copying the name of the task
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // Allow tasks switches again
    return ok;
    }
}

// destroyThread triggers SVC to perform kernel-level thread destruction.
// Actual cleanup is handled in the SVC handler to keep memory access atomic.
void destroyThread(_fn fn)
{
    __asm (" SVC #104");
}

// setThreadPriority updates the base priority for matching pid entries.
// currentPriority is not touched here; kernel may change it temporarily for inheritance.
void setThreadPriority(_fn fn, uint8_t priority)
{
    for(i=0;i<MAX_TASKS;i++)
    {
        if(tcb[i].pid == fn)                                                       //Sets the PID of the desired function
            tcb[i].priority = priority;                                            //Sets the required priority
    }
}

// Allocate a named semaphore (count initializes the available units).
struct semaphore* createSemaphore(char name[], uint8_t count)
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        strcpy(semaphores[semaphoreCount].name,name);                              //Copies the name of the Semaphore
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
    }
    return pSemaphore;
}

// Helper: extract SVC immediate from the stacked PC of the exception frame.
// This reads the SVC opcode immediate (imm8) which identifies the service.
// The kernel's exception stack frame places the stored PC at SP + offset; the
// instruction itself is at PC-2 when SVC is executed, so we read that byte.
uint32_t getSVC(uint32_t x)
{
    uint32_t svCallIsr_Stack_Pointer, svCallIsr_Program_Counter;
    svCallIsr_Stack_Pointer = getsp();                                                          //Get the stack pointer
    svCallIsr_Stack_Pointer = svCallIsr_Stack_Pointer + SVC_STACK_PC_OFFSET;                   //Calculate the offset from SP to PC
    svCallIsr_Program_Counter = (*((volatile uint32_t *)(svCallIsr_Stack_Pointer)));            //Go to where that Program Counter value points to
    svCallIsr_Program_Counter = svCallIsr_Program_Counter - 2;                                  //Decrement by 2 to get the address of the SVC number
    svCallIsr_Program_Counter = (*((volatile uint16_t *)(svCallIsr_Program_Counter)));          //Go to that address to get the SVC number
    svCallIsr_Program_Counter = svCallIsr_Program_Counter & 0x000000FF;                         // Only lower 8 bits are needed. Discard the rest
    return svCallIsr_Program_Counter;                                                           //Return the SVC number
}

// Yield requests a context switch via SVC (SVC handler will pend PENDSV).
void yield()
{
    __asm   (" SVC #100");
}

// Sleep triggers SVC which sets the calling thread to STATE_DELAYED for 'tick'
void sleep(uint32_t tick)
{
    __asm (" SVC #101");
}

// Wait (P) on semaphore. If not available, block the thread (handled in SVC).
void wait(struct semaphore *pSemaphore)
{
    __asm (" SVC #102");
}

// Post (V) to semaphore. Wake waiting task if any (handled in SVC).
void post(struct semaphore *pSemaphore)
{
    __asm (" SVC #103");
}

// SYSTICK ISR - called every 1ms. Handles sleep countdowns and periodic accounting.
// In preemptive mode it will pend PENDSV to request a context switch.
void systickIsr()
{
    uint32_t i;
    for (i = 0; i < MAX_TASKS; i++)
        if (tcb[i].state == STATE_DELAYED)                                                      //Checks for all threads if state is delayed
        {
            if (tcb[i].ticks == 0)                                                              //Checks if sleep time is over
            {
                tcb[i].state = STATE_READY;                                                     //If it is, sets the task's state to ready
            }
            else
                tcb[i].ticks--;                                                                //If it's not, simply decrements the ticks  of the timer
        }
    timer_count++;                                                                             //For the CPU % calculation
    if(timer_count == 2000)                                                                    //If 2 seconds
    {time_all_tasks1 = 0;                                                                      //Dump previous value
        for(i=0;i<MAX_TASKS;i++)
        {
            tcb[i].total_time1=0;                                                              //Dump previous value
            tcb[i].total_time1 = (9*tcb[i].total_time1 + tcb[i].time);                         //Integrate for new values and filtering
            time_all_tasks1 = time_all_tasks1 + tcb[i].total_time1;                            //Integrate for new values
            tcb[i].time = 0;                                                                   //Make the tcb[i].time==0 at the end of T
        }
    }
    if(timer_count == 4000)                                                                    //If 4 seconds
        {time_all_tasks2 =0;                                                                   //Dump previous value
            for(i=0;i<MAX_TASKS;i++)
            {
                tcb[i].total_time2 = 0;                                                        //Dump previous value
                tcb[i].total_time2 = (9*tcb[i].total_time2 + tcb[i].time);                     //Integrate for new values and filtering
                time_all_tasks2 = time_all_tasks2 + tcb[i].total_time2;                        //Integrate for new values
                tcb[i].time = 0;                                                               //Make tcb[i].time==0 at the end of T
            }
            timer_count = 0;                                                                   //Make the count zero to start the next cycle
        }
    if (preempt == ON)
        {
            NVIC_INT_CTRL_R |= 0x10000000;                                                     //If RTOS in preemption mode, PENDSV
        }
}

// pendSVISR saves current context and does a context switch.
// - Save a few registers (non-volatile R4-R7 already pushed before change of SP).
// - Record elapsed time for the current task using the wide timer.
// - Save current thread SP to TCB.
// - Restore system SP and decide next runnable task.
// - If next task is UNRUN, seed its stack so it returns into the thread function.
// - Restore timer and resume by popping registers.
void pendSVISR()
{
   __asm(" PUSH {R4, R5, R6, R7}");                                                          //Manual pushes to save context
   tcb[taskCurrent].time = tcb[taskCurrent].time+WTIMER5_TAV_R;                              //Store the values of time for the task
       WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;
    tcb[taskCurrent].sp = getsp();                                                           //Get system Stack Pointer
    setsp(stack_variable);                                                                   //Set the stack pointer to system stack pointer
    taskCurrent = rtosScheduler();                                                           //Schedule the next task

    if (tcb[taskCurrent].state == STATE_READY)                                               //If any tasks ready, set the stack pointer to their stack
    {
        setsp(tcb[taskCurrent].sp);
    }
    else if (tcb[taskCurrent].state == STATE_UNRUN)                                         //If task is unrun, seed the stack with dummy values to make it look like it has be run
    {
        // Seed the new thread's stack to simulate exception stack frame so it starts cleanly.
        stack[taskCurrent][255] = 0x61000000;
        stack[taskCurrent][254] = tcb[taskCurrent].pid;
        stack[taskCurrent][253] = tcb[taskCurrent].pid;
        stack[taskCurrent][252] = 12;
        stack[taskCurrent][251] = 3;
        stack[taskCurrent][250] = 2;
        stack[taskCurrent][249] = 1;
        stack[taskCurrent][248] = 0;
        stack[taskCurrent][247] = 0xFFFFFFF9;
        stack[taskCurrent][246] = 7;
        stack[taskCurrent][245] = 6;
        stack[taskCurrent][244] = 5;
        stack[taskCurrent][243] = 4;

        setsp(tcb[taskCurrent].sp - STACK_SEED_SIZE);                                                    //Manually set the Stack Pointer
        tcb[taskCurrent].state = STATE_READY;                                               //Set the state as Ready
    }
    WTIMER5_TAV_R = 0;                                                                      //Make timer data register as zero
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                                                        //Enable the timer for next task
    __asm(" POP {R7, R6, R5, R4}");                                                         //Pop the manually pushed registers
}

// SVC handler - central dispatch for kernel services. It reads SVC immediate
// and uses helper getR0/getR1/getR2 to access stacked register arguments.
void svCallISR()
{
    uint32_t x, R0, R1, R2, i, j, k, task_number;
    R0 = getR0();                                                                                     //Get R0
    semaphore_pointer = R0;                                                                          //Save it to access the members of the semaphore structure
    R1 = getR1();                                                                                   //Get R1
    R2 = getR2();                                                                                   //Get R2
    x = getSVC();                                                                                   //Get the SVC number to identify which case to run
    switch(x) {

        case SVC_YIELD:
            // Request a context switch by pended PENDSV; this leaves the current
            // thread state unchanged (ready/running) and schedules the next task.
            NVIC_INT_CTRL_R |= 0x10000000;                                                          //PENDSV to get the next ready task
            break;

        case SVC_SLEEP:
            // Put current thread to delayed state for R0 ticks; systick will
            // decrement ticks and make thread READY when time elapses.
            tcb[taskCurrent].ticks = R0;                                                            //Initializes the sleep time from R0
            tcb[taskCurrent].state = STATE_DELAYED;                                                 //Marks the state as delayed because task has to sleep
            NVIC_INT_CTRL_R |= 0x10000000;                                                         //PENDSV to get the next ready task
            break;

        case SVC_WAIT:
            // Semaphore P (wait): if count>0 take it and record current user.
            // Otherwise block the calling thread and enqueue it. If priority
            // inheritance is enabled, boost holder's currentPriority if needed.
            if ((*semaphore_pointer).count > 0)                                                    //Checks to see if Semaphores are available
            {
                (*semaphore_pointer).count--;                                                     //If available decrease the count
                (*semaphore_pointer).currentUser = taskCurrent;                                   //Save the task taking it
            }
            else if ((*semaphore_pointer).count == 0)                                             //If Semaphores not available
            {
                // Block this thread and append to semaphore queue.
                tcb[taskCurrent].state = STATE_BLOCKED;                                          //Mark task's state as Blocked
                tcb[taskCurrent].semaphore = (semaphore_pointer);                                  //Save the Semaphore being used by the task
                (*semaphore_pointer).processQueue[(*semaphore_pointer).queueSize] = taskCurrent;   //Put the task in Queue
                (*semaphore_pointer).queueSize++;                                                   //Increase the Queue Size
                if (priority_inheritance == ON)                                                     //Check if Priority Inheritance is turned on
                {
                    // If the waiting thread has higher priority (numerically lower),
                    // raise the owner's currentPriority to avoid priority inversion.
                    if (tcb[(*semaphore_pointer).currentUser].currentPriority > tcb[taskCurrent].priority)
                    {
                        tcb[(*semaphore_pointer).currentUser].currentPriority = tcb[taskCurrent].priority;
                    }
                    // Ensure idle task (index 0) doesn't hold an elevated priority
                    if(tcb[0].currentPriority != 7)
                    {
                        tcb[0].priority = 7;
                        tcb[0].currentPriority = tcb[0].priority;
                    }
                }
            }
            // After blocking, switch to next ready task.
            NVIC_INT_CTRL_R |= 0x10000000;                                                          //PENDSV to get the next ready task
            break;
         case SVC_POST:
            {
                // Semaphore V (post): if somebody waiting, make the first queued thread READY
                // and shift queue; otherwise increment count.
                tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;                               //Reset the currentPriority to its original value
                if ((*semaphore_pointer).queueSize > 0)                                                     //Check if somebody is waiting in queue
                {
                    tcb[(*semaphore_pointer).processQueue[0]].state = STATE_READY;                          //Set the first task in the queue as ready
                    (*semaphore_pointer).currentUser = (*semaphore_pointer).processQueue[0];
                    for(i=0;i<=(*semaphore_pointer).queueSize;i++)
                    {
                        (*semaphore_pointer).processQueue[i] = (*semaphore_pointer).processQueue[i + 1];    //Move all the members in the queue up by one place
                    }
                    (*semaphore_pointer).queueSize--;                                                       //Decrement the Semaphore Queue size
                }
                else if ((*semaphore_pointer).queueSize == 0)
                {
                    (*semaphore_pointer).count++;                                                           //Else, just increment the count
                }
                NVIC_INT_CTRL_R |= 0x10000000;                                                              //PENDSV to get the next ready task
            }
            break;
        case SVC_KILL:
        {
            // Kernel-level kill: find task by pid (R0), free any semaphore links,
            // remove from queues and mark invalid so it won't be scheduled.
            task_number=-1;                                                                                                                     //Flag so task doesn't re-enter
            for (k=0;k<MAX_TASKS;k++)
            {
                if (tcb[k].pid == R0)
                        task_number = k;                                                                                                        //Check if PID number of the task matches with the PID number of the task we need to kill. If it does, save the task number
            }
            if(task_number != -1)
            {
            if(tcb[task_number].semaphore != 0)                                                                                                 //Check if it is currently using a Semaphore
            {
                semaphore_pointer = tcb[task_number].semaphore;                                                                                 //Get the Semaphore
                if ((*semaphore_pointer).queueSize > 0)                                                                                         //Check if somebody is waiting in queue
                    {
                        (*semaphore_pointer).count++;                                                                                           //If somebody is, increment the Semaphore Count
                        tcb[(*semaphore_pointer).processQueue[0]].state = STATE_READY;                                                          //Mark the task as ready
                        (*semaphore_pointer).currentUser = (*semaphore_pointer).processQueue[0];                                                //Save it as last user
                        for(i=0;i<=(*semaphore_pointer).queueSize;i++)
                            {
                                (*semaphore_pointer).processQueue[i] = (*semaphore_pointer).processQueue[i + 1];                                //Move everybody up in the queue by one place
                            }
                        (*semaphore_pointer).queueSize--;                                                                                       //Decrement queue size
                     }
                }
            if (tcb[task_number].state == STATE_BLOCKED)                                                                                        //Check if state is Blocked
            {
                    for(j=0;j<=(*semaphore_pointer).queueSize;j++)
                    {
                        if ((*semaphore_pointer).processQueue[j] == task_number)                                                                //Check the queue for the semaphore to find he task
                            {
                            (*semaphore_pointer).processQueue[j] = 0;                                                                           //Remove it from the queue
                            for(i=j;i<=(*semaphore_pointer).queueSize;i++)
                                {
                                    (*semaphore_pointer).processQueue[i] = (*semaphore_pointer).processQueue[i + 1];                            //Move everybody up by one place
                                }
                            (*semaphore_pointer).queueSize--;                                                                                   //Decrement the Semaphore's queue size
                            }
                    }
            }
            tcb[task_number].state = STATE_INVALID;                                                                                             //Make the task's state as invalid
            tcb[task_number].pid = 0;                                                                                                           //Make its PID zero so it can't run again
            taskCount--;                                                                                                                        //Decrement Task Count
            NVIC_INT_CTRL_R |= 0x10000000;                                                                                                      //PENDSV to find the next ready task
            }
        }
        break;

        case SVC_IPCS:
        {
            // Print semaphore table info to UART
            putsUart0("Semaphore name      ");
            putsUart0("Semaphore Count      ");
            putsUart0("Semaphore Queuesize      ");
            putsUart0("Task Waiting");
            for(i=0;i<MAX_SEMAPHORES;i++)
            {
                putsUart0("\r\n  ");
                putsUart0(semaphores[i].name);                                  //Print the Semaphore's Name
                ltoa(semaphores[i].count, str1);
                putsUart0("                  ");
                putsUart0(str1);                                                //Print the Semaphore's Count
                ltoa(semaphores[i].queueSize, str1);
                putsUart0("                     ");
                putsUart0(str1);                                                //Print the Semaphore's Queue Size
                for(j=0;j<semaphores[i].queueSize;j++)
                {
                    x = semaphores[i].processQueue[j];                          //Print the names of the tasks that are waiting
                    putsUart0("               ");
                    putsUart0(tcb[x].name);
                }
            }
        }
        break;

        case SVC_PS:
        {
           // Print process list and CPU% accounting, formatting decimals for output.
           putsUart0("Process Name           ");
            putsUart0("Process PID         ");
            putsUart0("Process Priority     ");
            putsUart0("Process CPU%");
            if(timer_count <=2000)                                              //Condition to access buffer 1
            {
            for(i=0;i<MAX_TASKS;i++)
            {
                tcb[i].cpu = ((tcb[i].total_time1)*100000);                     //Calculating the CPU %
                tcb[i].cpu = tcb[i].cpu/time_all_tasks1;
                putsUart0("\r\n");
                putsUart0(tcb[i].name);                                         //Print the name of the task
                ltoa(tcb[i].pid, str1);
                putsUart0("                     ");
                putsUart0(str1);                                               //Print the PID of the task
                ltoa(tcb[i].currentPriority, str1);
                putsUart0("                  ");
                putsUart0(str1);                                               //Print the priority of the task
                ltoa(tcb[i].cpu,str1);
                tcb[i].cpu=0;                                                  //Make the CPU % value zero in the tcb structure after copying it to a string
                if (strlen(str1)>4)
                {
                for(j=2;j<=6;j++)
                {
                    str1[j+1] = str1[j];
                }
                str1[2] = 0x2E;
                }
                if (strlen(str1) == 3 || strlen(str1) == 4)
                {
                for(j=1;j<=6;j++)
                   {
                       str1[j+1] = str1[j];
                   }
                str1[1] = 0x2E;
                }
                if (strlen(str1) == 2)
                {
                for(j=0;j<=6;j++)
                   {
                       str1[j+2] = str1[j];
                   }
                str1[0] = 48;
                str1[1] = 0x2E;
                }
                if (strlen(str1) == 1)
                {
                str1[3] = str1[0];
                str1[1] = 0x2E;
                str1[0] = 48;
                str1[2] = 48;
                }
                putsUart0("                 ");
                putsUart0(str1);                                                    //Print the CPU % after putting decimal point
            }
            }
            if(timer_count >2000)                                                   //Condition to access buffer 2
            {
                for(i=0;i<MAX_TASKS;i++)
            {
                tcb[i].cpu = ((tcb[i].total_time1)*100000);                         //CPU % Calculation
                tcb[i].cpu = tcb[i].cpu/time_all_tasks1;
                putsUart0("\r\n");
                putsUart0(tcb[i].name);                                             //Print the name of the task
                ltoa(tcb[i].pid, str1);
                putsUart0("                     ");
                putsUart0(str1);                                                    //Print the PID of the task
                ltoa(tcb[i].currentPriority, str1);
                putsUart0("                  ");
                putsUart0(str1);                                                    //Print the Priority of the task
                ltoa(tcb[i].cpu,str1);
                tcb[i].cpu=0;                                                       //Make the CPU % value in the tcb structure zero
                if (strlen(str1)>4)
                {
                for(j=2;j<=6;j++)
                {
                    str1[j+1] = str1[j];
                }
                str1[2] = 0x2E;
                }
                if (strlen(str1) == 3 || strlen(str1) == 4)
                {
                for(j=1;j<=6;j++)
                   {
                       str1[j+1] = str1[j];
                   }
                str1[1] = 0x2E;
                }
                if (strlen(str1) == 2)
                {
                for(j=0;j<=6;j++)
                   {
                       str1[j+2] = str1[j];
                   }
                str1[0] = 48;
                str1[1] = 0x2E;
                }
                if (strlen(str1) == 1)
                {
                str1[3] = str1[0];
                str1[1] = 0x2E;
                str1[0] = 48;
                str1[2] = 48;
                }
                putsUart0("                 ");
                putsUart0(str1);                                                    //Print the CPU % after putting the decimal point
                }
            }
        }
    }
}
