#ifndef RTOS_H_

#define RTOS_H_

#include <stdint.h>
#include <stdbool.h>
#include<string.h>
#include<stdlib.h>
#include "tm4c123gh6pm.h"

// Bitbanding references for the off-board LEDs, SVC numbers and small constants
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board orange LED
#define Pushbutton_0 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // off-board pushbutton
#define Pushbutton_1 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // off-board pushbutton
#define Pushbutton_2 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // off-board pushbutton
#define Pushbutton_3 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) // off-board pushbutton
#define Pushbutton_4 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) // off-board pushbutton
#define SVC_YIELD                    100                                                        //SVC number for Yield
#define SVC_SLEEP                   101                                                        //SVC number for Sleep
#define SVC_WAIT                    102                                                        //SVC number for Wait
#define SVC_POST                    103                                                        //SVC number for Post
#define SVC_KILL                    104                                                        //SVC number for Kill
#define SVC_IPCS                    105                                                        //SVC number for IPCS
#define SVC_PS                      200                                                        //SVC number for PS
#define ON                           1                                                         //ON==1
#define OFF                          0                                                         //OFF==0
#define SVC_STACK_PC_OFFSET          88
#define STACK_SEED_SIZE              52
#define SYSTICK_RELOAD_1MS         40000

#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
#define MAX_TASKS 10       // maximum number of valid tasks

// task states
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

// function pointer
typedef void (*_fn)();

struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char name[20];
    uint32_t currentUser;
};

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // -8=highest to 7=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    int16_t skip;                  //Number of times a task can be skipped before it has to be scheduled
    int16_t skip_count;            //Number of times a task has been skipped
    uint64_t time;                 //Time taken by each task per interval for each thread
    uint64_t total_time1;          //Addition of all the times from the intervals-Buffer 1 for each thread
    uint64_t total_time2;          //Addition of all the times from the intervals-Buffer 2 for each thread
    uint64_t cpu;                  //CPU % for each thread
    int8_t processCounter;          //Used to create a thread using the command <process_name> &
};

// EXPORTS (externs) - the definitions live in 01_rtos.c
extern uint8_t max_chars, count, c, arg_count, position[], i, min_args, j;
extern char input_string[], type[], str1[];
extern char* string_verb;
extern volatile uint32_t timer_count;
extern uint32_t l, m, directory[];
extern volatile uint64_t time_all_tasks1, time_all_tasks2;
extern volatile bool scheduler, priority_inheritance, preempt;


extern struct semaphore semaphores[];
extern uint8_t semaphoreCount;
extern struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;
extern struct semaphore *semaphore_pointer;

extern volatile uint8_t taskCurrent;
extern uint8_t taskCount;
extern uint32_t stack[][256];
extern struct _tcb tcb[];
extern uint32_t stack_variable;

// Function prototypes (so files can call across boundaries)
// Kernel / thread API
void rtosInit(void);
int rtosScheduler(void);
void rtosStart(void);

bool createThread(_fn fn, char name[], int priority);
void destroyThread(_fn fn);
void setThreadPriority(_fn fn, uint8_t priority);

struct semaphore* createSemaphore(char name[], uint8_t count);

// SVC wrappers
void yield(void);
void sleep(uint32_t tick);
void wait(struct semaphore *pSemaphore);
void post(struct semaphore *pSemaphore);

// ISRs and helpers
void systickIsr(void);
void pendSVISR(void);
void svCallISR(void);
uint32_t getSVC(uint32_t x);

// Hardware / utility functions (implemented in hw.c or tasks.c)
void initHw(void);
uint8_t readPbs(void);
void waitMicrosecond(uint32_t us);

// Stack/register helpers (implemented in hw.c)
void setsp(uint32_t x);
uint32_t getsp(void);
uint32_t getR0(void);
uint32_t getR1(void);
uint32_t getR2(void);

// UART and shell utilities (implemented in tasks.c)
void putcUart0(char c);
void putsUart0(char* str);
char getcUart0(void);
void getstring(void);
void parse_string(void);
bool isCommand(char *verb2[]);

// Task prototypes
void idle(void);
void flash4Hz(void);
void oneshot(void);
void partOfLengthyFn(void);
void lengthyFn(void);
void readKeys(void);
void debounce(void);
void uncooperative(void);
void important(void);
void shell(void);
void pidof(void);
void reset(void);
void ipcs(void);
void ps(void);

#endif /* RTOS_H */
