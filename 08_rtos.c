// RTOS Framework - Spring 2020
// J Losh

// Student Name:
// TO DO: Nikita Nitin Kontamwar

// Add xx_ prefix to all files in your project
// 08_rtos.c
// 08_tm4c123gh6pm_startup_ccs.c
// 08_other files (except uart0.x and wait.x)
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PA2
// Orange: PA3
// Yellow: PA4
// Green:  PE0
// PBs on these pins
// PB0:    PC4
// PB1:    PC5
// PB2:    PC6
// PB3:    PC7
// PB4:    PD6
// PB5:    PD7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"
//#include <string.h>

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // off-board orange LED

#define BLUE_LED_MASK 4
#define RED_LED_MASK 4
#define ORANGE_LED_MASK 8
#define YELLOW_LED_MASK 16
#define GREEN_LED_MASK 1

#define PB0               (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))     //PC4
#define PB1              (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))      //PC5
#define PB2               (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))     //PC6
#define PB3               (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))     //PC7
#define PB4               (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4)))     //PD6
#define PB5               (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 7*4)))     //PD7

#define PUSH_BUTTON_0_MASK 16
#define PUSH_BUTTON_1_MASK 32
#define PUSH_BUTTON_2_MASK 64
#define PUSH_BUTTON_3_MASK 128
#define PUSH_BUTTON_4_MASK 64
#define PUSH_BUTTON_5_MASK 128
#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP\n")

extern void setPsp(uint32_t*);
extern uint32_t* getPsp();
extern void push_R4_R11();
extern void pop_R4_R11();
extern uint32_t get_R0();

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char sem_Name[15];
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;
struct semaphore *s;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 10      // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

uint32_t stack[10][512];

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;     // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];

///variables for shell

#define MAX_CHARS 30
char str[MAX_CHARS + 1];
#define SPECIAL_CHARS (str[i]== 9 || str[i]==' '|| str[i]== 47|| (str[i]>=33 && str[i]<=37) || (str[i]>=39 && str[i]<=44) || (str[i]>=58 && str[i]<=64) ||(str[i]>=91 && str[i]<=96) ||(str[i]>=123 && str[i]<=127))
#define ALPHA_NUM ((str[i+1]==38)||(str[i+1]==45)||(str[i+1]==46)||(str[i+1]>=48 && str[i+1]<=57)||(str[i+1]>=65 && str[i+1]<=90)||(str[i+1]>=97 && str[i+1]<=122))
char ans[MAX_CHARS][MAX_CHARS];

uint8_t pos[MAX_CHARS]; //position of each word that is the starting index of each word
uint8_t argCount;   //total number of words

uint32_t proc_pid;

static uint8_t preemptiveEnable;    // For Setting the Preemptive Scheduler Enable Mode
static uint8_t priorityEnable;        // For Setting the Priority Scheduler Enable Mode
static uint8_t priorityInheritance;                // For Enabling Priority Inheritance

struct cpu_time_calc
{
    uint32_t time_r;
    uint32_t time_a;
    uint32_t time_p;

} cpuTime[MAX_TASKS];

static uint32_t initial_time = 0, final_time = 0;
uint32_t flag = 0;

uint32_t t_time = 0;

//function prototypes

void copy_string(char *,char *);
void putcUart0(char);
void putsUart0(char*);
char getcUart0();
void getString(void);
void parseString();
int compare(char *,char *);
bool isCommand(char*,uint8_t);
char* getArgString(uint8_t);
uint32_t getArgInt(uint8_t);
char* reverse(char *);
char* itoa(int,char *);
int my_atoi(char*);
uint8_t getSvcNo(void);
void timer_init();
void waitMicrosecond(uint32_t);

void yield();
void sleep(uint32_t);
void wait(struct semaphore*);
void post(struct semaphore*);
void systickIsr();
void pendSvIsr();
void initHw();
uint8_t readPbs();

void idle();
void flash4Hz();
void oneshot();
void partOfLengthyFn();
void lengthyFn();
void readKeys();
void debounce();
void uncooperative();
void important();
void shell_cmds(void);
void shell();


void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");
    __asm("WMS_LOOP1:   SUB  R1, #1");
    __asm("             CBZ  R1, WMS_DONE1");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             B    WMS_LOOP1");
    __asm("WMS_DONE1:   SUB  R0, #1");
    __asm("             CBZ  R0, WMS_DONE0");
    __asm("             NOP");
    __asm("             B    WMS_LOOP0");
    __asm("WMS_DONE0:");
    // 40 clocks/us + error
}

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;

    preemptiveEnable = 1;
    priorityEnable = 1;
    priorityInheritance = 1 ;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    static uint8_t count = 0;
    int value = 0;
    int8_t p = 0;               //priority
    int i = 0;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        if (priorityEnable == 1)
        {
            while (p <= 15)
            {
                for (i = 1; i <= MAX_TASKS; i++)
                {
                    value = (i + count - 1) % MAX_TASKS;
                    ok = (tcb[value].state == STATE_READY
                            || tcb[value].state == STATE_UNRUN);
                    if (tcb[value].currentPriority == p && ok == true)
                    {
                        task = value;
                        count = value + 1;
                        return task;
                    }
                }
                p++;
            }
        }
        else
        {
            ok = (tcb[task].state == STATE_READY
                    || tcb[task].state == STATE_UNRUN);
            //  return task;
        }
    }
    return task;
}

bool createThread(_fn fn, char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name
    // add task if room in task list
    // allocate stack space for a thread and assign to sp below
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid == fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID)
            {
                i++;
            }
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = &stack[i][512];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            copy_string(tcb[i].name, name);
            // increment task count
            taskCount++;
            ok = true;
            tcb[i].spInit = tcb[i].sp;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
    __asm(" SVC #26");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm(" SVC #25");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    int k = 0;

    for(k=0;k<taskCount; k++)
    {
        if(tcb[k].pid == fn)
        {
            tcb[k].currentPriority = priority;
            break;
        }
    }
}

struct semaphore* createSemaphore(char* sem_Name, uint8_t count)
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
        copy_string(pSemaphore->sem_Name, sem_Name);
    }
    return pSemaphore;
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{
    _fn fn;
    taskCurrent = rtosScheduler();
    setPsp(tcb[taskCurrent].sp);
    fn = (_fn) tcb[taskCurrent].pid;
    tcb[taskCurrent].state = STATE_READY;
    //setPsp(tcb[taskCurrent].sp);
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    (*fn)();

}

void svCallIsr()
{
    //    BLUE_LED = 1;
    //    waitMicrosecond(100);
    //    BLUE_LED = 0;
    uint32_t R0 = get_R0();     //gets value stored in R0 register
    uint8_t N;
    uint32_t create_th;
    int a, b, c;
    int z;
    int i = 0, j,q;
    N = getSvcNo();         //gets SVC number value
    switch (N)
    {
    case 21:               //yield
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV; //send PendSvcIsr bit to switch task
        break;

    case 22:        //sleep
        tcb[taskCurrent].state = STATE_DELAYED; //state gets blocked and will not run until the state is ready
        tcb[taskCurrent].ticks = R0;                //sets sleep timeout value
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;

    case 23:        //wait
        s = (struct semaphore*) R0;
        if (s->count > 0)               //check semaphore count variable
        {
            s->count--;
            tcb[taskCurrent].semaphore = s;    //stores the pointer to semaphore
        }
        else
        {
            s->processQueue[s->queueSize] = (uint32_t) tcb[taskCurrent].pid; //task stored in semaphore process queue
            s->queueSize++;         //increment the queue size for next task
            tcb[taskCurrent].state = STATE_BLOCKED; //current task's state is blocked
            tcb[taskCurrent].semaphore = s;    //stores the pointer to semaphore

            if (priorityInheritance == 1)        // For Priority Inheritance
            {
                for (z = 0; z < MAX_TASKS; z++)
                {
                    if (tcb[z].semaphore == tcb[taskCurrent].semaphore) // check for semaphore being shared
                    {
                        if (tcb[z].currentPriority > tcb[taskCurrent].currentPriority)
                        {
                            tcb[z].currentPriority = tcb[taskCurrent].currentPriority;
                        }
                        // break;
                    }
                }
            }

            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        }

        break;

    case 24:            //post
        s = (struct semaphore*) R0; //argument passed in post()

        s->count++; //increase semaphore count so that other task can use the resource
        tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority; //pi reverted
        if (s->queueSize > 0)       //if task waiting in queue
        {
            for (j = 0; j < MAX_TASKS; j++)
            {
                if (s->processQueue[0] == (uint32_t) tcb[j].pid)
                {
                    tcb[j].state = STATE_READY; //make state ready from blocked state, of the task to be released
                    s->processQueue[0] = 0;      //release task waiting in queue
                    s->count--; //decrement semaphore count since resource freed by the task released

                    while (i < s->queueSize)
                    {
                        s->processQueue[i] = s->processQueue[i + 1]; //shift tasks of the semaphore process queue up by 1
                        i++;
                    }

                    s->queueSize--;

                    break;
                }

            }
        }

        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;

    case 25:        //kill

        proc_pid = R0;

        for (a = 0; a < MAX_TASKS; a++)
        {
            if (tcb[a].pid == (_fn) proc_pid) // Search for requested task in list of PIDs
            {
                tcb[a].state = STATE_INVALID; // Make the state invalid so that it cannot be scheduled
                taskCount--;
                if (tcb[a].semaphore != 0)
                {
                    s = tcb[a].semaphore;
                    for (b = 0; b < s->queueSize; b++)
                    {
                        if (s->processQueue[b] == proc_pid)
                        {
                            s->processQueue[b] = 0;
                            for (c = b; c < s->queueSize; c++)
                            {
                                s->processQueue[c] = s->processQueue[c + 1];

                            }
                            s->queueSize--;
                            break;
                        }
                    }
                    break;
                }

                proc_pid = 0;
                putsUart0("Thread Killed\t");

                //  tcb[a].pid = 0;
                //tcb[a].sp = 0;
                //tcb[a].semaphore = 0;
                // tcb[a].priority = 0;
                // tcb[a].currentPriority = 0;
                // for (z = 0; z < 16; z++)
                //   tcb[a].name[z] = '\0';
                break;

            }

        }

        break;

    case 26:        //create Thread
        create_th =R0;
        for(q=0;q<MAX_TASKS;q++)
        {
            if(tcb[q].pid==(_fn)create_th)
            {
                tcb[q].state= STATE_UNRUN;
                tcb[q].sp = tcb[q].spInit;
                break;
            }

        }
        break;
    }

}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    __asm(" SVC #21");
    //YELLOW_LED = 1;
    //waitMicrosecond(1000);
    // YELLOW_LED = 0;

}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{

    __asm(" SVC #22");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm(" SVC #23");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm(" SVC #24");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    int i;

    flag++;
    //  uint32_t time_x;
    uint8_t x=0,y=0,z=0;


    for (i = 0; i < MAX_TASKS; i++)
    {
        if (tcb[i].state == STATE_DELAYED)
        {

            if (tcb[i].ticks > 0)
                tcb[i].ticks--;
            else if (tcb[i].ticks == 0)
                tcb[i].state = STATE_READY;

        }

    }

    final_time = TIMER1_TAV_R;
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    cpuTime[taskCurrent].time_r = final_time - initial_time;


    if(flag == 100)
    {
        while(x < MAX_TASKS)
        {

            cpuTime[x].time_a = ((7 * cpuTime[x].time_a)
                    + cpuTime[x].time_r )>> 8;


            x++;

        }
        t_time = 0;

        while( y < MAX_TASKS)
        {
            t_time = t_time + cpuTime[y].time_a;
            y++;

        }
        while(z < MAX_TASKS)
        {
            cpuTime[z].time_p = (10000
                    * cpuTime[z].time_a) / t_time;
            cpuTime[z].time_r = 0;
            z++;
        }

        flag = 0;
    }
    if (preemptiveEnable == 1 && taskCurrent != 0)
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    //    RED_LED = 1;
    //    waitMicrosecond(100);
    //    RED_LED = 0;

    push_R4_R11();
    tcb[taskCurrent].sp = getPsp();
    //    __asm(" MOV R11,#1");
    //    __asm(" MOV R10,#2");
    //    __asm(" MOV R9,#3");
    //    __asm(" MOV R8,#4");
    //    __asm(" MOV R7,#5");
    //    __asm(" MOV R6,#6");
    //    __asm(" MOV R5,#7");
    //    __asm(" MOV R4,#8");
    final_time = TIMER1_TAV_R;
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_TAV_R = 0;

    if (tcb[taskCurrent].state != STATE_INVALID)        //to consider killed task
    {
        cpuTime[taskCurrent].time_r = final_time - initial_time;
        final_time = 0;
        initial_time = 0;
    }

    taskCurrent = rtosScheduler();

    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    initial_time = TIMER1_TAV_R;

    if (tcb[taskCurrent].state == STATE_READY)
    {
        setPsp(tcb[taskCurrent].sp);
        pop_R4_R11();
    }
    else
    {
        setPsp(tcb[taskCurrent].sp);
        uint32_t *p;
        p = getPsp();
        p--;
        *p = 16777216;       //PUSH xPSR
        p--;
        *p = (uint32_t) tcb[taskCurrent].pid;
        p--;
        p--;
        p--;
        p--;
        p--;
        p--;
        setPsp(p);
        tcb[taskCurrent].state = STATE_READY;
    }
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons, and uart
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO ports A,E and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC
            | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;



    GPIO_PORTA_DIR_R = RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK; // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTE_DIR_R = GREEN_LED_MASK;
    GPIO_PORTF_DIR_R = BLUE_LED_MASK;
    GPIO_PORTA_DR2R_R = RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DR2R_R = GREEN_LED_MASK;
    GPIO_PORTF_DR2R_R = BLUE_LED_MASK;
    GPIO_PORTA_DEN_R = RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK; // enable LEDs
    GPIO_PORTE_DEN_R = GREEN_LED_MASK;
    GPIO_PORTF_DEN_R = BLUE_LED_MASK;

    GPIO_PORTD_LOCK_R = 0X4C4F434B;
    GPIO_PORTD_CR_R = 0x000000FF;

    GPIO_PORTC_DEN_R = PUSH_BUTTON_0_MASK | PUSH_BUTTON_1_MASK
            | PUSH_BUTTON_2_MASK | PUSH_BUTTON_3_MASK;
    GPIO_PORTD_DEN_R = PUSH_BUTTON_4_MASK | PUSH_BUTTON_5_MASK;
    GPIO_PORTC_PUR_R = PUSH_BUTTON_0_MASK | PUSH_BUTTON_1_MASK
            | PUSH_BUTTON_2_MASK | PUSH_BUTTON_3_MASK; // enable internal pull-up for push button
    GPIO_PORTD_PUR_R = PUSH_BUTTON_4_MASK | PUSH_BUTTON_5_MASK;

    //UART Config
    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2; // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3; // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3; // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;       // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
    // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other UARTs in same status
    delay4Cycles();
    // wait 4 clock cycles
    UART0_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
    // enable TX, RX, and module

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;      //timer clock
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;

}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t button_val = 0;

    if (!PB0)
        button_val = 1;
    if (!PB1)
        button_val = 2;
    if (!PB2)
        button_val = 4;
    if (!PB3)
        button_val = 8;
    if (!PB4)
        button_val = 16;
    if (!PB5)
        button_val = 32;

    return button_val;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------


void copy_string(char *str_1, char *str_2)
{

    while (*str_2)
    {
        *str_1 = *str_2;

        str_1++;
        str_2++;

    }

    *str_1 = '\0';

}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF)
        yield();               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i, l;
    for (l = 0; str[l] != '\0'; ++l)
        ;
    for (i = 0; i < l; i++)
        putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
        yield();               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

void getString(void)
{
    uint8_t count = 0;
    char c;
    while (1)
    {
        A: c = getcUart0();
        putcUart0(c);
        if ((c == 8) || (c == 127))
        {
            if (count > 0)
            {
                count--;
                goto A;
            }
            else
            {
                goto A;
            }
        }
        else
        {
            if ((c == 10) || (c == 13))
            {
                B: str[count] = 0;
                break;
            }
            else
            {
                if (c >= 32)
                {
                    str[count++] = c;
                    if (count == MAX_CHARS)
                        goto B;
                    else
                        goto A;

                }

                else
                    goto A;

            }
        }
    }

}

void parseString()
{

    uint8_t i, l, j = 0, k = 0, p = 0;
    for (l = 0; str[l] != '\0'; ++l)
        ;

    if ((str[0] >= 48 && str[0] <= 57) || (str[0] >= 65 && str[0] <= 90)
            || (str[0] >= 97 && str[0] <= 122))
    {
        pos[0] = 0;
        p++;
    }
    for (i = 0; i <= l; i++)
    {
        if ((str[i] == ' ') || SPECIAL_CHARS)
        {
            if (ALPHA_NUM)
            {
                pos[p] = i + 1;
                p++;
                k++;
                j = 0;
            }
            str[i] = '\0';
        }
        else
        {
            ans[k][j] = str[i];
            j++;
        }
    }
    argCount = k + 1;
}

int compare(char *string1, char *string2)
{
    while (*string1 == *string2)
    {
        if (*string1 == '\0' && *string2 == '\0')
            return 1;
        string1++;
        string2++;
    }
    return 0;
}

bool isCommand(char* cmd, uint8_t argNo)
{
    bool cmp;
    cmp = compare(&str[pos[0]], cmd);
    if (argNo == (argCount - 1) && cmp == 1)
        //        putsUart0("valid command");
        //        putsUart0("\r\n");
        return true;

    else
    {
        //        putsUart0("invalid command");
        //        putsUart0("\r\n");
        return false;
    }
}

char* getArgString(uint8_t argNo)
{
    if (argNo < argCount)
        return &str[pos[argNo]];

    else
        return "invalid argument number";
}

uint32_t getArgInt(uint8_t argNo)
{

    uint32_t res = 0;
    int i;
    for (i = 0; str[i] != '\0'; ++i)
        res = res * 10 + str[i] - '0';
    return res;

}

char* reverse(char s[])             //site referred - https://en.wikibooks.org/wiki/C_Programming/stdlib.h/itoa
{
    int i, j,l;
    char c;
    for (l = 0; s[l] != '\0'; ++l);
    ;
    for (i = 0, j = l - 1; i < j; i++, j--)
    {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
    return s;
}

char* itoa(int n, char s[])             //site referred - https://en.wikibooks.org/wiki/C_Programming/stdlib.h/itoa
{
    int i, sign;

    if ((sign = n) < 0) /* record sign */
        n = -n; /* make n positive */
    i = 0;
    do
    { /* generate digits in reverse order */
        s[i++] = n % 10 + '0'; /* get next digit */
    }
    while ((n /= 10) > 0); /* delete it */
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    return reverse(s);
}

int my_atoi(char* str_x)            //site referred- https://www.geeksforgeeks.org/write-your-own-atoi/
{
    int res = 0, i; // Initialize result

    // Iterate through all characters of input string and
    // update result
    for (i = 0; str_x[i] != '\0'; ++i)
        res = res * 10 + str_x[i] - '0';

    // return result.
    return res;
}

uint8_t getSvcNo(void)
{
    uint32_t *x;
    uint32_t y;
    uint32_t *z;
    uint8_t w;
    x = getPsp();
    x++;
    x++;
    x++;
    x++;
    x++;
    x++;
    y = *x;
    y--;
    y--;
    z = y;
    w = *z;
    return w;
}

void shell_cmds(void)
{
    if (isCommand("ps", 0))         //ps
    {
        int k;
        char s_pid[10];
        char s_priority[10];
        char s_state[10];
        char s_time_1[10];
        char s_time_2[10];

        uint32_t p1;
        uint32_t p2;

        putsUart0("STATES:\r\n");
        putsUart0("0 - STATE_INVALID \r\n");
        putsUart0("1 - STATE_UNRUN \r\n");
        putsUart0("2 - STATE_READY \r\n");
        putsUart0("3 - STATE_DELAYED \r\n");
        putsUart0("4 - STATE_BLOCKED \r\n");

        putsUart0("\r\n");

        putsUart0("PID \t");
        putsUart0("PROCESS NAME \t");
        putsUart0("PRIORITY \t ");
        putsUart0("STATE \t");
        putsUart0("CPU TIME \t");

        for (k = 0; k < 9; k++)
        {
            putsUart0("\r\n");
            itoa((uint32_t) tcb[k].pid, s_pid);
            putsUart0(s_pid);
            putsUart0("  ");
            putsUart0(tcb[k].name);
            putsUart0("\t");
            putsUart0("\t");
            itoa((uint8_t) tcb[k].currentPriority, s_priority);
            putsUart0(s_priority);
            putsUart0("\t");
            putsUart0("\t  ");
            itoa((uint8_t) tcb[k].state, s_state);
            putsUart0(s_state);
            putsUart0("\t");
            p1 = cpuTime[k].time_p/100;
            itoa(p1, s_time_1);
            p2 = cpuTime[k].time_p%100;
            itoa(p2, s_time_2);
            putsUart0(s_time_1);
            putcUart0('.');
            putsUart0(s_time_2);
        }
    }


    if (isCommand("ipcs", 0))           //ipcs
    {
        uint32_t t = 0;
        char s1[33], s2[33];
        putsUart0("SEMAPHORE NAME \t");
        putsUart0("COUNT \t");
        putsUart0("QUEUE SIZE \t");
        while (t < 4)
        {
            putsUart0("\r\n");
            putsUart0(semaphores[t].sem_Name);
            putsUart0("\t");
            itoa(semaphores[t].count, s1);
            putsUart0(s1);
            putsUart0("\t");
            itoa(semaphores[t].queueSize, s2);
            putsUart0(s2);
            t++;

        }
    }

    if (isCommand("kill", 1))               //kill
    {
        int h;
        uint32_t kill_pid = my_atoi(getArgString(1));
        for(h=0;h<MAX_TASKS;h++)
        {
            if(tcb[h].pid==(_fn)kill_pid )
            {
                if(compare(tcb[h].name,"Idle")==1)
                {
                    putsUart0("Cannot kill Idle");
                    break;
                }
                else
                {
                    destroyThread((_fn) kill_pid);
                    break;
                }
            }
        }
    }


    if (isCommand("reboot", 0))         //to restart controller
    {
        putsUart0("System Reboot");
        NVIC_APINT_R = 0x04 | (0x05FA << 16);
    }

    if (isCommand("pidof", 1))          //to get process id
    {
        int c;
        char x[33];
        int i, flag = 0;

        putsUart0("pidof called \r\n");

        for (i = 0; i < MAX_TASKS; i++)
        {
            char *cmp = getArgString(1);
            c = compare(cmp, tcb[i].name);
            if (c == 1)
            {
                flag = 1;
                putsUart0("PID:");
                itoa((uint32_t) tcb[i].pid, x);
                putsUart0(x);
                putsUart0("\r\n");
            }
            if (flag == 1)
            {
                break;
            }

        }
    }

    if (isCommand("premption", 1))          //premption control
        if (isCommand("premption", 1))
        {
            if (compare(getArgString(1), "on") == 1)
            {
                if (preemptiveEnable == 1)
                    putsUart0("Preemptive scheduler already ON \r\n");

                else
                {
                    preemptiveEnable = 1;
                    putsUart0("Preemptive scheduler ON\r\n");
                }
            }

            else if (compare(getArgString(1), "off") == 1)
            {
                preemptiveEnable = 0;
                putsUart0("Preemptive scheduler OFF \r\n");

            }
        }

    if (isCommand("sched", 1))           //scheduler
    {
        if (compare(getArgString(1), "on") == 1)
        {
            if (priorityEnable == 1)
                putsUart0("Priority scheduler already ON \r\n");

            else
            {
                priorityEnable = 1;
                putsUart0("Priority scheduler ON\r\n");
            }
        }

        else if (compare(getArgString(1), "off") == 1)
        {
            priorityEnable = 0;
            putsUart0("Priority scheduler OFF and Round Robin Scheduler ON \r\n");

        }
    }



    if (isCommand("pi", 1))             //priority inheritance on/off
    {
        if (compare(getArgString(1), "on") == 1)
        {
            if (priorityInheritance == 1)
                putsUart0("Priority Inheritance already ON \r\n");

            else
            {
                priorityInheritance = 1;
                putsUart0("Priority Inheritance ON\r\n");
            }
        }

        else if (compare(getArgString(1), "off") == 1)
        {
            priorityInheritance = 0;
            putsUart0("Priority Inheritance OFF \r\n");

        }
    }


    if(compare("&",getArgString(1))==1)          //restart thread after killing it
    {
        int d;
        uint32_t create_pid;
        for(d=0;d<MAX_TASKS;d++)
        {
            if(compare(tcb[d].name,getArgString(0))==1)
            {
                create_pid = (uint32_t)tcb[d].pid;
                restartThread((_fn) create_pid);
            }
        }
    }

    if (isCommand("help", 0))
    {
        putsUart0("LIST OF COMMANDS\r\n");
        putsUart0("ps  -for process status\r\n");
        putsUart0("ipcs  -for Inter Process Communication Status\r\n");
        putsUart0("reboot   -to restart \r\n");
        putsUart0("pidof thread_name  -to get process ID of a thread\r\n");
        putsUart0("kill pid  -to kill a thread\r\n");
        putsUart0("fn_name &   -to restart a thread\r\n");
        putsUart0("sched   -on -> priority Scheduler    off -> Round-robin Scheduler  \r\n");
        putsUart0("premption on/off \r\n");
        putsUart0("pi   -on/off Priority Inheritance \r\n");

    }

}

void timer_init()
{
    NVIC_ST_CTRL_R = 0; // disable SysTick during setup
    NVIC_ST_CURRENT_R = 0;      // any write to current clears it
    NVIC_ST_RELOAD_R = 39999;  // reload value
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;       // enable SysTick with core clock and interrupts
}




// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while (true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

//void idle_2()
//{
//    while (true)
//    {
//        YELLOW_LED = 1;
//        waitMicrosecond(100000);
//        YELLOW_LED = 0;
//        yield();
//    }
//
//}

void flash4Hz()
{
    while (true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while (true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while (true)
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
    while (true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)         //PB0
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)         //PB1
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)         //PB2
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)         //PB3
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)        //PB4
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while (true)
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
    while (true)
    {
        while (readPbs() == 32)
        {
        }
        yield();
    }
}

void important()
{
    while (true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}


// REQUIRED: add processing for the shell commands through the UART here
//           your solution should not use C library calls for strings, as the stack will be too large
void shell()
{

    while (true)
    {
        putsUart0("\r\n");
        putsUart0("\r\n");
        putsUart0("Enter String - ");
        getString();
        parseString();
        putsUart0("\r\n");
        shell_cmds();

        yield();
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    timer_init();
    initRtos();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    putsUart0("\r\n");
    putsUart0("Enter help for commands \r\n");


    // Initialize semaphores
    keyPressed = createSemaphore("keyPressed", 1);
    keyReleased = createSemaphore("keyReleased", 0);
    flashReq = createSemaphore("flashReq", 5);
    resource = createSemaphore("resource", 1);

    // Add required idle process at lowest priority
    ok = createThread(idle, "Idle", 15, 1024);
    //  ok = createThread(idle_2, "Idle_2", 15, 1024);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
    ok &= createThread(oneshot, "OneShot", 4, 1024);
    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
    ok &= createThread(debounce, "Debounce", 12, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 12, 1024);
    ok &= createThread(shell, "Shell", 12, 1024);



    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
