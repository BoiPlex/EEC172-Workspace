//*****************************************************************************
// Multi-tap UART Messaging System
//*****************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_nvic.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "systick.h"

// Common interface includes
#include "uart_if.h"
#include "pinmux.h"


// the cc3200's fixed clock frequency of 80 MHz, note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 0;

extern void (* const g_pfnVectors[])(void);

volatile unsigned long SW2_intcount;
volatile unsigned long SW3_intcount;
volatile unsigned char SW2_intflag;
volatile unsigned char SW3_intflag;
static volatile int first_trigger = 1;

//volatile uint16_t data = 0;

volatile unsigned int received_data[32];  // Accumulated data bits
volatile unsigned int bit_count = 0;

const char* valid_chars[10] = {
    " ",     // 0
    NULL,    // 1 (unused)
    "ABC",   // 2
    "DEF",   // 3
    "GHI",   // 4
    "JKL",   // 5
    "MNO",   // 6
    "PQRS",  // 7
    "TUV",   // 8
    "WXYZ",  // 9
    NULL,    // 10 (unused)
    NULL,    // 11 - ENTER
    NULL     // 12 - DELETE
};

char message[128] = "";
int msg_index = 0;
char last_char = 0;
int btn_count = 0;
int last_btn = -1;
unsigned int last_btn_time = 0;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static const PinSetting switch2 = { .port = GPIOA2_BASE, .pin = 0x40};
static const PinSetting switch3 = { .port = GPIOA1_BASE, .pin = 0x4};

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

uint64_t current_time_us() {
    uint32_t current = SysTickValueGet();
    uint64_t elapsed_ticks = (SYSTICK_RELOAD_VAL - current);
    return TICKS_TO_US(elapsed_ticks + systick_cnt * SYSTICK_RELOAD_VAL);
}

/**
 * Reset SysTick Counter
 */
static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
    HWREG(NVIC_ST_CURRENT) = 1;

    // clear the global count variable
    systick_cnt = 0;
}



static void GPIOA1IntHandler(void) { // SW3 handler
    unsigned long ulStatus;
    // Clear the flag immediately to handle new interrupts
    ulStatus = MAP_GPIOIntStatus (GPIOA1_BASE, true);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus);        // clear interrupts on GPIOA1
    static uint32_t last_edge_time = 0;

    uint32_t current_time = SysTickValueGet();
    uint32_t delta_ticks;

    if (last_edge_time != 0) {
            // handle overflow if needed (SysTick is down-counting)
            if (last_edge_time > current_time) {
                delta_ticks = last_edge_time - current_time;
            } else {
                delta_ticks = (SYSTICK_RELOAD_VAL - current_time) + last_edge_time;
            }

            uint64_t delta_us = TICKS_TO_US(delta_ticks);

            if (delta_us >= 100 && delta_us <= 1500) {
                received_data[bit_count++] = 0;
            } else if (delta_us >= 1500 && delta_us <= 2500) {
                received_data[bit_count++] = 1;
            }

            if (bit_count >= 32) {
                SW3_intflag = 1;
                bit_count = 0;
                last_edge_time = 0;
                return;
            }
        }

        // Save time of this edge
        last_edge_time = current_time;
        SW3_intcount++;
}

static void GPIOA2IntHandler(void) {    // SW2 handler
    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus (switch2.port, true);
    MAP_GPIOIntClear(switch2.port, ulStatus);       // clear interrupts on GPIOA2
    SW2_intcount++;
    SW2_intflag=1;
}

/**
 * SysTick Interrupt Handler
 *
 * Keep track of whether the systick counter wrapped
 */
static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
}

/**
 * Initializes SysTick Module
 */
static void SysTickInit(void) {

    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();
}

static void
BoardInit(void) {
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);

    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

int get_btn(){
    uint16_t data = 0;
    int i;
    int bit_index = 0;
    for(i = 31; i > 0; i--)
    {
       if (received_data[i] == 0)
       {
           data |= (0 << bit_index);
           bit_index++;
       }
       else
       {
           data |= (1 << bit_index);
           bit_index++;
       }
    }
    // Report("\n %d \n", data);

    // Decode
    switch (data){
        case 0x18E7: return(0);
        case 0x807F: return(1);
        case 0x40BF: return(2);
        case 0xC03F: return(3);
        case 0xA05F: return(4);
        case 0x609F: return(5);
        case 0xE01F: return(6);
        case 0x906F: return(7);
        case 0x50AF: return(8);
        case 0xD02F: return(9);
        case 0x22DD: return(11); // ENTER
        case 0x926D: return(12); // DELETE
        default: return -1;
    }
// Reset bit count to 0
    if(bit_count >= 32)
    {
        bit_count = 0;
     }

}

static void handle_button(int btn){
    unsigned int current_time = systick_cnt; // current_time_us();

    if (btn == 11) { // ENTER
        message[msg_index] = '\0';
        Report("Sending message: %s\n", message);
        msg_index = 0;
        memset(message, 0, sizeof(message));
    }
    else if (btn == 12) { // DELETE
        if (msg_index > 0) {
            msg_index--;
            message[msg_index] = '\0';
        }
    }
    else if (btn >= 0 && btn <= 9) {
        if (btn == last_btn && current_time - last_btn_time < 20) {
            btn_count = (btn_count + 1) % strlen(valid_chars[btn]);
            message[msg_index - 1] = valid_chars[btn][btn_count];
        } else {
            btn_count = 0;
            message[msg_index++] = valid_chars[btn][btn_count];
        }
        last_btn = btn;
        last_btn_time = current_time;
    }

    Report("Message: %s\n", message);

//    if(btn == 0){
//        report(' ');
//    }
//    else if (btn == 11){//Enter

//    }
//    else if (btn == 12){//Delete

//    }
//    else{
//        if(last_btn != btn){//new char
//            pending_char = valid_char[btn-2][btn_count]
//        }
//        else{
//            if(btn_count < num_chars[btn-2]-1){
//                btn_count++;
//            }
//            else{
//                btn_count = 0;
//            }
//        }
//        del_char();
//        add_char(pending_char);
//        last_btn_time=Systick();
//        last_btn = btn;
//    }
}

//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//!
//! \return None.
//
//****************************************************************************
int main() {
    unsigned long ulStatus;

    BoardInit();
    PinMuxConfig();
    SysTickInit(); // Enable SysTick
    InitTerm();
    ClearTerm();

    // Register the interrupt handlers with each pin port
    MAP_GPIOIntRegister(GPIOA1_BASE, GPIOA1IntHandler);
    MAP_GPIOIntRegister(GPIOA2_BASE, GPIOA2IntHandler);
    // Configure rising edge interrupts
    MAP_GPIOIntTypeSet(switch3.port, switch3.pin, GPIO_FALLING_EDGE); // SW3
    MAP_GPIOIntTypeSet(switch2.port, switch2.pin, GPIO_FALLING_EDGE);    // SW2
    ulStatus = MAP_GPIOIntStatus(switch3.port, false);
    MAP_GPIOIntClear(switch3.port, ulStatus);           // clear interrupts on GPIOA1
    ulStatus = MAP_GPIOIntStatus(switch2.port, false);
    MAP_GPIOIntClear(switch2.port, ulStatus);           // clear interrupts on GPIOA2
    // clear global variables
    SW2_intcount=0;
    SW3_intcount=0;
    SW2_intflag=0;
    SW3_intflag=0;
    // Enable SW2 and SW3 interrupts
    MAP_GPIOIntEnable(switch3.port, switch3.pin);
    MAP_GPIOIntEnable(switch2.port, switch2.pin);

    Message("\t\t****************************************************\n\r");
    Message("\t\t\t IR Control Handler \n\r");
    Message("\t\t ****************************************************\n\r");
    Message("\n\n\n\r");
    // Report("SW2 ints = %d\tSW3 ints = %d\r\n",SW2_intcount,SW3_intcount);

    while (1) {
        if (SW3_intflag) {
            SW3_intflag=0;  // clear flag
            int btn = get_btn();
            Report("Button pressed: %i\n", btn);
            handle_button(btn);
        }
    }
    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
