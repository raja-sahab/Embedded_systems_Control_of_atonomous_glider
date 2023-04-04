/*
 * File:   main.c
 * Authors: Awais Tahir (S5174335)
 *          Raja Farooq Dilshad (5245187)
 *
 * Created on December 20, 2022, 1:47 PM
 */



// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <p30F4011.h>
#include <stdio.h>
#include <string.h>
#include "timer.h"
#include "spi.h"
#include "parser.h"
#include "circular_buffer.h"
#include <math.h>

#define MAX_TASKS 3
#define FIRST_ROW 0
#define SECOND_ROW 1

#define KP 1.0 // Proportional gain
#define DT 0.1 // Time step in seconds
#define MIN_BATTERY_POSITION 0.0
#define MAX_BATTERY_POSITION 100.0
#define MAX_RPM_MOTOR_3 100
#define MIN_RUDDER_ANGLE -30
#define MAX_RUDDER_ANGLE 30
#define MAX_PITCH 20
#define MIN_PITCH -20
#define MIN_SPEED -2
#define MAX_SPEED 2

double current_position = 50.0; // Current position in mm
double current_rudder = 0.0;
double current_pitch = 0.0; 

int rpm_m1;
int rpm_m2;
int rpm_m3;



// The buffer that will contain data received from the UART
volatile circular_buffer in_buffer;
// The buffer that will contain data to send to the UART
volatile circular_buffer out_buffer;

parser_state pstate;

// The variables necessary for handling multiple states
typedef enum {CONTROLLED, TIMEOUT, SAFE} WORKING_MODE;
volatile WORKING_MODE current_mode = 0;

typedef struct {
    int n;
    int N;
} heartbeat;

void blink_led() {
    LATBbits.LATB0 = !LATBbits.LATB0;
}

void blink_led_timeout(){
    if(current_mode == TIMEOUT)
    {
    LATBbits.LATB1 = !LATBbits.LATB1;
    }
    else{
        LATBbits.LATB1 = 0;
    }
}

// Function to convert character to decimal value
int extract_integer(const char* str, int* n) {
    int i = 0, number = 0, sign = 1;
    if (str[i] == '-') {
        sign = -1;
        i++;
    } else if (str[i] == '+') {
        sign = 1;
        i++;
    }
    while (str[i] != ',' && str[i] != '\0') {
        if ((str[i] - '0') < 0 || (str[i] - '0') > 9)
            return -1;
        number *= 10; // multiply the current number by 10;
        number += str[i] - '0'; // converting character to decimal number
        i++;
    }
    *n = sign*number;
    return 0;
}

// Function to move to the next value
int next_value(const char* msg, int i) {
    while (msg[i] != ',' && msg[i] != '\0') {
        i++;
    }
    if (msg[i] == ',')
        i++;
    return i;
}

// Structure to store all three desired values from serial interface
typedef struct {
    int speed;
    int pitch;
    int rudder;
} sensor_data;
sensor_data sdata;

// Function to fill the structure by extracting values
int parse_rlhref(const char * msg) {
    int i = 0;
    extract_integer(msg, &sdata.speed);
    i = next_value(msg, i);
    extract_integer(msg + i, &sdata.pitch);
    i = next_value(msg, i);
    extract_integer(msg + i, &sdata.rudder);
    return 0;
}

// Function Calculate Percentage Duty Cycle for all Motors
float calculatePercentageDutyCycle(int rpm)
{
    float percentageDutyCycle;

    // Calculate the percentage duty cycle
    percentageDutyCycle = (float) rpm / 100.0;

    // Ensure the percentage duty cycle is within the given range
    if (percentageDutyCycle < 0.0445) {
        percentageDutyCycle = 0.0445;
    } else if (percentageDutyCycle > 0.9555) {
        percentageDutyCycle = 0.9555;
    }

    return percentageDutyCycle;
}

// Function computes rpms of motor 2 and motor 3
int compute_motor_3_RPM(float rudder_angle) {
  return (int) round((rudder_angle - MIN_RUDDER_ANGLE) / (MAX_RUDDER_ANGLE - MIN_RUDDER_ANGLE) * MAX_RPM_MOTOR_3);
}

float compute_battery_position(float pitch) {
  return MIN_BATTERY_POSITION + (pitch - MIN_PITCH) / (MAX_PITCH - MIN_PITCH) * (MAX_BATTERY_POSITION - MIN_BATTERY_POSITION);
}

double control_position(double current_position, double desired_position) {
  double error = 0.0; // Error signal
  double control_action = 0.0; // Control action
  
  // Check if desired position is within the range
  if (desired_position < MIN_BATTERY_POSITION) desired_position = MIN_BATTERY_POSITION;
  if (desired_position > MAX_BATTERY_POSITION) desired_position = MAX_BATTERY_POSITION;
  
  // Calculate the error signal
  error = desired_position - current_position;
    
  // Calculate the control action
  control_action = error * KP;
  
  rpm_m2 = compute_motor_3_RPM(control_action);
    
  // Update the current position
  current_position = current_position + (control_action) * DT;
    
  return current_position;
}

double control_rudder(double current_rudder, double desired_rudder) {
  double error = 0.0; // Error signal
  double control_action = 0.0; // Control action
  
  // Check if desired rudder is within the range
  if (desired_rudder < MIN_RUDDER_ANGLE) desired_rudder = MIN_RUDDER_ANGLE;
  if (desired_rudder > MAX_RUDDER_ANGLE) desired_rudder = MAX_RUDDER_ANGLE;
  
  // Calculate the error signal
  error = desired_rudder - current_rudder;
    
  // Calculate the control action
  control_action = error * KP;
  
  rpm_m3 = compute_motor_3_RPM(control_action);
    
  // Update the current rudder
  current_rudder = current_rudder + (control_action) * DT;
    
  return current_rudder;
}

double control_pitch(double current_pitch, double desired_pitch) {
  double error = 0.0; // Error signal
  double control_action = 0.0; // Control action
  
  // Check if desired rudder is within the range
  if (desired_pitch < MIN_PITCH) desired_pitch = MIN_PITCH;
  if (desired_pitch > MAX_PITCH) desired_pitch = MAX_PITCH;
  
  // Calculate the error signal
  error = desired_pitch - current_pitch;
    
  // Calculate the control action
  control_action = error * KP;
      
  // Update the current rudder
  current_pitch = current_pitch + (control_action) * DT;
    
  return current_pitch;
}

// Function to change working mode.
void enter_working_mode(int mode)
{
    // Storing the current mode
    current_mode = mode;
    // Handling the initialization of every mode
    if(current_mode == CONTROLLED)
    {
        // Enabling the TIMEOUT timer
        T2CONbits.TON = 1;
        // The TIMEOUT timer is reset for when the SAFE is exited
        TMR2 = 0;
    }
    else if(current_mode == TIMEOUT)
    {
        //Setting RPMs to 0 in TIMEOUT State
        rpm_m1 = 0;
        rpm_m2 = 0;
        rpm_m3 = 0;
    }
}

// The Timer2 interrupt fires when no message is received for 5 seconds.
// In that case, the TIMEOUT mode logic must be executed.
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt()
{
    // Resetting the interrupt flag
    IFS0bits.T2IF = 0;
    // Switching to timeout mode
    enter_working_mode(TIMEOUT);
}

// The Timer2 interrupt fires when no message is received for 5 seconds.
// In that case, the TIMEOUT mode logic must be executed.
void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt()
{
    // Resetting the interrupt flag
    IFS0bits.T3IF = 0;
    IEC1bits.INT1IE = 1;
    IFS1bits.INT1IF = 0;
    T3CONbits.TON = 0;
    //IFS0bits.T2IF = 0;
    // Switching to timeout mode
    //enter_working_mode(TIMEOUT);
}
    char str[16];
    char str2[16];

void lcd_s6(){
    char str7[20];
    sprintf(str7, "R = %d  ", sdata.rudder);
    spi_clear_first_row();
    spi_move_cursor(FIRST_ROW, 0);
    spi_put_string(str7);
    sprintf(str7, "P = %d  ", sdata.pitch);
    spi_move_cursor(FIRST_ROW, 8);
    spi_put_string(str7);
    sprintf(str7, "R= %.1f ", current_rudder);
    spi_clear_second_row();
    spi_move_cursor(SECOND_ROW, 0);
    spi_put_string(str7);
    sprintf(str7, "P= %.1f", current_pitch);
    spi_move_cursor(SECOND_ROW, 8);
    spi_put_string(str7);
}
int s6;
// DEFINING THE BUTTON INTERRUPT
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt() 
    {
    
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 0;
    s6 = (s6 + 1) % 2;
    tmr_setup_period(TIMER3, 50);
        
   }

char str3[16];
char str8[16];
char str9[16];
double duty_cycle_m1, duty_cycle_m2, duty_cycle_m3;

void print_lcd(){
    if (s6 == 0){
        sprintf(str3, "Speed=%d", sdata.speed);
        spi_clear_first_row();
        spi_move_cursor(FIRST_ROW, 0);
        spi_put_string(str3);
        sprintf(str9, "R = %d", rpm_m1);
        spi_clear_second_row();
        spi_move_cursor(SECOND_ROW, 0);
        spi_put_string(str9);
    } else if (s6 == 1) {
        lcd_s6();
    }
}


char str5[20];
char str6[20];
int motor_2_RPM;

float desired_Battery_Position;
float desired_pitch;
float desired_rudder;

// Task 1
void task1() {
    // The UART logic that needs to be executed in the main loop
    uart_main_loop();
    // Handling all the data in the input buffer
    char word;
    while (in_buffer.count != 0)
    {
        cb_pop_front(&in_buffer, &word);
        // Check if there is a new message to handle

        if(parse_byte(&pstate, word) != NEW_MESSAGE)
            continue;
            if (strcmp(pstate.msg_type, "HLREF") == 0) {
                parse_rlhref(pstate.msg_payload);
                enter_working_mode(CONTROLLED);
            } else {
                uart_send("ERR2");
            }
        }
        
        // Motor 1 Control System
        if (sdata.speed < MIN_SPEED) sdata.speed = MIN_SPEED;
        if (sdata.speed > MAX_SPEED) sdata.speed = MAX_SPEED;
        rpm_m1 = sdata.speed * 5000;
        duty_cycle_m1 = calculatePercentageDutyCycle(rpm_m1);
        
        // Motor 2 control system
        current_pitch = control_pitch(current_pitch, sdata.pitch);
        desired_Battery_Position = compute_battery_position(sdata.pitch);
        current_position = control_position(current_position, desired_Battery_Position);
        if (desired_Battery_Position != current_position)
        {
        duty_cycle_m2 = calculatePercentageDutyCycle(rpm_m2);
        }
        
        // Motor 3 Control System
        desired_rudder = sdata.rudder;
        current_rudder = control_rudder(current_rudder, desired_rudder);
        if(current_rudder != desired_rudder)
        {
            duty_cycle_m3 = calculatePercentageDutyCycle(rpm_m3);
        }
        
        PDC1 = duty_cycle_m1 * 2 * PTPER;
        PDC2 = duty_cycle_m2 * 2 * PTPER;
        PDC3 = duty_cycle_m3 * 2 * PTPER;
        sprintf(str5, "$MCPWM=%d,%d,%d", rpm_m1, rpm_m2, rpm_m3);
        sprintf(str6, "$MCPOS=%.1f,%.1f", current_rudder, current_position);
        uart_send(str6);
        print_lcd();
}

// Task2
void task2() {
    blink_led_timeout();
    uart_send(str5);
}

// Task 3
void task3() {
    blink_led();
}

// Scheduler
void scheduler(heartbeat schedInfo[]) {
    int i;
    for (i = 0; i < MAX_TASKS; i++) {
        schedInfo[i].n++;
        if (schedInfo[i].n >= schedInfo[i].N) {
            switch (i) {
                case 0:
                    task1();
                    break;
                case 1:
                    task2();
                    break;
                case 2:
                    task3();
                    break;
            }
            schedInfo[i].n = 0;
        }
    }
}

int main(void) {
    heartbeat schedInfo[MAX_TASKS];
    
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 0;
    
    // UART Initialization
    char in[20];    
    char out[50];   
    cb_init(&in_buffer, in, 20);                 // Init UART input buffer ()
    cb_init(&out_buffer, out, 40);               // Init UART output buffer ()
    uart_init(4800, &in_buffer, &out_buffer);    // Init UART with buffers
    
    // Set the button S5 pin as input
    TRISDbits.TRISD0 = 1;
    // Enables interrupt related to button S5
    IEC1bits.INT1IE = 1;
    IFS1bits.INT1IF = 0;

    // Timer Setup
    tmr_setup_period(TIMER1, 5);
    tmr_setup_period(TIMER2, 5000);

    // SPI Configuration
    SPI1CONbits.PPRE = 0b11; // 1:1
    SPI1CONbits.SPRE = 0b110; // 2:1
    SPI1CONbits.MSTEN = 1; //master
    SPI1CONbits.MODE16 = 0; // 8 bits
    SPI1STATbits.SPIEN = 1; // enable

    // Parser Configuration
    pstate.state = STATE_DOLLAR;
    pstate.index_type = 0;
    pstate.index_payload = 0;

    // UART Configuration
    U2BRG = 11; // 9600
    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN = 1;
    IEC1bits.U2RXIE = 1;

    //Scheduling Configuration
    schedInfo[0].n = 0;
    schedInfo[0].N = 20;
    schedInfo[1].n = 0;
    schedInfo[1].N = 40;
    schedInfo[2].n = 0;
    schedInfo[2].N = 200;

    // Init PWM to set the voltage to the armature of the DC motor
    PTCONbits.PTMOD = 0;        // free running mode
    PTCONbits.PTCKPS = 0b0;     // prescaler
    PWMCON1bits.PEN1H = 1;      // output high bit for PWM1
    PWMCON1bits.PEN1L = 1;      // output low bit for PWM1
    PWMCON1bits.PEN2H = 1;      // output high bit for PWM2
    PWMCON1bits.PEN2L = 1;      // output low bit for PWM2
    PWMCON1bits.PEN3H = 1;      // output high bit for PWM3
    PWMCON1bits.PEN3L = 1;      // output low bit for PWM3
    PTPER = 1842;               // time period
    PDC1 = 0;                   // duty cycle, at the beginning the motor is still
    PDC2 = 0;                   // duty cycle, at the beginning the motor is still
    PDC3 = 0;                   // duty cycle, at the beginning the motor is still
    DTCON1bits.DTAPS = 0b00;    // dead time prescaler
    DTCON1bits.DTA = 6;         // dead time
    PTCONbits.PTEN = 1;         // enable the PWM

    // Enabling the interrupt related to TIMER2
    IEC0bits.T2IE = 1;
    // Enabling the interrupt related to TIMER3
    IEC0bits.T3IE = 1;
    // Initializing the controlled execution mode
    enter_working_mode(CONTROLLED);
    
    while (1) {
        scheduler(schedInfo);
        tmr_wait_period(TIMER1);
    }
    return 0;
}

