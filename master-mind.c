/*
 * MasterMind implementation: template; see comments below on which parts need to be completed
 * CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
 * This repo: https://gitlab-student.macs.hw.ac.uk/f28hs-2021-22/f28hs-2021-22-staff/f28hs-2021-22-cwk2-sys

 * Compile: 
 gcc -c -o lcdBinary.o lcdBinary.c
 gcc -c -o master-mind.o master-mind.c
 gcc -o master-mind master-mind.o lcdBinary.o
 * Run:     
 sudo ./master-mind

 OR use the Makefile to build
 > make all
 and run
 > make run
 and test
 > make test

 ***********************************************************************
 * The Low-level interface to LED, button, and LCD is based on:
 * wiringPi libraries by
 * Copyright (c) 2012-2013 Gordon Henderson.
 ***********************************************************************
 * See:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
*/

/* ======================================================= */
/* SECTION: includes                                       */
/* ------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <unistd.h>
#include <string.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

/* --------------------------------------------------------------------------- */
/* Config settings */
/* you can use CPP flags to e.g. print extra debugging messages */
/* or switch between different versions of the code e.g. digitalWrite() in Assembler */
#define DEBUG
#undef ASM_CODE

// =======================================================
// Tunables
// PINs (based on BCM numbering)
// For wiring see CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
// GPIO pin for green LED
#define LED 13
// GPIO pin for red LED
#define LED2 5
// GPIO pin for button
#define BUTTON 19
// =======================================================
// delay for loop iterations (mainly), in ms
// in mili-seconds: 0.2s
#define DELAY   200
// in micro-seconds: 3s
#define TIMEOUT 3000000
// =======================================================
// APP constants   ---------------------------------
// number of colours and length of the sequence
#define COLS 3
#define SEQL 3
// =======================================================

// generic constants

#ifndef	TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(1==2)
#endif

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define	INPUT			 0
#define	OUTPUT			 1

#define	LOW			 0
#define	HIGH			 1


// =======================================================
// Wiring (see inlined initialisation routine)

#define STRB_PIN 24
#define RS_PIN   25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

/* ======================================================= */
/* SECTION: constants and prototypes                       */
/* ------------------------------------------------------- */

// =======================================================
// char data for the CGRAM, i.e. defining new characters for the display

static unsigned char newChar [8] = 
{
  0b11111,
  0b10001,
  0b10001,
  0b10101,
  0b11111,
  0b10001,
  0b10001,
  0b11111,
} ;

/* Constants */

static const int colors = COLS;
static const int seqlen = SEQL;

static char* color_names[] = { "red", "green", "blue" };

static int* theSeq = NULL;

static int *seq1, *seq2, *cpy1, *cpy2;

/* --------------------------------------------------------------------------- */

// data structure holding data on the representation of the LCD
struct lcdDataStruct
{
  int bits, rows, cols ;
  int rsPin, strbPin ;
  int dataPins [8] ;
  int cx, cy ;
} ;

static int lcdControl ;

/* ***************************************************************************** */
/* INLINED fcts from wiringPi/devLib/lcd.c: */
// HD44780U Commands (see Fig 11, p28 of the Hitachi HD44780U datasheet)

#define	LCD_CLEAR	0x01
#define	LCD_HOME	0x02
#define	LCD_ENTRY	0x04
#define	LCD_CTRL	0x08
#define	LCD_CDSHIFT	0x10
#define	LCD_FUNC	0x20
#define	LCD_CGRAM	0x40
#define	LCD_DGRAM	0x80

// Bits in the entry register

#define	LCD_ENTRY_SH		0x01
#define	LCD_ENTRY_ID		0x02

// Bits in the control register

#define	LCD_BLINK_CTRL		0x01
#define	LCD_CURSOR_CTRL		0x02
#define	LCD_DISPLAY_CTRL	0x04

// Bits in the function register

#define	LCD_FUNC_F	0x04
#define	LCD_FUNC_N	0x08
#define	LCD_FUNC_DL	0x10

#define	LCD_CDSHIFT_RL	0x04

// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define	PI_GPIO_MASK	(0xFFFFFFC0)

static unsigned int gpiobase ;
static uint32_t *gpio ;

static int timed_out = 0;

/* ------------------------------------------------------- */
// misc prototypes

int failure (int fatal, const char *message, ...);
void waitForEnter (void);
void waitForButton (uint32_t *gpio, int button);

/* ======================================================= */
/* SECTION: hardware interface (LED, button, LCD display)  */
/* ------------------------------------------------------- */
/* low-level interface to the hardware */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Either put them in a separate file, lcdBinary.c, and use   */
/* inline Assembler there, or use a standalone Assembler file */
/* You can also directly implement them here (inline Asm).    */
/* ********************************************************** */

/* These are just prototypes; you need to complete the code for each function */

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
void digitalWrite (uint32_t *gpio, int pin, int value){
  if(value == 1){                                            //If the value is equal to 1 then GPIO is set to high
        asm(
            "\tMOV R1, %[gpio] \n"                           // moves value of the GPIO pointer to R1
            "\tMOV R2, #0b1 \n"                              // moves 1 to R2
            "\tLSL R2, %[pin] \n"                            // shifts the value by the number of bits specified by Pin
            "\tSTR R2, [R1, #28] \n"                         //stores the value in R2 and SET register for the GPIO pin
            :
            : [gpio] "r" (gpio)                              //Provides input/output operands (gpio and pin input) (R1 and R2 output)
            , [pin] "r" (pin)
            : "r1", "r2", "cc" 
            );
    }else{
        asm(                                                //else the value is not equal to 1 then GPIO i set to low
            "\tMOV R1, %[gpio]\n"
            "\tMOV R2, #0b1\n"
            "\tLSL R2, %[pin]\n"
            "\tSTR R2, [R1, #40]\n"
            :
            : [gpio] "r" (gpio)
            , [pin] "r" (pin)
            : "r1", "r2", "cc" 
            );
    }
}


/* set the @mode@ of a GPIO @pin@ to INPUT or OUTPUT; @gpio@ is the mmaped GPIO base address */
void pinMode(uint32_t *gpio, int pin, int mode){
  int fsel, shift, res;

  //Switch and case statements that determines fsel (function select) and shift ammount (shift)        
  switch (pin)
  {
  case LED:
    fsel = 1;
    shift = 9;
    break;

  case LED2:
    fsel = 0;
    shift = 15;
    break;

  case BUTTON:
    fsel = 1;
    shift = 27;
    break;
  }

  asm(
      "\tB _bonzo0\n"
      "_bonzo0:\n"
      "\tLDR R1, %[gpio]\n"           // GPIO loaded into R1
      "\tADD R0, R1, %[fsel]\n"       // R0 set to fsel * 4
      "\tLDR R1, [R0, #0]\n"          // value loaded into R1
      "\tMOV R2, #0b111\n"
      "\tLSL R2, %[shift]\n"          // 0b111 shifts the bits
      "\tBIC R1, R1, R2\n"            // clears the registers
      "\tMOV R2, %[mode]\n"
      "\tLSL R2, %[shift]\n"          // value shifts to the left    
      "\tORR R1, R2\n"                // sets specific bit using ORR
      "\tSTR R1, [R0, #0]\n"
      "\tMOV %[result], R1\n"
      : [result] "=r"(res)            // value is moved into res 
      : [act] "r"(pin), [gpio] "m"(gpio), [fsel] "r"(fsel * 4),
        [shift] "r"(shift), [mode] "r"(mode)
      : "r0", "r1", "r2", "cc");
}

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
/* can use digitalWrite(), depending on your implementation */
void writeLED(uint32_t *gpio, int led, int value){
  int off, res;

  switch (led)
  {
  case LED:                             //Begins the switch statement based on LED value
    off = (value == LOW) ? 10 : 7;      // off is set to 10 or 7 based on the value
    break;

  case LED2:                            //Begins the switch statement based on LED value
    off = (value == LOW) ? 10 : 7;      // off is set to 10 or 7 based on the value
    break;

  default:                              // if led does not match any of the cases
    failure(TRUE, "writeLED: pin %d not supported\n", led);// failure line will be printed
  }

  asm volatile(
      "\tB   _bonzo1\n"
      "_bonzo1:\n"
      "\tLDR R1, %[gpio]\n"         // loads the value to the pointer gpio into R1
      "\tADD R0, R1, %[off]\n"      // adds value of off * 4 to and stores it in R0
      "\tMOV R2, #1\n"              // moved the value into R1 and stores into R2
      "\tMOV R1, %[led]\n"          // this moves the value of led into R1
      "\tAND R1, #31\n"             // shows 5 least bits
      "\tLSL R2, R1\n"              // shifts left in R2 by the number specified by R1       
      "\tSTR R2, [R0, #0]\n"        // stores in R2 to the memory address pointed to R0
      "\tMOV %[result], R2\n"       // moves the value R2 to the output operand 
      : [result] "=r"(res)
      : [led] "r"(led), [gpio] "m"(gpio), [off] "r"(off * 4)
      : "r0", "r1", "r2", "cc");
}

/* read a @value@ (LOW or HIGH) from pin number @pin@ (a button device); @gpio@ is the mmaped GPIO base address */
int readButton(uint32_t *gpio, int button){
  int res;
  int off;

  switch (button)                // switch cases to check value of Button
  {
  case BUTTON:
    off = 13;                   // if off is set to 13 button matches the BUTTON
    break;

  default:                      //else failure is printed 
    failure(TRUE, "readButton: pin %d not supported\n", button);  //prints out readButton failure
    break;
  }

  asm(
      "\tB   _bonzo2\n"
      "_bonzo2:\n"
      "\tLDR R0, %[gpio]\n"    // gpio pointer into R0
      "\tADD R1, R0, %[off]\n" // off * 4 to R0 and stores it in R1 
      "\tLDR R0, [R1]\n"       // Loads the content by R1 to R0
      "\tMOV R2, #1\n"         // sets R2 to 1
      "\tMOV R1, %[button]\n"  // sets R1 to value button
      "\tAND R1, #31\n"
      "\tLSL R2, R1\n"         // shifts R2 left by the value R1 and stores it in R2 
      "\tAND R2, R0\n"         // AND of R0 contents with contents of R2
      "\tMOV %[result], R2\n"  // Moves R2 contents to res
      : [result] "=r"(res)
      : [button] "r"(button), [gpio] "m"(gpio), [off] "r"(off * 4)
      : "r0", "r1", "r2", "cc");

  return res;
}

/* wait for a button input on pin number @button@; @gpio@ is the mmaped GPIO base address */
/* can use readButton(), depending on your implementation */
void waitForButton (uint32_t *gpio, int button){
  while (readButton(gpio, button) == 0)
  {
  }
}

/* ======================================================= */
/* SECTION: game logic                                     */
/* ------------------------------------------------------- */
/* AUX fcts of the game logic */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* initialise the secret sequence; by default it should be a random sequence */
void initSeq() {
  theSeq = (int *)malloc(seqlen * sizeof(int));

  srand(time(0));

  for (int i = 0; i < seqlen; i++)
  {
    theSeq[i] = (rand() % 3) + 1;
  }
}

/* display the sequence on the terminal window, using the format from the sample run in the spec */
void showSeq(int *seq) {                 // This fuction takes a pointer to a integer as a parameter 
   fprintf(stdout, "Secret: ");          // prints secret
  for (int i = 0; i < seqlen; i++)       // In this loop it iterates 0 to value of seqlen
  {
    fprintf(stdout, "%d ", seq[i]);      //This prints the value of seq[i] to the output stream
  }

  fprintf(stdout, "\n");                //prints newline character to the output 
} 

#define NAN1 8
#define NAN2 9

/* counts how many entries in seq2 match entries in seq1 */
/* returns exact and approximate matches, either both encoded in one value, */
/* or as a pointer to a pair of values */
int *countMatches(int *seq1, int *seq2) {
  int *data = (int *)malloc(2 * sizeof(int)); // variable to store the matches

  int res_exact = 0;
  int res_approx = 0;

  asm(
      "start:\n"
      "\tMOV R0, #0\n"            // moves 0 to R0
      "\tMOV R3, #0\n"            // moves 0 to R3
      "\tMOV R1, %[seq1]\n"       // moves seq1 into R1
      "\tMOV R2, %[seq2]\n"       // moves seq2 into R2
      "\tMOV R4, #0\n"            // moves 0 to R4
      "\tMOV R5, #0\n"            // moves 0 to R5
      "\tMOV R7, #0\n"            // moves 0 to R7
      "\tMOV R6, #0\n"            // moves 0 to R6
      "\tMOV R8, #0\n"            // moves 0 to R8
      "\tB main_loop\n"           // main loop

      "main_loop:\n"
      "\tCMP R5, #3\n"              // compares value of R5 with the value 3 
      "\tBEQ exit_routine\n"        // exit routine

      "\tLDR R9, [R1, R7]\n"        // loads value from the memory R1+R7 into R9
      "\tLDR R10, [R2, R7]\n"       // loads value of mem addr by the sum of the values  in R2 and R7 into R10

      "\tCMP R9, R10\n"              // compare R9 to the value of R10
      "\tBEQ add_exact\n"            // if equal goes down

      "\tMOV R6, #0\n"               // moves 0 to R6
      "\tMOV R8, #0\n"               // moves 0 to R8 
      "\rB approx_loop\n"            // loop
      "\tB loop_increment1\n"

      "loop_increment1:\n"
      "\tADD R5, R5, #1\n"          // increments the value by 1
      "\tADD R7, R7, #4\n"          // increments the value by 4
      "\tB main_loop\n"

      "add_exact:\n"
      "\tADD R0, R0, #1\n"          // adds 1 to R0
      "\tCMP R10, R4\n"             // compares R10 to the value in R4
      "\tBEQ check_approx_size\n"   // if equal checks approx
      "\tB loop_increment1\n"

      "check_approx_size:\n"
      "\tCMP R3, #0\n"              // compares R3 to 0
      "\tBNE decrement_approx\n"
      "\tB loop_increment1\n"

      "decrement_approx:\n"
      "\tSUB R3, R3, #1\n"          // subtracts 1 from the value in R3
      "\tB loop_increment1\n"

      "approx_loop:\n"
      "\tCMP R6, #3\n"              // compares R6 to 3
      "\tBEQ loop_increment1\n"     

      "\tLDR R10, [R2, R8]\n"       // loads value of mem addr the sum of values in R2 and R8 into R10

      "\tCMP R9, R10\n"             // compares value in R9 to R10
      "\tBEQ index_check\n"
      "\tB loop_increment2\n"

      "loop_increment2:\n"
      "\tADD R6, R6, #1\n"          // increments R6 by 1 
      "\tADD R8, R8, #4\n"          // increments R8 by 4
      "\tB approx_loop\n"

      "index_check:\n"
      "\tCMP R5, R6\n"              // compares R5 and R6
      "\tBNE approx_check\n"        
      "\tB loop_increment2\n"

      "approx_check:\n"
      "\tCMP R10, R4\n"             // compares R4 and R10
      "\tBNE add_approx\n"
      "\tB loop_increment2\n"

      "add_approx:\n"
      "\tMOV R4, R10\n"              // moves the value from R10 to R4
      "\tADD R3, R3, #1\n"           // adds 1 to R3 and stores it in R3
      "\tMOV R6, #2\n"               // moves value 2 into R6
      "\tB loop_increment2\n"

      "exit_routine:\n"
      "\tMOV %[result_exact], R0\n"  // moves the value in R0 to result_exact
      "\tMOV %[result_approx], R3\n" // moves the value in R3 to the result_approx


      : [result_exact] "=r"(res_exact), [result_approx] "=r"(res_approx)
      : [seq1] "r"(seq1), [seq2] "r"(seq2), [seqlen] "r"(seqlen)
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "cc");

  data[0] = res_exact;
  data[1] = res_approx;

  return data;

}
  

/* show the results from calling countMatches on seq1 and seq1 */
void showMatches(int *code, int *seq1, int *seq2, int lcd_format) {
  code = countMatches(seq1, seq2);                // calls counts matches with seq 1 and seq 2 array as arguments and returns the value to the code 
  fprintf(stdout, "%d exact\n", code[0]);         //prints output of number of exact matches between seq1 and seq 2
  fprintf(stdout, "%d approximate\n", code[1]);   //prints output of number of approximate matches between seq1 and seq 2
}

/* parse an integer value as a list of digits, and put them into @seq@ */
/* needed for processing command-line with options -s or -u            */
void readSeq(int *seq, int val) {
  int length = seqlen;          //in this function it taskes two arguments , pointer to integer array 'seq' and integer value \val'

  while (length != 0)           //starts while loop and continiues until
  {
    
    seq[length - 1] = val % 10;     //extracts the right most digit of val by taking the reminder
    val /= 10;                      //when val is divided by 10 and stores it in the element of  seq at index length -1  
    length--;
  }
}

/* read a guess sequence fron stdin and store the values in arr */
/* only needed for testing the game logic, without button input */
int readNum(int max) {         //The Function readNum recives an integer argument max 
  int seq[max];                //integer array seq of size max
  int tmp;                     //int tmp is declared

  for (int i = 0; i < max; i++)  //for loop is used to iterrate max times 
  {
    
    scanf("Enter number: %d", &tmp);   //input is stored in the integer tmp
    seq[i] = tmp;                      //the value of tmp is assigined to seq[i]
  }

  return tmp;                  //returns the value of tmp
}

/* ======================================================= */
/* SECTION: TIMER code                                     */
/* ------------------------------------------------------- */
/* TIMER code */

/* timestamps needed to implement a time-out mechanism */
static uint64_t startT, stopT;

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* you may need this function in timer_handler() below  */
/* use the libc fct gettimeofday() to implement it      */
uint64_t timeInMicroseconds(){      //defines function timeinmicroseconds that returns 64-bit unsigned integer
  struct timeval timev;             //this strucure has current time in seconds and micro seconds 
  uint64_t current;                 //decleares variable now to stors the current time 
  gettimeofday(&timev, NULL);       //this function gets current time into microseconds  and stores it in 'tv' 
  current = (uint64_t)timev.tv_sec * (uint64_t)1000000 + (uint64_t)timev.tv_usec; // calculates the current time in micro seconds
  return (uint64_t)current;         //returns the current time as a 64-bit unsigned integer
}

/* this should be the callback, triggered via an interval timer, */
/* that is set-up through a call to sigaction() in the main fct. */
void timer_handler (int signum) {    //This function timer_handler takes an integer parameter signum
  static int count = 0;              //decleares a static integer count
  stopT = timeInMicroseconds();      //asigns the return variable to to stopT
  count++;                           //increments the value of the count variable by 1
  fprintf(stderr, "Timer expired %d times. Time took: %f\n", count, (stopT - startT) / 1000000.0);//In this the string contains two place holders that will be replced by the value of count
  timed_out = 1;                  //indecating that the time is expired
}


/* initialise time-stamps, setup an interval timer, and install the timer_handler callback */
void initITimer(uint64_t timeout){
  struct sigaction siga;
  struct itimerval timer;

  /* setting the signale handler for when the timer expires */
  memset(&siga, 0, sizeof(siga));   // initalizes it to 0 using memset
  siga.sa_handler = &timer_handler; // signal handler for when time expires 

  sigaction(SIGALRM, &siga, NULL);  // registers signal handler with SIGALRM and sigaction

  /* non recurring timer */
  timer.it_value.tv_sec = timeout;  
  timer.it_value.tv_usec = 0;

  timer.it_interval.tv_sec = 0;
  timer.it_interval.tv_usec = 0;

  setitimer(ITIMER_REAL, &timer, NULL); // passing flag to indicate real-time timer

  startT = timeInMicroseconds();
}

/* ======================================================= */
/* SECTION: Aux function                                   */
/* ------------------------------------------------------- */
/* misc aux functions */

int failure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal) //  && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
  vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}

/*
 * waitForEnter:
 *********************************************************************************
 */

void waitForEnter (void)
{
  printf ("Press ENTER to continue: ") ;
  (void)fgetc (stdin) ;
}

/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay (unsigned int howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}

/* From wiringPi code; comment by Gordon Henderson
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicroseconds (unsigned int howLong)
{
  struct timespec sleeper ;
  unsigned int uSecs = howLong % 1000000 ;
  unsigned int wSecs = howLong / 1000000 ;

  /**/ if (howLong ==   0)
    return ;
#if 0
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
#endif
  else
  {
    sleeper.tv_sec  = wSecs ;
    sleeper.tv_nsec = (long)(uSecs * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }
}

/* ======================================================= */
/* SECTION: LCD functions                                  */
/* ------------------------------------------------------- */
/* medium-level interface functions (all in C) */

/* from wiringPi:
 * strobe:
 *	Toggle the strobe (Really the "E") pin to the device.
 *	According to the docs, data is latched on the falling edge.
 *********************************************************************************
 */

void strobe (const struct lcdDataStruct *lcd)
{

  // Note timing changes for new version of delayMicroseconds ()
  digitalWrite (gpio, lcd->strbPin, 1) ; delayMicroseconds (50) ;
  digitalWrite (gpio, lcd->strbPin, 0) ; delayMicroseconds (50) ;
}

/*
 * sentDataCmd:
 *	Send an data or command byte to the display.
 *********************************************************************************
 */

void sendDataCmd (const struct lcdDataStruct *lcd, unsigned char data)
{
  register unsigned char myData = data ;
  unsigned char          i, d4 ;

  if (lcd->bits == 4)
  {
    d4 = (myData >> 4) & 0x0F;
    for (i = 0 ; i < 4 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
    strobe (lcd) ;

    d4 = myData & 0x0F ;
    for (i = 0 ; i < 4 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
  }
  else
  {
    for (i = 0 ; i < 8 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (myData & 1)) ;
      myData >>= 1 ;
    }
  }
  strobe (lcd) ;
}

/*
 * lcdPutCommand:
 *	Send a command byte to the display
 *********************************************************************************
 */

void lcdPutCommand (const struct lcdDataStruct *lcd, unsigned char command)
{
#ifdef DEBUG
  fprintf(stderr, "lcdPutCommand: digitalWrite(%d,%d) and sendDataCmd(%d,%d)\n", lcd->rsPin,   0, lcd, command);
#endif
  digitalWrite (gpio, lcd->rsPin,   0) ;
  sendDataCmd  (lcd, command) ;
  delay (2) ;
}

void lcdPut4Command (const struct lcdDataStruct *lcd, unsigned char command)
{
  register unsigned char myCommand = command ;
  register unsigned char i ;

  digitalWrite (gpio, lcd->rsPin,   0) ;

  for (i = 0 ; i < 4 ; ++i)
  {
    digitalWrite (gpio, lcd->dataPins [i], (myCommand & 1)) ;
    myCommand >>= 1 ;
  }
  strobe (lcd) ;
}

/*
 * lcdHome: lcdClear:
 *	Home the cursor or clear the screen.
 *********************************************************************************
 */

void lcdHome (struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdHome: lcdPutCommand(%d,%d)\n", lcd, LCD_HOME);
#endif
  lcdPutCommand (lcd, LCD_HOME) ;
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
}

void lcdClear (struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdClear: lcdPutCommand(%d,%d) and lcdPutCommand(%d,%d)\n", lcd, LCD_CLEAR, lcd, LCD_HOME);
#endif
  lcdPutCommand (lcd, LCD_CLEAR) ;
  lcdPutCommand (lcd, LCD_HOME) ;
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
}

/*
 * lcdPosition:
 *	Update the position of the cursor on the display.
 *	Ignore invalid locations.
 *********************************************************************************
 */

void lcdPosition (struct lcdDataStruct *lcd, int x, int y)
{
  // struct lcdDataStruct *lcd = lcds [fd] ;

  if ((x > lcd->cols) || (x < 0))
    return ;
  if ((y > lcd->rows) || (y < 0))
    return ;

  lcdPutCommand (lcd, x + (LCD_DGRAM | (y>0 ? 0x40 : 0x00)  /* rowOff [y] */  )) ;

  lcd->cx = x ;
  lcd->cy = y ;
}



/*
 * lcdDisplay: lcdCursor: lcdCursorBlink:
 *	Turn the display, cursor, cursor blinking on/off
 *********************************************************************************
 */

void lcdDisplay (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_DISPLAY_CTRL ;
  else
    lcdControl &= ~LCD_DISPLAY_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

void lcdCursor (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_CURSOR_CTRL ;
  else
    lcdControl &= ~LCD_CURSOR_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

void lcdCursorBlink (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_BLINK_CTRL ;
  else
    lcdControl &= ~LCD_BLINK_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

/*
 * lcdPutchar:
 *	Send a data byte to be displayed on the display. We implement a very
 *	simple terminal here - with line wrapping, but no scrolling. Yet.
 *********************************************************************************
 */

void lcdPutchar (struct lcdDataStruct *lcd, unsigned char data)
{
  digitalWrite (gpio, lcd->rsPin, 1) ;
  sendDataCmd  (lcd, data) ;

  if (++lcd->cx == lcd->cols)
  {
    lcd->cx = 0 ;
    if (++lcd->cy == lcd->rows)
      lcd->cy = 0 ;
    
    // TODO: inline computation of address and eliminate rowOff
    lcdPutCommand (lcd, lcd->cx + (LCD_DGRAM | (lcd->cy>0 ? 0x40 : 0x00)   /* rowOff [lcd->cy] */  )) ;
  }
}


/*
 * lcdPuts:
 *	Send a string to be displayed on the display
 *********************************************************************************
 */

void lcdPuts (struct lcdDataStruct *lcd, const char *string)
{
  while (*string)
    lcdPutchar (lcd, *string++) ;
}

/* ======================================================= */
/* SECTION: aux functions for game logic                   */
/* ------------------------------------------------------- */

/* ********************************************************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ********************************************************** */

/* --------------------------------------------------------------------------- */
/* interface on top of the low-level pin I/O code */

/* blink the led on pin @led@, @c@ times */
void blinkN(uint32_t *gpio, int led, int c) {   
   for (int i = 0; i < c; i++)
  {
    /* turns the led on and off with  certain delay */
    writeLED(gpio, led, HIGH);   // turns the LED on
    delay(800);                  // delays for seconds
    writeLED(gpio, led, LOW);    // turns the LED off
    delay(800);                  // delays for seconds
  }

  delay(500);     // additional delay before returning from function
}

/* ======================================================= */
/* SECTION: main fct                                       */
/* ------------------------------------------------------- */

int main (int argc, char *argv[])
{ // this is just a suggestion of some variable that you may want to use
  struct lcdDataStruct *lcd ;
  int bits, rows, cols ;
  unsigned char func ;

  int found = 0, attempts = 0, i, j, *result;
  int c, d, buttonPressed, rel, foo;
  int *attSeq;

  int pinLED = LED, pin2LED2 = LED2, pinButton = BUTTON;
  int fSel, shift, pin,  clrOff, setOff, off, res;
  int fd ;

  int  exact, contained;
  char str1[32];
  char str2[32];
  
  struct timeval t1, t2 ;
  int t ;

  char buf [32] ;

  // variables for command-line processing
  char str_in[20], str[20] = "some text";
  int verbose = 0, debug = 0, help = 0, opt_m = 0, opt_n = 0, opt_s = 0, unit_test = 0, res_matches = 0;
  
  // -------------------------------------------------------
  // process command-line arguments

  // see: man 3 getopt for docu and an example of command line parsing
  { // see the CW spec for the intended meaning of these options
    int opt;
    while ((opt = getopt(argc, argv, "hvdus:")) != -1) {
      switch (opt) {
      case 'v':
	verbose = 1;
	break;
      case 'h':
	help = 1;
	break;
      case 'd':
	debug = 1;
	break;
      case 'u':
	unit_test = 1;
	break;
      case 's':
	opt_s = atoi(optarg); 
	break;
      default: /* '?' */
	fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
	exit(EXIT_FAILURE);
      }
    }
  }

  if (help) {
    fprintf(stderr, "MasterMind program, running on a Raspberry Pi, with connected LED, button and LCD display\n"); 
    fprintf(stderr, "Use the button for input of numbers. The LCD display will show the matches with the secret sequence.\n"); 
    fprintf(stderr, "For full specification of the program see: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf\n"); 
    fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
    exit(EXIT_SUCCESS);
  }
  
  if (unit_test && optind >= argc-1) {
    fprintf(stderr, "Expected 2 arguments after option -u\n");
    exit(EXIT_FAILURE);
  }

  if (verbose && unit_test) {
    printf("1st argument = %s\n", argv[optind]);
    printf("2nd argument = %s\n", argv[optind+1]);
  }

  if (verbose) {
    fprintf(stdout, "Settings for running the program\n");
    fprintf(stdout, "Verbose is %s\n", (verbose ? "ON" : "OFF"));
    fprintf(stdout, "Debug is %s\n", (debug ? "ON" : "OFF"));
    fprintf(stdout, "Unittest is %s\n", (unit_test ? "ON" : "OFF"));
    if (opt_s)  fprintf(stdout, "Secret sequence set to %d\n", opt_s);
  }

  seq1 = (int*)malloc(seqlen*sizeof(int));
  seq2 = (int*)malloc(seqlen*sizeof(int));
  cpy1 = (int*)malloc(seqlen*sizeof(int));
  cpy2 = (int*)malloc(seqlen*sizeof(int));

  // check for -u option, and if so run a unit test on the matching function
  if (unit_test && argc > optind+1) { // more arguments to process; only needed with -u 
    strcpy(str_in, argv[optind]);
    opt_m = atoi(str_in);
    strcpy(str_in, argv[optind+1]);
    opt_n = atoi(str_in);
    // CALL a test-matches function; see testm.c for an example implementation
    readSeq(seq1, opt_m); // turn the integer number into a sequence of numbers
    readSeq(seq2, opt_n); // turn the integer number into a sequence of numbers
    if (verbose)
      fprintf(stdout, "Testing matches function with sequences %d and %d\n", opt_m, opt_n);
    res_matches = countMatches(seq1, seq2);
    showMatches(res_matches, seq1, seq2, 1);
    exit(EXIT_SUCCESS);
  } else {
    /* nothing to do here; just continue with the rest of the main fct */
  }

  if (opt_s) { // if -s option is given, use the sequence as secret sequence
    if (theSeq==NULL)
      theSeq = (int*)malloc(seqlen*sizeof(int));
    readSeq(theSeq, opt_s);
    if (verbose) {
      fprintf(stderr, "Running program with secret sequence:\n");
      showSeq(theSeq);
    }
  }
  
  // -------------------------------------------------------
  // LCD constants, hard-coded: 16x2 display, using a 4-bit connection
  bits = 4; 
  cols = 16; 
  rows = 2; 
  // -------------------------------------------------------

  printf ("Raspberry Pi LCD driver, for a %dx%d display (%d-bit wiring) \n", cols, rows, bits) ;

  if (geteuid () != 0)
    fprintf (stderr, "setup: Must be root. (Did you forget sudo?)\n") ;

  // init of guess sequence, and copies (for use in countMatches)
  attSeq = (int*) malloc(seqlen*sizeof(int));
  cpy1 = (int*)malloc(seqlen*sizeof(int));
  cpy2 = (int*)malloc(seqlen*sizeof(int));

  // -----------------------------------------------------------------------------
  // constants for RPi2
  gpiobase = 0x3F200000 ;

  // -----------------------------------------------------------------------------
  // memory mapping 
  // Open the master /dev/memory device

  if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
    return failure (FALSE, "setup: Unable to open /dev/mem: %s\n", strerror (errno)) ;

  // GPIO:
  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpiobase) ;
  if ((int32_t)gpio == -1)
    return failure (FALSE, "setup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

  // -------------------------------------------------------
  // Configuration of LED and BUTTON

  pinMode(gpio, pinLED, OUTPUT);
  pinMode(gpio, pin2LED2, OUTPUT);
  pinMode(gpio, pinButton, INPUT);

  // init of guess sequence, and copies (for use in countMatches)
  attSeq = (int *)malloc(seqlen * sizeof(int));
  
  // -------------------------------------------------------
  // INLINED version of lcdInit (can only deal with one LCD attached to the RPi):
  // you can use this code as-is, but you need to implement digitalWrite() and
  // pinMode() which are called from this code
  // Create a new LCD:
  lcd = (struct lcdDataStruct *)malloc (sizeof (struct lcdDataStruct)) ;
  if (lcd == NULL)
    return -1 ;

  // hard-wired GPIO pins
  lcd->rsPin   = RS_PIN ;
  lcd->strbPin = STRB_PIN ;
  lcd->bits    = 4 ;
  lcd->rows    = rows ;  // # of rows on the display
  lcd->cols    = cols ;  // # of cols on the display
  lcd->cx      = 0 ;     // x-pos of cursor
  lcd->cy      = 0 ;     // y-pos of curosr

  lcd->dataPins [0] = DATA0_PIN ;
  lcd->dataPins [1] = DATA1_PIN ;
  lcd->dataPins [2] = DATA2_PIN ;
  lcd->dataPins [3] = DATA3_PIN ;
  // lcd->dataPins [4] = d4 ;
  // lcd->dataPins [5] = d5 ;
  // lcd->dataPins [6] = d6 ;
  // lcd->dataPins [7] = d7 ;

  // lcds [lcdFd] = lcd ;

  digitalWrite (gpio, lcd->rsPin,   0) ; pinMode (gpio, lcd->rsPin,   OUTPUT) ;
  digitalWrite (gpio, lcd->strbPin, 0) ; pinMode (gpio, lcd->strbPin, OUTPUT) ;

  for (i = 0 ; i < bits ; ++i)
  {
    digitalWrite (gpio, lcd->dataPins [i], 0) ;
    pinMode      (gpio, lcd->dataPins [i], OUTPUT) ;
  }
  delay (35) ; // mS

// Gordon Henderson's explanation of this part of the init code (from wiringPi):
// 4-bit mode?
//	OK. This is a PIG and it's not at all obvious from the documentation I had,
//	so I guess some others have worked through either with better documentation
//	or more trial and error... Anyway here goes:
//
//	It seems that the controller needs to see the FUNC command at least 3 times
//	consecutively - in 8-bit mode. If you're only using 8-bit mode, then it appears
//	that you can get away with one func-set, however I'd not rely on it...
//
//	So to set 4-bit mode, you need to send the commands one nibble at a time,
//	the same three times, but send the command to set it into 8-bit mode those
//	three times, then send a final 4th command to set it into 4-bit mode, and only
//	then can you flip the switch for the rest of the library to work in 4-bit
//	mode which sends the commands as 2 x 4-bit values.

  if (bits == 4)
  {
    func = LCD_FUNC | LCD_FUNC_DL ;			// Set 8-bit mode 3 times
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    func = LCD_FUNC ;					// 4th set: 4-bit mode
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcd->bits = 4 ;
  }
  else
  {
    failure(TRUE, "setup: only 4-bit connection supported\n");
    func = LCD_FUNC | LCD_FUNC_DL ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
  }

  if (lcd->rows > 1)
  {
    func |= LCD_FUNC_N ;
    lcdPutCommand (lcd, func) ; delay (35) ;
  }

  // Rest of the initialisation sequence
  lcdDisplay     (lcd, TRUE) ;
  lcdCursor      (lcd, FALSE) ;
  lcdCursorBlink (lcd, FALSE) ;
  lcdClear       (lcd) ;

  lcdPutCommand (lcd, LCD_ENTRY   | LCD_ENTRY_ID) ;    // set entry mode to increment address counter after write
  lcdPutCommand (lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL) ;  // set display shift to right-to-left

  // END lcdInit ------
  
  // -----------------------------------------------------------------------------
  // Start of game
  lcdClear(lcd);    //Clears the lcd screen before starting the game
  printf("-----------------------------------------------------------------\n");
  fprintf(stderr,"Game is starting...\n");            // prints on terminal
  lcdPosition(lcd,0,0) ; lcdPuts(lcd, "MasterMind");  //prints on lcd screen
  //printf("Hello mate\n");
  /* ***  COMPLETE the code here  ***  */

  // creates the secret sequence
  if (!opt_s)
    initSeq();
  if (debug)
    showSeq(theSeq);

  // optionally one of these 2 calls:
  // waitForEnter () ; 
  // waitForButton (gpio, pinButton) ;
  

  // -----------------------------------------------------------------------------
  // +++++ main loop
  fprintf(stdout, "\n");

  // While loop if sequence is not found
  while (!found) {
    attempts++;    // increments attempts

    blinkN(gpio, pin2LED2, 3);  // blinks LED
    fprintf(stdout, "Round %d\n", attempts);  // prints on terminal
    printf("---------------------------------\n");

    // defining the guess sequence numbers to calculate the input
    attSeq[0] = 0;  // round 1 sub 1...
    attSeq[1] = 0;  // round 1 sub 2...
    attSeq[2] = 0;  // round 1 sub 3...

    for (int i = 0; i < seqlen; i++)
    {
      
      waitForButton(gpio, pinButton); // waits for button to be pressed

      timed_out = 0; // set timer to 0
      initITimer(5); // 5 seconds to put number of inputs

      while (!timed_out)
      {
        
        if (readButton(gpio, pinButton) != 0) 
        {
          attSeq[i]++;
          fprintf(stderr, "Button Pressed\n"); // prints on terminal screen
        }

        delay(DELAY);   //adds delay 
      }

      if (attSeq[i] > 3) //if input is greater than 3
      {
        
        attSeq[i] = 3; // assumes input is 3 cannot go more
      }

      fprintf(stdout, "Input: %d\n", attSeq[i]); // prints the inputted number to the stdout

      blinkN(gpio, pin2LED2, 1);       // blinks LED
      blinkN(gpio, pinLED, attSeq[i]); // responds to the number on inputs and blinks
      fprintf(stdout, "\n");
    }

    blinkN(gpio, pin2LED2, 2);    // blinks LED indicate new round


    result = countMatches(theSeq, attSeq); 

    if (result[0] == 3)    // when array is equal to 3 
    {
      found = 1;           // sets the found to 1
    }

    else if (attempts == 6)  // maximum rounds is 6 not more
    {
      /* exists the game after 6 rounds */
      break;
    }

    else
    {
      lcdClear(lcd);   // lcd is cleared
      char buf[16];    // creates character set of 16
      lcdPosition(lcd, 0,0);
      snprintf(buf, sizeof (buf), "Exact : %d", result[0]);   // prints on lcd
      lcdPuts(lcd,buf);
      lcdPosition(lcd, 0,1);
      snprintf(buf, sizeof (buf), "Approximate : %d", result[1]); //prints on lcd
      lcdPuts(lcd,buf);
      showMatches(result, theSeq, attSeq, 1); // prints on terminal
      fprintf(stdout, "\n");



      blinkN(gpio, pinLED, result[0]); 
      blinkN(gpio, pin2LED2, 1);       
      blinkN(gpio, pinLED, result[1]); 
    }
  }
  if (found) {
      fprintf(stdout, "Game completed in %d rounds\n", attempts); // prints on terminal

    writeLED(gpio, pin2LED2, HIGH);
    blinkN(gpio, pinLED, 3);
    writeLED(gpio, pin2LED2, LOW);

    lcdClear(lcd);  // lcd is cleared 
    char buf[16];   // creates character set of 16
    lcdPosition(lcd,0,1);
    snprintf(buf, sizeof(buf), "Rounds %d", attempts);  // prints on lcd
    lcdPuts(lcd,buf); 

    fprintf(stdout, "SUCCESS\n"); // prints on terminal 
    lcdPosition(lcd,0,0) ; lcdPuts(lcd, "Success"); // prints on lcd
  } else {
    fprintf(stdout, "Sequence not found\n"); // prints on terminal 
    lcdClear(lcd); // lcd is cleared 
    lcdPosition(lcd,0,0) ; lcdPuts(lcd, "Failed"); // prints on lcd
    char buf[16];        // creates character set of 16
    lcdPosition(lcd,0,1);
    snprintf(buf, sizeof(buf), "Rounds %d", attempts); // prints on lcd
    lcdPuts(lcd,buf);
  }
  return 0;
}
