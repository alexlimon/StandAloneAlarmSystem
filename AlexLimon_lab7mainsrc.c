/*******************************
 * Name: Alex Limon
 * Building a PIC18F4520 Standalone Alarm System with EUSART Communication 
 ********************************/


/* Included libraries */
#include <p18F4520.h>
#include <delays.h>
#include <stdio.h>
#include <stdlib.h>


/* All the configuration bits that are generated for the microcontroller*/
// CONFIG1H
#pragma config OSC = HS    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

#define HIGH 1 //defining some basic logic for keypad readability
#define LOW 0

/*Below are all the definitions for the specific pin peripherals */
#define TEMPSENSOR PORTAbits.RA0

#define R1 PORTDbits.RD4
#define R2 PORTDbits.RD5
#define R3 PORTDbits.RD6
#define R4 PORTDbits.RD7

#define RUNNINGLED PORTAbits.RA5
#define ALARMLED PORTEbits.RE0
#define TEMPSENSORLED PORTEbits.RE1
#define KEYPADLED PORTEbits.RE2
#define CLOCKIN PORTAbits.RA7
#define CLOCKOUT PORTAbits.RA6

#define PICkitinput4 PORTBbits.RB7
#define PICkitinput5 PORTBbits.RB6 
#define C4 PORTBbits.RB5
#define C2 PORTBbits.RB4
#define C3 PORTBbits.RB3
#define C1 PORTBbits.RB2
#define MOTIONSENSOR PORTBbits.RB0 
/*connect VDD and GROUND*/
#define RXD PORTCbits.RC7
#define TXD PORTCbits.RC6


#define ALARMTRIGGERED INTCONbits.INT0F   //interrupt flags
#define TIMERTRIGGERED INTCONbits.TMR0IF

#define MAXBUF 200
#define PASSCODELENGTH 4

#define TimerValH 0x67 //these are the timer values needed to load into the timer registers to equal 1 second
#define TimerValL 0x69


/*Function prototypes organized by return type
 For specifics on it's functionality see the individual functions*/
unsigned char insertChar( char unsigned newchar ); 
unsigned char deleteChar();
unsigned char readEEByte(unsigned char address);

int writeEEByte(unsigned char address, unsigned char data);
int numChar();
int passcodeListener(int firstime);
int verifypasscodes();
int menuOptionListener(int minoption,int maxoption);
int keypadhandling();
int thresholdListener();
int motionTListener();


void ISRHigh(void);
void ISRLow(void);


void mainMenu();
void changePasscode();
void firsttime();
void motionsensorHandling();
void motionsensorIntON();
void motionsensorIntOFF();
void temperaturesensorHandling();
void tempsensorIntON();
void tempsensorIntOFF();
void tempsensorALARMON();
void tempsensorALARMOFF();
void Delay127m();
void Delay50ms();
void Delay1Sec();
void keypadSettings();
void debounce();
void motionSTime();




/*Actual buffer itself for input routines*/
unsigned char buffer[MAXBUF];
/*Actual passcode stored temporarily*/
unsigned char passcode[4];
/*This is used to check against a passcode*/
unsigned char passcodechecker[4];
unsigned int temperatureF=0;

char thresholdStr[3]; // this keeps the threshold wanted for temperature sensor
char motionTimStr[3]; // this keeps the time wanted before the alarm triggers

/* Global variable initializations to 0
 * tail pointer is where we start writing
 * head pointer is where we read from
 * buffer size starts at 0
*/
int headptr = 0; 
int tailptr = 0;
int buffersize = 0;


int access = 0; //gives access to the main menu
int RUN =1;// always on unless something terrible happens
int CHECKTEMP = 0;// high if user wants to trigger the temperature
int threshold = 999; // initialize the threshold high to avoid the interrupt comparing garbage
int THRESHOLDTRIGGERED = 0; // a flag that signifies the temp interrupt has happened
int MOTIONSENSORTRIGGERED =0;// a flag that signifies that the motion sensor has recently happened
int KEYPADON = 0; 
int KEYPRESSED = 0; // used for keypad checking the key
int MOTIONON = 0; //used to trigger if the motion sensor is on
int MotionTsetting= 0; // used for timer triggering of temperature

int motioncountD = 99999; // intialize to a really high time so when we turn on the counter it wont explode

int motionCount=0; // actual time in seconds the user places


/*HIPRI is an actual interrupt that is triggered */
#pragma code HIPRI = 0x0008
void HIPRI ( void ) 
{
  _asm
    GOTO ISRHigh
  _endasm 


}
//same concept but this is a lower priority
#pragma code LOWPRI = 0x00018
void LOWPRI( void ) 
{
  
  _asm
    GOTO ISRLow
  _endasm 

}

/*Interrupt service routine for high priority
 this will be used when the motion sensor is triggered
 the user will have the ability to either turn it off or on
 */
#pragma interrupt ISRHigh
void ISRHigh( void )
{
    int motionsetting;
     if( ALARMTRIGGERED ) // alarm triggered is the interrupt flag
     {
          ALARMLED = 1; 
          Delay10KTCYx(20);
          printf("\n");
          printf("\n");
          printf("You no longer have access to the menu due a trigger in the Motion Sensor.\n"); //self explanitory user interface
          printf("Please enter the passcode:");
          passcodeListener(0);// this will listen to serial communication for a passcode, more below
          while(!verifypasscodes()) // checking if that passwords match
          {
              printf("\nWhoops! Wrong password. Try again.\n");
              printf("Enter the passcode:");
              passcodeListener(0); // keep listening for a passcode
              
         }
         printf("\nPasscode Accepted!\n");
         printf("\nWould you like to turn off the motion sensor alarm?\n"); //give the user the ability to turn off alarm 
         printf("1 for Yes 0 for No: ");
         while(menuOptionListener(0,1) == -1)
         {
           printf("\nEnter a valid option!\n");
           printf("\n");
           printf("Enter 1 for Yes and 0 for No: ");       
         }
         MOTIONON = (deleteChar() - 48); // converting into integer
         if( MOTIONON == 1 ) MOTIONON = 0; // this controls whether we should turn off the motion sensor or keep it on
         else MOTIONON = 1;
         
         if( MOTIONON == 1 ) motionsensorIntON(); 
         else 
         {
           motionsensorIntOFF(); // turn off the motion sensor interrupt
           MotionTsetting = 0;// if we also turn it off, it removes the timer triggered motion sensor
         
         }
         
         access = 1; // allow access to the menu below
         MOTIONSENSORTRIGGERED = 1; // signify to the system that we have just come back from an interrupt
       
         
         
     }
       ALARMLED = 0; // turn off the LED
       ALARMTRIGGERED = 0;// reset the interrupt flag until the next one occurs
       
}
/*This interrupt service routine is used solely for the timer and capturing temperature from the sensor*/
#pragma interruptlow ISRLow
void ISRLow( void )
{
  unsigned int readingHIGH, readingLOW, readingCONCAT; // our variables for manipulating data read from sensor
  int tempsetting;
  float voltage,temperatureC;
    
    if(TIMERTRIGGERED) // check if timer was triggered
    {
      TMR0H = TimerValH; // load the preload values again to set another second 
      TMR0L = TimerValL;
      
      if(TEMPSENSORLED == 1) TEMPSENSORLED = 0; // toggle the LED 
      else TEMPSENSORLED = 1;
    
    
      Delay50ms();// surround with small delays to capture from sensor to allow ADC conversion time
      ADCON0bits.GO = 1; // set off the ADC
      while(!ADCON0bits.DONE); // poll until its finished
      Delay50ms();// wait a little for things to settle in the reading registers
    
      readingHIGH = ADRESH; // load the high and low digital values of the voltage
      readingLOW = ADRESL;
      readingCONCAT = ((readingHIGH << 8) + readingLOW); // place into one register
   
      voltage = readingCONCAT * 0.004882814; // convert the digital values of voltage back into 0-5V
      temperatureC = (voltage - 0.5) * 100; // account for the offset and convert into celsius
      temperatureF = ((int) temperatureC * (9.0/5.0) + 32.0)+6; // convert into farenheit and add a 6 offset for sensor defect
      if( CHECKTEMP ) // if the user wants to trigger a temperature, check the flag to start
      {
          if(temperatureF > threshold) // if the user inputted temperature is lower than the actual temperature deny user access
          {
            printf("\n");
            printf("\n");
            printf("You no longer have access to the menu due to an increase in temperature.\n"); //same process as above
            printf("Please enter the passcode:");
            passcodeListener(0);
            while(!verifypasscodes())
            {
              printf("\nWhoops! Wrong password. Try again.\n");
              printf("Enter the passcode:");
              passcodeListener(0);
              
            }
            printf("\nPasscode Accepted! \n");
            access = 1;
            THRESHOLDTRIGGERED = 1; // this notifies the system that a rise in temperature causes the trigger
            printf("\nWould you like to turn off the temperature alarm?\n"); //allow the user to turn off the alarm
            printf("1 for Yes 0 for No: ");
            while(menuOptionListener(0,1) == -1)
            {
              printf("\nEnter a valid option!\n");
              printf("\n");
              printf("Enter 1 for Yes and 0 for No: ");       
            }
            tempsetting = deleteChar() - 48;
            if(tempsetting == 1)
            {
               tempsensorALARMOFF(); //turn off the temp sensor alarm
                
            }
            else tempsensorALARMON(); 
          
          }
       
      
      
    }
      if(MotionTsetting == 1) // if the user wants to have a timer for when to kick off alarm
       {
           if(motioncountD == 0) motionsensorIntON(); // if we get to the specified amount turn the alarm on
           motioncountD--;        // decrement until we reach that time.
       }
       
      
      TIMERTRIGGERED =0; // clear the interrupt flag, very IMPORTANT
  }
}
void main()
{
    int passwordret;// password function return value
    int FIRSTTIME;// check if its a hard or soft reset
    int motionsetting; //will be used as return values from EEPROM
    int tempsetting;
    int thresholdsetting;
    
    TRISEbits.RE0 = 0; // turns the LED to output
    TRISA= 0b00000001;
    TRISDbits.RD4 = 0; // turns the keypad outputs 
    TRISDbits.RD5 = 0;
    TRISDbits.RD6 = 0;
    TRISDbits.RD7 = 0;
    
    TRISEbits.RE0 = 0;// turns more LEDs to output
    TRISEbits.RE1 = 0;
    TRISEbits.RE2 = 0;
    
    /*configurations for ADC*/
    ADCON0 = 0b00000001; 
    ADCON1 = 0b00001110;//set AN0 as analog but the rest digital 
    ADCON2 = 0b10010100;
    /*Set all the keypad columns and motion sensor to input*/
    TRISB = 0b00111111;
	ALARMTRIGGERED = 0; // make sure the interrupt flag is cleared
    INTCON3 = 0;
    INTCONbits.TMR0IE =1;// enables the timer
    INTCON2bits.TMR0IP = 0; // makes the timer a low priority 
   
    T0CON= 0b00000110; //sets the settings to prescaler of 128
    
    
    
    
    OSCCONbits.SCS =1; 
   
    TXSTA = 0x00; // reset all the settings for communication
    RCSTA =0x00;
    BAUDCON =0x00;
    BAUDCONbits.RCIDL =1;
    SPBRG = 31; //choose 9.6k baud at 20MHz HS 
    TXSTAbits.TXEN = 1; //enable transmission 
    RCSTAbits.SPEN = 1; //turn on peripheral
    
    RCONbits.IPEN = 1; // this turns on priorities in the interrupts
    INTCONbits.INT0IE = 0; // this turns off the interrupt 0 enable for clarifying purposes
    INTCONbits.GIE_GIEH = 1; // enable global interrupts
    INTCONbits.PEIE_GIEL = 1;// enable peripheral interrupts
    INTCONbits.INT0IF = 0;// set the triggered flag to 0
    INTCON2bits.INTEDG0 = 0; // we need to have this interrupt be negative triggered because the motion sensor is active low
    tempsensorIntON(); //
   
    /*turning off all the LEDs*/
    RUNNINGLED = 0;
    ALARMLED = 0;
    TEMPSENSORLED = 0;
    KEYPADLED = 0;
    
    
    
    /*Creating this boot delay because of ugly serial comm problems that it would cause*/
    printf("Booting.\n");
    Delay1Sec();
    printf("Booting..\n");
   
    while(RUN)
	{   
         if(readEEByte(0x00) != 1) firsttime();//checking if its our first time
         else if( !THRESHOLDTRIGGERED && !MOTIONSENSORTRIGGERED) //checking if we are not here because of an interrupt or reset
         {// this branch off is based on a soft reset that just happened
           
           motionsetting = readEEByte(0x10);//reads to see if the motion sensor interrupt should be on
           if (motionsetting == 1) motionsensorIntON();// turn on or off appropriately 
           else motionsensorIntOFF();
           
           tempsetting = readEEByte(0x12);// reads to see if the 
           
           if ( tempsetting == 1) tempsensorALARMON(); // turn on or off based on read eeprom value
           else tempsensorALARMOFF();
           
           threshold = readEEByte(0x14);// remember the threshold
           
           KEYPADON = readEEByte(0x16); // remember if the keypad was on or not
           
           if (KEYPADON == 1) KEYPADLED = 1; // turn on the LED to show we are using keypad
           
           MotionTsetting = readEEByte(0x18); // remember if we are using the timer set off motion sensor 
           motioncountD = readEEByte(0x20); // the time used to set off motion sensor
           
           
           printf("\n");
           printf("\n");
           printf("You no longer have access to the menu due to a reset.\n");
           printf("\n");
          
           printf("Please enter the passcode:");
           passcodeListener(0); // passcode authentication
           while(!verifypasscodes())
           {
              printf("\nWhoops! Wrong password. Try again.\n");
              printf("Enter the passcode:");
              passcodeListener(0);
              
           }
         
           access = 1; //give access
          
         }
         else{}
         RUNNINGLED = 1; // we are now up and running
         while(access)
         {
             if( MOTIONSENSORTRIGGERED == 1 ) // see if the user turned off or on the motion
             {
               writeEEByte(0x10,MOTIONON);
               MOTIONSENSORTRIGGERED = 0; // clear the flag to see where we came from
             }
             if( THRESHOLDTRIGGERED == 1 ) // see if user turn off or on the threshold
             {     
               writeEEByte(0x12,CHECKTEMP);
               THRESHOLDTRIGGERED = 0;     // clear the flag to see where we came from 
             }
         
             mainMenu(); // run the text interface menu
             
         }
        
       
        
     } //end of while(1)

	
    
    
    
} //end of void main()


int keypadhandling()
{
  int i=0;
  int SENDINPUT = 0;
  int characterspressed=0;
  KEYPRESSED = 0;
    /*Start the handling for the 1st Row*/
  while ( SENDINPUT == 0 ) // check to see if the enter key has been pressed so we can return or not
  { 
    while( KEYPRESSED == 0 ) // check to see if a key has been pressed
    {
      for(i = 0; i < 4 ; i++)    // iterate through each of the rows to find out the key
      {
      
        if( i == 0 ) // this checks the first row
        {
          R2 = R3 = R4 = LOW; // set all the other rows to low 
          R1 = HIGH;
          if(C1 == HIGH) // if this triggered a high in the column
          {
            void Delay50ms(); // wait a little to settle
            while(C1)
            {
                debounce(); // debounce the key
                insertChar('1'); // insert the character into the buffer
            }   
          }
          else if(C2 == HIGH) // the rest of the logic is very similar 
          {
            void Delay50ms();
            while(C2) 
            {
                debounce();
                insertChar('2');
            }           
                    
          }
          else if(C3 == HIGH)
          {
            void Delay50ms();
            while(C3) 
            {
                debounce();
                insertChar('3');
            }       
          }
          else if(C4 == HIGH)
          {
            void Delay50ms();
            /*SENDINPUT = 1;
            break;*/
            while(C4) 
            {
                debounce();
                insertChar('A');
            }       
          }
          else{}
        } 
   
        if( i == 1 )
        {
  /*Start the handling for the 2nd Row*/
          R1 = R3 = R4 = LOW;
          R2 = HIGH;
   
          if(C1 == HIGH)
          {
            void Delay50ms();
            while(C1) 
            {
                debounce();
                insertChar('4');
            }       
          }
          else if(C2 == HIGH)
          {
            void Delay50ms();
            while(C2) 
            {
                debounce();
                insertChar('5');
            }       
          }
          else if(C3 == HIGH)
          {
            void Delay50ms();
            while(C3) 
            {
                debounce();
                insertChar('6');
            }       
          }
          else if(C4 == HIGH)
          {
            void Delay50ms();
            while(C4) 
            {
                debounce();
                insertChar('B');
            }       
          }
          else{}
    
        } 
    
        if( i == 2 )
        {
    /*Start the handling for the 3rd Row*/
          R1 = R2 = R4 = LOW;
          R3 = HIGH;
    
          if(C1 == HIGH)
          {
            void Delay50ms();
            while(C1) 
            {
                debounce();
                insertChar('7');
            }       
          }
          else if(C2 == HIGH)
          {
            void Delay50ms();
            while(C2) 
            {
                debounce();
                insertChar('8');
            }       
          }
          else if(C3 == HIGH)
          {
            void Delay50ms();
            while(C3) 
            {
                debounce();
                insertChar('9');
            }       
          }
          else if(C4 == HIGH)
          {
            void Delay50ms();
            while(C4) 
            {
                debounce();
                insertChar('C');
            }       
        
          }
          else{}
        }
    
        if( i == 3 )
        {
         /*Start the handling for the 4th Row*/
          R1 = R2 = R3 = LOW;
          R4 = HIGH;
          if(C1 == HIGH)
          {
            void Delay50ms();
            while(C1) 
            {
                debounce();
                insertChar('*');
            }       
          }
          else if(C2 == HIGH)
          {
            void Delay50ms();
            while(C2) 
            {
                debounce();
                insertChar('0');
            } 
            
          }
          else if(C3 == HIGH && numChar() > 0) //this is where we go if the # is pressed which is the enter key
          {
            void Delay50ms();
            while(C3)
            {
              debounce(); // debounce and signify the input has been sent
              SENDINPUT = 1;
            } 
            break;
           
          }
          else if(C4 == HIGH)
          {
            void Delay50ms();
            while(C4) 
            {
                debounce();
                insertChar('D');
            } 
            
          }
          else {}
        }//end of if
    }// end of for
   if (SENDINPUT == 1) break;   // if the input has to be sent leave the loop
  }
  KEYPRESSED = 0; // reset the keypressed so we can keep checking for more keys
  if(SENDINPUT == 0) characterspressed++; // check how many characters have been placed in the buffer 
  
 }  
 return characterspressed; // return it to any of the listeners
}



/*The insertChar function takes the character the user inputs
 *and places it in the buffer based on the pointer.
 *The pointer mods with the buffersize to go
 *to the beginning if we are at the end the array.
 *Returns -1 if the buffer is full
*/

unsigned char insertChar( unsigned char newchar )
{
  printf("*", newchar);
  
  KEYPRESSED = 1; 
  
  if( buffersize < MAXBUF )
  { 
    buffer[tailptr] = newchar;
    tailptr++;
    tailptr %= MAXBUF;
    buffersize++;
    return 0;
  }
  
  else
  {
    return -1;
  }
   
  
}

/*The deleteChar() function removes the last character
 *in the buffer (AKA reads it).
 *Advances pointer based on the last char read
 *and mods with maxbuff to create circular effect
 *Returns the character read or
 *returns -1 if the buffer is empty
 */


void putch(unsigned char new) // this function overrides a function in the printf that allows us to show output
{   
    while(!TXSTAbits.TRMT);
    TXREG = new;    
}

unsigned char deleteChar()// this removes a character from the buffer after its been placed and returns -1 if an error has occurred
{
  unsigned char readchar;
  
  if( buffersize > 0 )
  {
    readchar = buffer[headptr];
    headptr++;
    headptr %= MAXBUF;
    buffersize--;
    return readchar;
  }
  else
  {
    return -1;
  }
}

/*The numChar() function simply encapsulates the buffersize global variable 
 * and can return its value at any time
*/
int numChar()
{
  return buffersize;
}
/*The debounce function is the perfect delay for a key debounce in the keypad*/
void debounce()
{
    Delay10KTCYx(255);   
}
/*This is a calculation of the perfect delay for a second*/
void Delay1Sec()
{
    int i;
    for(i = 0; i < 8; i++) Delay10KTCYx(255);
    
}
/*This function creates the smallest delay possible in the PIC*/
void Delay50ms()
{
    Delay1KTCYx(1);
    
}

/*This function is the same as the debounce function but its more clear what the time delay is*/
void Delay127m()
{
   
    Delay10KTCYx(255);
    
    
}
/*The passcode listener function listens for keyboard or keypad input
 * using serial communcation. 
 * The parameter it uses "firsttime" tells the function
 * whether we should be checking the passcode against the current passcode or whether 
 * this is a new passcode that will be used. 
 * It return -1 if the passcode is not valid ie there were too many characters in that buffer
 * this function also clears the buffer in the case that there is nonsense in there
 */
int passcodeListener(int firstime)
{
  
  int i=0,j=0;
  int ENTERNOTPRESSED = 1;
  char tempchar;
  
 if( KEYPADON != 1 ) // this will not be executed if we are keypad mode
 {
     
 
  TRISC = 128; //setting RX bit as input
  RCSTA = 0x90; //enable serial port and receiver
  SPBRG = 31; //choose 9.6k baud at 10MHz xtal
  
  while( ENTERNOTPRESSED ) // while an enter is not pressed
  {
    while(PIR1bits.RCIF == 0); //wait to receive
    tempchar = RCREG; // store the character in a variable
    if(tempchar == 13) ENTERNOTPRESSED = 0; //check if the user pressed enter
    else
    {
      if(tempchar > 32 && tempchar < 126) //only go in if its a valid character
      {
        insertChar(tempchar); // insert the character in the buffer
        i++; // keep track of how many things are being dumped in the buffer
      }
    }
  }
 }
 else
 {
     i= keypadhandling();// this will redirect to keypad
 }
   if (i != 4) // if the passcode was not 4 characters
   {  
       j = numChar();
       for(i = 0; i < j; i++) deleteChar(); //clear the buffer of nonsense
       return -1; // return an error
   }
   else if(firstime) // if its our first time, place the new passcode in the buffer
   {
       for( i = 0; i < PASSCODELENGTH ; i++) passcode[i] = deleteChar();
   }
   else // place the passcode in the beginning of the checking stage
   {
        for( i = 0; i < PASSCODELENGTH ; i++) passcodechecker[i] = deleteChar();
   }
  return 0;
    
}

/*This function reads memory from the EEPROM
 used especially after a soft reset
 returns the data in that specific EE prom address
 */
unsigned char readEEByte(unsigned char address) 
{
  EEADR=address; // load address of EEPROM to read
  EECON1bits.EEPGD=0; // access EEPROM data memory
  EECON1bits.CFGS=0; // do not access configuration registers
  EECON1bits.RD=1; // initiate read 
  return EEDATA; // return EEPROM byte
}

// Write Byte to internal EEPROM and return -1 if an error has accord
int writeEEByte(unsigned char address, unsigned char data)
{
  EECON1bits.WREN=1; // allow EEPROM writes
  EEADR=address; // load address of write to EEPROM
  EEDATA=data; // load data to write to EEPROM
  EECON1bits.EEPGD=0;// access EEPROM data memory
  EECON1bits.CFGS=0; // do not access configuration registers
  INTCONbits.GIE=0; // disable interrupts for critical EEPROM write sequence
  //===============//
  EECON2=0x55;
  EECON2=0xAA;
  EECON1bits.WR=1;
  //==============//
  INTCONbits.GIE=1; // enable interrupts, critical sequence complete
  while (EECON1bits.WR==1); // wait for write to complete
  EECON1bits.WREN=0; // do not allow EEPROM writes
 
  

  //Verify write operation
  if (readEEByte(address)==data) // read the byte we just wrote to EEPROM
  return 0; // write was successful
  else
  return -1; // write failed
}

/*The first time function is the splash screen of the alarm that greets the user for a new passcode and writes the 
 passcode into EEPROM memory*/
void firsttime() 
{
    int passwordret;
    printf("\nWelcome to our Alarm System!\n");
    printf("\n");
    while(passwordret != 0)
    {
      printf("Enter your new 4-Digit passcode: ");
      passwordret = passcodeListener(1);
      if( passwordret == -1 ) printf("\nPlease enter a valid passcode!\n");
    }
     access = 1;
     writeEEByte(0x00,1);
     writeEEByte(0x02,passcode[0]);
     writeEEByte(0x04,passcode[1]);
     writeEEByte(0x06,passcode[2]);
     writeEEByte(0x08,passcode[3]);
     writeEEByte(0x16,0);
    
    
}
/*The verify passcodes function takes in the current passcode from the EEPROM
 *and checks it against the passcode checker used in the passscodelistener
 * if the check has failed it will return 0
 */
int verifypasscodes()
{
    int i;
    passcode[0] = readEEByte(0x02); // grabbing passcode from eeprom
    passcode[1] = readEEByte(0x04);
    passcode[2] = readEEByte(0x06);
    passcode[3] = readEEByte(0x08);
    
    
    for(i = 0 ; i < PASSCODELENGTH; i++)
    {
        if( passcode[i] != passcodechecker[i]) return 0; //whoops! not the same passcode
        
    }
    return 1;
    
 }
/*The main menu function is the main UI for the alarm system that redirects the user
 based on any of the settings the user wishes to change
 it also serves as showing the current status of the system by accessing the EEPROM memory directly
 the status includes the temperature, whether certain things are enabled or disabled 
 it can only be accessed by the main
 */
void mainMenu()
{   int option;
    
   
    printf("\n");
    printf("\n");
    printf("\n");
   /*Print out all the current status of the system*/
    printf("\n*******************************");
    if(readEEByte(0x10)==1)  printf("\n* Motion Sensor: ENABLED      *\n");
    else printf("\n* Motion Sensor: DISABLED     *\n");
    if(readEEByte(0x12)==1)  printf("* Temperature Alarm: ENABLED  *\n");
    else printf("* Temperature Alarm: DISABLED *\n",temperatureF);
    printf("* Temperature: %dF            *\n",temperatureF);
    if(readEEByte(0x16)==1)  printf("* Input: Keypad               *\n");
    else printf("* Input: Keyboard             *\n");
    printf("*******************************");
   
    /*Print out all the possibly options for the user to see through serial comm*/
    printf("\nAlarm System Menu:\n");
    printf("\n");
    printf("1. Change Passcode.\n");
    printf("2. Enable/Disable Motion Sensor\n");
    printf("3. Temperature Sensor Settings\n");
    printf("4. Switch Input Method\n");
    printf("5. Motion Timer Settings\n");
    printf("6. Refresh\n");
           
    printf("Enter Option: ");
    if(THRESHOLDTRIGGERED || MOTIONSENSORTRIGGERED) return; // if we were in the middle of an interrupt return to verify access
    while( menuOptionListener(1,8) == -1 ) // check the listener to see if the values are appropriate 
    {
      if(THRESHOLDTRIGGERED || MOTIONSENSORTRIGGERED) // if at this point there was an interrupt triggered we need to refresh the menu
      {
          insertChar("6");
          break;
      }
      printf("\nEnter a valid option!\n"); // if no interrupt happened at this point in the code that means the user didnt
      printf("\n"); // enter a valid option
      printf("Enter Option: ");
    }
    
    option = deleteChar() - 48; //convert to integer a have an option ready for the switch
    if(THRESHOLDTRIGGERED || MOTIONSENSORTRIGGERED) return; // if an interrupt happened here go back to main
    switch(option)
    {
        case 1:
            changePasscode(); // this will allow the user to change passcode
            break;
        case 2:
            motionsensorHandling();  // this will allow the user to turn on the motion sensor interrupt
            break;
        case 3:
            temperaturesensorHandling(); // this will allow the user to turn on the temperature threshold
            break;
        case 4:
            keypadSettings(); // this will allow the user to use both the keypad and keyboard
            break;
        case 5:
            motionSTime(); // this will allow the user to set a time to turn on motion sensor
            break;
        case 6:
            break; // refresh the menu
        default:
            break;        
        
        
    }
    
    
    
    
}
/*The menu option listener takes two parameters
 the first is the min option which is minimum number that the user
 can enter in the case of a certain setting
 and the max option is the maximum number the user can enter
 this serves as an alternative to the passcode listener that does the same things
 this is used multiple times when there is a range in a menu that needs to be accessed
 the function returns -1 when it is not within the range of the options and it is too long
 ie more than one number
 */

int menuOptionListener(int minoption, int maxoption)
{
  int i=0;
  int j=0;
  int ENTERNOTPRESSED = 1;
  char tempchar;
 
  TRISC = 128; //setting RX bit as input
  RCSTA = 0x90; //enable serial port and receiver
  SPBRG = 31; //choose 9.6k baud at 20mHz
  minoption +=48; //converting the integers to characters
  maxoption +=48;
  
  if( KEYPADON != 1 ) // if the keypad is not on use check for user input
  { 
    while(ENTERNOTPRESSED) // escape with an enter key
    {
      while(PIR1bits.RCIF == 0); //wait to receive
      tempchar = RCREG;
      if(tempchar == 13) ENTERNOTPRESSED = 0;// time to leave
      else
      {
        if(tempchar >= minoption && tempchar <= maxoption) // check if the character is within the range what was given
        {
          insertChar(tempchar);
          i++;
        
        }
      }
    }
  }
  else
  {
     i= keypadhandling(); // use the keypad to handle issues
  }
  if (i != 1) // if the user inputted too much or to little its time to clear the buffer
  {
    j = numChar();
    for(i = 0; i < j; i++) deleteChar(); //clear the buffer of nonsense
    return -1;
  }
  return 0;
}

/*The change passcode function is the UI for changing the passcode
 it checks the passcode with the current one to verify you are authenticated to use it
 it then allows you to change the passcode via the passcode listener above
 */
void changePasscode()
{
  printf("\n");
  printf("Enter your current passcode: ");
    
  if(passcodeListener(0) == -1 || verifypasscodes() == 0) // check both if the user is entering the right input and if the passcode is correct
  {
    printf("\nThe passcode you have entered is invalid\n");
  } //exit to menu if the user wasn't ready to change passcode
  else
  {
    printf("\nEnter your new 4-digit passcode: "); // we are authenticated
    while(passcodeListener(1) == -1) // brute force user to enter a valid new passcode
    {
        printf("\nEnter a valid new 4-digit passcode!\n");
        printf("\nEnter your new 4-digit passcode: ");
    }
    writeEEByte(0x02,passcode[0]); // write the new passcode to the EEPROM
    writeEEByte(0x04,passcode[1]);
    writeEEByte(0x06,passcode[2]);
    writeEEByte(0x08,passcode[3]);
    
  }    
                
        
        
}
/*The motion sensor handling function is the UI for changing whether the interrupts triggered by motion should be on or off
 this is straightforward logic and UI and it reads the EEPROM to figure out the current state of the motion sensor*/
void motionsensorHandling()
{
    int motionsetting;
    if(readEEByte(0x10) == 1) // check the state to see if its on 
    {
        printf("\n");
        printf("\nThe motion sensor is currently on. Would you like to turn it off?\n");
        printf("Enter 1 for Yes and 0 for No: ");
        
        while(menuOptionListener(0,1) == -1) //using menu option listener again
        {
          printf("\nEnter a valid option!\n");
          printf("\n");
          printf("Enter 1 for Yes and 0 for No: ");       
        }
         motionsetting = deleteChar() - 48;
         motionsetting = !motionsetting; //inverse the logic to be 1 if the user wants it on 
         
         
        
    }
    else // this is the off state
    {
      printf("\n");
      printf("\nThe motion sensor is currently off. Would you like to turn it on?\n");
      printf("Enter 1 for Yes and 0 for No: ");
      
      while(menuOptionListener(0,1) == -1)
      {
          printf("\nEnter a valid option!\n");
          printf("\n");
          printf("Enter 1 for Yes and 0 for No: ");       
      }
      motionsetting = deleteChar() - 48;
         
         
    }
    writeEEByte(0x10,motionsetting); //write the settings to the EEPROM
    Delay127m(); //delay to settle in the EEPROM and interrupts
    if(motionsetting == 1) motionsensorIntON(); //turn on motion sensor interrupt based on user input
    else motionsensorIntOFF();
    
    
    
    
}
/*motionsensoron function just turns on the motion sensor interrupt by abstracting the control bits*/
void motionsensorIntON()
{
   INTCONbits.INT0IE = 1;  
}
/* motion sensor off just turns off the motion sensor interrupt by abstracting the control bits*/
void motionsensorIntOFF()
{
   INTCONbits.INT0IE = 0;  
}
/*The threshold listener is the exact copy of the passcode listener with a twist, it makes sure the digits are in the correct
 *place because we will use the to compare the temperature later
 * it returns -1 on a failed threshold input */
int thresholdListener()
{
  int i=0,j=0;
  int ENTERNOTPRESSED = 1;
  char tempchar;
  
  if( KEYPADON != 1 ) //check if the keypad is on once again
  {
    
    TRISC = 128; //setting RX bit as input
    RCSTA = 0x90; //enable serial port and receiver
    SPBRG = 31; //choose 9.6k baud at 20MHz xtal
  
    while(ENTERNOTPRESSED) // while enter is not pressed
    {
      while(PIR1bits.RCIF == 0); //wait to receive
      tempchar = RCREG;
      if(tempchar == 13) ENTERNOTPRESSED = 0;
      else
      {
        if(tempchar > 47 && tempchar < 58) //only accept from 0-9 digits
        {
          insertChar(tempchar);
          i++;
        }
      }
    }
  }
  else
  {
    i= keypadhandling(); // revert to keypad for input of threshold 
  }
  
  if (i > 3 || i < 0) //make sure we are just right in the amount of numbers for threshold
  {  
    j = numChar();
    for(i = 0; i < j; i++) deleteChar(); //clear the buffer of nonsense
    return -1;
  }
  else
  {
     
   if( i == 1 ) // in the case of a one digit number
   {
     thresholdStr[0] = '0';
     thresholdStr[1] = '0';
     thresholdStr[2] = deleteChar();
   }
   else if( i == 2 ) // in the case of a two digit number
   {
     thresholdStr[0] = '0';
     thresholdStr[1] = deleteChar();
     thresholdStr[2] = deleteChar();
       
   }
   else // in the case of a three digit number
   {
     thresholdStr[0] = deleteChar();
     thresholdStr[1] = deleteChar();
     thresholdStr[2] = deleteChar();
     
   }
 
  return 0; 
  }    
}
/*This is the UI for the state of the temperature sensor threshold and will
 allow the user to turn it off or on*/
void temperaturesensorHandling()
{
    int tempsetting;
    if(readEEByte(0x12) == 1) //if the threshold is on 
    {
        printf("\n");
        printf("\nThe temperature sensor alarm is currently on. Would you like to turn it off?\n");
        printf("Enter 1 for Yes and 0 for No: ");
        
        while(menuOptionListener(0,1) == -1)
        {
          printf("\nEnter a valid option!\n");
          printf("\n");
          printf("Enter 1 for Yes and 0 for No: ");       
        }
         tempsetting = deleteChar() - 48;
         tempsetting = !tempsetting;
         
         
         
         
        
    }
    else // if the threshold is off
    {
      printf("\n");
      printf("\nThe temperature sensor alarm is currently off. Would you like to turn it on?\n");
      printf("Enter 1 for Yes and 0 for No: ");
      
      while(menuOptionListener(0,1) == -1) // here is our friend the menu option listener
      {
        printf("\nEnter a valid option!\n");
        printf("\n");
        printf("Enter 1 for Yes and 0 for No: ");       
      }
      tempsetting = deleteChar() - 48;
     
         
    }
   
   
    if( tempsetting == 1) //if the user decided to want the threshold trigger
    {
        
      printf("\nEnter a threshold to trigger the alarm: ");  
      while(thresholdListener() == -1)
      {
          printf("\nEnter a valid option!\n");
          printf("\n");
          printf("Enter a threshold to trigger the alarm: ");     
      }
    
      threshold = atoi(thresholdStr);// convert the threshold to an integer to be stored properly
      writeEEByte(0x12,tempsetting); // store the threshold on or off in the EEPROM
      Delay127m();
      if(tempsetting == 1) tempsensorALARMON(); // turn on the threshold alarm if necessary
      else tempsensorALARMOFF();
      writeEEByte(0x14, threshold);    //store the threshold in memory
        
    }
      
    
    
}
//turn on the timer for temp sensor
void tempsensorIntON()
{
   T0CONbits.TMR0ON = 1;
   TMR0H = TimerValH;
   TMR0L = TimerValL;
    
    
}
//turn the timer off, hint hint never happens! but have it just in case
void tempsensorIntOFF()
{
    
    T0CONbits.TMR0ON = 0;
 
    
}
// actually turn on the ability to check threshold
void tempsensorALARMON()
{
    CHECKTEMP = 1;
}
//actually turn on the ability to check the threshold
void tempsensorALARMOFF()
{
    CHECKTEMP = 0;
}
//Be sure to have a blank line at the end of your program
void keypadSettings() // this will trigger whether the user wants the keypad on or off
{
    if(readEEByte(0x16) == 1) //checking the EEPROM for the state of the keypad if its on
    {
        printf("\n");
        printf("\nThe keypad is currently on. Would you like to turn it off?\n");
        printf("Enter 1 for Yes and 0 for No: ");
        
        while(menuOptionListener(0,1) == -1)
        {
          printf("\nEnter a valid option!\n");
          printf("\n");
          printf("Enter 1 for Yes and 0 for No: ");       
        }
         KEYPADON = deleteChar() - 48;
         KEYPADON = !KEYPADON;
         
         
        
    }
    else // we are using keyboard at this point
    {
      printf("\n");
      printf("\nThe keypad is currently off. Would you like to turn it on?\n");
      printf("Enter 1 for Yes and 0 for No: ");
      
      while(menuOptionListener(0,1) == -1)
      {
          printf("\nEnter a valid option!\n");
          printf("\n");
          printf("Enter 1 for Yes and 0 for No: ");       
      }
      KEYPADON = deleteChar() - 48;
         
         
    }
    if (KEYPADON == 1) KEYPADLED = 1; // turn on the LED to signify we are now in KEYPAD mode!
    else KEYPADLED = 0;
    writeEEByte(0x16, KEYPADON); // write to the EEPROM to save that we are in keypad mode
    Delay127m();
    
    
    
}
/*This function is the UI for changing the settings of when the motion sensor should be kicked off
 it will read the state of the EEPROM and ask the user accordingly based on what it read
 it also allows the user to set a certain amount of when the motion sensor should be kick off*/
void motionSTime()
{
    
    
    if(readEEByte(0x18) == 1)// check the state of the motion timer setting
    {
        printf("\n");
        printf("\nThe motion timer on is currently on. Would you like to turn it off?\n");
        printf("Enter 1 for Yes and 0 for No: ");
        
        while(menuOptionListener(0,1) == -1)
        {
          printf("\nEnter a valid option!\n");
          printf("\n");
          printf("Enter 1 for Yes and 0 for No: ");       
        }
         MotionTsetting = deleteChar() - 48;
         MotionTsetting = !MotionTsetting;
         
         
         
         
        
    }
    else
    {
      printf("\n");
      printf("\nThe motion timer on is currently off. Would you like to turn it on?\n");
      printf("Enter 1 for Yes and 0 for No: ");
      
      while(menuOptionListener(0,1) == -1)
      {
          printf("\nEnter a valid option!\n");
          printf("\n");
          printf("Enter 1 for Yes and 0 for No: ");       
      }
      MotionTsetting = deleteChar() - 48;
     
         
    }
   
   
    if( MotionTsetting == 1)
    {
        
      printf("\nEnter a time to turn on motion sensor (0-999 seconds): ");
      while(motionTListener() == -1)
      {
          printf("\nEnter a valid option!\n");
          printf("\n");
          printf("Enter a time to turn on motion sensor(0-999 seconds): ");     
      }
    
      motionCount = atoi(motionTimStr); // convert the string into an integer
      writeEEByte(0x18,MotionTsetting); // write the on bit to the prom
      Delay127m();
     
      writeEEByte(0x20, motionCount);  //  but the time amount in the prom as well
      motioncountD = motionCount;
        
    }
      
    
    
    
    
}
/*The motion time listener is an exact copy of the threshold listener except it modifies the time string 
 rather than the threshold string
 it also returns -1 in the case of an invalid 3 digit input
 */
int motionTListener()
{
  int i=0,j=0;
  int ENTERNOTPRESSED = 1;
  char tempchar;
  
  if( KEYPADON != 1 )
  {
    
    TRISC = 128; //setting RX bit as input
    RCSTA = 0x90; //enable serial port and receiver
    SPBRG = 31; //choose 9.6k baud at 10MHz xtal
  
    while(ENTERNOTPRESSED)
    {
      while(PIR1bits.RCIF == 0); //wait to receive
      tempchar = RCREG;
      if(tempchar == 13) ENTERNOTPRESSED = 0;
      else
      {
        if(tempchar > 47 && tempchar < 58)
        {
          insertChar(tempchar);
          i++;
        }
      }
    }
  }
  else
  {
    i= keypadhandling();
  }
  
  if (i > 3 || i < 0)
  {  
    j = numChar();
    for(i = 0; i < j; i++) deleteChar(); //clear the buffer of nonsense
    return -1;
  }
  else
  {
     
   if( i == 1 ) // 1 digit number
   {
     motionTimStr[0] = '0';
     motionTimStr[1] = '0';
     motionTimStr[2] = deleteChar();
   }
   else if( i == 2 ) //2 digit number
   {
     motionTimStr[0] = '0';
     motionTimStr[1] = deleteChar();
     motionTimStr[2] = deleteChar();
       
   }
   else // must be a 3 digit number
   {
     motionTimStr[0] = deleteChar();
     motionTimStr[1] = deleteChar();
     motionTimStr[2] = deleteChar();
     
   }
 
  return 0; 
  }    
}


    
    
