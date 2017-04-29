;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Author: 	Drew Bell				;
; Date: 	4/27/17					;
; Version: 	0.1					;
; Title: 	ME218 Lab10				;
; 							;
; Description: 						;
; This is an assembly language program to create a	; 
; small communication network node. This is node B.     ;
;							;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
   
    errorlevel -302             ;ignore banksel macro error reminder
    LIST        P=PIC12F752	;tell assembler which PIC to use
    #include "p12F752.inc"	;include for chip defines
    __CONFIG (_CP_OFF & _WDTE_OFF & _MCLRE_OFF & _PWRTE_ON & _FOSC0_INT)	;configure HW features
 
;       Defines
    
#define BitZero	    0
#define BitOne      1
#define BitTwo	    2
#define BitThree    3
#define BitFour	    4
#define BitFive	    5
    
#define BitZeroHi   1
#define BitOneHi    2
#define BitTwoHi    4
#define BitThreeHi  8
#define BitFourHi   16
#define BitFiveHi   32
    
#define BitsHi	    ffh
    
;
;		Variable definitions
;
    
; INSERT VARIABLES HERE
    
        org         0           ;set location 0 in program memory as instruction to go to Main
        goto        Main
       ; org         4           ;set location 4 in program memory as instruction to go to ISR
       ; goto        ISR
        org         5           ;set location 5 in program memory as start of rest of code    
        
        
        
Main:       ;Initializations
            call        InitGPIO                 ; initialize GPIO pins    
            
Run:	    Nop                                       
	    Nop
	    goto	Run

HUAD:       goto        $                  ; hang here at the end  
    
;******************** Initialialization Routines ******************;

InitGPIO:   ;Set up Pins initialized to the following:
            ; RA0 = OUTPUT  (Red LED)
            ; RA1 = OUTPUT  (Yellow LED)
            ; RA2 = OUTPUT  (Green LED)
            ; RA4 = OUTPUT  (Blue LED)
            ; RA5 = INPUT   (Comm Line)

            banksel      PORTA           ; Go to bank containing PortA
            clrf         PORTA           ; Init PORTA
            banksel      LATA            ; Go to bank containing PortA Data Latch
            clrf         LATA            ; clear latch 
            banksel      ANSELA          ; Go to bank containing ANSELA
            clrf         ANSELA          ; clear ANSELA to enable digital I/O
            banksel      TRISA           ; Go to bank containing TRISA
            movlw        b'00100000'     ; Prep WREG to set RA<5> as input and set RA<4,2:0> as outputs
            movwf        TRISA           ; move WREG into TRISA
            banksel      LATA            ; switch to bank containing LATA
            movlw        b'00010111'     ; Set all LED pins high to test LEDs
            movwf        LATA            ; Write to the latch
            return    
;
    end                     ; tell the assembler we are done