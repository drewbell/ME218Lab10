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
    
#define BIT_ZERO	0
#define BIT_ONE		1
#define BIT_TWO		2
#define BIT_THREE	3
#define BIT_FOUR	4
#define BIT_FIVE	5
    
#define BIT_ZERO_HI	1
#define BIT_ONE_HI	2
#define BIT_TWO_HI	4
#define BIT_THREE_HI	8
#define BIT_FOUR_HI	16
#define BIT_FIVE_HI	32

; bits in RecvStatus Byte
#define RecvStarting 0	    ;make sure default to decimal is set for these defines
#define RecvDataReady 1	    
#define RecvFramingErr 2
    
#define ALL_BITS_HI	    ffh
    
;
;		Variable definitions
;
WREG_TEMP	     equ     0x70       ; for saving WREG in ISR, saved into common RAM
STATUS_TEMP	     equ     0x71       ; for saving STATUS in ISR, saved into common RAM
PCLATH_TEMP	     equ     0x72       ; for saving PCLATH in ISR, saved into common RAM
RecvShiftCounter     equ     0x73       ; index for moving through tables
RecvShiftRegister    equ     0x74       ; var for setting direction of spin
RecvDataRegister     equ     0x75       ; var for setting mode of spinning
RecvStatus	     equ     0x76       ; byte containing RecvStarting (0), RecvDataReady (1), 
					; RecvFramingErr (2), and RecvActive (3)
    
        org         0           ;set location 0 in program memory as instruction to go to Main
        goto        Main
        org         4           ;set location 4 in program memory as instruction to go to ISR
        goto        ISR
        org         5           ;set location 5 in program memory as start of rest of code    
      
        
Main:       ;Initializations
            
	    call        InitGPIO		    ; initialize GPIO pins   
	    call	InitRxIOC		    ; initialize RA5 for interrupt on falling edge
	    call	InitRxLEDTimer		    ; intialize Timer1 for 0.5s LED pulse upon RX
	    clrf	RecvStatus		    ; clears RecvStarting, RecvDataReady, RecvFramingErr, RecvActive
	    clrf	RecvShiftCounter	    ; clear RecvShiftCounter
	    banksel     INTCON			    ; switch to bank 0 for INTCON 
	    bsf		INTCON, GIE		    ; enable interrupts globally
            
Run:	    Nop                                       
	    Nop
	    goto	Run

HUAD:       goto        $                  ; hang here at the end  
    
;******************** Interrupt Service Routine *******************;
                            
ISR: ;start of ISR
Push          movwf      WREG_TEMP       ; save away W Register right off the bat
              movf       STATUS,w        ; move status into W to prepare for saving
              clrf       STATUS          ; clear status bits to change to file register bank 0
              movwf      STATUS_TEMP     ; save away Status and
              movf       PCLATH,w        ; store PCLATH in WREG
              movwf      PCLATH_TEMP     ; save PCLATH value
              clrf       PCLATH          ; ISR should configure bank as required

;ISR_BODY:
;Label ISR:
;If IOCIF set (service interrupts for interrupts on change)
;If RX interrupt is active
;Program Timer2 to fire interrupt ½ bit time 
;Enable Timer2
;Set RecvStarting to 1
;Set RecShiftRegister to 7
;Clear IOCAF bit associated with RX
;Disable IOC on RX
;		Endif IOC on RX
;		Process other IOC interrupts as needed
;Endif (IOCF set)
;
;if TMR2 interrupt is active
;if RecvStarting set (doing start bit)
;clear RecvStarting
;if RX is low (we got good start bit)
;program Timer2 to fire interrupt after 1 bit time
;Clear Timer1 count register for RxLEDPulse timer
;Enable Timer1 with TMR1ON bits of T1CON 
;Raise RA4 to turn on blue LED
;else ( RX is high, so bad start bit)
;program IOC Negative on RX and enable interrupt
;endif (RX is low) 
;else (Doing data bit)
;decrement RecvShiftCounter
;if RecvShiftCounter not 0
;shift RecvShiftRegister 1 position right
;Copy state of RX to MSB of RecvShiftRegister
;else
;copy RecvShiftRegister to RecvDataRegister
;if RX is low (bad stop bit)
;set RecvFramingErr
;endif (RX is low)
;set RecvDataReady to 1
;disable TMR2 interrupt
;program IOC Negative on RX and enable interrupt
;endif (RecvShiftCounter not 0 )
;endif (RecvStarting set)
;clear TMR2IF
;endif (TMR2 interrupt )

;If Timer1 interrupt is active
;	Lower RA4 to turn off BLUE LED 
;Disable Timer1 with TMR1ON bits of T1CON
;	Clear TMR1GIF in PIR1
;Endif 

 
Pop            movf        PCLATH_TEMP,w       ;store saved PCLATH value in WREG
               movwf       PCLATH              ;restore PCLATH
               movf        STATUS_TEMP,w       ;store saved STATUS value in WREG
               movwf       STATUS              ;restore STATUS
               swapf       WREG_TEMP,f         ;prepare WREG to be restore by swapping nibbles once
               swapf       WREG_TEMP,w         ;restore WREG keeping STATUS bits
               retfie                          ;return from interrupt, reenabling global interrupts

	       
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
            movlw        b'00000000'     ; Set all LED pins LOW to start
            movwf        LATA            ; Write to the latch
            return    

InitRxIOC:	;to set up RA5 to receive message
	    banksel	INTCON	        ;change to bank containing INCON 
	    bsf		INTCON, IOCIE	;edit interrupt control register to turn on interrupt on change 
	    banksel	IOCAN		;change to bank containing IOCAN to 
	    bsf		IOCAN, RA5	;set RA5 as interrupt on falling edge
	    return 

InitRxLEDTimer:
	;Set Timer1 prescale to 1:8 in T1CKPS bits of T1CON
	;Set TMR1H and TMR1L to zero to reset timer
	;Enable peripheral by setting TMR1GIE in PIE1
    
   
    end                     ; tell the assembler we are done