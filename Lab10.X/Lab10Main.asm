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
    
#define BIT_ZERO	    0
#define BIT_ONE		    1
#define BIT_TWO		    2
#define BIT_THREE	    3
#define BIT_FOUR	    4
#define BIT_FIVE	    5
    
#define BIT_ZERO_HI	    1
#define BIT_ONE_HI	    2
#define BIT_TWO_HI	    4
#define BIT_THREE_HI	    8
#define BIT_FOUR_HI	    16
#define BIT_FIVE_HI	    32
    
#define	RED_LED		    00h
#define YELLOW_LED	    01h
#define GREEN_LED	    02h
#define BLUE_LED	    10h
    
#define	RED_LED_BIT	    04h
#define YELLOW_LED_BIT	    08h
#define GREEN_LED_BIT	    10h

; bits in RecvStatus Byte
#define RecvStarting 0	    ;make sure default to decimal is set for these defines
#define RecvDataReady 1	    
#define RecvFramingErr 2
#define RecvActive
    
#define ALL_BITS_HI	    ffh
    
#define HALF_BIT_TIME	    79
#define ONE_BIT_TIME	    158
    158
;
;		Variable definitions
;
WREG_TEMP	    equ     0x70       ; for saving WREG in ISR, saved into common RAM
STATUS_TEMP	    equ     0x71       ; for saving STATUS in ISR, saved into common RAM
PCLATH_TEMP	    equ     0x72       ; for saving PCLATH in ISR, saved into common RAM
RecvShiftCounter    equ     0x73       ; index for moving through tables
RecvShiftRegister   equ     0x74       ; var for setting direction of spin
RecvDataRegister    equ     0x75       ; var for setting mode of spinning
RecvStatus	    equ     0x76       ; byte containing RecvStarting (0), RecvDataReady (1), 
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
Push        movwf      WREG_TEMP       ; save away W Register right off the bat
            movf       STATUS,w        ; move status into W to prepare for saving
            clrf       STATUS          ; clear status bits to change to file register bank 0
            movwf      STATUS_TEMP     ; save away Status and
            movf       PCLATH,w        ; store PCLATH in WREG
            movwf      PCLATH_TEMP     ; save PCLATH value
            clrf       PCLATH          ; ISR should configure bank as required

ISR_BODY:
	    ;If IOCIF set (service interrupts for interrupts on change)
	    banksel	IOCAF		    ;change to bank containing IOC flags for falling edges
	    btfss	IOCAF, IOCAF5	    ;If RX interrupt is active, skip down to deal with IOCAF5 routine
	    goto	ISR_TIMER2	    ;otherwise, skip down to next interrupt
	    
RX_FALLING_EDGE
	    banksel	PR2		    ;change to bank with PR2
	    movlw	HALF_BIT_TIME	    ;Program Timer2 to fire interrupt ½ bit time 
	    banksel	T2CON		    ; move to bank containing T2CON
	    bsf		T2CON, TMR2ON	    ; turn ON the timer2
	    bsf		BitStatus, Recv	    ;Set RecvStarting to 1 
	    movlw	07h		    ;Prepare WREG to set RecvShiftCounter to 7	
	    movwf	RecvShiftCounter    ;write WREG to RecvShiftCounter
	    banksel	IOCAF		    ;change to bank with IOC falling edge flags
	    bcf		IOCAF, IOCAF5	    ;Clear IOCAF bit associated with RX
	    bcf		IOCAN, RA5	    ;Disable falling IOC on RX
	    ;Endif IOC on RX
	    ;Process other IOC interrupts as needed
	    ;Endif (IOCF set)

ISR_TIMER2	;if TMR2 interrupt flag is active, response to subcases
		banksel	    PIR1
		btfss	    PIR1, TMR2IF	    ; skip next command if there is a pending Timer2 interrupt
		goto	    ISR_TIMER1		    ; otherwise go to check if Timer1 interrupt has fired
	    
Timer2_Reponse	btfss	    RecvStatus, RecvStarting    ;if RecvStarting set (doing start bit)
		goto	    DataBit
		
StartBit	bcf	    RecvStatus, RecvStarting    ;clear RecvStarting
		btfsc	    PortA,RA5		    ; if PortA pin RA5 (RX) is low, we got good start bit
		goto	    BadStartBit		    ; otherwise goto BadStartBit
		
GoodStartBit	movlw	    ONE_BIT_TIME	    ; load up number representing one bit time into WREG
		banksel	    PR2			    ; change to bank containing PR2
		movwf	    PR2			    ; program Timer2 to fire interrupt after 1 bit time
		banksel	    TMR1H		    ; change to bank 0 with TMR1L and TMR1H
		clrf	    TMR1H		    ; clearing HI 8 bits
		clrf	    TMR1L		    ; clearing LO 8 bits	
		bsf	    T1CON, TMR1ON	    ; Enable Timer1 with TMR1ON bits of T1CON 
		banksel	    LATA		    ; change to bank with LATA
		bsf	    LATA, BLUE_LED	    ; Raise RA4 to turn on blue LED
		goto	    ClearTMR2Flag	    ; jump down to clear flag
		
BadStartBit	;else ( RX is high, so bad start bit)
		banksel	    IOCAF		    ;change to bank with IOC falling edge flags
		bsf	    IOCAN, RA5		    ;program IOC Negative on RX and enable interrupt
		goto	    ISR_TIMER1		    ;skip down to if Timer1 ISR

DataBit		;else (Doing data bit)
		decfsz	    RecvShiftCounter,f	    ; decrement RecvShiftCounter, store result to file, and skip next instr if 0
		goto	    NewDataBit		    ; process new data bit
		goto	    StopBit		    ; process stop bit
		
NewDataBit	bcf	    Status, C		    ; clear carry bit to prepare for right shift
		rrf	    RecvShiftRegister,f	    ; shift RecvShiftRegister 1 position right
		movf	    PortA,w		    ; move value on PortA to WREG
		btfsc	    W , RA5		    ; if RA5 is low, skip next insruction
		bsf	    RecvShiftRegister,RA5   ; write that value on line was HIGH
		btfss	    W , RA5		    ; if RA5 is high, skip next instruction
		bcf	    RecvShiftRegister,RA5   ; write that value on line was LOW
		goto	    ClearTMR2Flag	    ; skip down to clear Timer2 flag
StopBit	    ;else
		; shift RecvShiftRegister bits right 2 times to right-justify data, clearing Carry bit each time
		bcf	    Status, C		    ; clear carry bit to prepare for right shift
		rrf	    RecvShiftRegister,f	    ; shift RecvShiftRegister 1 position right
		bcf	    Status, C		    ; clear carry bit to prepare for right shift
		rrf	    RecvShiftRegister,f	    ; shift RecvShiftRegister 1 position right	    
		movf	    RecvShiftRegister,w	    ; load RecvShiftRegister into WREG
		movwf	    RecvDataRegister	    ; copy WREG (RecvShiftRegister) to RecvDataRegister
		btfss	    Porta, RA5		    ; if RA5 is high for stop bit as it should be, skip next instruction
		bsf	    RecvStatus, RecvFramingErr	    ; RX LOW means bad stop bit, set RecvFramingErr
		bsf	    RecvStatus, RecvDataReady	    ;set RecvDataReady to 1
		call	    UpdateLEDs		    ; call routine to update LEDs
		banksel	    T2CON		    ; move to bank containing T2CON
		bcf	    T2CON, TMR2ON	    ; disable TMR2 interrupt
		banksel	    IOCAN		    ; change to bank with IOC falling edge enable register
		bsf	    IOCAN, RA5		    ; Enable falling IOC on RX;program IOC Negative on RX and enable interrupt

ClearTMR2Flag	banksel	    PIR1		    ; go to bank with PIR1 (Peripheral Interrut Request Register 1)
		bcf	    IR1, TMR2IF		    ; clear Timer2Interrupt
	       
		; End of TMR2 interrupt response


ISR_TIMER1
		banksel	    PIR1		    ; go to register containing the peripheral interrupt flags
		btfss	    PIR1, TMR1IF	    ; If Timer1 interrupt is active, skip next instruction
		goto	    POP			    ; skip down to POP since Timer1 interrupt is not active
		bcf	    LATA, BLUE_LED	    ; Lower RA4 to turn off BLUE LED 
		bcf	    TICON, TMR1ON	    ; Disable Timer1 with TMR1ON bits of T1CON
		bcf	    PIR1, TMR1IF	    ; Clear TMR1IF in PIR1

 
Pop            movf        PCLATH_TEMP,w       ;store saved PCLATH value in WREG
               movwf       PCLATH              ;restore PCLATH
               movf        STATUS_TEMP,w       ;store saved STATUS value in WREG
               movwf       STATUS              ;restore STATUS
               swapf       WREG_TEMP,f         ;prepare WREG to be restore by swapping nibbles once
               swapf       WREG_TEMP,w         ;restore WREG keeping STATUS bits
               retfie                          ;return from interrupt, reenabling global interrupts

;******************** Interrupt Subroutines ******************;	       
	       
UpdateLEDs:
	
UpdateRedLED
		btfsc	   RecvDataRegister, RED_LED	   ;skip next instruction if RED_
    
    
UpdateYellowLED
    

UpdateGreenLED
	       
	       
	       
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

InitRxIOC:	    ;to set up RA5 to detect falling edge IOC at start of message
	    banksel	INTCON	        ;change to bank containing INCON 
	    bsf		INTCON, IOCIE	;edit interrupt control register to turn on interrupt on change 
	    banksel	IOCAN		;change to bank containing IOCAN to 
	    bsf		IOCAN, RA5	;set RA5 as interrupt on falling edge
	    return 

InitRxLEDTimer:	    ;init Timer1 for Rx LED pulse
	    ;Set Timer1 prescale to 1:8 by setting T1CKPS bits of T1CON to 1,1
	    banksel	T1CON		; change to bank containing T1CON
	    bsf		T1CON,T1CKPS0	; set T1CKPS0 hi
	    bsf		T1CON,T1CKPS1	; set T1CKPS1 hi
	    ;Clear TMR1H and TMR1L to reset timer to zero
	    banksel TMR1H		; change to bank 0 with TMR1L and TMR1H
	    clrf	TMR1H		; clearing HI 8 bits
	    clrf	TMR1L		; clearing LO 8 bits
	    banksel	PIE1		; change to data memory bank with PIE1
	    bsf		PIE1, TMR1IE	;Enable peripheral interrupts by setting TMR1GIE in PIE1
	    return
	    
InitTimer2:	    ;Sets up Timer2: TMR2 = 0, PR2 = 75 = half bit time from reset
	    banksel	PIE1		    ; switch to bank containing PIE1
	    bsf		PIE1, TMR2IE	    ; in PIE1, timer2 interrupt
	    banksel	T2CON		    ; switch to bank with T2CON
	    movlw	b'01111000'	    ; prescale of 1, post-scale of 16, timer off
	    movwf	T2CON               ; move value from WREG into T2CON
	    banksel	PR2		    ; move to bank containing PR2
	    movlw	HALF_BIT_TIME	    ; move starting value of PR2 to WREG
	    movwf	PR2		    ; write to PR2 reg value of 250
	    return
   
    end                     ; tell the assembler we are done