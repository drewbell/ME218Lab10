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
    
#define BIT_ZERO	    d'0'
#define BIT_ONE		    d'1'
#define BIT_TWO		    d'2'
#define BIT_THREE	    d'3'
#define BIT_FOUR	    d'4'
#define BIT_FIVE	    d'5'
    
#define BIT_ZERO_HI	    b'0000 0001'
#define BIT_ONE_HI	    b'0000 0010'
#define BIT_TWO_HI	    b'0000 0100'
#define BIT_THREE_HI	    b'0000 1000'
#define BIT_FOUR_HI	    b'0001 0000'
#define BIT_FIVE_HI	    b'0010 0000'
        
#define	RED_LED		    d'0'	; Pin RA0
#define YELLOW_LED	    d'1'	; Pin RA1
#define GREEN_LED	    d'2'        ; Pin RA2
#define BLUE_LED	    d'4'        ; Pin RA5
    
#define	RED_LED_BIT	    d'2'	; Bit Number 2 from Msg
#define YELLOW_LED_BIT	    d'3'	; Bit Number 3 from Msg
#define GREEN_LED_BIT	    d'4'	; Bit Number 4 from Msg


#define RecvStarting	    d'0'	; bits numbers in RecvStatus Byte    
#define RecvDataReady	    d'1'	    
#define RecvFramingErr	    d'2'
#define RecvActive	    d'3'
    
#define ALL_BITS_HI	    ffh
#define	ADDRESS_MASK	    b'0000 0011'
#define	NODE_B_ADDR	    b'0000 0010'
    
#define HALF_BIT_TIME	    d'79'
#define ONE_BIT_TIME	    d'158'
#define	FullShiftCount	    d'8'
#define MSB		    d'7'
    
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
      
        
Main:		;Initializations
            
		call	    InitGPIO		    ; initialize GPIO pins   
		call	    InitRxIOC		    ; initialize RA5 for interrupt on falling edge
		call	    InitRxLEDTimer	    ; intialize Timer1 for 0.5s LED pulse upon RX
		call	    InitTimer2
		clrf	    RecvStatus		    ; clears RecvStarting, RecvDataReady, RecvFramingErr, RecvActive
		clrf	    RecvShiftCounter	    ; clear RecvShiftCounter
		banksel     INTCON		    ; switch to bank 0 for INTCON 
		bsf	    INTCON, GIE		    ; enable interrupts globally
		bsf	    INTCON, PEIE	    ; enable peripheral interrupts
            
Run:		Nop                                       
		Nop
 		goto	    Run

HUAD:		goto        $               ; hang here at the end  
    
;******************** Interrupt Service Routine *******************;
                            
ISR: ;start of ISR
Push		movwf	    WREG_TEMP       ; save away W Register right off the bat
		movf	    STATUS,w        ; move status into W to prepare for saving
		clrf	    STATUS          ; clear status bits to change to file register bank 0
		movwf	    STATUS_TEMP     ; save away Status and
		movf	    PCLATH,w        ; store PCLATH in WREG
		movwf	    PCLATH_TEMP     ; save PCLATH value
		clrf	    PCLATH          ; ISR should configure bank as required

ISR_BODY:
	    ;If IOCIF set (service interrupts for interrupts on change)
		btfss	    INTCON, IOCIF	    ; if the interrupt on change for falling edge 
		goto	    ISR_TIMER2		    ; if not skipped, go to Timer2 interrupt
		banksel	    IOCAF		    ;change to bank containing IOC flags for falling edges
		btfss	    IOCAF, IOCAF5	    ;If RX interrupt is active, skip down to deal with IOCAF5 routine
		goto	    ISR_TIMER2	    ;otherwise, skip down to next interrupt
	    
RX_FALLING_EDGE 
		banksel	    PR2			    ; change to bank with PR2
		movlw	    HALF_BIT_TIME	    ; program Timer2 to fire interrupt ½ bit time 
		movwf	    PR2			    ; move half bit time into PR2
		banksel	    T2CON		    ; move to bank containing T2CON
		bsf	    T2CON, TMR2ON	    ; turn ON the timer2
		bsf	    RecvStatus, RecvStarting	    ;Set RecvStarting to 1 
		movlw	    FullShiftCount	    ;Prepare WREG to set RecvShiftCounter to 8	
		movwf	    RecvShiftCounter	    ;write WREG to RecvShiftCounter
		banksel	    IOCAF		    ;change to bank with IOC falling edge flags
		bcf	    IOCAF, IOCAF5	    ;Clear IOCAF bit associated with RX
		banksel	    IOCAN		    ; change to bank containing IOCAN
		bcf	    IOCAN, IOCAN5	    ;Disable falling IOC on RX
		;Endif IOC on RX
		;Process other IOC interrupts as needed
		;Endif (IOCF set)

ISR_TIMER2	;if TMR2 interrupt flag is active, response to subcases
		banksel	    PIR1
		btfss	    PIR1, TMR2IF	    ; skip next command if there is a pending Timer2 interrupt
		goto	    ISR_TIMER1		    ; otherwise go to check if Timer1 interrupt has fired
	    
Timer2_Response	btfss	    RecvStatus, RecvStarting    ;if RecvStarting set (doing start bit)
		goto	    DataBit
		
StartBit	bcf	    RecvStatus, RecvStarting    ;clear RecvStarting
		btfsc	    PORTA,RA5		    ; if PortA pin RA5 (RX) is low, we got good start bit
		goto	    BadStartBit		    ; otherwise goto BadStartBit
		
GoodStartBit	movlw	    ONE_BIT_TIME	    ; load up number representing one bit time into WREG
		banksel	    PR2			    ; change to bank containing PR2
		movwf	    PR2			    ; program Timer2 to fire interrupt after 1 bit time
		decf	    RecvShiftCounter	    ; decrement shift counter
		goto	    ClearTMR2Flag	    ; jump down to clear flag
		
BadStartBit	;else ( RX is high, so bad start bit)
		banksel	    IOCAN		    ;change to bank with IOC falling edge flags
		bsf	    IOCAN, RA5		    ;program IOC Negative on RX and enable interrupt
		goto	    ClearTMR2Flag		    ;skip down to if Timer1 ISR

DataBit		;else (Doing data bit)
		decfsz	    RecvShiftCounter,f	    ; decrement RecvShiftCounter, store result to file, and skip next instr if 0
		goto	    NewDataBit		    ; process new data bit
		goto	    StopBit		    ; process stop bit
		
NewDataBit	bcf	    STATUS, C		    ; clear carry bit to prepare for right shift
		rrf	    RecvShiftRegister,f	    ; shift RecvShiftRegister 1 position right
		btfsc	    PORTA, RA5		    ; if RA5 is low, skip next insruction
		bsf	    RecvShiftRegister,MSB   ; write that value on line was HIGH (0x74)
		btfss	    PORTA, RA5		    ; if RA5 is high, skip next instruction
		bcf	    RecvShiftRegister,MSB   ; write that value on line was LOW (0x74)
		goto	    ClearTMR2Flag	    ; skip down to clear Timer2 flag
		
StopBit		;else
		; shift RecvShiftRegister bits right 2 times to right-justify data, clearing Carry bit each time
		bcf	    STATUS, C		    ; clear carry bit to prepare for right shift
		rrf	    RecvShiftRegister,f	    ; shift RecvShiftRegister 1 position right
		bcf	    STATUS, C		    ; clear carry bit to prepare for right shift
		rrf	    RecvShiftRegister,f	    ; shift RecvShiftRegister 1 position right	    
		movf	    RecvShiftRegister,w	    ; load RecvShiftRegister into WREG
		movwf	    RecvDataRegister	    ; copy WREG (RecvShiftRegister) to RecvDataRegister
		btfss	    PORTA, RA5		    ; if RA5 is high for stop bit as it should be, skip next instruction
		bsf	    RecvStatus, RecvFramingErr	    ; RX LOW means bad stop bit, set RecvFramingErr
		bsf	    RecvStatus, RecvDataReady	    ;set RecvDataReady to 1
		
		;if the address bits <1:0> of RecvDataRegister are (1,0) , then report switch bits <4:2> on the LEDs
		btfsc	    RecvDataRegister, BIT_ZERO	    ; if bit 0 of RecvDataRegister is clear, skip next instruction
		goto	    DisableT2			    ; Bit0 of address wrong, skip past LED update to DisableT2
		btfss	    RecvDataRegister, BIT_ONE	    ; if bit 1 is set, skip next instruction
		goto	    DisableT2			    ; Bit1 of address wrong, skip past LED update to DisableT2
		call	    UpdateDataLEDs		    ; call subroutine to update LEDs
		call	    TurnOnBlueLED		    ; call rubroutine to start pulse of blue LED

DisableT2		
		banksel	    T2CON		    ; move to bank containing T2CON
		bcf	    T2CON, TMR2ON	    ; disable TMR2 interrupt
		banksel	    IOCAN		    ; change to bank with IOC falling edge enable register
		bsf	    IOCAN, RA5		    ; Enable falling IOC on RX;program IOC Negative on RX and enable interrupt

ClearTMR2Flag
		banksel	    PIR1		    ; go to bank with PIR1 (Peripheral Interrut Request Register 1)
		bcf	    PIR1, TMR2IF		    ; clear Timer2Interrupt
	       
		; End of TMR2 interrupt response

ISR_TIMER1
		banksel	    PIR1		    ; go to register containing the peripheral interrupt flags
		btfss	    PIR1, TMR1IF	    ; If Timer1 interrupt is active, skip next instruction
		goto	    Pop			    ; skip down to POP since Timer1 interrupt is not active
		bcf	    T1CON, TMR1ON	    ; Disable Timer1 with TMR1ON bits of T1CON
		bcf	    PIR1, TMR1IF	    ; Clear TMR1IF in PIR1
		banksel	    LATA		    ; change to bank containing LATA
		bcf	    LATA, BLUE_LED	    ; Lower RA4 to turn off BLUE LED 
 
Pop            movf        PCLATH_TEMP,w	    ;store saved PCLATH value in WREG
               movwf       PCLATH		    ;restore PCLATH
               movf        STATUS_TEMP,w	    ;store saved STATUS value in WREG
               movwf       STATUS		    ;restore STATUS
               swapf       WREG_TEMP,f		    ;prepare WREG to be restore by swapping nibbles once
               swapf       WREG_TEMP,w		    ;restore WREG keeping STATUS bits
               retfie				    ;return from interrupt, reenabling global interrupts

;******************** Interrupt Subroutines ******************;	       
	       
UpdateDataLEDs:
	
UpdateRedLED
		btfsc	    RecvDataRegister, RED_LED_BIT   ; skip next instruction if RED LED BIT in data received is clear
		bsf	    LATA, RED_LED		    ; turn red LED ON
		btfss	    RecvDataRegister, RED_LED_BIT   ; skip next instruction if RED LED BIT in data received is set
		bcf	    LATA, RED_LED		    ; turn red LED OFF
    
UpdateYellowLED
		btfsc	    RecvDataRegister, YELLOW_LED_BIT	; skip next instruction if YELLOW LED BIT in data received is clear
		bsf	    LATA, YELLOW_LED			; turn YELLOW LED ON
		btfss	    RecvDataRegister, YELLOW_LED_BIT	; skip next instruction if YELLOW LED BIT in data received is set
		bcf	    LATA, YELLOW_LED			; turn YELLOW LED OFF

UpdateGreenLED
		btfsc	    RecvDataRegister, GREEN_LED_BIT	; skip next instruction if GREEN LED BIT in data received is clear
		bsf	    LATA, GREEN_LED			; turn GREEN LED ON
		btfss	    RecvDataRegister, GREEN_LED_BIT	; skip next instruction if GREEN LED BIT in data received is set
		bcf	    LATA, GREEN_LED			; turn GREEN LED OFF
		
		return	    ; end of helper routine
		
TurnOnBlueLED
		banksel	    TMR1H		    ; change to bank 0 with TMR1L and TMR1H
		clrf	    TMR1H		    ; clearing HI 8 bits
		clrf	    TMR1L		    ; clearing LO 8 bits	
		bsf	    T1CON, TMR1ON	    ; Enable Timer1 with TMR1ON bits of T1CON 
		banksel	    LATA		    ; change to bank with LATA
		bsf	    LATA, BLUE_LED	    ; Raise RA4 to turn on blue LED
		return
	    
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
            movlw        b'00000001'     ; Set all LED pins LOW to start
            movwf        LATA            ; Write to the latch
            return    

InitRxIOC:	    ;to set up RA5 to detect falling edge IOC at start of message
	    banksel	INTCON	        ;change to bank containing INCON 
	    bsf		INTCON, IOCIE	;edit interrupt control register to turn on interrupt on change 
	    banksel	IOCAN		;change to bank containing IOCAN to 
	    bsf		IOCAN, RA5	;set RA5 as interrupt on falling edge
	    return 

InitRxLEDTimer:	    ;init Timer1 for Rx LED pulse
	    ;Set Timer1 prescale to 1:2 by setting T1CKPS bits of T1CON to 0,1
	    banksel	T1CON		; change to bank containing T1CON
	    bsf		T1CON,T1CKPS0	; set T1CKPS0 hi
	    bcf		T1CON,T1CKPS1	; set T1CKPS1 lo
	    ;Clear TMR1H and TMR1L to reset timer to zero
	    banksel	TMR1H		; change to bank 0 with TMR1L and TMR1H
	    clrf	TMR1H		; clearing HI 8 bits
	    clrf	TMR1L		; clearing LO 8 bits
	    banksel	PIE1		; change to data memory bank with PIE1
	    bsf		PIE1, TMR1IE	;Enable the timer peripheral interrupts by setting TMR1IE in PIE1
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