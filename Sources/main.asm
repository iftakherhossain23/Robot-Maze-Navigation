;*****************************************************************
;* COE538 Final Project                                          *
;* Iftakher Hossain, Piyush Patel                                *
;*****************************************************************
              XDEF Entry, _Startup ;
              ABSENTRY Entry 
              INCLUDE "derivative.inc"

; Variable and state equates

CLEAR_HOME    EQU   $01                   
INTERFACE     EQU   $38                   
CURSOR_OFF    EQU   $0C                   
SHIFT_OFF     EQU   $06                   
LCD_SEC_LINE  EQU   64                    
LCD_CNTR      EQU   PTJ                   ; LCD control port, bits - PJ7(E),PJ6(RS)
LCD_DAT       EQU   PORTB                 ; LCD data port, bits - PB7,...,PB0
LCD_E         EQU   $80                   ; LCD Enable-signal pin
LCD_RS        EQU   $40                   ; LCD Reset-signal pin
NULL          EQU   00                    
CR            EQU   $0D                   
SPACE         EQU   ' '                   
START         EQU   0
FWD           EQU   1                     
ALL_STOP      EQU   2                     
LEFT_TURN      EQU   3
RIGHT_TURN     EQU   4
REV_TURN       EQU   5                     
LEFT_ALIGN    EQU   6                     
RIGHT_ALIGN   EQU   7 
TIME_LEFT        EQU   8                           
TIME_RIGHT       EQU   8                             

; variable/data section

              ORG   $3800

BASE_LINE     FCB   $9D                  ;Sensor Calibration
BASE_BOW      FCB   $CA                  ;These are the base readings of the sensors on the line
BASE_MID      FCB   $CA                  
BASE_PORT     FCB   $CC
BASE_STBD     FCB   $CC

LINE_VARIANCE           FCB   $21           ; Allowable tolerance in sensor reading
BOW_VARIANCE            FCB   $21           ; 
PORT_VARIANCE           FCB   $21            
MID_VARIANCE            FCB   $20
STARBOARD_VARIANCE      FCB   $21

TOP_LINE      RMB   20                      
              FCB   NULL                    
              
BOT_LINE      RMB   20                      
              FCB   NULL                    

CLEAR_LINE    FCC   '                  '    
              FCB   NULL                    

TEMP          RMB   1                       
                                            
                                            
SENSOR_LINE   FCB   $01                    ; Placeholder values  
SENSOR_BOW    FCB   $23                     
SENSOR_PORT   FCB   $45  
SENSOR_MID    FCB   $67                    
SENSOR_STBD   FCB   $89                    
SENSOR_NUM    RMB   1 


              ORG   $3850                   ; Where TOF counter register is located
TOF_COUNTER   dc.b  0                       ; The timer, incremented at 23Hz
CRNT_STATE    dc.b  2                       ; Current state register
T_TURN        ds.b  1                       ; time turning
TEN_THOUS     ds.b  1                       ; 10,000 digit
THOUSANDS     ds.b  1                       ; 1,000 digit
HUNDREDS      ds.b  1                       ; 100 digit
TENS          ds.b  1                       ; 10 digit
UNITS         ds.b  1                       ; 1 digit
NO_BLANK      ds.b  1                       ; Blnk
BCD_SPARE     RMB   2


; Code Section
***************************************************************************************************
              ORG   $4000
Entry:                                                                       
_Startup: 

              LDS   #$4000                 
              CLI                          
              JSR   INIT                   
              JSR   openADC                
              JSR   initLCD                
              JSR   CLR_LCD_BUF            
              BSET  DDRA,%00000011         
              BSET  DDRT,%00110000         
              JSR   initAD                 
              JSR   initLCD                
              JSR   clrLCD                 
              LDX   #msg1                     ; Display msg1                              
              JSR   putsLCD                                                           
                                                ;                                         
              LDAA  #$8A                      ; Move LCD cursor to the end of msg1        
              JSR   cmd2LCD                                                           
              LDX   #msg2                     ; Display msg2                              
              JSR   putsLCD                                                           
                                              ;                                           
              LDAA  #$C0                      ; Move LCD cursor to the 2nd row            
              JSR   cmd2LCD                                                           
              LDX   #msg3                     ; Display msg3                              
              JSR   putsLCD                                                         
                                            ;                                           
              LDAA  #$C7                      ; Move LCD cursor to the end of msg3        
              JSR   cmd2LCD                                                           
              LDX   #msg4                     ; Display msg4                              
              JSR   putsLCD    
              JSR   ENABLE_TOF             ; Jump to TOF initialization

MAIN        
              JSR   G_LEDS_ON              
              JSR   READ_SENSORS           
              JSR   G_LEDS_OFF             
              JSR   UPDT_DISPL         
              LDAA  CRNT_STATE         
              JSR   DISPATCHER         
              BRA   MAIN               

; Data Section
***************************************************************************************************
********************************************************************************************
          msg1: dc.b  "S:",0                   ; Current state label
          msg2: dc.b  ":",0                    ; Sensor readings label
          msg3: dc.b  "V:",0                   ;Battery voltage label
          msg4: dc.b  "ALIVE",0                    ; Bumper status label
          
           tab: dc.b  "START  ",0               ; States
                dc.b  "FWD    ",0               
                dc.b  "REV    ",0               
                dc.b  "RT_TURN ",0               
                dc.b  "LT_TURN ",0              
                dc.b  "RevTrn ",0               
                dc.b  "STANDBY",0                 
                dc.b  "RTimed ",0  
                                                                                                             
; subroutine section 
*********************************************************************************************************|    
DISPATCHER        JSR   VERIFY_START                        
                  RTS                                                                                    
                                                            
VERIFY_START      CMPA  #START                              
                  BNE   VERIFY_FORWARD                      
                  JSR   START_ST                            
                  RTS                                       
                                                            
VERIFY_FORWARD    CMPA  #FWD                                
                  BNE   VERIFY_STOP                         
                  JSR   FWD_ST                              
                  RTS                                       
                                                            
VERIFY_REV_TURN    CMPA  #REV_TURN                          
                  BNE   VERIFY_LEFT_ALIGN                   
                  JSR   REV_TURN_ST                         
                  RTS                                       
                                                            
VERIFY_STOP       CMPA  #ALL_STOP                           
                  BNE   VERIFY_LEFT_TURN                    
                  JSR   ALL_STOP_ST                         
                  RTS                                       
                                                            
VERIFY_LEFT_TURN   CMPA  #LEFT_TURN                         
                  BNE   VERIFY_RIGHT_TURN                   
                  JSR   LEFT                                
                  RTS                                       
                                                            
VERIFY_LEFT_ALIGN CMPA  #LEFT_ALIGN                         
                  BNE   VERIFY_RIGHT_ALIGN                  
                  JSR   LEFT_ALIGN_DONE                     
                  RTS                                       
                                                            
VERIFY_RIGHT_TURN  CMPA  #RIGHT_TURN                        
                  BNE   VERIFY_REV_TURN                     
                  JSR   RIGHT                               
                                                            
VERIFY_RIGHT_ALIGN CMPA  #RIGHT_ALIGN                       
                  JSR   RIGHT_ALIGN_DONE                    
                  RTS                                       
                                                                                                         
*********************************************************************************************************
START_ST          BRCLR   PORTAD0, %00000100,RELEASE                                    
                  JSR     INIT_FWD                                                               
                  MOVB    #FWD, CRNT_STATE

RELEASE           RTS                                                                                                                                  

*********************************************************************************************************

FWD_ST            BRSET   PORTAD0, $04, NO_FWD_BUMP          
                  MOVB    #REV_TURN, CRNT_STATE              
                                                                                           
                  JSR     UPDT_DISPL                         
                  JSR     INIT_REV                                                                
                  LDY     #12000                                                                   
                  JSR     del_50us                                                                
                  JSR     INIT_RIGHT                                                              
                  LDY     #6000                                                                   
                  JSR     del_50us                                                             
                  LBRA    EXIT                                                                    

NO_FWD_BUMP       BRSET   PORTAD0, $04, NO_FWD_REAR_BUMP     
                  MOVB    #ALL_STOP, CRNT_STATE              
                  JSR     INIT_STOP                          
                  LBRA    EXIT 
                  
NO_FWD_REAR_BUMP  LDAA    SENSOR_BOW                                                              
                  ADDA    BOW_VARIANCE                                                               
                  CMPA    BASE_BOW                                                                
                  BPL     NOT_ALIGNED                                                                
                  LDAA    SENSOR_MID                                                              
                  ADDA    MID_VARIANCE                                                                
                  CMPA    BASE_MID                                                                
                  BPL     NOT_ALIGNED                                                               
                  LDAA    SENSOR_LINE                                                             
                  ADDA    LINE_VARIANCE                                                                
                  CMPA    BASE_LINE                                                               
                  BPL     CHECK_RIGHT_ALIGN                                                          
                  LDAA    SENSOR_LINE          
                  SUBA    LINE_VARIANCE        
                  CMPA    BASE_LINE            
                  BMI     CHECK_LEFT_ALIGN     

***************************************************************************************************                                                                  

NOT_ALIGNED       LDAA    SENSOR_PORT        
                  ADDA    PORT_VARIANCE                                                               
                  CMPA    BASE_PORT                                                              
                  BPL     PARTIAL_LEFT_TURN    
                  BMI     NO_PORT             
                                             
NO_PORT           LDAA    SENSOR_BOW                                                            
                  ADDA    BOW_VARIANCE                                                                 
                  CMPA    BASE_BOW                                                                
                  BPL     EXIT                                                                    
                  BMI     NO_BOW                                                              

NO_BOW            LDAA    SENSOR_STBD                                                             
                  ADDA    STARBOARD_VARIANCE                                                               
                  CMPA    BASE_STBD                                                               
                  BPL     PARTIAL_RIGHT_TURN                                                         
                  BMI     EXIT                 

***************************************************************************************************

PARTIAL_LEFT_TURN  LDY     #6000                 
                  jsr     del_50us                                                                
                  JSR     INIT_LEFT                                                               
                  MOVB    #LEFT_TURN, CRNT_STATE                                                  
                  LDY     #6000                     
                  JSR     del_50us                                                                
                  BRA     EXIT                                                                    

CHECK_LEFT_ALIGN  JSR     INIT_LEFT                                                               
                  MOVB    #LEFT_ALIGN, CRNT_STATE                                                 
                  BRA     EXIT

*************************************************************************************************** 

PARTIAL_RIGHT_TURN LDY     #6000                                                                  
                  jsr     del_50us                     
                  JSR     INIT_RIGHT                                                              
                  MOVB    #RIGHT_TURN, CRNT_STATE      
                  LDY     #6000                                                                   
                  JSR     del_50us                                                                
                  BRA     EXIT                                                                   

CHECK_RIGHT_ALIGN JSR     INIT_RIGHT                                                              
                  MOVB    #RIGHT_ALIGN, CRNT_STATE                                                
                  BRA     EXIT                                                                                                                                                         

EXIT              RTS 

***************************************************************************************************                                                                            
                                       
LEFT              LDAA    SENSOR_BOW   
                  ADDA    BOW_VARIANCE 
                  CMPA    BASE_BOW                                                               
                  BPL     LEFT_ALIGN_DONE                                                        
                  BMI     EXIT

LEFT_ALIGN_DONE   MOVB    #FWD, CRNT_STATE                                                        
                  JSR     INIT_FWD                                                                
                  BRA     EXIT                                                                    

RIGHT             LDAA    SENSOR_BOW                                                              
                  ADDA    BOW_VARIANCE                                                                
                  CMPA    BASE_BOW                                                                
                  BPL     RIGHT_ALIGN_DONE                                                        
                  BMI     EXIT 

RIGHT_ALIGN_DONE  MOVB    #FWD, CRNT_STATE                                                        
                  JSR     INIT_FWD                                                                
                  BRA     EXIT                                                                    

***************************************************************************************************                                       
REV_TURN_ST       LDAA    SENSOR_BOW    
                  ADDA    BOW_VARIANCE  
                  CMPA    BASE_BOW      
                  BMI     EXIT                                                                    
                  JSR     INIT_LEFT                                                               
                  MOVB    #FWD, CRNT_STATE                                                        
                  JSR     INIT_FWD                                                                
                  BRA     EXIT                                                                    

ALL_STOP_ST       BRSET   PORTAD0, %00000100, NO_START_BUMP                                       
                  MOVB    #START, CRNT_STATE                                                      

NO_START_BUMP     RTS                                                                             

***************************************************************************************************
INIT_RIGHT        BSET    PORTA,%00000010          
                  BCLR    PORTA,%00000001        
                  LDAA    TOF_COUNTER               
                  ADDA    #TIME_RIGHT
                  STAA    T_TURN
                  RTS

INIT_LEFT        
                  BSET    PORTA,%00000001         
                  BCLR    PORTA,%00000010 
          
                  LDAA    TOF_COUNTER               
                  ADDA    #TIME_LEFT                
                  STAA    T_TURN                    
                  RTS

INIT_FWD          BCLR    PORTA, %00000011          
                  BSET    PTT, %00110000            
                  LDY     #100 
                  JSR     del_50us  
                  RTS 

INIT_REV          BSET    PORTA,%00000011           
                  BSET    PTT,%00110000             
                  RTS

INIT_STOP         BCLR    PTT, %00110000            
                  RTS


***************************************************************************************************
INIT              BCLR   DDRAD,$FF 
                  BSET   DDRA,$FF  
                  BSET   DDRB,$FF  
                  BSET   DDRJ,$C0  
                  RTS


***************************************************************************************************        
openADC           MOVB   #$80,ATDCTL2 
                  LDY    #1           
                  JSR    del_50us     
                  MOVB   #$20,ATDCTL3 
                  MOVB   #$97,ATDCTL4 
                  RTS

********************************************************************************
; This routine writes characters (ascii 20) into the LCD display
; buffer in order to prepare it for the building of a new display buffer.
; Done only once.
CLR_LCD_BUF       LDX   #CLEAR_LINE
                  LDY   #TOP_LINE
                  JSR   STRCPY

CLB_SECOND        LDX   #CLEAR_LINE
                  LDY   #BOT_LINE
                  JSR   STRCPY

CLB_EXIT          RTS

*********************************************************************************      
; String Copy
; Copies a null-terminated string (including the null) from one location to another.
; X = starting address of null-terminated string
; Y =  first address of destination
STRCPY            PSHX            ; Protect the registers used
                  PSHY
                  PSHA

STRCPY_LOOP       LDAA 0,X        ; Get a source character
                  STAA 0,Y        ; Copy it to the destination
                  BEQ STRCPY_EXIT ; If it was the null, then exit
                  INX             ; Else increment the pointers
                  INY
                  BRA STRCPY_LOOP ; Repeat steps again in a loop

STRCPY_EXIT       PULA            ; Restore the registers
                  PULY
                  PULX
                  RTS  

;* **************************************************************************************************      
;*                                   Guider LEDs ON                                                 |
;* This routine enables the guider LEDs such that the sensor correspond to the lights.              |
;* Passed: Nothing (For both ON and OFF)                                                            |
;* Returns:Nothing (For both ON and OFF)                                                            |
;* Side: PORTA bit 5 is changed                                                                     |
G_LEDS_ON         BSET PORTA,%00100000 ; Set bit 5                                                  |
                  RTS                                                                             ; |
;*--------------------------------------------------------------------------------------------------*      
;*                                  Guider LEDs OFF                                                 |
;* This routine disables the guider LEDs. Readings of the sensor correspond to the ambient lighting |
;* Side: PORTA bit 5 is changed                                                                     |
;*                                                                                                  |
G_LEDS_OFF        BCLR PORTA,%00100000 ; Clear bit 5                                                |
                  RTS                  ;                                                            |    


***************************************************************************************************  
READ_SENSORS      CLR   SENSOR_NUM     ; Select sensor number 0
                  LDX   #SENSOR_LINE   ; Point at the start of the sensor array

RS_MAIN_LOOP      LDAA  SENSOR_NUM     ; Select the correct sensor input
                  JSR   SELECT_SENSOR  ; on the hardware
                  LDY   #400           ; 20 ms delay to allow the
                  JSR   del_50us       ; sensor to stabilize
                  LDAA  #%10000001     ; Start A/D conversion on AN1
                  STAA  ATDCTL5
                  BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done
                  LDAA  ATDDR0L        ; A/D conversion is complete in ATDDR0L
                  STAA  0,X            ; so copy it to the sensor register
                  CPX   #SENSOR_STBD   ; If this is the last reading
                  BEQ   RS_EXIT        ; Then exit
                  INC   SENSOR_NUM     ; Else, increment the sensor number
                  INX                  ; and the pointer into the sensor array
                  BRA   RS_MAIN_LOOP   ; and do it again

RS_EXIT           RTS


*************************************************************************************************     
SELECT_SENSOR     PSHA                ; Save the sensor number for the moment
                  LDAA PORTA          ; Clear the sensor selection bits to zeros
                  ANDA #%11100011
                  STAA TEMP           ; and save it into TEMP
                  PULA                ; Get the sensor number
                  ASLA                ; Shift the selection number left, twice
                  ASLA 
                  ANDA #%00011100     ; Clear irrelevant bit positions
                  ORAA TEMP           ; OR it into the sensor bit positions
                  STAA PORTA          ; Update the hardware
                  RTS
 *************************************************************************************************    
DP_FRONT_SENSOR   EQU TOP_LINE+3    
DP_PORT_SENSOR    EQU BOT_LINE+0    
DP_MID_SENSOR     EQU BOT_LINE+3    
DP_STBD_SENSOR    EQU BOT_LINE+6    
DP_LINE_SENSOR    EQU BOT_LINE+9    

DISPLAY_SENSORS   LDAA  SENSOR_BOW        ; Get the FRONT sensor value
                  JSR   BIN2ASC           ; Convert to ascii string in D
                  LDX   #DP_FRONT_SENSOR  ; Point to the LCD buffer position
                  STD   0,X               ; and write the 2 ascii digits there
                  LDAA  SENSOR_PORT       ; Repeat for the PORT value
                  JSR   BIN2ASC
                  LDX   #DP_PORT_SENSOR
                  STD   0,X
                  LDAA  SENSOR_MID        ; Repeat for the MID value
                  JSR   BIN2ASC
                  LDX   #DP_MID_SENSOR
                  STD   0,X
                  LDAA  SENSOR_STBD       ; Repeat for the STARBOARD value
                  JSR   BIN2ASC
                  LDX   #DP_STBD_SENSOR
                  STD   0,X
                  LDAA  SENSOR_LINE       ; Repeat for the LINE value
                  JSR   BIN2ASC
                  LDX   #DP_LINE_SENSOR
                  STD   0,X
                  LDAA  #CLEAR_HOME       ; Clear the display and home the cursor
                  JSR   cmd2LCD           
                  LDY   #40               ; Wait 2 ms until "clear display" command is complete
                  JSR   del_50us
                  LDX   #TOP_LINE         ; Now copy the buffer top line to the LCD
                  JSR   putsLCD
                  LDAA  #LCD_SEC_LINE     ; Position the LCD cursor on the second line
                  JSR   LCD_POS_CRSR
                  LDX   #BOT_LINE         ; Copy the buffer bottom line to the LCD
                  JSR   putsLCD
                  RTS
********************************************************************************************
UPDT_DISPL      LDAA  #$82                      
                JSR   cmd2LCD                   
                
                LDAB  CRNT_STATE                
                LSLB                            
                LSLB                            
                LSLB                            
                LDX   #tab                      
                ABX                             
                JSR   putsLCD                   

                LDAA  #$8F                      
                JSR   cmd2LCD                   
                LDAA  SENSOR_BOW                
                JSR   BIN2ASC                   
                JSR   putcLCD                   
                EXG   A,B                       
                JSR   putcLCD                   

                LDAA  #$92                      
                JSR   cmd2LCD                   
                LDAA  SENSOR_LINE               
                JSR   BIN2ASC                   
                JSR   putcLCD                   
                EXG   A,B                       
                JSR   putcLCD                   

                LDAA  #$CC                      
                JSR   cmd2LCD                   
                LDAA  SENSOR_PORT               
                JSR   BIN2ASC                   
                JSR   putcLCD                   
                EXG   A,B                       
                JSR   putcLCD                   

                LDAA  #$CF                      
                JSR   cmd2LCD                   
                LDAA  SENSOR_MID                
                JSR   BIN2ASC                   
                JSR   putcLCD                   
                EXG   A,B                       
                JSR   putcLCD                   

                LDAA  #$D2                      
                JSR   cmd2LCD                   
                LDAA  SENSOR_STBD               
                JSR   BIN2ASC                   
                JSR   putcLCD                   
                EXG   A,B                       
                JSR   putcLCD                   
        
                MOVB  #$90,ATDCTL5              
                BRCLR ATDSTAT0,$80,*            
                LDAA  ATDDR0L                   
                LDAB  #39                       
                MUL                             
                ADDD  #600                      
                JSR   int2BCD
                JSR   BCD2ASC
                LDAA  #$C2                      
                JSR   cmd2LCD                                   
                LDAA  TEN_THOUS                 
                JSR   putcLCD                   
                LDAA  THOUSANDS                 
                JSR   putcLCD                   
                LDAA  #$2E                      
                JSR   putcLCD                   
                LDAA  HUNDREDS                  
                JSR   putcLCD                                   

                LDAA  #$C9                      
                JSR   cmd2LCD
                
                BRCLR PORTAD0,#%00000100,bowON  
                LDAA  #$20                      
                JSR   putcLCD                   
                BRA   stern_bump                ; Display 'B'
         bowON: LDAA  #$42                      
                JSR   putcLCD                   
          
    stern_bump: BRCLR PORTAD0,#%00001000,sternON
                LDAA  #$20                      
                JSR   putcLCD                   
                BRA   UPDT_DISPL_EXIT           ; Display 'S' 
       sternON: LDAA  #$53                      
                JSR   putcLCD                   
UPDT_DISPL_EXIT RTS                             
                
***************************************************************************************************
ENABLE_TOF        LDAA    #%10000000
                  STAA    TSCR1           ; Enable TCNT
                  STAA    TFLG2           ; Clear TOF
                  LDAA    #%10000100      ; Enable TOI and select prescale factor equal to 16
                  STAA    TSCR2
                  RTS

TOF_ISR           INC     TOF_COUNTER
                  LDAA    #%10000000      ; Clear
                  STAA    TFLG2           ; TOF
                  RTI


; Utility Subroutines, not directly related to function
*******************************************************************
initLCD:
            BSET DDRB,%11111111 ; configure all pins of port B for output
            BSET DDRJ,%11000000 ; configure pins PJ7,PJ6 for output
            LDY #2000 ; wait for LCD to be ready
            JSR del_50us ; -"-
            LDAA #$28 ; set 4-bit data, 2-line display
            JSR cmd2LCD ; -"-
            LDAA #$0C ; display on, cursor off, blinking off
            JSR cmd2LCD ; -"-
            LDAA #$06 ; move cursor right after entering a character
            JSR cmd2LCD ; -"-
            RTS
*******************************************************************
cmd2LCD:    BCLR LCD_CNTR,LCD_RS ; select the LCD Instruction Register (IR)
            JSR dataMov ; send data to IR
            RTS
*******************************************************************
putsLCD:    LDAA 1,X+ ; get one character from the string
            BEQ donePS ; reach NULL character?
            JSR putcLCD
            BRA putsLCD
donePS      RTS
*******************************************************************
clrLCD:
            LDAA #$01 ; clear cursor and return to home position
            JSR cmd2LCD ; -"-
            LDY #40 ; wait until "clear cursor" command is complete
            JSR del_50us ; -"-
            RTS
*******************************************************************
; 50 Microsecond Delay
del_50us    PSHX ; (2 E-clk) Protect the X register
eloop       LDX #300 ; (2 E-clk) Initialize the inner loop counter
iloop       NOP ; (1 E-clk) No operation
            DBNE X,iloop ; (3 E-clk) If the inner cntr not 0, loop again
            DBNE Y,eloop ; (3 E-clk) If the outer cntr not 0, loop again
            PULX ; (3 E-clk) Restore the X register
            RTS ; (5 E-clk) Else return
*******************************************************************
; Send a character in accumulator in A to LCD
putcLCD     BSET LCD_CNTR,LCD_RS ; select the LCD Data register
            JSR dataMov ; send data to IR or DR of the LCD
            RTS
*******************************************************************
; Send data to the LCD IR or DR depending on the RS signal
dataMov     BSET LCD_CNTR,LCD_E ; pull the LCD E-sigal high
            STAA LCD_DAT ; send the 8 bits of data to LCD
            NOP
            NOP
            NOP
            BCLR LCD_CNTR,LCD_E ; pull the E signal low to complete the write operation
            LDY #1 ; adding this delay will complete the internal
            JSR del_50us ; operation for most instructions
            RTS
*******************************************************************
initAD      MOVB #$C0,ATDCTL2 ; power up AD, select fast flag clear
            JSR del_50us      ; wait for 50 us
            MOVB #$00,ATDCTL3 ; 8 conversions in a sequence
            MOVB #$85,ATDCTL4 ; res=8, conv-clks=2, prescal=12
            BSET ATDDIEN,$0C  ; configure pins AN03,AN02 as digital inputs
            RTS
*******************************************************************
int2BCD     XGDX            ; Save the binary number into .X
            LDAA #0         ; Clear the BCD_BUFFER
            STAA TEN_THOUS
            STAA THOUSANDS
            STAA HUNDREDS
            STAA TENS
            STAA UNITS
            STAA BCD_SPARE
            STAA BCD_SPARE+1

            CPX #0         ; Check for a zero input
            BEQ CON_EXIT   ; and if so, exit
           
            XGDX           ; Not zero, get the binary number back to .D as dividend
            LDX #10        ; Setup 10 (Decimal!) as the divisor
            IDIV           ; Divide: Quotient is now in .X, remainder in .D
            ANDB #$0F      ; Clear high nibble of remainder
            STAB UNITS     ; and store it.
            CPX #0         ; If quotient is zero,
            BEQ CON_EXIT   ; then exit

            XGDX           ; else swap first quotient back into .D
            LDX #10        ; and setup for another divide by 10
            IDIV
            ANDB #$0F
            STAB TENS
            CPX #0
            BEQ CON_EXIT

            XGDX           ; Swap quotient back into .D
            LDX #10        ; and setup for another divide by 10
            IDIV
            ANDB #$0F
            STAB HUNDREDS
            CPX #0
            BEQ CON_EXIT

            XGDX           ; Swap quotient back into .D
            LDX #10        ; and setup for another divide by 10
            IDIV
            ANDB #$0F
            STAB THOUSANDS
            CPX #0
            BEQ CON_EXIT

            XGDX           ; Swap quotient back into .D
            LDX #10        ; and setup for another divide by 10
            IDIV
            ANDB #$0F
            STAB TEN_THOUS

CON_EXIT    RTS            ; We're done the conversion
*******************************************************************
;---------------------------------------------------------------------------
; Position the Cursor
; This routine positions the display cursor in preparation for the writing
; of a character or string.
; For a 20x2 display:
; The first line of the display runs from 0 .. 19.
; The second line runs from 64 .. 83.
; The control instruction to position the cursor has the format
; 1aaaaaaa
; where aaaaaaa is a 7 bit address.
; Passed: 7 bit cursor Address in ACCA
; Returns: Nothing
; Side Effects: None
LCD_POS_CRSR ORAA #%10000000 ; Set the high bit of the control word
            JSR cmd2LCD ; and set the cursor address
            RTS
*******************************************************************
; Binary to ASCII
; Converts an 8 bit binary value in ACCA to the equivalent ASCII character 2
; character string in accumulator D
; Uses a table-driven method rather than various tricks.
; Passed: Binary value in ACCA
; Returns: ASCII Character string in D
; Side Fx: ACCB is destroyed
HEX_TABLE   FCC "0123456789ABCDEF" ; Table for converting values
BIN2ASC     PSHA ; Save a copy of the input number on the stack
            TAB ; and copy it into ACCB
            ANDB #%00001111 ; Strip off the upper nibble of ACCB
            CLRA ; D now contains 000n where n is the LSnibble
            ADDD #HEX_TABLE ; Set up for indexed load
            XGDX
            LDAA 0,X ; Get the LSnibble character
            PULB ; Retrieve the input number into ACCB
            PSHA ; and push the LSnibble character in its place
            RORB ; Move the upper nibble of the input number
            RORB ; into the lower nibble position.
            RORB
            RORB
            ANDB #%00001111 ; Strip off the upper nibble
            CLRA ; D now contains 000n where n is the MSnibble
            ADDD #HEX_TABLE ; Set up for indexed load
            XGDX
            LDAA 0,X ; Get the MSnibble character into ACCA
            PULB ; Retrieve the LSnibble character into ACCB
            RTS
*******************************************************************
BCD2ASC     LDAA #0 ; Initialize the blanking flag
            STAA NO_BLANK

C_TTHOU     LDAA TEN_THOUS ; Check the "ten_thousands" digit
            ORAA NO_BLANK
            BNE NOT_BLANK1

ISBLANK1    LDAA #' ' ; It"s blank
            STAA TEN_THOUS ; so store a space
            BRA C_THOU ; and check the "thousands" digit

NOT_BLANK1  LDAA TEN_THOUS ; Get the "ten_thousands" digit
            ORAA #$30 ; Convert to ascii
            STAA TEN_THOUS
            LDAA #$1 ; Signal that we have seen a "non-blank" digit
            STAA NO_BLANK

C_THOU      LDAA THOUSANDS ; Check the thousands digit for blankness
            ORAA NO_BLANK ; If it"s blank and "no-blank" is still zero
            BNE NOT_BLANK2

ISBLANK2    LDAA #' ' ; Thousands digit is blank
            STAA THOUSANDS ; so store a space
            BRA C_HUNS ; and check the hundreds digit

NOT_BLANK2  LDAA THOUSANDS ; (similar to "ten_thousands" case)
            ORAA #$30
            STAA THOUSANDS
            LDAA #$1
            STAA NO_BLANK

C_HUNS      LDAA HUNDREDS ; Check the hundreds digit for blankness
            ORAA NO_BLANK ; If it"s blank and "no-blank" is still zero
            BNE NOT_BLANK3

ISBLANK3    LDAA #' ' ; Hundreds digit is blank
            STAA HUNDREDS ; so store a space
            BRA C_TENS ; and check the tens digit

NOT_BLANK3  LDAA HUNDREDS ; (similar to "ten_thousands" case)
            ORAA #$30
            STAA HUNDREDS
            LDAA #$1
            STAA NO_BLANK

C_TENS      LDAA TENS ; Check the tens digit for blankness
            ORAA NO_BLANK ; If it"s blank and "no-blank" is still zero
            BNE NOT_BLANK4

ISBLANK4    LDAA #' '   ; Tens digit is blank
            STAA TENS   ; so store a space
            BRA C_UNITS ; and check the units digit

NOT_BLANK4  LDAA TENS ; (similar to "ten_thousands" case)
            ORAA #$30
            STAA TENS

C_UNITS     LDAA UNITS ; No blank check necessary, convert to ascii.
            ORAA #$30
            STAA UNITS

            RTS ; We"re done
*******************************************************************
* Interrupt Vectors *
*******************************************************************
            ORG $FFFE
            DC.W Entry ; Reset Vector
            ORG $FFDE
            DC.W TOF_ISR ; Timer Overflow Interrupt Vector