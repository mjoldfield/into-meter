;;;
;;; Intervalometer firmware
;;; 
;;; Copyright (C) 2011 Martin Oldfield <m@mjo.tc>
;;;
;;; Version 0.1, 2011-06-01 M J Oldfield
;;;	      
;;; This program is free software: you can redistribute it and/or modify
;;; it under the terms of the GNU General Public License as published by
;;; the Free Software Foundation, either version 3 of the License, or
;;; (at your option) any later version.
;;; 
;;; This program is distributed in the hope that it will be useful,
;;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;;; GNU General Public License for more details.
;;; 
;;; You should have received a copy of the GNU General Public License
;;; along with this program.  If not, see <http://www.gnu.org/licenses/>.
;;; ;;;
;;; OVERVIEW     
;;;	      
;;; This firmware runs a simple intervalometer based around a PIC 16F690.
;;;
;;; The basic idea is to trigger a camera into taking a photo on a regular
;;; basis. We can choose:
;;;    a. An amount: 1,3,10,30
;;;    b. A unit: seconds, minutes, hours, days
;;;    c. Whether we want to control some other device e.g. power to the camera
;;;       or a light
;;; 
;;; Thus there are 32 possible modes: 4 x 4 x 2 = 32.
;;; 
;;; The output from the device is a 3-bit binary code, each bit driving
;;; its own opto-isolator. A fourth bit controls a status LED. That
;;; leaves a couple of spare output bits: one has pulses which
;;; are high for four clock ticks and occur at about 16Hz , the other
;;; displays a string of short pulses with a period of 256s. We
;;; use these to calibrate the
;;; oscillator, but they could easily be used for debugging too.
;;; 
;;; The PIC should be clocked by the LP oscillator driven by a 32kHz
;;; crystal. Essentially everything is driven by the timer1 interrupt, so
;;; if one could SLEEP without stopping the timer 1 we'd do that. However
;;; one can't, so we don't! Happily, the chip is pretty frugal when
;;; clocked so slowly.
;;;
;;; On the PIC each instruction takes four clock ticks i.e. we'll
;;; execute 8192 instructions per second. Since the outputs will only
;;; change when we execute an instruction, it makes sense to clock
;;; timer1 at 8kHz too: otherwise transitions will occur
;;; mid-instruction which leads to interesting phase effects.
;;; 
;;; We set timer1 to divide this 8kHz clock by about 512, giving us a
;;; 16Hz clock which we use for scanning the switches and controlling
;;; the outputs.
;;; 
;;; The outputs are actually driven by a dinky little state machine
;;; which gets kicked at 4Hz. It maintains an internal 24-bit counter,
;;; which gives a maximum time between takes of:
;;; 
;;;   2^24 * 0.25s ~ 4e6s ~ 1/8 year.
;;; 
;;; The code makes no attempt to be clever or concise. The PIC's woefully
;;; overspecified for the task, but I've got a lot of them lying
;;; around. The only tricky part is an attempt to allow for the tuning
;;; of the clock rate.
;;; 
;;; Obviously we can tweak the divider value of 512 to tune the
;;; frequency, but consider the smallest possible tweak:
;;;   
;;;  1/512 ~ 0.2% ~ 7s / hour ~ 3 minutes / day
;;;
;;; This is rather too coarse to be useful: after we'd expect that the
;;; crystal itself is accurate to better than 20ppm which is roughly
;;; 2s per day.
;;; 
;;; To do better, we'll need to apply different dividers in sequence:
;;; suppose we change our atom to be 4096 of these cycles. Then we
;;; can adjust the count (and hence the frequency) to about 1 part
;;; in 2 million or 0.5ppm. Going any further seems pointless unless
;;; we're going to start applying temperature compensation and the
;;; like. Besides, those 2 million cycles will take nearly 5 minutes
;;; at 8kHz.
;;;
;;; So, in summary we'll generate a 4Hz clock to drive the camera
;;; which might have some jitter, but that should average out over
;;; periods longer than about 5 minutes.

#include <P16F690.inc> 

        __CONFIG (_LP_OSC & _WDT_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF &  _IESO_OFF & _FCMEN_OFF)

       radix dec

       constant	calib_led   = 16
       constant	status_led  = 8
       constant power_led   = 4
       constant shutter_led = 2
       constant focus_led   = 1
	
       cblock 0x20
;;; These registers are free for any code to use locally i.e. they
;;; might get trashed across a CALL
scratch0:	
scratch1:	
scratch2:	
scratch3:	

;;; Main system clock
sys_clk:			; increments at 16Hz (fast) clock

;;; Registers for tweaking the clock rate
adj_clk_h:			; 16 bit counter (increments at 16 per tick)
adj_clk_l:	
tmr1_inc_l:			; the actual adjustment applied to TMR1 L

;;; Flags OR'd with LED state for calibration signal
calib_flags:
	
;;; LED state machine state
sm_i:				; position in state machine
sm_clk_l:			; 24-bit counter incrementing at 4Hz
sm_clk_m:
sm_clk_h:	

;;; Basically the state of the front panel switches
mode:				; 
old_mode:	

;;; status of LEDs and other outputs
led_status:			

;;; state machine transition parameters
;;; in a better implementation, we'd probably stash these in flash
t_start:			; 8-bit time to start power 
t_focus:			;               engage focus
t_shutter:			; 	        press shutter
t_release:			;               release focus and shutter
t_powerdown:			;               disable power
t_end_l:        		; 24-bit total cycle time
t_end_m:        
t_end_h:        
       
        endc

        org     0x000
        goto    main

        org     0x004
        goto    int_handler

;;; These registers need to be common to all banks for the interrupt handler
        cblock 0x71
w_inth_save
status_inth_save
pclath_inth_save
        endc

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Interrupt handler
;;; 
int_handler:    
        ;; Save status
        movwf   w_inth_save
        swapf   STATUS, w
        clrf    STATUS		    ; ensure, inter alia, that we're in bank 0
        movwf   status_inth_save
        movf    PCLATH, w
        movwf   pclath_inth_save
        clrf    PCLATH
	
        btfsc   PIR1, TMR1IF	; Timer1 is used for timekeeping
        goto    timer_int_vector
   
error_handler:
	bsf	PORTC, 3
	call	delay
	bcf	PORTC, 3
	call	delay
        goto    error_handler

int_return:
        movfw   pclath_inth_save
        movwf   PCLATH
        swapf   status_inth_save, w
        movwf   STATUS
        swapf   w_inth_save,f
        swapf   w_inth_save,w

        retfie
	 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Timer 1 code
;;
;; The sole aim of the code is to 'call sys_tick' every 16Hz
;;
	;; In essence the next 4 registers should be (16 x) the number
	;; of cycles in exactly 256s.
	
	;; higher number => shorter period
	constant	tmr1_dh   = 0xfe ; 0xfe00 -> 0x10000 = 512 => 16Hz fast clock		
	constant	tmr1_dl   = 0x01 ; these 2 cycles are lost when we reload

	;; tweak setting (only the 12 most significant bits matter)
	;; this is device/crystal specific
	constant	tmr1_adj_h = 0xff ; 
	constant	tmr1_adj_l = 0x70 ;

	;; increment to adjustment clock (0x80 => 32s cycle, 0x40 = 64s cycle, ..)
	constant	adj_clk_inc = 0x10

timer_init:
	bcf    	T1CON, TMR1ON   ; stop timer
	movlw	tmr1_dh
	movwf	TMR1H

	bsf    	STATUS, RP0     ; Bank 1
	bsf    	PIE1, TMR1IE    ; Enable timer 1 interrupt

	bcf    	STATUS, RP0     ; Bank 0
	movlw  	0x09            ; 4:1 prescale, enabled, sync, external
	movwf  	T1CON

	movlw	0x20
	movwf	calib_flags

	clrf	adj_clk_h
	clrf	adj_clk_l

	movlw	tmr1_dl
	movwf	tmr1_inc_l
	
	return

timer_int_vector:
        bcf    	PIR1, TMR1IF

	;;; time critical code starts here
        ;;; -- the code in this block should always take the same number of ticks
	
	;; potentially pulse bit 4 of PORTC to generate a clock for calibration
	;; always pulse bit 5 for exacly 4 cycles
	movfw	led_status
	andlw	0x0f
	iorwf	calib_flags, w	
	movwf	PORTC

	bcf	calib_flags, 4

	movfw	led_status
	andlw	0x0f
	movwf	PORTC
	
	;; increment the timer
	movlw	tmr1_dh
	movwf	TMR1H

	;; we're ignoring the possibility of a carry here.
	movfw	tmr1_inc_l
	addwf	TMR1L, f
	
	;;;  time critical code ends here

	;; increment (by some value) a 16-bit counter
	;; smaller increments => longer timing periods => longer to test, but better precision
	;; if we rollover, then pulse the calibration signal on the next timer interrupt
	movlw	adj_clk_inc
	addwf	adj_clk_l, f

	movlw	0
	btfsc	STATUS, C
	movlw	1
	addwf	adj_clk_h, f

	btfsc	STATUS, C
	bsf	calib_flags, 4
	
	;; figure out what adjusment to apply to the next tick:
	;; the high-byte is fixed so look at the low---
	;; normally we add tmr1_dl, but for some fraction of the adj_clk cycle
	;; we add (tmr1_dl + 1) instead
	;;
	;; to see which, just add an offset to the 16-bit counter

	movfw	adj_clk_l
	addlw	tmr1_adj_l

	movfw	adj_clk_h
	xorwf	adj_clk_l, w	
	
	btfsc	STATUS, C
	incfsz	adj_clk_h, w
	addlw	tmr1_adj_h

	movlw	tmr1_dl
	btfsc	STATUS, C
	movlw	(tmr1_dl + 1)
	movwf	tmr1_inc_l

	;; call the system stuff
	call    sys_tick	
        goto    int_return
        
;;; A simple delay loop. Wait for about 0.5s 
delay:
	clrf	scratch0
	movlw	0x14
	movwf	scratch1
delay1:		
        decfsz 	scratch0,f
	goto	delay1
        decfsz 	scratch1,f
	goto	delay1
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Main program
;;; 
main:
	;; Standard 16F690 initialization
        clrf    STATUS          
        clrf    INTCON
        clrf    PIR1

	bsf	STATUS,RP1
	clrf    ANSEL
	clrf    ANSELH

	bcf	STATUS,RP1
	bsf     STATUS,RP0      
        clrf    PIE1       
        movlw   0xff
        movwf   TRISA		; Start off with all inputs
        movwf   TRISB		; Start off with all inputs
	movlw  	0xc0
        movwf  	TRISC		; make lower 6 bits of C into outputs

	bcf    	STATUS,RP0     	
	clrf	PORTA		
	clrf	PORTC
	
	;; Application initialization
	call	read_mode	;

;	call	delay		
	
	call	timer_init
	
        bsf    INTCON, PEIE	; interrupts are GO!
        bsf    INTCON, GIE

loop:

        goto 	loop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Code to handle our own local clocks, which hang off the interrupt
;;

system_init:	
	call    init_transitions
	call	sm_init
	clrf	sys_clk
	return	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;	
;;; This is essentially the main entry point to the code:
;;; call this repeatedly to make stuff work
;;;
;;; Basically we've got two concerns:
;;; 
;;;   - poll the front panel and see if anything's changed: if it has
;;; 	then reconfigure the state machine appropriately
;;; 
;;;   - increment the slow clock and give the main state machine a
;;; 	chance to change the LEDs
;;;
;;; The only subtlety, if that's not too posh a word for it, is to make
;;; the machine play dead for a short while if it sees things changing
;;; or invalid.
;;; 
       constant	lockout_time = 4

sys_tick:
	movfw	mode
	movwf	old_mode	; bit 7 => consistency, bit 6 => we were running

	call	read_mode	; bit 7 => consistency

	btfsc	mode, 7
	goto	hold_off	; new state is inconsistent

	btfss	old_mode, 6
	goto	was_stopped

	;; getting here means that we were running last time and new state is good
	movfw	mode
	xorwf	old_mode, w
	andlw	0x3f
	btfsc	STATUS, Z	; Z => same state
	goto	tick_on		; was running and still in the same state

hold_off:	
	clrf	led_status
	goto	tick_down

was_stopped:
	;; getting here => we were stopped but new state is good
	movfw	sys_clk
	sublw	lockout_time
	btfss	STATUS, Z
	goto	tick_down	; state is noew good, but hasn't been stable for long

	;; state has just become good
	call	system_init

tick_on:
	bsf	mode, 6

tick_down:

	;; the fast clock ticks at 16Hz, but the state machine should tick at 4Hz
	movfw	sys_clk
	andlw	0x03		; slow clock divider
	btfsc	STATUS, Z
	call    sm_tick	        ; sets LED status

	call    write_led	; update LEDs
	incf	sys_clk, f

        return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; LED control
;;;
;;; Read led_status register for bits 0--2, then write PORTC
;;; appropriately, inferring the state of the fourth bit.
;;; 
;;;     1.. => power   => flashing (on 1/8 the time)
;;;     .1. => shutter => always
;;;     ..1 => focus   => flashing (on 1/2 the time)
;;;
;;; The actual LEDs are connected to PORTC:0-3

write_led:
        ;; default is for status to be off
        bcf     led_status, 3

	btfsc	led_status, 1 	; shutter
	goto	led_status_on	; => always on

	movfw	sys_clk		; 
	andlw	0x01
	btfss	STATUS, Z
	goto	led_status_done

led_query0:	
	btfsc	led_status, 0	; focus
	goto	led_status_on	; flashing: on half of the time

led_query2:	
	btfss	led_status, 2	; power
	goto	led_status_done	

	movfw	sys_clk		; flashing: on 1/8th of the time
	andlw	0x06
	btfss	STATUS, Z
	goto	led_status_done

led_status_on:
	bsf	led_status, 3

led_status_done:	
        movfw	led_status
	movwf	PORTC
	return
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Given the mode, set the t_ registers
;;; 
;;; Basically there are different numbers for powered and unpowered
;;; modes (because if we're controlling the power we wait a bit for
;;; things to settle before taking the picture). There are also
;;; special cases for short periods (< 1 minute).
;;; 
;;; All the counts in the timetables below are in 0.25s
;;; mode is puunn, where:
;;;     p - 1 if power should be switched
;;;    uu - units for cycle time
;;;    dd - how many units
;;;
;;; Note that the full 24-bit period is set in init_time_end
;;; 
init_transitions:
	call	init_time_end	

	clrf	t_start
	
	movfw	mode
	andlw	0x0f
	btfss	STATUS, Z
	goto 	init_seconds

write_smt macro focus, shutter, release, powerdown
        movlw   focus
        movwf   t_focus
        movlw   shutter
        movwf   t_shutter
        movlw   release
	movwf	t_release
        movlw   powerdown
	movwf	t_powerdown
	endm

init_1s:	
	write_smt 0,3,4,4
	return;	
	
init_seconds:
	movfw	mode
	andlw	0x0c
	btfss	STATUS, Z
	goto 	init_normal	

	write_smt 0, 8, 10, 0
        movfw	t_end_l
	movwf	t_powerdown
	return
	
init_normal:	
        btfsc   mode, 4
        goto    init_powered

init_unpowered: 
	write_smt 0,16,28,28
	return
        
init_powered:
	write_smt 40, 56, 68, 240
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Given the mode, set the t_end_ registers
;;; 
;;; Computed GOTOs here, so let's stay in page 0x0200
	org     0x0200

;;; t is actually in seconds, and assumes a 4Hz clock

;;; each of these MUST BE 8 words long!
write_te macro t
        movlw   (t <<  2) & 0xff
	movwf	t_end_l
        movlw   (t >>  6) & 0xff
	movwf	t_end_m        
        movlw   (t >> 14) & 0xff
	movwf	t_end_h
        return
        nop
	endm	

init_time_end:
        movfw   mode
        andlw   0x0f

	movwf	scratch0
	bcf	STATUS, C
        rlf    	scratch0, f
        rlf    	scratch0, f
        rlf    	scratch0, f

	movlw	HIGH t_base
	movwf	PCLATH
	movfw	scratch0
	addwf	PCL, f

t_base:	
t_1s:	write_te    1
t_3s:	write_te    3
t_10s:	write_te   10
t_30s:	write_te   30

t_1m:	write_te   60
t_3m:	write_te  180
t_10m:	write_te  600
t_30m:	write_te 1800

t_1h:	write_te    1 * 3600
t_3h:	write_te    3 * 3600
t_10h:	write_te   10 * 3600
t_30h:	write_te   30 * 3600

t_1d:	write_te    1 * 86400
t_3d:	write_te    3 * 86400
t_10d:	write_te   10 * 86400
t_30d:	write_te   30 * 86400

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; The main state machine which controls the intervalometer
;;;

	org 0x0300 ;; computed GOTOs are used, so let's stay in page 0x0300

sm_init:
	;; sm_i is a pointer to an instruction (offset from sm_base)
	movlw	sm_phase0 - sm_base
	movwf	sm_i

	;; sm_clk_X form a 24-bit counter
	clrf	sm_clk_l
	clrf	sm_clk_m
	clrf	sm_clk_h

	return

;;; The normal entry point
sm_tick:
	call	sm_handler
	
	incfsz	sm_clk_l, f	; increment the 24-bit cycle counter
	goto	inced
	incfsz	sm_clk_m, f
	goto	inced
	incf	sm_clk_h, f
inced:
	return
	
sm_retry:       
        movwf   sm_i
sm_handler:
	movlw	HIGH sm_base
	movwf	PCLATH
        movfw	sm_i
	addwf	PCL, f
sm_base:                

	;; All states return through here
sm_return:
	;; in an unpowered mode the power LED should always be off
	btfss	mode, 4	
	bcf	led_status, 2	
	return

sm_std_phase macro t,leds,next
	movfw	sm_clk_l		
	subwf	t, w
	btfss	STATUS, Z
        goto	sm_return
        
	movlw	leds
	movwf	led_status

	movlw	next - sm_base
        goto    sm_retry
	endm

sm_phase0:  sm_std_phase t_start,     (power_led),                           sm_phase1
sm_phase1:  sm_std_phase t_focus,     (power_led | focus_led),               sm_phase2
sm_phase2:  sm_std_phase t_shutter,   (power_led | focus_led | shutter_led), sm_phase3
sm_phase3:  sm_std_phase t_release,   (power_led),                           sm_phase4
sm_phase4:  sm_std_phase t_powerdown, 0,                                     sm_phase5h

;;; phases 5x don't mess with the LEDs but do look beyond the lowest byte => need a new macro :)

sm_end_phase macro t, cycle, next
	movfw	cycle
	subwf	t, w
	btfss	STATUS, Z
        goto	sm_return

	movlw	next - sm_base
        goto    sm_retry
	endm

sm_phase5h:  sm_end_phase t_end_h, sm_clk_h, sm_phase5m
sm_phase5m:  sm_end_phase t_end_m, sm_clk_m, sm_phase5l

sm_phase5l:     
	movfw	sm_clk_l		
        subwf   t_end_l, w
	btfss	STATUS, Z
        goto	sm_return

	call	sm_init
	movlw	sm_phase0 - sm_base
	goto	sm_retry

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; Code to scan the front panel switches, setting mode appropriately
;;;
;;; mode = 000puunn (power, unit, number) if all's OK
;;;        1xxxxxxx on error
;;; The input switches are arranged in a matrix.
;;; PORTA:2 and PORTC:6,7 are inputs which float high. PORTB:4-7 are outputs which
;;; pull the inputs low if the appropriate switch is closed.
;;; 

	org     0x0400;; computed GOTOs are used, so let's stay in page 0x0400

read_mode:
	clrf	scratch0
	clrf	scratch1

	movlw	0xef
	movwf	scratch2

	bsf     STATUS,RP0
	movlw	0x0f
	movwf	TRISB

	movlw	0xc0
	movwf	TRISC
        bcf     STATUS,RP0

	clrf	mode
rloop:	
	;; set portb outputs so one bit is low
	movfw	scratch2
	movwf	PORTB

	bcf	STATUS, C
	rrf	scratch0, f
	rrf	scratch1, f

	;; Check the three inputs
	btfss	PORTC, 7
	bsf	scratch0, 3	; set a bit in scratch0

	btfss	PORTC, 6
	bsf	scratch1, 3 	; set a bit in scratch1

	btfss	PORTA, 2
	bsf	mode, 0		; set the mode bit directly

	bsf	STATUS, C
	rlf	scratch2, f

	incf	scratch2, w
	btfss	STATUS, Z
	goto	rloop		; loop over all four output lines

	call	accum_mode	; do bits 2-3 of mode

	movfw	scratch1
	movwf	scratch0
	call	accum_mode	; do bits 0-1 of mode

	return

;;; roll the bottom two bits of mode into bits 2:3, then
;;; set bits 0:1 from scratch0
;;;
;;; if bit:7's set, which indicates an error, come back at once.
accum_mode
	btfsc	mode, 7
	return

	;; Clear 2 LSBs of mode
	bcf	STATUS, C
	rlf	mode, f
	rlf	mode, f

	movlw	HIGH read_bits
	movwf	PCLATH

	;; Read input (4 bits) and move to LSBs
	movfw	scratch0
	andlw	0x0f

	;; Convert to two bits (bit 7 set => error)
	call	read_bits

	;; Write mode
	iorwf	mode, f

	return

;;; bit counting!
;;; convert 0001 => 0, 0010 => 1, 0100 => 2, 1000 => 3
;;;         everything else to 0x80 (error)
read_bits:
	addwf	PCL,f

	retlw	0x80
	retlw	0x00
	retlw	0x01
	retlw	0x80

	retlw	0x02
	retlw	0x80
	retlw	0x80
	retlw	0x80

	retlw	0x03
	retlw	0x80
	retlw	0x80
	retlw	0x80

	retlw	0x80
	retlw	0x80
	retlw	0x80
	retlw	0x80
end
