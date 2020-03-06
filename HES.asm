.include "m2560def.inc"


.def key_ins = r19			;key ins instead
.def temp1 = r20
.def temp2 = r21

.def temp3 = r24
.def temp4 = r25

.equ PORTLDIR =0xF0			; use PortL for input/output from keypad: PF7-4, output, PF3-0, input
.equ INITCOLMASK = 0xEF		; scan from the leftmost column, the value to mask output
.equ INITROWMASK = 0x01		; scan from the bottom row
.equ ROWMASK  =0x0F			; low four bits are output from the keypad. This value mask the high 4 bits.

.equ LCD_CTRL_PORT = PORTA
.equ LCD_CTRL_DDR = DDRA
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

.equ LCD_DATA_PORT = PORTF
.equ LCD_DATA_DDR = DDRF
.equ LCD_DATA_PIN = PINF

.dseg
.org 0x200
col: .byte 1
row: .byte 1
cmask: .byte 1
rmask: .byte 1
start_fly: .byte 1
timer_1s: .byte 1
position_x: .byte 1						;helicopter left right			;0-50
position_y: .byte 1						;helicopter forward backward	;0-50
position_z: .byte 1						;helicopter up down height		;0-10
heli_speed: .byte 1						;helicopter speed 				;1m/s, 2m/s, 3m/s, 4m/s
heli_ex_speed: .byte 1					;ex speed 
heli_state: .byte 1
heli_ex_state: .byte 1					;ex state
fly_dis1: .byte 1						;distance byte 1-6
fly_dis2: .byte 1
fly_dis3: .byte 1
fly_dis4: .byte 1
fly_dis5: .byte 1
fly_dis6: .byte 1
fly_time1: .byte 1						;time byte 1-6
fly_time2: .byte 1
fly_time3: .byte 1
fly_time4: .byte 1
fly_time5: .byte 1
fly_time6: .byte 1


.cseg
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;LCD macro function;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;STORE												;;;;;||
.macro STORE																				;;;;;||
	.if @0 > 63																				;;;;;||
		sts @0, @1																			;;;;;||
	.else																					;;;;;||
		out @0, @1																			;;;;;||
	.endif																					;;;;;||
.endmacro																					;;;;;||
																							;;;;;||
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;LOAD												;;;;;||	
.macro LOAD																					;;;;;||
	.if @1 > 63																				;;;;;||
		lds @0, @1																			;;;;;||
	.else																					;;;;;||
		in @0, @1																			;;;;;||
	.endif																					;;;;;||
.endmacro																					;;;;;||
																							;;;;;||
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;do_lcd_command										;;;;;||
.macro do_lcd_command																		;;;;;||
	ldi r16, @0																				;;;;;||
	rcall lcd_wait																			;;;;;||
	rcall lcd_command																		;;;;;||
.endmacro																					;;;;;||
																							;;;;;||
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;do_lcd_data										;;;;;||
.macro do_lcd_register																		;;;;;||
	mov r16, @0																				;;;;;||
	rcall lcd_wait																			;;;;;||
	rcall lcd_data																			;;;;;||
.endmacro																					;;;;;||
																							;;;;;||
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;do_lcd_data										;;;;;||
.macro do_lcd_data																			;;;;;||
	ldi r16, @0																				;;;;;||
	rcall lcd_wait																			;;;;;||
	rcall lcd_data																			;;;;;||
.endmacro																					;;;;;||
																							;;;;;||
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;LCD_location										;;;;;||
.macro lcd_location																			;;;;;||
	.if @0 == 2																				;;;;;||
		ldi r16, 0b11000000																	;;;;;||
	.else																					;;;;;||
		ldi r16, 0b10000000																	;;;;;||
	.endif																					;;;;;||
	ldi r17, @1																				;;;;;||
	dec r17																					;;;;;||
	add r16, r17																			;;\  ||  /
	rcall lcd_wait																			;;;\ || /
	rcall lcd_command																		;;;;\||/
.endmacro																					;;;;;\/
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;end LCD macro;;;;;;;;;;;	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;2 unit number ascii;;;;;
.macro count_2u_num
	clr temp2
	lds temp1, @0
	cpi temp1, 10
	brsh get_dec
	jmp end_count_2u_num					;count number unit
get_dec:
	inc temp2
	subi temp1, 10
	cpi temp1, 10
	brsh get_dec
end_count_2u_num:
	subi temp2, -'0'						;get ascii
	subi temp1, -'0'
.endmacro
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


.org 0x0000
	jmp RESET

.org OVF0addr									;timer address
	jmp Timer0OVF

Timer0OVF:										;timer run											
	push key_ins
	push temp1
	push temp2
	push temp3
	push temp4

	lds temp1, timer_1s
	inc temp1
	cpi temp1, 61
	brlo end_Timer0OVF							;keep every 61 for in once about 0.9994s

	lds temp1, start_fly
	cpi temp1, 0				;0 grand
	breq None_fly
	cpi temp1, 3				;3 land
	breq Jump_results_display
	cpi temp1, 4				;4 crash
	breq crash_display
	cpi temp1, 5				;5 landing
	breq Jump_landing_display

	rcall position_count
	rcall data_count

	lds temp1, start_fly
	cpi temp1, 4				;4 crash
	breq crash_display

	rcall state_dispaly

	ldi temp1, 0
	jmp end_Timer0OVF

None_fly:										;none fly state display
	ldi temp1, 0
	jmp end_Timer0OVF

Jump_landing_display:							;landing state display
	rcall position_count
	rcall data_count
	lds temp1, start_fly
	cpi temp1, 3				;3 land
	breq Jump_results_display
	rcall state_dispaly
	ldi temp1, 0
	jmp end_Timer0OVF

Jump_results_display:
	rcall results_display
	ldi temp1, 0
	jmp end_Timer0OVF

crash_display:									;crash state display
	rcall show_carsh_location
	ldi temp1, 0
	jmp end_Timer0OVF

end_Timer0OVF:
	sts timer_1s, temp1

	pop temp4
	pop temp3
	pop temp2
	pop temp1
	pop key_ins
	reti

RESET:
	cli							;disable Global Interrupt
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;data_init
	clr temp2
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;PORT_INIT
	ser temp1					; PORTC is outputs
	out DDRC, temp1	
	out PORTC, temp1			; initialize state
	out DDRE, temp1

	clr temp1
	out DDRD, temp1	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;KEYPAD_INIT
	ldi temp1, PORTLDIR			; columns are outputs, rows are inputs
	sts	DDRL, temp1
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;LCD_INIT
	ldi r16, low(RAMEND)
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

	ser r16
	STORE LCD_DATA_DDR, r16
	STORE LCD_CTRL_DDR, r16
	clr r16
	STORE LCD_DATA_PORT, r16
	STORE LCD_CTRL_PORT, r16

	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001100 ;	Cursor off, no blink;
	lcd_location 1, 1		  ;set lcd location
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;PWM PORETE INIT
	ldi temp1, 0b11111111
	out DDRE, temp1 						; Bit 3 will function as OC3A.

	ldi temp1, 0xff 						; the value controls the PWM duty cycle
	sts OCR3AL, temp1
	clr temp1
	sts OCR3AH, temp1
											; Set Timer3 to Phase Correct PWM mode.
	ldi temp1, (1 << CS30)|(1 << CS31)
	sts TCCR3B, temp1
	ldi temp1, (1<< WGM30)|(1<<COM3A1)|(1<<COM3A0)
	sts TCCR3A, temp1
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Timer0 interrupt INIT

	ldi temp1, 0b00000000
	out TCCR0A, temp1
	ldi temp1, 0b00000101					;1/1024	timer
	out TCCR0B, temp1
	ldi temp1, 1<<TOIE0
	sts TIMSK0, temp1 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;port_test
	ser temp1
	out PORTC, temp1
	ldi temp1, 0xaa
	sts OCR3AL, temp1
	;;;;;;;;
	rcall sleep_500ms					;as test time
	;;;;;;;;
	ldi temp1, 0xff
	sts OCR3AL, temp1
	clr temp1
	out PORTC, temp1

	do_lcd_command 0x01
	do_lcd_data 'S'
	do_lcd_data 't'
	do_lcd_data 'a'
	do_lcd_data 'r'
	do_lcd_data 't'
	do_lcd_data ':'
	
	rcall set_inti_data					;set date inti

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
end_RESET:
	sei ; enable Global Interrupt
	rjmp main
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;init_data
set_inti_data:
	clr temp1								;used data clear

	sts col, temp1
	sts row, temp1
	sts cmask, temp1
	sts rmask, temp1

	sts position_z, temp1

	sts heli_speed, temp1
	sts heli_ex_speed, temp1

	sts timer_1s, temp1

	sts start_fly, temp1

	sts fly_time1, temp1
	sts fly_time2, temp1
	sts fly_time3, temp1
	sts fly_time4, temp1
	sts fly_time5, temp1
	sts fly_time6, temp1

	sts fly_dis1, temp1
	sts fly_dis2, temp1
	sts fly_dis3, temp1
	sts fly_dis4, temp1
	sts fly_dis5, temp1
	sts fly_dis6, temp1

	ldi temp1, 25

	sts position_x, temp1 
	sts position_y, temp1

	ldi temp1, '-'

	sts heli_state, temp1
	sts heli_ex_state, temp1
end_set_inti_data:
	ret										;function finish
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
main:
	lds temp2, start_fly
	cpi temp2, 4					;4 crash
	breq end_fly

	rcall keypad_scan				;input stroe in temp1 with ascii
	rcall fly_ctrl
	
	lds temp2, start_fly
	cpi temp2, 5					;5 landing no ctrl
	breq end_fly	
	
	cpi temp2, 4					;4 crash
	breq end_fly																			

main_loop_finish:
	rjmp main

end_fly:
	rjmp end_fly					;finish one fly or crash
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;position_coun;;;;;;;;
position_count:
	lds temp1, heli_state
											;find heli state and change position
	cpi temp1, 'U'
	breq JUMP_renew_z_add
	cpi temp1, 'D'
	breq JUMP_renew_z_sub

	cpi temp1, 'F'
	breq JUMP_renew_y_add
	cpi temp1, 'B'
	breq JUMP_renew_y_sub

	cpi temp1, 'L'
	breq JUMP_renew_x_sub
	cpi temp1, 'R'
	breq JUMP_renew_x_add

	jmp end_position_count

JUMP_renew_z_add:									;avoid cp over jump range
	jmp renew_z_add
JUMP_renew_z_sub:
	jmp renew_z_sub 
JUMP_renew_y_add:
	jmp renew_y_add
JUMP_renew_y_sub:
	jmp renew_y_sub
JUMP_renew_x_add:
	jmp renew_x_add
JUMP_renew_x_sub:
	jmp renew_x_sub

renew_z_add:										;z count
	lds temp1, position_z	
	lds temp2, heli_speed
	add temp1, temp2
	cpi temp1, 10
	brsh z_add_crash
	sts position_z, temp1
	jmp end_position_count
z_add_crash:										;z crash
	ldi temp1, 10
	sts position_z, temp1
	ldi temp1, 4
	sts start_fly, temp1
	rcall show_carsh_location
	jmp end_position_count

renew_z_sub:										;avoid overflow before sub check range to sub
	lds temp1, position_z
	lds temp2, heli_speed
	cp temp1, temp2
	brlo z_sub_crash
	breq z_sub_crash
	sub temp1, temp2
	sts position_z, temp1
	jmp end_position_count
z_sub_crash:										;z crash
	ldi temp1, 0
	sts position_z, temp1
	lds temp1, start_fly
	cpi temp1, 5
	breq landing_safely
	ldi temp1, 4
	sts start_fly, temp1
	rcall show_carsh_location
	jmp end_position_count

landing_safely:										;only z position could get safety landing
	ldi temp1, 3
	sts start_fly, temp1
	jmp end_position_count

renew_y_add:										;y count
	lds temp1, position_y
	lds temp2, heli_speed
	add temp1, temp2
	cpi temp1, 50
	brsh y_add_crash
	sts position_y, temp1
	jmp end_position_count
y_add_crash:										;y crash
	ldi temp1, 50
	sts position_y, temp1
	ldi temp1, 4
	sts start_fly, temp1
	rcall show_carsh_location
	jmp end_position_count

renew_y_sub:										;avoid overflow before sub check range to sub
	lds temp1, position_y
	lds temp2, heli_speed
	cp temp1, temp2
	brlo y_sub_crash
	breq y_sub_crash
	sub temp1, temp2
	sts position_y, temp1
	jmp end_position_count
y_sub_crash:										;y crash
	ldi temp1, 0
	sts position_y, temp1
	ldi temp1, 4
	sts start_fly, temp1
	rcall show_carsh_location
	jmp end_position_count

renew_x_add:										;x count
	lds temp1, position_x
	lds temp2, heli_speed
	add temp1, temp2
	cpi temp1, 50
	brsh x_add_crash
	sts position_x, temp1
	jmp end_position_count
x_add_crash:										;x crash
	ldi temp1, 50
	sts position_x, temp1
	ldi temp1, 4
	sts start_fly, temp1
	rcall show_carsh_location
	jmp end_position_count

renew_x_sub:										;avoid overflow before sub check range to sub
	lds temp1, position_x
	lds temp2, heli_speed
	cp temp1, temp2
	brlo x_sub_crash
	breq x_sub_crash
	sub temp1, temp2
	sts position_x, temp1
	jmp end_position_count
x_sub_crash:										;x crash
	ldi temp1, 0
	sts position_x, temp1
	ldi temp1, 4
	sts start_fly, temp1
	rcall show_carsh_location
	jmp end_position_count

end_position_count:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;show_carsh_location;;
show_carsh_location:
	lcd_location 1, 1
	do_lcd_data '*'
	do_lcd_data 'C'
	do_lcd_data 'r'
	do_lcd_data 'a'
	do_lcd_data 's'
	do_lcd_data 'h'
	do_lcd_data ' '
	do_lcd_data 'L'
	do_lcd_data 'o'
	do_lcd_data 'c'
	do_lcd_data 'a'
	do_lcd_data 't'
	do_lcd_data 'i'
	do_lcd_data 'o'
	do_lcd_data 'n'
	do_lcd_data '*'
	lcd_location 2, 1
	do_lcd_data 126
	do_lcd_data 126
	do_lcd_data 126
	do_lcd_data '('
	count_2u_num position_x		;change 2 unit number to 1 unit and back ASCII
	do_lcd_register temp2
	do_lcd_register temp1
	do_lcd_data ','
	count_2u_num position_y		;change 2 unit number to 1 unit and back ASCII
	do_lcd_register temp2
	do_lcd_register temp1
	do_lcd_data ','
	count_2u_num position_z		;change 2 unit number to 1 unit and back ASCII
	do_lcd_register temp2
	do_lcd_register temp1
	do_lcd_data ')'
	do_lcd_data 127
	do_lcd_data 127
	do_lcd_data 127

	rcall black_box
end_show_carsh_location:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;black_box;;;;;;;;;;;;
black_box:						;simulate black box signal flash
	ldi temp1, 0xff
	out PORTC, temp1
	rcall sleep_100ms
	rcall sleep_100ms
	ldi temp1, 0x00
	out PORTC, temp1
	rcall sleep_100ms

	ldi temp1, 0xff
	out PORTC, temp1
	rcall sleep_100ms
	rcall sleep_100ms
	ldi temp1, 0x00
	out PORTC, temp1
	rcall sleep_100ms

	ldi temp1, 0xff
	out PORTC, temp1
	rcall sleep_100ms
	rcall sleep_100ms
	ldi temp1, 0x00
	out PORTC, temp1
end_black_box:						;function would be call every 1 second
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;fly_ctrl;;;;;;;;;;;;;
fly_ctrl:
	cli								;keep only one thread use display function
	lds temp2, start_fly
	cpi temp2, 0					;0 grand only # could ctrl
	breq grand_st

	cpi temp1, '#'					;taking off and landing.
	breq JUMP_state_change

	cpi temp1, '*'					;hovering 
	breq JUMP_ctrl_hovering

	cpi temp1, '1'					;up
	breq JUMP_ctrl_up
	
	cpi temp1, '3'					;down
	breq JUMP_ctrl_down
	
	cpi temp1, '4'					;left
	breq JUMP_ctrl_left
	
	cpi temp1, '6'					;right
	breq JUMP_ctrl_right
		
	cpi temp1, '2'					;forward
	breq JUMP_ctrl_forward

	cpi temp1, '8'					;back
	breq JUMP_ctrl_back

	cpi temp1, 'a'					;speed_up
	breq JUMP_ctrl_speed_up

	cpi temp2, 2					;2 hovering without					
	breq no_case

	cpi temp1, 'b'					;speed_down
	breq JUMP_ctrl_speed_down

no_case:
	jmp end_fly_ctrl

JUMP_state_change:					;avoid cp over jump range 
	jmp state_change
JUMP_ctrl_hovering:
	jmp ctrl_hovering
JUMP_ctrl_up:
	jmp ctrl_up
JUMP_ctrl_down:
	jmp ctrl_down
JUMP_ctrl_left:
	jmp ctrl_left
JUMP_ctrl_right:
	jmp ctrl_right
JUMP_ctrl_forward:
	jmp ctrl_forward
JUMP_ctrl_back:
	jmp ctrl_back
JUMP_ctrl_speed_up:
	jmp ctrl_speed_up
JUMP_ctrl_speed_down:
	jmp ctrl_speed_down

grand_st:
	cpi temp1, '#'					;taking off and landing.
	breq JUMP_state_change
	jmp no_case

JUMP_fly_ctrl_landing:
	jmp fly_ctrl_landing

ctrl_up:
	ldi temp1, 'U'					;change director state to up
	sts heli_state, temp1
	ldi temp1, 1
	sts start_fly, temp1
	rcall assit_speed
	rcall state_dispaly
	jmp end_fly_ctrl

ctrl_down:
	ldi temp1, 'D'					;change director state to down
	sts heli_state, temp1
	ldi temp1, 1
	sts start_fly, temp1
	rcall assit_speed
	rcall state_dispaly
	jmp end_fly_ctrl

ctrl_left:
	ldi temp1, 'L'					;change director state to left
	sts heli_state, temp1
	ldi temp1, 1
	sts start_fly, temp1
	rcall assit_speed
	rcall state_dispaly
	jmp end_fly_ctrl

ctrl_right:
	ldi temp1, 'R'					;change director state to right
	sts heli_state, temp1
	ldi temp1, 1
	sts start_fly, temp1
	rcall assit_speed
	rcall state_dispaly
	jmp end_fly_ctrl

ctrl_forward:
	ldi temp1, 'F'					;change director state to front
	sts heli_state, temp1
	ldi temp1, 1
	sts start_fly, temp1
	rcall assit_speed
	rcall state_dispaly
	jmp end_fly_ctrl

ctrl_back:	
	ldi temp1, 'B'					;change director state to back
	sts heli_state, temp1
	ldi temp1, 1
	sts start_fly, temp1
	rcall assit_speed
	rcall state_dispaly
	jmp end_fly_ctrl

ctrl_speed_up:						;limit speed range and late 0.5s dispaly
	rcall assit_dir
	rcall sleep_500ms
	lds temp1, heli_speed
	inc temp1
	cpi temp1, 4
	brsh max_speed
end_speed_up:
	sts heli_speed, temp1
	rcall state_dispaly
	jmp end_fly_ctrl
max_speed:
	ldi temp1, 4
	jmp end_speed_up

ctrl_speed_down:					;limit speed range and late 0.5s dispaly
	rcall assit_dir
	rcall sleep_500ms
	lds temp1, heli_speed
	dec temp1
	cpi temp1, 1
	brlo min_speed
end_speed_down:
	sts heli_speed, temp1
	rcall state_dispaly
	jmp end_fly_ctrl
min_speed:
	ldi temp1, 1
	jmp end_speed_down

ctrl_hovering:
	lds temp1, start_fly
	cpi temp1, 1
	breq save_state
	cpi temp1, 2
	breq load_state
	jmp end_fly_ctrl				;for crash

save_state:
	lds temp1, heli_speed
	sts heli_ex_speed, temp1
	lds temp1, heli_state
	sts heli_ex_state, temp1

	ldi temp1, 0
	sts heli_speed, temp1
	ldi temp1, '-'
	sts heli_state, temp1

	ldi temp1, 2					;2 hovering
	sts start_fly, temp1
	jmp end_ctrl_hovering

load_state:
	lds temp1, heli_ex_speed
	sts heli_speed, temp1
	lds temp1, heli_ex_state
	sts heli_state, temp1

	ldi temp1, 1					;resumes 1
	sts start_fly, temp1
	jmp end_ctrl_hovering

end_ctrl_hovering:
	rcall state_dispaly
	jmp end_fly_ctrl

state_change:
	lds temp1, start_fly
	cpi temp1, 0					;0 stop begin
	breq fly_ctrl_up
	cpi temp1, 1					;1 fly state
	breq fly_ctrl_landing
	cpi temp1, 2					;2 hoving
	breq fly_ctrl_landing
	jmp end_fly_ctrl

fly_ctrl_up:
	rcall set_inti_data
	ldi temp1, 1					;reset data start
	sts start_fly, temp1
	ldi temp1, 'U'
	sts heli_state, temp1
	ldi temp1, 2
	sts heli_speed, temp1
	rcall state_dispaly
	jmp end_fly_ctrl

fly_ctrl_landing:
	ldi temp1, 5					;5 landing
	sts start_fly, temp1
	ldi temp1, 'D'
	sts heli_state, temp1
	rcall sleep_500ms
	ldi temp1, 1
	sts heli_speed, temp1
	rcall state_dispaly
	jmp end_fly_ctrl

end_fly_ctrl:
	sei								;enable timer interrupt
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;assit for hoving
assit_speed:						;save hovering state
	lds temp1, heli_speed
	cpi temp1, 0
	brne end_assit_speed
	lds temp1, heli_ex_speed
	sts heli_speed, temp1
end_assit_speed:
	ret

assit_dir:							;save hovering director
	lds temp1, heli_state
	cpi temp1, '-'
	brne end_assit_dir
	lds temp1, heli_ex_state
	sts heli_state, temp1
end_assit_dir:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;motor simulate
motor_simulate:
	lds temp2, start_fly				;get state and give speed simulate
	cpi temp2, 0
	breq motor_stop
	cpi temp2, 3
	breq motor_stop
	cpi temp2, 4
	breq motor_stop

	lds temp1, heli_speed
	cpi temp1, 0
	breq speed_change0
	cpi temp1, 1
	breq speed_change1
	cpi temp1, 2
	breq speed_change2
	cpi temp1, 3
	breq speed_change3
	cpi temp1, 4
	breq speed_change4
	jmp end_moter_simulate

motor_stop:								;0
	ldi temp1, 0xff
	sts OCR3AL,temp1
	jmp end_moter_simulate

speed_change0:							;hovering
	ldi temp1, 0xcc
	sts OCR3AL,temp1
	jmp end_moter_simulate

speed_change1:							;1
	ldi temp1, 0x99
	sts OCR3AL,temp1
	jmp end_moter_simulate

speed_change2:							;2
	ldi temp1, 0x77
	sts OCR3AL,temp1
	jmp end_moter_simulate

speed_change3:							;3
	ldi temp1, 0x55
	sts OCR3AL,temp1
	jmp end_moter_simulate

speed_change4:							;4
	ldi temp1, 0x00
	sts OCR3AL,temp1
	jmp end_moter_simulate

end_moter_simulate:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;results_display;;;;;;;;
results_display:
	lcd_location 1, 1
	do_lcd_data 'D'
	do_lcd_data 'i'
	do_lcd_data 's'
	do_lcd_data 't'
	do_lcd_data 'a'
	do_lcd_data 'n'
	do_lcd_data 'c'
	do_lcd_data 'e'
	do_lcd_data ':'
	ldi temp2, 0						;compare 0 every unit for hide no meaning 0
	lds temp1, fly_dis6
	or temp2, temp1
	cpi temp2, 0
	breq next_dis5
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
next_dis5:
	lds temp1, fly_dis5
	or temp2, temp1
	cpi temp2, 0
	breq next_dis4
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
next_dis4:
	lds temp1, fly_dis4
	or temp2, temp1
	cpi temp2, 0
	breq next_dis3
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
next_dis3:
	lds temp1, fly_dis3
	or temp2, temp1
	cpi temp2, 0
	breq next_dis2	
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
next_dis2:
	lds temp1, fly_dis2
	or temp2, temp1
	cpi temp2, 0
	breq next_dis1
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
next_dis1:
	lds temp1, fly_dis1
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
	do_lcd_data 'm'
	do_lcd_data ' '
	do_lcd_data ' '
	do_lcd_data ' '
	do_lcd_data ' '
	do_lcd_data ' '

	lcd_location 2, 1
	do_lcd_data 'D'
	do_lcd_data 'u'
	do_lcd_data 'r'
	do_lcd_data 'a'
	do_lcd_data 't'
	do_lcd_data 'i'
	do_lcd_data 'o'
	do_lcd_data 'n'
	do_lcd_data ':'
	ldi temp2, 0						;compare 0 every unit for hide no meaning 0
	lds temp1, fly_time6
	or temp2, temp1
	cpi temp2, 0
	breq next_time5
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
next_time5:
	lds temp1, fly_time5
	or temp2, temp1
	cpi temp2, 0
	breq next_time4
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
next_time4:
	lds temp1, fly_time4
	or temp2, temp1
	cpi temp2, 0
	breq next_time3
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
next_time3:
	lds temp1, fly_time3
	or temp2, temp1
	cpi temp2, 0
	breq next_time2
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
next_time2:
	lds temp1, fly_time2
	or temp2, temp1
	cpi temp2, 0
	breq next_time1
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
next_time1:
	lds temp1, fly_time1
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
	do_lcd_data 's'
	do_lcd_data ' '
	do_lcd_data ' '
	do_lcd_data ' '
	do_lcd_data ' '
	do_lcd_data ' '

end_results_display:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;data_count;;;;;;;;;;;
data_count:
	rcall motor_simulate				;change motor speed
time_count:								;time count
	lds temp1, fly_time1				;count time every unit up to 10
	inc temp1							;clear and next unit increase 1
	sts fly_time1, temp1
	cpi temp1, 10
	brlo end_time_count 
	
	clr temp1							;2nd
	sts fly_time1, temp1
	lds temp1, fly_time2
	inc temp1
	sts fly_time2, temp1
	cpi temp1, 10
	brlo end_time_count
	
	clr temp1							;3rd
	sts fly_time2, temp1
	lds temp1, fly_time3
	inc temp1
	sts fly_time3, temp1
	cpi temp1, 10
	brlo end_time_count
	
	clr temp1							;4th
	sts fly_time3, temp1
	lds temp1, fly_time4
	inc temp1
	sts fly_time4, temp1
	cpi temp1, 10
	brlo end_time_count
	
	clr temp1							;5th
	sts fly_time4, temp1
	lds temp1, fly_time5
	inc temp1
	sts fly_time5, temp1
	cpi temp1, 10
	brlo end_time_count
	
	clr temp1							;6th
	sts fly_time5, temp1
	lds temp1, fly_time6
	inc temp1
	sts fly_time6, temp1
	cpi temp1, 10
	brlo end_time_count
	
	clr temp1							;overflow clear
	sts fly_time1, temp1
	sts fly_time2, temp1
	sts fly_time3, temp1
	sts fly_time4, temp1
	sts fly_time5, temp1
	sts fly_time6, temp1
end_time_count:

	lds temp1, fly_dis1					;count distance every unit up to 10
	lds temp2, heli_speed				;clear and next unit increase 1
	add temp1, temp2
	sts fly_dis1, temp1
	cpi temp1, 10
	brlo end_dis_count 
	
	subi temp1, 10						;2nd
	sts fly_dis1, temp1
	lds temp1, fly_dis2
	inc temp1
	sts fly_dis2, temp1
	cpi temp1, 10
	brlo end_dis_count
	
	subi temp1, 10						;3rd
	sts fly_dis2, temp1
	lds temp1, fly_dis3
	inc temp1
	sts fly_dis3, temp1
	cpi temp1, 10
	brlo end_dis_count
	
	subi temp1, 10						;4th
	sts fly_dis3, temp1
	lds temp1, fly_dis4
	inc temp1
	sts fly_dis4, temp1
	cpi temp1, 10
	brlo end_dis_count
	
	subi temp1, 10						;5th
	sts fly_dis4, temp1
	lds temp1, fly_dis5
	inc temp1
	sts fly_dis5, temp1
	cpi temp1, 10
	brlo end_dis_count
	
	subi temp1, 10						;6th
	sts fly_dis5, temp1
	lds temp1, fly_dis6
	inc temp1
	sts fly_dis6, temp1
	cpi temp1, 10
	brlo end_dis_count
	
	clr temp1							;overflow clear
	sts fly_dis1, temp1
	sts fly_dis2, temp1
	sts fly_dis3, temp1
	sts fly_dis4, temp1
	sts fly_dis5, temp1
	sts fly_dis6, temp1
end_dis_count:
	
end_data_count:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;state_dispaly;;;;;;;;
state_dispaly:
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;line1
	lcd_location 1, 1
	do_lcd_data ' '
	do_lcd_data ' '
	do_lcd_data ' '
	do_lcd_data 'P'
	do_lcd_data 'o'
	do_lcd_data 's'
	do_lcd_data ' '
	do_lcd_data ' '
	do_lcd_data ' '
	do_lcd_data 'D'
	do_lcd_data 'i'
	do_lcd_data 'r'
	do_lcd_data ' '
	do_lcd_data 'S'
	do_lcd_data 'p'
	do_lcd_data ' '
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;line2
	lcd_location 2, 1
	do_lcd_data '('
	count_2u_num position_x
	do_lcd_register temp2
	do_lcd_register temp1
	do_lcd_data ','
	count_2u_num position_y
	do_lcd_register temp2
	do_lcd_register temp1
	do_lcd_data ','
	lds temp1, position_z
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
	do_lcd_data ')'
	do_lcd_data ' '
	lds temp1, heli_state
	do_lcd_register temp1
	do_lcd_data ' '
	lds temp1, heli_speed
	subi temp1, -'0'					;get ASCII code
	do_lcd_register temp1
	do_lcd_data 'm'
	do_lcd_data '/'
	do_lcd_data 's'
end_state_display:
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
JUMP_end_keypad_input:
	rjmp end_keypad_input
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;keypad_scan;;;;;;;;
keypad_scan:																				;;;;;/\
		ldi key_ins, INITCOLMASK															;;;;/||\
		sts cmask, key_ins			; initial column mask									;;;/ || \
		clr	key_ins					; initial column										;;/  ||  \
		sts col, key_ins																	;;;;;||
	colloop:																				;;;;;||
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
		lds temp2, start_fly
		cpi temp2, 4					;4 crash jump out
		breq JUMP_end_keypad_input
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
		lds key_ins, col																	;;;;;||				;;;;;||
		cpi key_ins, 4																		;;;;;||
		breq keypad_scan																	;;;;;||
		lds key_ins, cmask																	;;;;;||
		sts	PORTL, key_ins			; set column to mask value (one column off)				;;;;;||
		ldi temp1, 0xFF																		;;;;;||
	delay:																					;;;;;||
		dec temp1																			;;;;;||
		brne delay																			;;;;;||
																							;;;;;||
		lds	temp1, PINL				; read PORTL											;;;;;||
		andi temp1, ROWMASK																	;;;;;||
		cpi temp1, 0xF				; check if any rows are on								;;;;;||
		breq nextcol																		;;;;;||
							; if yes, find which row is on									;;;;;||
		ldi key_ins, INITROWMASK															;;;;;||
		sts rmask, key_ins			; initialise row check									;;;;;||
		clr	key_ins					; initial row											;;;;;||
		sts row, key_ins																	;;;;;||
	rowloop:																				;;;;;||
		lds key_ins, row																	;;;;;||
		cpi key_ins, 4																		;;;;;||
		breq nextcol																		;;;;;||
		mov temp2, temp1																	;;;;;||
		lds key_ins, rmask																	;;;;;||
		and temp2, key_ins				; check masked bit									;;;;;||
		breq keypad_input 				; if bit is clear, convert the bitcode				;;;;;||
		lds key_ins, row																	;;;;;||
		inc key_ins						; else move to the next row							;;;;;||
		sts row, key_ins																	;;;;;||
		lds key_ins, rmask																	;;;;;||
		lsl key_ins					; shift the mask to the next bit						;;;;;||
		sts rmask, key_ins																	;;;;;||
		jmp rowloop																			;;;;;||
	nextcol:																				;;;;;||
		lds key_ins, cmask																	;;;;;||
		lsl key_ins					; else get new mask by shifting and 					;;;;;||
		inc key_ins					;!!!@#@@@@												;;;;;||
		sts cmask, key_ins																	;;;;;||
		lds key_ins, col																	;;;;;||
		inc key_ins						; increment column value							;;;;;||				
		sts col, key_ins																	;;;;;||
		jmp colloop					; and check the next column								;;;;;||
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;end Sacn keypad;;;;;;;;;;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;convert & max find ascii;;;;;;;;;;;	
	keypad_input:																			;;;;;/\
		lds key_ins, col																	;;;;/||\
		cpi key_ins, 3					; if column is 3 we have a letter					;;;/ || \
		breq letters																		;;/	 ||	 \
		lds key_ins, row																	;;;;;||
		cpi key_ins, 3					; if row is 3 we have a symbol or 0					;;;;;||
		breq symbols																		;;;;;||
																							;;;;;||
		lds temp1, row				; otherwise we have a number in 1-9						;;;;;||
		lsl temp1																			;;;;;||
		lds key_ins, row																	;;;;;||
		add temp1, key_ins			; temp1 = row * 3										;;;;;||
		lds key_ins, col																	;;;;;||
		add temp1, key_ins				; add the column address to get the value			;;;;;||
		subi temp1, -'1'			; add the value of character '0'						;;;;;||
		jmp check_release																	;;;;;||
																							;;;;;||
	letters:																				;;;;;||
		ldi temp1, 'a'																		;;;;;||
		lds key_ins, row																	;;;;;||
		add temp1, key_ins				; increment the character 'A' by the row value		;;;;;||
		jmp check_release																	;;;;;||
																							;;;;;||
	symbols:																				;;;;;||
		lds key_ins, col																	;;;;;||
		cpi key_ins, 0					; check if we have a star							;;;;;||
		breq star																			;;;;;||
		cpi key_ins, 1					; or if we have zero								;;;;;||
		breq zero																			;;;;;||
		ldi temp1, '#'				; if not we have hash									;;;;;||
		jmp check_release																	;;;;;||
	star:																					;;;;;||
		ldi temp1, '*'				; set to star											;;;;;||
		jmp check_release																	;;;;;||
	zero:																					;;;;;||
		ldi temp1, '0'				; set to zero											;;;;;||
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;every time only one key;;;;;;;;;||
check_release:																				;;;;;||
	lds temp2, PINL																			;;;;;||
	andi temp2, 0x0F																		;;;;;||
	cpi temp2, 0x0F																		    ;;;;;||
	breq released																			;;;;;||										
	rjmp check_release																		;;;;;||
released:																					;;;;;||	
end_keypad_input:																			;;;;;||
	ret																						;;;;;||
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;end convert ascii save in temp1;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;LCD Command;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;lcd_set										;;;;;/\
.macro lcd_set																			;;;;/||\
	sbi LCD_CTRL_PORT, @0																;;;/ || \
.endmacro																				;;/  ||  \
																						;;;;;||
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;lcd_clr										;;;;;||
.macro lcd_clr																			;;;;;||
	cbi LCD_CTRL_PORT, @0																;;;;;||
.endmacro																				;;;;;||
																						;;;;;||
;																						;;;;;||
; Send a command to the LCD (r16)														;;;;;||
;																						;;;;;||
																						;;;;;||
lcd_command:																			;;;;;||
	STORE LCD_DATA_PORT, r16															;;;;;||
	rcall sleep_1ms																		;;;;;||
	lcd_set LCD_E																		;;;;;||
	rcall sleep_1ms																		;;;;;||
	lcd_clr LCD_E																		;;;;;||
	rcall sleep_1ms																		;;;;;||
	ret																					;;;;;||
																						;;;;;||
lcd_data:																				;;;;;||
	STORE LCD_DATA_PORT, r16															;;;;;||
	lcd_set LCD_RS																		;;;;;||
	rcall sleep_1ms																		;;;;;||
	lcd_set LCD_E																		;;;;;||
	rcall sleep_1ms																		;;;;;||
	lcd_clr LCD_E																		;;;;;||
	rcall sleep_1ms																		;;;;;||
	lcd_clr LCD_RS																		;;;;;||
	ret																					;;;;;||
																						;;;;;||
lcd_wait:																				;;;;;||
	push r16																			;;;;;||
	clr r16																				;;;;;||
	STORE LCD_DATA_DDR, r16																;;;;;||
	STORE LCD_DATA_PORT, r16															;;;;;||
	lcd_set LCD_RW																		;;;;;||
lcd_wait_loop:																			;;;;;||
	rcall sleep_1ms																		;;;;;||
	lcd_set LCD_E																		;;;;;||
	rcall sleep_1ms																		;;;;;||
	LOAD r16, LCD_DATA_PIN																;;;;;||
	lcd_clr LCD_E																		;;;;;||
	sbrc r16, 7																			;;;;;||
	rjmp lcd_wait_loop																	;;;;;||
	lcd_clr LCD_RW																		;;;;;||
	ser r16																				;;;;;||
	STORE LCD_DATA_DDR, r16																;;;;;||
	pop r16																				;;;;;||
	ret																					;;;;;||
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Sleep function 1ms ~ 1000ms;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.equ F_CPU = 16000000																	;;;;;/\
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4													;;;;/||\
; 4 cycles per iteration - setup/call-return overhead									;;;/ || \
sleep_1ms:																				;;/  ||  \
	push r26																			;;;;;||
	push r27																			;;;;;||
	ldi r27, high(DELAY_1MS)															;;;;;||
	ldi r26, low(DELAY_1MS)																;;;;;||
																						;;;;;||
delayloop_1ms:																			;;;;;||
	sbiw r27:r26, 1																		;;;;;||
	brne delayloop_1ms																	;;;;;||
	pop r27			 																	;;;;;||
	pop r26																				;;;;;||
	ret																					;;;;;||
																						;;;;;||
sleep_5ms:																				;;;;;||
	rcall sleep_1ms																		;;;;;||
	rcall sleep_1ms																		;;;;;||
	rcall sleep_1ms																		;;;;;||
	rcall sleep_1ms																		;;;;;||
	rcall sleep_1ms																		;;;;;||
	ret																					;;;;;||
																						;;;;;||
sleep_20ms:																				;;;;;||
	rcall sleep_5ms																		;;;;;||
	rcall sleep_5ms																		;;;;;||
	rcall sleep_5ms																		;;;;;||
	rcall sleep_5ms																		;;;;;||
	ret																					;;;;;||
																						;;;;;||
sleep_50ms:																				;;;;;||
	rcall sleep_20ms																	;;;;;||
	rcall sleep_20ms																	;;;;;||
	rcall sleep_5ms																		;;;;;||
	rcall sleep_5ms																		;;;;;||
	ret																					;;;;;||
																						;;;;;||
sleep_100ms:																			;;;;;||
	rcall sleep_20ms																	;;;;;||
	rcall sleep_20ms																	;;;;;||
	rcall sleep_20ms																	;;;;;||
	rcall sleep_20ms																	;;;;;||
	rcall sleep_20ms																	;;;;;||
	ret																					;;;;;||
																						;;;;;||
sleep_500ms:																			;;;;;||
	rcall sleep_100ms																	;;;;;||
	rcall sleep_100ms																	;;;;;||
	rcall sleep_100ms																	;;;;;||
	rcall sleep_100ms																	;;;;;||
	rcall sleep_100ms																	;;;;;||
	ret																					;;;;;||
																						;;;;;||
sleep_1000ms:																			;;;;;||
	rcall sleep_500ms																	;;;;;||
	rcall sleep_500ms																	;;;;;||
	ret																					;;;;;||
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;end Sleep function;;;;;;;;;;;;;
