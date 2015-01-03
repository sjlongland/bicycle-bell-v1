/*
 *  Bicycle Bell/Horn firmware
 *  Copyright (C) 2015  Stuart Longland
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "bellsnd.h"
#include "hornsnd.h"

/*
 * Connections:
 * - Port B Pin 4:	External Source (active low)
 * - Port B Pin 5:	Green LED
 * - Port B Pin 6:	Blue LED
 * - Port B Pin 7:	Sound output (PWM)
 * - Port C Pin 7:	Red LED
 * - Port D Pin 6:	Power On (active low)
 * - Port D Pin 7:	Mode
 * - Port E Pin 6:	Bell (active low)
 *
 * Test points:
 * - Port C Pin 6:	PWM Next Sample
 * - Port D Pin 4:	Main loop
 * - Port D Pin 3:	PWM Buffer Wait
 * - Port D Pin 2:	PWM Buffer Switch
 * - Port D Pin 1:	Bell down
 * - Port D Pin 0:	Bell release
 * - Port F Pin 7:	Bell state (main loop)
 */

#define MODE_SW		(PIND & (1 << DDB7))
#define BELL_SW		(!(PINE & (1 << DDB6)))
#define EXT_SW		(!(PINB & (1 << DDB4)))
#define PWR_ON_PORT	PORTD
#define PWR_ON_PIN	(1 << DDB6)
#define RED_LED_PORT	PORTC
#define GREEN_LED_PORT	PORTB
#define BLUE_LED_PORT	PORTB
#define RED_LED_PIN	(1 << DDB7)
#define GREEN_LED_PIN	(1 << DDB5)
#define BLUE_LED_PIN	(1 << DDB6)

/*
 * Timer configuration.  Freetronics picked GPIO B7 as their output for the
 * sound, which means we're stuck with either using timers 0 (8-bit) or 1
 * (16-bit).  I could pick a different pin, but sod it, I've already wired up
 * the board so there's no going back now.
 *
 * Timers 0 and 1 are fed off the system clock, so we're stuck with a maximum
 * of 16MHz as the input frequency.  (If we had Timer4 available to us, then
 * we've got 64MHz.)  Timer 1 *can* do 16-bits, but at that resolution, it'll
 * give us a 244Hz carrier, which is utterly useless for audio.  So we'll
 * suffer with 8-bits, which gives us a 62.5kHz carrier.
 *
 * Timer 0 is probably our easiest choice for this.  Timer 1 can also do 8-bit
 * but why complicate things with 16-bit registers?  Our output will be OC0A.
 * We will feed the samples into OCR0A.
 */

#define T0_COMA	(2)	/* Output A: Clear on match */
#define T0_COMB (0)	/* Output B: Not used */
#define T0_WGM	(3)	/* Fast PWM mode */
#define T0_CS	(1)	/* 16MHz clock (no prescale) */
#define T0_FCA	(0)	/* Do not force A */
#define T0_FCB	(0)	/* Do not force B */
#define TCCR0A_VAL		\
	(	(T0_COMA << 6)	\
	|	(T0_COMB << 4)	\
	|	(T0_WGM & 0x3)	)
#define TCCR0B_VAL		\
	(	(T0_FCA << 7)	\
	|	(T0_FCB << 6)	\
	|	((T0_WGM & 0x04) << 1) \
	|	T0_CS)
/*
 * Timer 3 is well suited to give us a stable sample rate clock at sample
 * rates up to 32kHz.  Multiples of 8kHz should be smack on (according to the
 * crystal), and multiples of 11025Hz should be less than 1% off.  48kHz will
 * be off, but you'd be insane to expect DVD-quality audio out of the PWM
 * output of an 8-bit micro.
 *
 * We set OCR3A according to the equation:
 * 	OCR3A = (F_CPU / f_sample) - 1
 *
 * The interrupt vector name is TIMER3_COMPA.
 */
#define T3_COMA	(0)	/* Output A: Not used */
#define T3_COMB	(0)	/* Output B: Not used */
#define T3_COMC	(0)	/* Output C: Not used */
#define T3_WGM	(4)	/* Mode: CTC */
#define T3_CS	(1)	/* 16MHz clock (no prescale) */
#define T3_FCA	(0)	/* Do not force A */
#define T3_FCB	(0)	/* Do not force B */
#define T3_FCC	(0)	/* Do not force C */
#define T3_ICNC	(0)	/* Don't care */
#define T3_ICES	(0)	/* Don't care */
#define TCCR3A_VAL		\
	(	(T3_COMA << 6)	\
	|	(T3_COMB << 4)	\
	|	(T3_COMC << 2)	\
	|	(T3_WGM & 0x3)	)
#define TCCR3B_VAL			\
	(	(T3_ICNC << 7)		\
	|	(T3_ICES << 6)		\
	|	((T3_WGM & 0xc) << 1)	\
	|	T3_CS)
#define OCR3A_VAL(freq)	\
	((F_CPU / freq) - 1)
#define TIMSK3_VAL	(1 << 1)

/* 
 * The following is our output sample buffers, two buffers that get rotated
 * around (double-buffering) to ensure we're not writing to the one we're
 * reading from, buffer selector and the buffer pointer.
 */
#define BUFFER_SZ	256
#define BUFFER_NUM	2
static volatile uint8_t pwm_buffer[BUFFER_NUM][BUFFER_SZ];
static volatile uint8_t buffer_ready 	= 0;
static volatile uint8_t buffer_sel 	= 0;
static volatile uint8_t buffer_ptr 	= 0;
static volatile uint8_t buffer_wait	= 0;
static volatile uint8_t pwm_on		= 0;

/*
 * That leaves us timer 1 for a tick counter, which we can tune as needed.
 * Best rate would be about 10Hz to give us 100msec time slices that we can
 * give us a nice tick counter.  The primary aim of this will be to delay the
 * system power-down after the bell or external source signals are
 * de-asserted.
 *
 * We set OCR1A according to the equation:
 * 	OCR1A = (F_CPU / (1024*f_sample)) - 1
 *
 * This can be hard-coded since it will not be changing at runtime.  The
 * interrupt vector name is TIMER1_COMPA.
 */
#define T1_COMA	(0)	/* Output A: Not used */
#define T1_COMB	(0)	/* Output B: Not used */
#define T1_COMC	(0)	/* Output C: Not used */
#define T1_WGM	(4)	/* Mode: CTC */
#define T1_CS	(5)	/* 15.625kHz clock (16MHz / 1024) */
#define T1_FCA	(0)	/* Do not force A */
#define T1_FCB	(0)	/* Do not force B */
#define T1_FCC	(0)	/* Do not force C */
#define T1_ICNC	(0)	/* Don't care */
#define T1_ICES	(0)	/* Don't care */
#define T1_FREQ	(10)	/* 10Hz */
#define TCCR1A_VAL		\
	(	(T1_COMA << 6)	\
	|	(T1_COMB << 4)	\
	|	(T1_COMC << 2)	\
	|	(T1_WGM & 0x3)	)
#define TCCR1B_VAL			\
	(	(T1_ICNC << 7)		\
	|	(T1_ICES << 6)		\
	|	((T1_WGM & 0xc) << 1)	\
	|	T1_CS)
#define OCR1A_VAL	\
	((F_CPU / (1024*T1_FREQ)) - 1)
#define TIMSK1_VAL	(1 << 1)

/* This is our global tick counter variable */
static volatile uint8_t system_tick	= 0;

/* LED state information */
static volatile uint8_t led_colour	= 0;

/* Audio set-up routine, buffer 0 better be ready! */
void start_audio(uint16_t sample_rate) {
	/* Stop interrupts momentarily */
	if (pwm_on) {
		/* We already have PWM running??? */
		RED_LED_PORT |= RED_LED_PIN;
		GREEN_LED_PORT &= ~GREEN_LED_PIN;
		BLUE_LED_PORT &= ~BLUE_LED_PIN;
		while(1);
	}
	cli();
	/* Set up buffer pointers */
	buffer_sel	= 1;
	buffer_ptr	= BUFFER_SZ-1;
	/* Set up timer 0 */
	OCR0A 	= UINT8_MAX/2;
	TCCR0A	= TCCR0A_VAL;
	TCCR0B	= TCCR0B_VAL;
	/* Set up timer 3 */
	OCR3A	= OCR3A_VAL(sample_rate);
	TCCR3A	= TCCR3A_VAL;
	TCCR3B	= TCCR3B_VAL;
	TIMSK3	= TIMSK3_VAL;
	/* Resume interrupts */
	sei();
	/* Wait for audio interrupt to tick */
	//led_colour 	= 0xc;
	while(!pwm_on);
}

/* Audio tear-down routine */
void stop_audio() {
	/* Stop interrupts momentarily */
	cli();
	//led_colour 	= 0xc;
	/* Re-set buffer pointers and PWM state */
	buffer_sel	= 0;
	buffer_ptr	= 0;
	buffer_ready	= 0;
	buffer_wait	= 0;
	pwm_on		= 0;
	/* Silence output */
	OCR0A 	= UINT8_MAX/2;
	/* Stop timer 3 */
	TIMSK3	= 0;
	TCCR3A	= 0;
	TCCR3B	= 0;
	OCR3A	= 0;

	/* Stop timer 0 */
	TCCR0A	= 0;
	TCCR0B	= 0;
	/* Clear buffers */
	memset(pwm_buffer, 0, sizeof(pwm_buffer));
	/* Resume interrupts */
	sei();
}

/* Write audio to the output buffer */
uint16_t write_audio(const uint8_t* audio, uint16_t offset,
		uint16_t len, uint8_t is_ram, uint8_t loop) {
	/* Wait until the interrupt handler switches buffers */
	while(buffer_ready);

	/* Pick the buffer not being read */
	uint8_t buf_num = buffer_sel ? 0 : 1;
	uint16_t buf_rem = BUFFER_SZ;
	uint16_t buf_ptr = 0;
	uint16_t in_rem = len - offset;
	volatile uint8_t* out = pwm_buffer[buf_num];
	const uint8_t* in = &audio[offset];

	while(buf_rem && in_rem) {
		if (is_ram)
			*out = *in;
		else
			*out = pgm_read_byte(in);
		out++;
		buf_rem--;
		in++;
		in_rem--;
		if (loop && (!in_rem)) {
			in_rem = len;
			in = audio;
		}
	}
	/* Mark the buffer as ready */
	buffer_ready = 1;
	/* Return where we got to */
	return len - in_rem;
}

/* The loop point in the bell effect */
#define BELL_LOOP_SZ	(2054)

/* Bell states */
#define BELL_IDLE	0
#define BELL_DOWN	1
#define BELL_RELEASE	2
#define BELL_STOP	3
static volatile uint8_t 	bell_state	= BELL_IDLE;
static volatile uint8_t		bell_released	= 0;
static volatile uint16_t	bell_ptr	= 0;
static uint8_t			bell_mode	= 0;
static const uint8_t*		bell_snd	= NULL;
static uint16_t			bell_loop_sz	= 0;
static uint16_t			bell_sz		= 0;
static uint8_t			bell_loop	= 0;

/* What do we do when the bell is idle? */
void bell_idle(void) {
	if (BELL_SW) {
		/*
		 * Someone has pressed the bell button, enter the "down"
		 * state, load the initial buffer then start the audio.
		 */
		bell_state	= BELL_DOWN;
		bell_mode	= MODE_SW;
		if (bell_mode) {
			bell_snd	= horn;
			bell_loop_sz	= HORN_LOOP_SZ;
			bell_sz		= HORN_SZ - HORN_LOOP_OFFSET;
		} else {
			bell_snd	= bell;
			bell_loop_sz	= BELL_LOOP_SZ;
			bell_sz		= BELL_SZ;
		}
		bell_ptr 	= write_audio(
				bell_snd, 0, bell_loop_sz, 0, 1);
		start_audio(BELL_RATE);
	}
}

/* What do we do while the button is held? */
void bell_down(void) {
	if (BELL_SW) {
		/* The button is still down, is there room? */
		if (!buffer_ready) {
			/* There is, put some more dinging noises in */
			if (bell_mode && !bell_loop) {
				/* 
				 * We've played the initial part, now for
				 * the rest.
				 */
				bell_snd 	= &horn[HORN_LOOP_OFFSET];
				bell_loop_sz 	= HORN_LOOP_SZ - HORN_LOOP_OFFSET;
				bell_ptr	-= HORN_LOOP_OFFSET;
				bell_sz		-= HORN_LOOP_OFFSET;
				bell_loop	= 1;
			}
			bell_ptr = write_audio(
					bell_snd, bell_ptr, bell_loop_sz,
					0, 1);
		}
	} else {
		/* Button just released?  Or switch bounce */
		bell_state 	= BELL_RELEASE;
		bell_released	= system_tick;
	}
}

/* What do we do when the button is released? */
void bell_release(void) {
	if (BELL_SW) {
		/* The button is bouncing */
		bell_state 	= BELL_DOWN;
		bell_down();
	} else if ((system_tick - bell_released) > 2) {
		/* I'll call this released. */
		bell_state 	= BELL_STOP;
	} else if (!buffer_ready) {
		/* Keep making the dinging noises in the meantime */
		bell_ptr = write_audio(
				bell_snd, bell_ptr, bell_loop_sz,
				0, 1);
	}
}

/* Waiting for the final ding */
void bell_stop(void) {
	if (bell_ptr < bell_sz) {
		if (!buffer_ready) {
			/* One more ding since there's room */
			bell_ptr = write_audio(
					bell_snd, bell_ptr, bell_sz, 0, 0);
		}
	} else {
		/* We're done, wait for the buffer to finish */
		while(!buffer_wait);
		stop_audio();
		bell_state	= BELL_IDLE;
	}
}

/* Our main loop */
int main(void) {
	/* Our last-activity time */
	uint8_t last_act	= 0;

	/* Ensure interrupts are off */
	cli();

	/* Ensure audio is not running */
	pwm_on = 0;
	stop_audio();
	PORTB |= (1 << DDB0);

	/* Set up inputs */
	PORTE |= (1 << 6);
	PORTD |= (1 << 6);

	/* Set up outputs */
        DDRB |= (1 << DDB5)|(1 << DDB6)|(1 << DDB7)|(1 << DDB0);
	DDRC |= (1 << DDB7)|(1 << DDB6);
	DDRD |= (1 << DDB6)|(1 << DDB5)|
		(1 << DDB3)|(1 << DDB2)|(1 << DDB1)|(1 << DDB0);

	/* Hold the power on */
	PWR_ON_PORT |= PWR_ON_PIN;

	/* Set up Timer 1 */
	OCR1A 	= OCR1A_VAL;
	TIMSK1	= TIMSK1_VAL;
	TCCR1A 	= TCCR1A_VAL;
	TCCR1B 	= TCCR1B_VAL;

	/* Enable interrupts */
	sei();
	while(1) {
		if (bell_state == BELL_IDLE) {
			led_colour = 1;
			bell_idle();
		} else if (bell_state == BELL_DOWN) {
			led_colour = 3;
			bell_down();
			PORTD ^= (1 << DDB1);
		} else if (bell_state == BELL_RELEASE) {
			led_colour = 2;
			bell_release();
			PORTD ^= (1 << DDB0);
		} else {
			led_colour = 5;
			bell_stop();
		}

		/* Turn power off if self-powered */
		if (pwm_on)
			PWR_ON_PORT |= PWR_ON_PIN;
		else
			PWR_ON_PORT &= ~PWR_ON_PIN;
	}
}

ISR(TIMER1_COMPA_vect) {
	/*
	 * Executed every tick to time things like shutdown delay and
	 * to blink the LEDs.
	 */
	uint8_t c = led_colour;
	system_tick++;
	if ((c & 8) && (!(system_tick & 0x02)))
		c = 0;

	if (c & 1)
		RED_LED_PORT |= RED_LED_PIN;
	else
		RED_LED_PORT &= ~RED_LED_PIN;

	if (c & 2)
		GREEN_LED_PORT |= GREEN_LED_PIN;
	else
		GREEN_LED_PORT &= ~GREEN_LED_PIN;

	if (c & 4)
		BLUE_LED_PORT |= BLUE_LED_PIN;
	else
		BLUE_LED_PORT &= ~BLUE_LED_PIN;
}

ISR(TIMER3_COMPA_vect) {
	/* 
	 * Executed to pull data from the buffer and stuff it into
	 * the PWM output.  We begin by reading the sample at the current
	 * buffer location and writing that to PWM.
	 */
	OCR0A = pwm_buffer[buffer_sel][buffer_ptr];
	/* Is this the end of the buffer? */
	if (buffer_ptr < (BUFFER_SZ-1)) {
		/* No, move on */
		buffer_ptr++;
		//led_colour 	= 0x4;
		PORTC ^= (1 << DDB6);
	/* It is, is the other buffer ready? */
	} else if (buffer_ready) {
		/* Swap */
		buffer_sel	= buffer_sel ? 0 : 1;
		buffer_ptr	= 0;
		buffer_ready	= 0;
		buffer_wait	= 0;
		PORTD ^= (1 << DDB2);
	/* We're waiting on a buffer */
	} else if (!buffer_wait) {
		buffer_wait 	= 1;
		//led_colour 	= 0x9;
		PORTD ^= (1 << DDB3);
	}
	pwm_on = 1;
}
