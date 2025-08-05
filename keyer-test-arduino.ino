// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025 SASANO Takayoshi <uaa@uaa.org.uk>

#include "keyer-test-arduino.h"

#define OUT_0 4 // D4 -> PD4 
#define OUT_1 5 // D5 -> PD5 
#undef  OUT_2
#undef  OUT_3
#define OUT_PORT PORTD
#define OUT_DDR DDRD

#define IN_0 1 // D15 -> PC1
#define IN_1 0 // D14 -> PC0
#undef  IN_2
#undef  IN_3
#define IN_PORT PORTC
#define IN_PIN PINC
#define IN_DDR DDRC

static struct event event[MAX_ENTRY];
static struct event result[MAX_ENTRY];
static unsigned char event_entry, event_index;
static unsigned char result_index;
static unsigned short max_pos = MAX_POS;

static void timer_init(void)
{
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;
	TIMSK1 = 0;
	TCCR1B = 0x05;	// F_CLK / 1024 (1tick = 64us @ 16MHz)
}

inline unsigned short timer_get(void)
{
	return TCNT1;
}

inline void timer_set(unsigned short t)
{
	TCNT1 = t;
}

static void event_init(void)
{
	memset(event, 0, sizeof(event));
	event_entry = event_index = 0;
}

static void result_init(void)
{
	memset(result, 0, sizeof(result));
	result_index = 0;
}

static void gpio_init(void)
{
	unsigned char d;

	/* set output pins */
	d = OUT_DDR;
#ifdef OUT_0
	d |= (1 << OUT_0);
#endif
#ifdef OUT_1
	d |= (1 << OUT_1);
#endif
#ifdef OUT_2
	d |= (1 << OUT_2);
#endif
#ifdef OUT_3
	d |= (1 << OUT_3);
#endif
	OUT_DDR = d;

	/* set input pins */
	d = IN_DDR;
#ifdef IN_0
	d &= ~(1 << IN_0);
#endif
#ifdef IN_1
	d &= ~(1 << IN_1);
#endif
#ifdef IN_2
	d &= ~(1 << IN_2);
#endif
#ifdef IN_3
	d &= ~(1 << IN_3);
#endif
	IN_DDR = d;

	/* enable pull-up for input */
	MCUCR &= 0xef; // clear PUD (-> pull-up enable)
	d = IN_PORT;
#ifdef IN_0
	d |= (1 << IN_0);
#endif
#ifdef IN_1
	d |= (1 << IN_1);
#endif
#ifdef IN_2
	d |= (1 << IN_2);
#endif
#ifdef IN_3
	d |= (1 << IN_3);
#endif
	IN_PORT = d;
}

static void gpio_out(unsigned char v)
{
	unsigned char d = 0;

#ifdef OUT_0
	if (v & 0x01) d |= (1 << OUT_0);
#endif
#ifdef OUT_1
	if (v & 0x02) d |= (1 << OUT_1);
#endif
#ifdef OUT_2
	if (v & 0x04) d |= (1 << OUT_2);
#endif
#ifdef OUT_3
	if (v & 0x08) d |= (1 << OUT_3);
#endif
	OUT_PORT = d;
}

static unsigned char gpio_in(void)
{
	unsigned char d = IN_PIN, v = 0;

#ifdef IN_0
	if (!(d & (1 << IN_0))) v |= 0x01;
#endif
#ifdef IN_1
	if (!(d & (1 << IN_1))) v |= 0x02;
#endif
#ifdef IN_2
	if (!(d & (1 << IN_2))) v |= 0x04;
#endif
#ifdef IN_3
	if (!(d & (1 << IN_3))) v |= 0x08;
#endif
	return v;
}

static int do_log(void)
{
	unsigned short curr_timer, prev_timer = 0xffff;
	unsigned char curr_status, prev_status = 0xf0;

	noInterrupts();

	event_index = 0;
	result_index = 0;
	timer_set(0xffff);

	while (1) {
		/* wait for next timing */
		while ((curr_timer = timer_get()) == prev_timer);
		prev_timer = curr_timer;

		/* timer overflow */
		if (curr_timer >= max_pos)
			goto fin0;

		/* do output event */
		if (event_index < event_entry &&
		    event[event_index].evt == EVT_SET &&
		    event[event_index].pos == curr_timer) {
			gpio_out(event[event_index].val);
			event_index++;
		}

		/* capture input event */
		if ((curr_status = gpio_in()) != prev_status &&
		    result_index < MAX_ENTRY) {
			prev_status = curr_status;
			result[result_index].pos = curr_timer;
			result[result_index].val = curr_status;
			result_index++;
		}

		/* wait signal change and restart logging */
		if (event_index < event_entry &&
		    event[event_index].evt == EVT_CHGSTS &&
		    event[event_index].val) {

			/* result[0]: status before signal change */
			result_index = 0;
			result[result_index].pos =
				(prev_timer = event[event_index].pos);
			result[result_index].val = curr_status;
			if (result_index < MAX_ENTRY) result_index++;

			/* wait for event change */
			timer_set(0x0000);
			while (1) {
				/* timeout */
				if (timer_get() >= max_pos)
				goto fin0;

				/* event change */
				if ((gpio_in() ^ curr_status) &
					event[event_index].val)
				break;
			}

			/* renew timer at new value (start log immediately) */
			timer_set(prev_timer + 1);
			event_index++;
		}
	}

fin0:
	interrupts();

	return 0;
}

static int send_result(void)
{
	Serial.write(result_index - 1);
	for (int i = 0; i < MAX_ENTRY; i++) {
		Serial.write((unsigned char *)&result[i], sizeof(struct event));
		Serial.flush();
	}

	return 0;
}

static int recv_event(void)
{
	while (Serial.available() < 1);
	if ((event_entry = Serial.read() + 1) > MAX_ENTRY)
		return -1;

	for (int i = 0; i < event_entry; i++) {
		while (Serial.available() < sizeof(struct event));
		if (Serial.readBytes((unsigned char *)&event[i],
		    sizeof(struct event)) < sizeof(struct event)) return -1;
	}

	return 0;
}

static int set_maxpos(void)
{
	byte c;
	unsigned short n;

	while (Serial.available() < 1);
	if ((n = Serial.read() + 1) <= (MAX_POS >> 8)) {
		max_pos = n << 8;
		return 0;
	}
		
	return -1;
}

void setup(void)
{
	Serial.begin(115200);
	event_init();
	result_init();
	timer_init();
	gpio_init();
}

void loop(void)
{
	unsigned char c;
	int r;

	while (1) {
		while (Serial.available() < 1);
		c = Serial.read();

		switch (c) {
		case CMD_READY:
			r = 0;
			break;
		case CMD_RESET:
			event_init();
			result_init();
			max_pos = MAX_POS;
			r = 0;
			break;
		case CMD_EVENT:
			r = recv_event();
			break;
		case CMD_LOG:
			r = do_log();
			break;
		case CMD_RESULT:
			r = send_result();
			break;
		case CMD_MAXPOS:
			r = set_maxpos();
			break;			
		default:
			r = -1;
			break;
		}

		Serial.write((r < 0) ? RESP_NAK : RESP_ACK);
		Serial.flush();
	}
}
