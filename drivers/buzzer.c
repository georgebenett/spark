#include <string.h>

#include <nrf_gpio.h>
#include <nrf_pwm.h>

#include "drivers/buzzer.h"
#include "drivers/pwm.h"
#include "eventpump.h"
#include "util.h"

#define PWM_PRESCALER		NRF_PWM_CLK_4MHz
#define PWM_FREQUENCY		(16000000 >> (PWM_PRESCALER))
#define FREQ_TO_TICKS(f)	(PWM_FREQUENCY / (f))
#define MS_TO_PERIODS(f, t)	(((f) * (t)) / 1000)
#define PAUSE_FREQUENCY		1000
#define MAX_RAM_NOTES		128

#define NOTE(n, o, d)	{.freq_hz = (NOTE_##n) >> (9-(o)), .len_ms = (d)}
#define PAUSE(d)		{.freq_hz = 0, .len_ms = (d)}
#define TUNE(x)			{.p_notes = (struct note *)(x), .len = ARRAY_SIZE(x)}

#define NOTE_C	8372
#define NOTE_Db	8870
#define NOTE_D	9397
#define NOTE_Eb	9956
#define NOTE_E	10548
#define NOTE_F	11175
#define NOTE_Gb	11840
#define NOTE_G	12544
#define NOTE_Ab	13290
#define NOTE_A	14080
#define NOTE_Bb	14917
#define NOTE_B	15804

/* SEQEND callback called at start of last seq step so add a dummy value */
enum pwm_idx {
	PWM_IDX_OUTPUT_VAL = 0,
	PWM_IDX_DUMMY_VAL = 1,
	NBR_OF_PWM_IDXS
};

struct note {
	uint16_t freq_hz;
	uint16_t len_ms;
};

struct tune {
	struct note *p_notes;
	uint8_t len;
};

static const struct note classic_jingle[] = {
	NOTE(E, 9, 38), NOTE(Db, 9, 45), NOTE(Bb, 8, 55), NOTE(A, 8, 58),
	NOTE(G, 8, 63), NOTE(Gb, 8, 68), NOTE(E, 8, 75), NOTE(D, 8, 85),
	NOTE(C, 8, 95), NOTE(B, 7, 105), NOTE(A, 7, 113)
};

static const struct note dms_on_sound[] = {
	NOTE(E, 8, 75), NOTE(A, 7, 113), NOTE(A, 7, 113)
};

static const struct note dms_off_sound[] = {
	NOTE(A, 7, 75), NOTE(A, 7, 113), NOTE(E, 8, 75)
};

static const struct note esc_precharged[] = {
	NOTE(A, 8, 25), NOTE(A, 7, 95), PAUSE(40), NOTE(A, 8, 25),
	NOTE(A, 7, 175)
};

static const struct note gps_fix[] = {
	NOTE(A, 5, 110), NOTE(B, 6, 110), NOTE(C, 7, 110),
	NOTE(G, 7, 100), PAUSE(100), NOTE(C, 8, 25),
	NOTE(C, 7, 195)
};

static struct {
	const struct buzzer_board	*p_board;
	NRF_PWM_Type				*p_pwm;
	struct note					ram_notes[MAX_RAM_NOTES];

	/* PWM instance settings */
	nrf_pwm_values_common_t		pwm_val[NBR_OF_PWM_IDXS];
	nrf_pwm_sequence_t			pwm_seq;

	/* currently playing tune */
	const struct tune			*p_tune;
	uint8_t						tune_pos;
} me;

/* mapping from buzzer_seq to tune */
static struct tune tunes[NBR_OF_BUZZ_SEQ] = {
	[BUZZ_SEQ_START]			= TUNE(classic_jingle),
	[BUZZ_SEQ_DMS_ON]			= TUNE(dms_on_sound),
	[BUZZ_SEQ_DMS_OFF]			= TUNE(dms_off_sound),
	[BUZZ_SEQ_ESC_PRECHARGED]	= TUNE(esc_precharged),
	[BUZZ_SEQ_GPS_FIX]			= TUNE(gps_fix),
	[BUZZ_SEQ_RAM_TUNE]			= { me.ram_notes, 0 }
};

static err_code load_note(const struct note *p_note)
{
	if (!p_note)
		return EBUZZ_INVALID_ARG;

	if (p_note->freq_hz == 0) {
		/* pause */
		pwm_channels_config(me.p_pwm, PWM_PRESCALER,
			FREQ_TO_TICKS(PAUSE_FREQUENCY), PWM_DECODER_LOAD_Common);
		me.pwm_val[PWM_IDX_OUTPUT_VAL] = FREQ_TO_TICKS(PAUSE_FREQUENCY);
		me.pwm_seq.repeats = MS_TO_PERIODS(PAUSE_FREQUENCY, p_note->len_ms);
	} else {
		/* note */
		pwm_channels_config(me.p_pwm, PWM_PRESCALER,
			FREQ_TO_TICKS(p_note->freq_hz), PWM_DECODER_LOAD_Common);
		me.pwm_val[PWM_IDX_OUTPUT_VAL] = (FREQ_TO_TICKS(p_note->freq_hz) / 2);
		me.pwm_seq.repeats = MS_TO_PERIODS(p_note->freq_hz, p_note->len_ms);
	}

	return ERROR_OK;
}

static err_code play_next_note(void)
{
	const struct note *p_next_note;
	err_code r;

	if (!me.p_pwm)
		return EBUZZ_INVALID_PWM;

	if (!me.p_tune || !me.p_tune->p_notes)
		return EBUZZ_INVALID_ARG;

	if (me.tune_pos >= me.p_tune->len)
		return ERROR_OK;

	p_next_note = &me.p_tune->p_notes[me.tune_pos];

	r = load_note(p_next_note);
	ERR_CHECK(r);
	r = pwm_channels_start_seq(me.p_pwm, &me.pwm_seq);
	ERR_CHECK(r);

	me.tune_pos++;
	return ERROR_OK;
}

static void buzz_seq_end_cb(void)
{
	if (ERROR_OK == pwm_stop(me.p_pwm))
		(void)play_next_note();
}

err_code buzzer_run(enum buzzer_seq buzz_seq)
{
	if (!me.p_board)
		return EBUZZ_NO_INIT;

	if (buzz_seq >= NBR_OF_BUZZ_SEQ)
		return EBUZZ_INVALID_ARG;

	me.p_tune = &tunes[buzz_seq];
	me.tune_pos = 0;
	return play_next_note();
}

static err_code event_handler(const struct eventpump_param * const p_event)
{
	if (me.p_board == NULL)
		return ERROR_OK;

	return buzzer_run(BUZZ_SEQ_START);
}
REGISTER_EVENT_HANDLER(EVENT_SYSINIT, event_handler);

err_code buzzer_upload(const uint32_t *p_data, uint32_t len)
{
	struct note note;
	uint32_t i;

	if (!p_data || (len > MAX_RAM_NOTES))
		return EBUZZ_INVALID_ARG;

	for (i = 0; i < len; i++) {
		note = ((struct note *)p_data)[i];
		tunes[BUZZ_SEQ_RAM_TUNE].p_notes[i] = note;
	}
	tunes[BUZZ_SEQ_RAM_TUNE].len = len;

	return ERROR_OK;
}

err_code buzzer_init(const struct buzzer_board * const p_board)
{
	err_code r;

	if (me.p_board)
		return ERROR_OK;

	if (!p_board)
		return EBUZZ_INVALID_ARG;

	r = util_validate_pins(&p_board->gpio, sizeof(p_board->gpio));
	ERR_CHECK(r);

	r = pwm_channels_reg(&me.p_pwm, &p_board->gpio, sizeof(p_board->gpio));
	ERR_CHECK(r);

	pwm_channels_config(me.p_pwm, PWM_PRESCALER, 0, PWM_DECODER_LOAD_Common);

	r = pwm_set_seq_end_handler(me.p_pwm, buzz_seq_end_cb);
	ERR_CHECK(r);

	me.pwm_val[PWM_IDX_DUMMY_VAL] = 0;

	me.pwm_seq.values.p_common = me.pwm_val;
	me.pwm_seq.length = NBR_OF_PWM_IDXS;
	me.pwm_seq.end_delay = 0;

	me.p_board = p_board;

	return ERROR_OK;
}
