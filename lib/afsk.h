#ifndef AFSK_H
#define AFSK_H
#include "aprs.h"
#include "DRA818v.h"
#include "Arduino.h"
#include "aprs_global.h"
#include <stdint.h>
//Frequency Constants
static const int MARK_FREQ = 1200;
static const int SPACE_FREQ = 2200;
static const float PREEMPHASIS_RATIO = 0.75;

//Time Constants
static const int BIT_RATE = 1200; //APRS standard.
static const uint32_t SAMPLE_RATE = 96000;
static const uint32_t SAMPLES_PER_BIT = SAMPLE_RATE / BIT_RATE;
static const int DEBUG_PRESCALER = 1;//Set to 1 for full speed, higher to slow down interrupts by that factor

//Phase Delta Constants
static const int SINE_TABLE_LENGTH = 512;
static const int SINE_WAVE_MAX = pow(2,SINE_WAVE_RESOLUTION + 1);

static const int ANGLE_RESOLUTION_PRESCALER = 1000; 
static const uint32_t MARK_INCREMENT = ANGLE_RESOLUTION_PRESCALER * SINE_TABLE_LENGTH * MARK_FREQ / SAMPLE_RATE;
static const uint32_t SPACE_INCREMENT = ANGLE_RESOLUTION_PRESCALER * SINE_TABLE_LENGTH * SPACE_FREQ / SAMPLE_RATE;
// Exported functions

void afsk_modulate_packet(volatile uint8_t* buffer, int size);
void afsk_timer_begin();
void afsk_timer_stop();
void resetVolatiles();

#if SINE_WAVE_RESOLUTION == 12
static const uint16_t sineTable[128] = {
2047, 2072, 2097, 2122, 2147, 2173, 2198, 2223, 2248, 2273, 2298, 2323, 2348, 2372, 2397, 2422,
2447, 2471, 2496, 2520, 2545, 2569, 2593, 2617, 2642, 2666, 2689, 2713, 2737, 2761, 2784, 2807, 
2831, 2854, 2877, 2900, 2923, 2945, 2968, 2990, 3012, 3035, 3056, 3078, 3100, 3121, 3143, 3164,
3185, 3206, 3226, 3247, 3267, 3287, 3307, 3327, 3346, 3366, 3385, 3404, 3422, 3441, 3459, 3477,
3495, 3513, 3530, 3547, 3564, 3581, 3598, 3614, 3630, 3646, 3662, 3677, 3692, 3707, 3721, 3736,
3750, 3764, 3777, 3791, 3804, 3816, 3829, 3841, 3853, 3865, 3876, 3887, 3898, 3909, 3919, 3929,
3939, 3949, 3958, 3967, 3975, 3984, 3992, 3999, 4007, 4014, 4021, 4027, 4034, 4040, 4045, 4051,
4056, 4060, 4065, 4069, 4073, 4076, 4080, 4083, 4085, 4087, 4089, 4091, 4093, 4094, 4094, 4095,
};
#else
#define SINE_WAVE_RESOLUTION 8
static const uint8_t sineTable[128] = 
{
    128, 129, 131, 132, 134, 135, 137, 138, 140, 142, 143, 145, 146, 148, 149, 151,
    152, 154, 155, 157, 158, 160, 162, 163, 165, 166, 167, 169, 170, 172, 173, 175,
    176, 178, 179, 181, 182, 183, 185, 186, 188, 189, 190, 192, 193, 194, 196, 197,
    198, 200, 201, 202, 203, 205, 206, 207, 208, 210, 211, 212, 213, 214, 215, 217,
    218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233,
    234, 234, 235, 236, 237, 238, 238, 239, 240, 241, 241, 242, 243, 243, 244, 245,
    245, 246, 246, 247, 248, 248, 249, 249, 250, 250, 250, 251, 251, 252, 252, 252,
    253, 253, 253, 253, 254, 254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255,
};
#endif
#endif // AFSK_H
