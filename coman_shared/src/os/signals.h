#include <math.h>
#include <stdlib.h>

#include "utils.h"


/* y(t) = A sin (2 PI f t + teta) */
inline double sine_wave(double freq_Hz, double A, double teta)
{
    double wave;
    unsigned long long time_ns = get_time_ns();

    wave = A * sin((2.0 * M_PI * freq_Hz * time_ns)/1e9 + teta);

    return  wave;
}

/* y(t) = sgn ( A sin (2 PI f t + teta) ) */
inline double square_wave(double freq_Hz, double A, double teta)
{
    double wave;

    wave = (sine_wave(freq_Hz, A, teta) < 0 ? -A : A);

    return  wave;
}

/* y(t) = A 2 (t/a - floor(t/a + 1/2) */
inline double saw_wave(double freq_Hz, double A, double teta)
{
    double wave;
    unsigned long long time_ns = get_time_ns();

    wave = A * 2 * ((time_ns*freq_Hz/1e9) - floor((time_ns*freq_Hz/1e9) + 0.5));

    return  wave;
}

/* y(t) = A/PI_2 asin(sin(2 PI f t + teta)) */
inline double triangle_wave(double freq_Hz, double A, double teta)
{
    double wave;

    wave = A / M_PI_2 * asin(sine_wave(freq_Hz, 1, teta));

    return  wave;
}

#define SIN1HZ      sine_wave(1,3,0)
#define SIN10HZ     sine_wave(10,3,0)
#define SIN50HZ     sine_wave(50,3,0)
#define SIN100HZ    sine_wave(100,3,0)
#define SIN200HZ    sine_wave(200,3,0)
#define SIN500HZ    sine_wave(500,3,0)
#define SIN1MHZ     sine_wave(1e3,3,0)

#define SQR100HZ    square_wave(100,3,0)

#define SAW1HZ      saw_wave(1,3,0)
#define SAW50HZ     saw_wave(50,3,0)
#define SAW100HZ    saw_wave(100,3,0)
#define SAW200HZ    saw_wave(200,3,0)

#define TRG1HZ      triangle_wave(1,3,0)
#define TRG50HZ     triangle_wave(50,3,0)
#define TRG100HZ    triangle_wave(100,3,0)
#define TRG200HZ    triangle_wave(200,3,0)

