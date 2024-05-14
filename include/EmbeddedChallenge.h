// Macro to calculate error bounds based on a given percentage
#define ERROR_BOUND(bound, x) bound * ((float) x / 100)

// Macro to test if the bits denoted by a bit mask, y, are set in a value, x
#define TEST_BIT_MASK(x, y) (x & y) == y

// Constants for array-related sizes
#define DATA_SIZE 64
#define AVG_SIZE 10

// Constants for FFT bin sum tremor tresholds found via empirical testing but
// can be adjusted here
#define MIN_TREMOR_THRESHOLD 20
#define MED_TREMOR_THRESHOLD 50
#define HIGH_TREMOR_THRESHOLD 100

// Constant for timing threshold
#define MINUTE_THRESHOLD 10

// Constant for gravitational acceleration, taken from
// https://en.wikipedia.org/wiki/Gravitational_acceleration
#define GRAVITY 9.80665

// Note that the following code snippet was borrowed from the Adafruit
// Zero FFT library written by Dean Miller for Adafruit Industries
// under the BSD license.

#define FFT_BIN(num, fs, size)                                                 \
  (num *                                                                       \
   ((float)fs / (float)size)) ///< return the center frequency of FFT bin 'num'
                              ///< based on the sample rate and FFT stize

// Note that the following code snippet was borrowed from the Adafruit Circuit 
// Playground microphone library by Phil Burgess / Paint Your Dragon.
// Fast Fourier transform section is derived from
// ELM-ChaN FFT library (see comments in ffft.S).

/**************************************************************************/
/*! 
    @brief  16 bit complex data type
*/
/**************************************************************************/
typedef struct {
  int16_t r; ///< real portion
  int16_t i; ///< imaginary portion
} complex_t;

extern "C" { // In ffft.S
  void fft_input(const int16_t *, complex_t *),
       fft_execute(complex_t *),
       fft_output(complex_t *, uint16_t *);
} 
