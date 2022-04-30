#include <Arduino.h>
#include "FFT.h" // include the library

#define SAMPLES 256              // Must be a power of 2
#define SAMPLING_FREQUENCY 10000 // Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds; // current time since the Arduino board started

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6)
const int SIGNAL_PIN = 34;

double vReal[SAMPLES];
double vImag[SAMPLES];

float fft_input[SAMPLES];
float fft_output[SAMPLES];

char print_buf[300];

float max_magnitude = 0;
float fundamental_freq = 0;

void setup()
{
  Serial.begin(115200);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
}

void loop()
{
  /*SAMPLING*/
  long int t1 = micros();

  for (int i = 0; i < SAMPLES; i++)
  {
    microseconds = micros(); // Overflows after around 70 minutes
    vReal[i] = analogRead(SIGNAL_PIN);
    vImag[i] = 0;

    while (micros() < (microseconds + sampling_period_us))
    { // We take sampling_period_us microseconds to read the next sample
    }
  }
  long int t2 = micros();

  // Serial.println(vReal[0]);
  /*FFT*/

  fft_config_t *real_fft_plan = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

  for (int k = 0; k < SAMPLES; k++)
    real_fft_plan->input[k] = (float)vReal[k];

  // Execute transformation
  fft_execute(real_fft_plan);

  // Serial.println(t2 - t1);
  max_magnitude = 0;
  fundamental_freq = 0;
  // Print the output
  for (int k = 1; k < real_fft_plan->size / 2; k++)
  {
    /*The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output*/
    float mag = sqrt(pow(real_fft_plan->output[2 * k], 2) + pow(real_fft_plan->output[2 * k + 1], 2)) / 1;
    float freq = (k * 1.0 / ((t2 - t1))) * 1000000;
    // sprintf(print_buf, "%f Hz : %f", freq, mag);
    // Serial.println(print_buf);

    if (mag > max_magnitude)
    {
      max_magnitude = mag;
      fundamental_freq = freq;
    }
  }

  // /*Multiply the magnitude of the DC component with (1/FFT_N) to obtain the DC component*/
  // sprintf(print_buf, "DC component : %f g\n", (real_fft_plan->output[0]) / 10000 / SAMPLES); // DC is at [0]
  // Serial.println(print_buf);

  /*Multiply the magnitude at all other frequencies with (2/FFT_N) to obtain the amplitude at that frequency*/
  sprintf(print_buf, "%f", fundamental_freq);
  Serial.println(print_buf);
  // sprintf(print_buf, "Mag: %f g\n", (max_magnitude / 10000) * 2 / SAMPLES);
  // Serial.println(print_buf);

  // Serial.print("Time taken: ");
  // Serial.print((t2 - t1) * 1.0 / 1000);
  // Serial.println(" milliseconds!");

  // Clean up at the end to free the memory allocated
  fft_destroy(real_fft_plan);
}

// #include "FFT.h" // include the library
// #include "FFT_signal.h"

// #define sensorPin GPIO36

// void setup()
// {
//   Serial.begin(115200); // use the serial port
//   char print_buf[300];
//   fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

//   for (int k = 0; k < FFT_N; k++)
//     real_fft_plan->input[k] = (float)fft_signal[k];

//   long int t1 = micros();
//   // Execute transformation
//   fft_execute(real_fft_plan);

//   // Print the output
//   for (int k = 1; k < real_fft_plan->size / 2; k++)
//   {
//     /*The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output*/
//     float mag = sqrt(pow(real_fft_plan->output[2 * k], 2) + pow(real_fft_plan->output[2 * k + 1], 2)) / 1;
//     float freq = k * 1.0 / TOTAL_TIME;
//     sprintf(print_buf, "%f Hz : %f", freq, mag);
//     Serial.println(print_buf);
//     if (mag > max_magnitude)
//     {
//       max_magnitude = mag;
//       fundamental_freq = freq;
//     }
//   }
//   long int t2 = micros();

//   Serial.println();
//   /*Multiply the magnitude of the DC component with (1/FFT_N) to obtain the DC component*/
//   sprintf(print_buf, "DC component : %f g\n", (real_fft_plan->output[0]) / 10000 / FFT_N); // DC is at [0]
//   Serial.println(print_buf);

//   /*Multiply the magnitude at all other frequencies with (2/FFT_N) to obtain the amplitude at that frequency*/
//   sprintf(print_buf, "Fundamental Freq : %f Hz\t Mag: %f g\n", fundamental_freq, (max_magnitude / 10000) * 2 / FFT_N);
//   Serial.println(print_buf);

//   Serial.print("Time taken: ");
//   Serial.print((t2 - t1) * 1.0 / 1000);
//   Serial.println(" milliseconds!");

//   // Clean up at the end to free the memory allocated
//   fft_destroy(real_fft_plan);
// }

// void loop()
// {
// }
