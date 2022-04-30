#define FFT_N 2048          // Must be a power of 2
#define TOTAL_TIME 9.391904 // The time in which data was captured. This is equal to FFT_N/sampling_freq

float fft_input[FFT_N];
float fft_output[FFT_N];

float max_magnitude = 0;
float fundamental_freq = 0;

/* Dummy data (Output of an accelerometer)
 * Frequency: 5 Hz
 * Amplitude: 0.25g
 */
double fft_signal[FFT_N] = {
    11100, 10600, 11200, 11700, 12200, 12900, 12900, 13100, 11600, 12900, 13600, 13000, 12100, 11700, 11000, 11500, 10400, 10800, 9800, 9800, 9400, 9700, 8700, 8700, 8200, 8200, 7700, 7800, 7200, 7200, 6900, 5900, 5600, 7400, 8100, 8300, 8600, 9400, 9300, 9600, 10300, 11000, 10700, 11200, 11200, 11200, 11800, 11700, 12900, 12700, 12700, 11900, 11500, 11100, 11700, 13700, 12900, 11700, 10600, 11000, 10900, 11100, 9800, 9600, 9900, 9400, 9000, 8300, 8100, 7600, 7100, 7600, 7000, 9700, 8200, 8200, 7600, 7000, 7500, 8500, 8400, 9400, 9100, 9400, 10200, 10400, 10100, 10600, 11000, 11700, 11700, 12700, 12100, 12200, 12000, 12200, 14200, 13700, 12400, 11100, 11000, 11500, 10800, 10700, 10500, 10400, 9800, 9400, 9300, 9300, 9200, 9000, 8000, 7600, 7600, 7400, 7500, 3600, 6100, 6700, 9000, 8500, 7800, 8000, 8800, 9100, 10000, 9600, 10900, 10600, 11000, 11400, 11800, 11700, 11100, 12100, 12600, 13400, 11000, 12800, 12100, 13100, 12800, 12100, 11300, 10900, 11200, 10500, 10200, 9500, 9900, 9800, 9000, 8600, 8300, 8000, 7900, 7300, 6800, 6700, 9500, 9300, 7500, 7800, 7100, 8000, 8400, 8700, 8900, 10000, 10300, 10300, 9900, 10300, 11300, 11800, 12000, 11200, 12200, 13100, 13400, 13100, 13500, 12900, 13100, 10900, 11900, 11500, 11700, 10400, 10400, 10700, 9900, 10200, 9600, 9300, 8700, 8500, 8500, 8300, 8000, 7700, 6700, 7200, 4800, 6300, 8400, 8000, 7600, 8100, 9500, 9600, 9600, 9600, 10700, 10400, 11000, 10800, 11300, 11700, 11200, 11900, 12500, 12800, 12500, 10900, 12500, 12600, 11900, 11800, 11800, 11200, 11500, 11000, 10600, 10500, 9900, 9800, 9300, 8700, 9200, 8600, 7900, 7600, 7500, 6600, 7500, 7400, 8800, 6200, 6500, 7500, 8400, 8700, 8700, 9400, 9500, 10000, 10400, 10900, 11200, 11500, 11500, 11700, 11900, 13000, 12000, 13700, 12900, 11600, 13600, 12100, 11300, 11500, 11800, 11600, 10500, 10900, 10600, 10600, 9900, 9000, 9000, 9000, 9100, 8000, 7700, 7700, 8100, 6700, 6800, 4300, 7300, 7400, 8500, 7500, 8500, 9100, 9900, 8900, 9600, 9700, 10200, 10700, 10900, 11500, 11300, 11600, 11800, 13300, 13000, 12900, 11100, 12500, 12300, 13100, 12200, 11800, 11000, 11200, 11000, 10700, 10000, 10400, 9900, 9400, 8800, 8800, 8400, 8300, 8100, 7300, 6800, 7400, 8700, 5700, 5900, 9500, 8900, 9000, 8200, 8800, 8600, 9400, 10400, 10500, 10200, 10500, 12300, 10700, 12100, 11900, 13100, 13300, 12600, 12200, 11700, 11800, 11800, 12700, 12200, 11800, 11000, 10900, 10500, 10000, 9900, 9600, 10000, 9200, 9100, 8500, 8200, 7800, 7700, 7300, 6800, 8600, 9100, 8000, 8200, 7400, 7500, 8300, 8900, 9100, 9500, 9900, 10400, 10600, 11200, 11000, 11800, 11700, 11800, 12700, 12700, 12800, 12700, 13300, 13700, 13700, 10800, 10900, 11400, 11600, 11400, 10900, 10000, 10300, 10000, 9400, 9200, 9100, 8900, 8700, 8300, 8000, 7600, 6700, 7300, 5400, 6100, 6200, 8600, 8500, 8200, 8300, 8100, 9900, 9800, 10000, 9800, 10300, 10700, 12000, 12100, 12100, 11700, 12600, 12600, 12800, 11500, 11900, 11100, 12000, 12900, 12300, 11500, 11200, 11100, 10500, 10200, 9700, 9700, 10300, 9300, 8900, 8300, 8200, 7700, 7500, 7000, 6900, 9700, 8600, 6000, 6000, 7100, 7400, 8400, 9200, 9100, 9700, 9800, 10600, 10200, 11300, 11300, 11600, 12000, 11400, 12200, 12700, 12700, 12800, 12800, 13700, 13800, 11800, 11700, 11700, 11600, 10900, 10800, 10400, 10400, 9400, 9300, 8900, 9300, 9000, 8000, 7900, 8000, 7500, 6700, 8800, 4500, 6700, 7100, 8300, 7400, 8600, 9100, 9400, 9700, 10000, 9500, 9800, 11300, 11900, 12000, 11100, 11000, 12100, 11800, 12600, 12700, 11300, 12300, 11800, 13400, 12700, 11000, 11000, 11200, 11100, 11000, 9900, 9900, 9800, 9300, 9200, 8600, 8600, 8300, 8000, 7800, 7100, 7500, 9000, 9700, 8600, 6400, 7200, 7300, 9100, 9200, 9100, 9700, 9400, 10100, 10700, 11500, 11800, 11200, 12100, 11600, 13100, 12500, 13000, 12900, 13100, 13900, 12300, 11900, 11200, 11700, 11300, 10900, 10300, 10300, 10800, 10300, 9500, 8700, 9700, 9500, 8300, 8200, 7600, 6900, 7500, 7100, 4400, 6400, 6100, 8800, 8200, 8300, 9000, 9400, 9700, 10100, 10500, 10100, 11400, 11700, 12000, 11900, 11000, 11800, 12300, 13000, 13100, 11300, 12100, 11500, 12900, 12900, 12800, 10700, 11200, 11000, 10800, 9900, 9400, 9600, 9800, 9100, 8400, 8000, 7500, 7900, 6900, 7400, 7100, 9600, 7600, 6900, 6400, 7300, 8500, 8400, 9000, 9400, 9400, 9600, 10300, 10500, 10500, 11200, 11400, 11800, 11800, 12900, 12500, 13000, 11900, 13500, 12800, 12700, 11000, 12100, 11400, 11400, 11000, 11100, 10300, 10500, 9300, 9700, 9500, 9400, 8100, 8000, 7700, 8300, 7500, 6800, 6800, 4400, 8000, 7800, 7900, 8200, 7900, 9100, 9200, 9300, 9300, 10800, 11200, 10900, 11100, 10900, 11900, 12100, 12100, 12300, 12500, 12800, 10900, 13400, 13500, 11800, 11400, 11000, 10800, 10600, 10000, 9800, 10500, 9900, 10000, 9400, 9000, 8900, 8700, 8500, 7800, 7900, 6900, 6900, 7800, 7800, 6500, 6400, 7500, 7500, 8000, 8000, 8300, 9600, 9800, 10600, 10200, 10300, 11600, 11300, 12100, 10800, 12200, 12400, 12400, 12400, 12700, 11900, 11800, 11900, 12400, 12100, 11400, 11200, 10700, 10500, 9700, 9600, 9700, 9600, 9000, 8800, 8100, 8000, 8500, 7200, 7300, 7000, 4000, 7200, 8900, 8700, 7800, 8100, 9000, 8900, 9300, 9500, 10300, 10600, 10800, 10700, 10900, 11200, 12100, 12500, 12500, 12400, 12700, 10600, 13200, 13500, 13300, 12100, 12200, 11200, 11500, 11000, 10400, 10000, 9800, 9700, 9400, 9100, 9000, 8600, 8500, 7900, 7400, 6800, 7300, 9000, 7100, 7400, 6800, 7400, 7600, 9100, 8700, 9500, 9700, 10400, 10700, 10100, 11000, 10800, 11800, 11600, 11300, 12200, 12300, 12800, 11300, 12000, 11100, 11300, 13300, 12800, 11200, 10700, 10900, 10900, 10400, 10100, 9300, 9700, 9500, 9200, 8300, 8200, 7600, 8000, 7000, 7000, 8100, 8200, 8300, 7800, 6800, 7200, 8500, 9000, 8900, 9300, 9500, 10600, 10900, 11300, 10500, 10800, 11400, 10900, 13100, 12700, 13100, 11900, 12500, 14000, 13600, 11300, 11000, 12000, 11600, 11100, 11800, 9800, 9500, 9400, 9800, 9300, 9000, 8800, 8200, 7600, 8100, 7400, 7400, 7300, 3700, 7900, 7800, 9200, 8300, 8400, 8900, 8800, 9700, 9700, 9600, 10400, 10800, 11300, 10900, 11500, 11500, 12600, 12400, 12400, 13200, 11100, 13200, 12700, 12100, 12000, 11800, 11500, 11700, 11200, 10700, 9800, 10100, 9600, 9700, 9500, 8600, 8300, 8400, 7900, 6800, 7600, 6600, 8600, 8600, 6700, 6800, 7200, 7800, 8400, 8900, 10100, 10400, 10500, 10700, 11100, 10300, 10600, 11800, 12100, 11600, 12400, 12500, 12600, 12100, 13400, 12700, 12800, 11000, 12000, 12100, 11600, 10400, 10600, 11000, 10400, 9500, 9800, 9100, 9000, 8800, 7800, 8000, 8000, 7500, 7000, 6900, 5900, 6200, 8700, 7600, 7400, 8400, 8400, 8800, 9200, 9600, 9700, 10800, 11700, 12100, 11500, 11500, 11500, 12100, 11700, 13100, 12800, 11300, 12600, 12800, 12900, 12200, 11500, 11200, 11700, 10900, 10500, 9800, 9600, 10100, 9000, 8800, 8600, 8300, 8300, 7600, 7100, 6800, 7000, 9800, 7800, 7800, 6300, 7100, 7400, 8300, 8100, 9200, 9500, 10200, 10000, 10500, 11300, 10900, 12200, 12300, 11400, 12800, 12500, 12900, 12400, 11400, 11000, 11500, 10700, 12200, 12600, 11400, 10500, 10900, 10900, 10300, 10200, 9600, 9400, 8900, 9000, 8100, 8200, 8000, 7600, 6900, 7400, 4900, 8300, 9000, 8100, 7700, 7700, 8100, 9700, 10300, 9600, 10000, 9700, 10400, 11200, 11100, 11200, 11200, 11400, 12400, 12000, 13100, 11300, 12300, 12800, 12700, 13100, 12200, 11700, 10900, 10800, 10500, 10300, 10100, 9400, 10100, 9400, 9000, 8400, 8200, 7600, 7600, 7600, 7400, 9600, 8800, 8200, 8300, 7000, 7800, 8300, 8600, 8100, 8500, 9000, 10500, 10600, 10800, 10700, 11600, 11900, 12100, 12700, 12900, 12800, 12300, 13300, 13900, 13900, 11000, 11100, 11100, 11900, 10900, 11000, 10700, 10000, 9800, 9300, 9100, 9400, 8900, 8000, 8100, 8000, 7500, 7200, 6700, 5900, 8000, 5500, 8700, 8100, 8600, 7700, 8200, 9000, 9700, 10500, 10600, 10700, 11000, 11100, 11600, 11600, 11700, 13100, 12700, 12500, 11100, 12000, 11700, 12800, 12800, 12100, 11200, 10900, 10900, 10400, 10500, 9900, 9200, 9700, 9300, 9400, 7900, 7800, 7700, 7500, 7300, 6800, 9600, 7900, 8400, 8900, 7400, 8100, 7700, 8800, 8800, 9100, 9500, 9800, 11200, 11200, 11300, 11000, 11000, 11600, 11500, 12700, 12700, 12300, 12400, 12500, 13800, 11500, 11600, 11200, 12100, 11600, 10800, 9700, 9900, 9800, 9500, 9200, 9000, 8100, 8600, 8200, 8300, 7500, 7500, 6900, 7300, 5800, 5700, 6900, 7900, 8800, 8400, 8300, 9600, 9600, 10900, 10200, 10500, 10400, 12000, 11800, 12300, 12000, 12200, 12400, 12600, 11300, 11900, 11200, 11400, 13300, 12600, 11600, 10900, 11000, 10900, 10200, 9900, 9700, 9100, 9100, 8700, 8500, 8100, 7900, 7000, 7000, 7100, 9200, 9000, 8700, 7000, 7400, 7000, 8900, 9000, 9400, 9600, 9900, 10200, 10700, 11300, 11000, 11600, 11500, 12300, 13100, 12700, 12400, 12600, 13400, 14000, 12700, 11700, 11200, 12200, 12100, 10600, 10900, 9800, 10700, 9400, 9300, 9100, 9400, 8900, 8800, 8200, 8000, 7600, 7400, 6900, 4400, 6500, 7100, 9100, 8200, 8000, 8700, 9100, 9700, 10400, 9400, 10200, 10900, 10500, 12000, 12100, 11700, 11200, 12700, 12400, 12500, 11000, 11800, 11800, 11900, 13000, 11800, 11400, 11000, 10900, 11100, 9900, 10400, 9500, 10000, 9100, 8800, 8200, 8300, 7500, 7200, 6800, 7000, 9700, 9600, 7700, 7900, 6500, 7800, 8500, 8800, 9200, 9400, 9400, 10300, 10300, 11100, 11400, 11500, 11900, 11800, 11700, 12700, 12800, 12800, 12700, 13400, 13500, 11200, 11700, 11800, 11300, 10700, 10400, 10500, 10700, 10000, 9200, 8900, 9100, 9300, 8600, 7500, 7900, 7300, 7300, 7200, 4600, 7400, 6900, 8200, 7300, 8300, 8400, 9300, 9700, 9800, 10200, 10600, 10900, 12100, 11600, 11400, 11600, 11600, 12100, 12400, 12800, 10600, 11400, 12000, 13500, 12100, 11900, 10600, 10600, 11000, 10500, 10400, 9600, 9800, 9800, 9000, 9200, 8300, 8300, 7500, 7200, 6900, 7300, 10000, 8400, 9100, 6400, 6900, 7300, 8000, 9000, 9300, 9500, 10100, 10500, 10200, 10700, 11300, 10900, 11900, 12000, 12300, 12700, 12500, 12900, 12400, 13400, 12200, 11100, 11500, 11600, 11500, 10600, 11000, 10700, 10100, 10100, 9000, 8400, 9000, 9300, 8500, 8300, 8000, 7300, 7200, 7200, 5000, 9000, 7600, 8600, 7100, 8000, 8400, 8800, 9700, 9400, 9700, 10800, 10600, 11300, 11100, 11400, 11700, 11100, 12000, 12600, 13200, 10800, 12000, 11400, 13500, 13700, 12000, 10600, 10900, 10800, 10400, 10100, 9500, 9800, 9400, 8700, 8400, 8800, 8300, 7800, 7400, 7000, 7100, 8600, 6800, 5800, 6900, 7600, 8200, 8500, 8500, 8800, 10000, 10300, 10600, 10100, 10700, 11300, 12400, 11600, 12800, 12400, 12300, 12100, 11900, 12700, 11200, 11800, 12600, 11500, 12000, 11100, 10600, 10800, 10500, 9600, 9500, 9500, 9100, 9000, 8600, 8400, 8100, 8300, 7500, 7000, 7500, 5400, 8300, 8600, 7800, 7800, 7700, 8700, 9600, 9800, 8800, 9900, 10700, 10800, 11700, 11000, 11200, 12100, 11700, 12200, 12200, 13400, 11000, 14000, 13200, 12200, 10800, 10600, 11200, 11600, 10900, 10000, 10400, 10000, 9600, 10100, 8500, 9000, 8800, 8900, 8300, 7600, 7200, 7300, 4100, 6000, 6100, 6700, 8100, 8400, 8500, 9200, 9500, 10000, 9800, 9600, 10600, 11100, 11800, 11200, 11600, 11000, 12200, 13000, 13000, 11500, 12500, 11400, 12400, 12400, 12900, 11300, 11500, 10500, 10900, 10600, 9900, 10500, 9400, 9100, 8800, 8800, 8300, 8200, 7600, 7700, 7100, 7600, 8500, 8000, 9200, 7100, 7300, 7900, 8500, 9100, 9500, 9000, 10100, 9800, 11400, 11100, 11500, 11300, 11600, 12200, 12400, 12300, 12100, 12200, 13700, 13700, 11000, 11500, 11900, 11600, 10800, 10800, 10000, 9800, 10400, 9300, 9400, 9000, 8800, 8500, 8000, 8300, 7400, 7600, 7600, 4700, 6200, 6100, 8600, 7900, 8500, 8700, 9300, 9700, 9600, 9800, 10100, 11900, 11500, 11400, 11900, 11300, 12400, 12300, 12400, 12000, 10900, 13400, 11400, 13200, 12600, 12500, 11000, 11300, 10900, 10500, 10300, 10100, 9500, 9700, 9000, 9000, 8100, 8100, 7600, 7500, 7200, 6700, 9400, 8800, 8500, 6800, 7600, 7700, 7900, 8800, 9300, 8300, 10400, 10100, 11000, 11000, 11100, 11600, 11900, 12500, 11800, 12700, 13300, 13000, 13000, 13600, 13300, 12100, 10900, 11100, 12000, 10800, 11200, 10600, 10100, 10000, 9700, 9500, 9300, 8900, 8100, 8000, 8400, 7200, 7500, 7200, 5200, 5700, 5700, 8000, 7800, 9100, 8800, 8700, 9000, 9300, 10000, 9900, 11100, 10900, 11700, 11600, 11100, 11600, 12100, 12500, 13200, 11100, 12100, 11300, 11300, 14000, 12800, 11000, 10600, 11600, 10100, 10500, 9900, 9500, 9500, 9200, 9200, 8400, 7800, 7500, 7900, 7700, 7000, 9100, 9200, 9100, 7800, 7300, 7200, 7900, 9100, 9100, 9100, 9600, 9700, 10300, 11000, 11200, 10900, 11000, 11800, 12300, 12800, 13100, 13100, 11700, 13300, 13800, 12100, 11100, 10900, 11200, 11000, 10800, 10100, 10100, 9900, 8700, 9500, 8800, 8600, 8400, 7700, 7500, 7100, 7800, 7000, 6500, 5300, 5300, 5900, 7700, 9000, 8800, 9300, 9800, 9600, 10000, 10000, 9900, 10900, 10900, 11700, 12000, 11700, 12100, 12600, 12800, 11300, 12100, 11600, 11400, 13200, 12800, 10900, 10900, 10500, 10500, 10100, 9800, 9700, 9600, 9300, 8100, 8400, 7900, 8100, 7400, 7400, 7000, 8000, 7700, 8600, 8400, 7500, 7700, 8200, 9200, 9300, 9200, 9500, 10000, 11000, 11500, 11700, 10900, 11800, 12200, 12100, 12500, 12200, 12800, 12300, 14200, 14100, 11600, 11100, 10900, 11400, 11400, 10800, 9800, 10100, 9800, 9300, 8900, 9100, 9200, 8500, 8200, 8200, 8100, 7500, 7100, 7700, 5400, 5900, 6600, 6900, 8700, 8700, 8000, 9700, 9600, 10300, 9800, 10000, 10700, 11600, 12000, 11700, 12000, 12500, 12500, 12600, 12000, 12700, 13500, 11700, 10800, 11100, 12400, 11200, 10800, 10400, 10100, 10200, 10100, 9400, 9300, 9300, 9300, 8200, 8100, 7800, 7400, 7500, 7500, 3800, 7100, 6900, 8600, 8400, 8700, 8900, 9200, 9500, 10100, 10400, 10400, 10500, 10600, 12000, 11700, 11700, 11000};