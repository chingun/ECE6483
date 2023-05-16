#include <mbed.h>
#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"
#include <fftw3.h>  // Include the FFTW header file

using namespace std;

extern "C" void wait_ms(int ms) { wait_us(1000*ms); }

struct GyroData {
    int16_t x, y, z;
};

class Gyroscope {
public:
    Gyroscope(SPI& spi, DigitalOut& cs) : spi(spi), cs(cs) {}

    void initialize() {
        cs = 1;
        cs = 0;
        spi.write(0x20);
        spi.write(0xCF);
        cs = 1;
        spi.format(8, 3);
        spi.frequency(100000);
    }

    GyroData readData() {
        cs = 0;
        spi.write(0xE8);
        int xl = spi.write(0x00);
        int xh = spi.write(0x00);
        int yl = spi.write(0x00);
        int yh = spi.write(0x00);
        int zl = spi.write(0x00);
        int zh = spi.write(0x00);
        cs = 1;

        GyroData data;
        data.x = (xh << 8) | (xl);
        data.y = (yh << 8) | (yl);
        data.z = (zh << 8) | (zl);

        wait_us(50000);

        return data;
    }

private:
    SPI& spi;
    DigitalOut& cs;
};

class GyroSequenceMatcher {
public:
    GyroSequenceMatcher(double threshold) : threshold(threshold) {}

    bool compare(const vector<GyroData>& seq1, const vector<GyroData>& seq2) {
        int n = min(seq1.size(), seq2.size());

        vector<int16_t> x1(n, 0), y1(n, 0), z1(n, 0);
        vector<int16_t> x2(n, 0), y2(n, 0), z2(n, 0);

        for (int i = 0; i < n; i++) {
            x1[i] = seq1[i].x;
            y1[i] = seq1[i].y;
            z1[i] = seq1[i].z;

            x2[i] = seq2[i].x;
            y2[i] = seq2[i].y;
            z2[i] = seq2[i].z;
        }

        double distX = computeCrossCorrelation(x1, x2);
        double distY = computeCrossCorrelation(y1, y2);
        double distZ = computeCrossCorrelation(z1, z2);

        printf("Dist: %d, %d, %d\n", (int)distX, (int)distY, (int)distZ);
        bool correct = distX < threshold && distY < threshold && distZ < threshold;
        printf("Correct? %d\n", correct);

        return correct;
    }

private:
    double computeCrossCorrelation(const std::vector<int16_t>& x, const std::vector<int16_t>& y) {
        int n = std::min(x.size(), y.size());
        int paddedSize = 2 * pow(2, ceil(log2(n))); // Next power of 2 for efficient FFT
        std::vector<std::complex<double>> fftX(paddedSize, 0.0);
        std::vector<std::complex<double>> fftY(paddedSize, 0.0);

        // Create FFTW plans
        fftw_complex* inX = reinterpret_cast<fftw_complex*>(fftX.data());
        fftw_complex* inY = reinterpret_cast<fftw_complex*>(fftY.data());
        fftw_plan planX = fftw_plan_dft_1d(paddedSize, inX, inX, FFTW_FORWARD, FFTW_ESTIMATE);
        fftw_plan planY = fftw_plan_dft_1d(paddedSize, inY, inY, FFTW_FORWARD, FFTW_ESTIMATE);

        // Perform FFT on x
        for (int i = 0; i < n; i++) {
            fftX[i] = x[i];
        }
        fftw_execute(planX);

        // Perform FFT on y
        for (int i = 0; i < n; i++) {
            fftY[i] = y[i];
        }
        fftw_execute(planY);

        // Calculate cross-correlation in frequency domain
        std::vector<std::complex<double>> crossCorr(paddedSize, 0.0);
        for (int i = 0; i < paddedSize; i++) {
            crossCorr[i] = std::conj(fftX[i]) * fftY[i];
        }
        fftw_plan planInv = fftw_plan_dft_1d(paddedSize, reinterpret_cast<fftw_complex*>(crossCorr.data()),
                                            reinterpret_cast<fftw_complex*>(crossCorr.data()), FFTW_BACKWARD, FFTW_ESTIMATE);
        fftw_execute(planInv);

        // Find maximum value in cross-correlation
        double maxCorrelation = 0.0;
        for (int i = 0; i < n; i++) {
            double correlation = std::abs(crossCorr[i]);
            maxCorrelation = std::max(maxCorrelation, correlation);
        }

        // Clean up FFTW plans and allocated memory
        fftw_destroy_plan(planX);
        fftw_destroy_plan(planY);
        fftw_destroy_plan(planInv);

        return maxCorrelation;
    }

    // double computeCrossCorrelation(const vector<int16_t>& x, const vector<int16_t>& y) {
    //     int n = min(x.size(), y.size());
    //     int paddedSize = 2 * pow(2, ceil(log2(n))); // Next power of 2 for efficient FFT
    //     vector<complex<double>> fftX(paddedSize, 0.0);
    //     vector<complex<double>> fftY(paddedSize, 0.0);

    //     // Perform FFT on x
    //     for (int i = 0; i < n; i++) {
    //         fftX[i] = x[i];
    //     }
    //     fft(fftX);

    //     // Perform FFT on y
    //     for (int i = 0; i < n; i++) {
    //         fftY[i] = y[i];
    //     }
    //     fft(fftY);

    //     // Calculate cross-correlation in frequency domain
    //     vector<complex<double>> crossCorr(paddedSize, 0.0);
    //     for (int i = 0; i < paddedSize; i++) {
    //         crossCorr[i] = conj(fftX[i]) * fftY[i];
    //     }
    //     ifft(crossCorr);

    //     // Find maximum value in cross-correlation
    //     double maxCorrelation = 0.0;
    //     for (int i = 0; i < n; i++) {
    //         double correlation = abs(crossCorr[i]);
    //         maxCorrelation = max(maxCorrelation, correlation);
    //     }

    //     return maxCorrelation;
    // }
    // double computeCrossCorrelation(const vector<int16_t>& x, const vector<int16_t>& y) {
    //     int n = min(x.size(), y.size());
    //     double distance = 0;
    //     int N = 0;

    //     for (int i = 0; i < n; i++) {
    //         if (x[i] && y[i]) {
    //             distance += abs(x[i] - y[i]);
    //             N++;
    //         }
    //     }

    //     return N ? distance / N : 0;
    // }

    double threshold;
};

class LCDDisplay {
public:
    LCDDisplay() {
        HAL_Init();
        BSP_LCD_Init();
        BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);
        BSP_LCD_SelectLayer(0);
        BSP_LCD_DisplayOn();
        BSP_LCD_Clear(LCD_COLOR_WHITE);
    }
    void displayText(const string& text, int line, uint32_t color) {
        BSP_LCD_Clear(LCD_COLOR_WHITE);
        BSP_LCD_SetTextColor(color);
        BSP_LCD_DisplayStringAt(0, LINE(line), (uint8_t*)text.c_str(), CENTER_MODE);
    }
};

enum State { IDLE, RECORDING, WAITING, UNLOCKING, CHECKING, LOCKED };

int main() {
    SPI spi(PF_9, PF_8, PF_7);
    DigitalOut cs(PC_1);
    Gyroscope gyroscope(spi, cs);
    LCDDisplay lcdDisplay;
    vector<GyroData> recordData;
    vector<GyroData> attemptData;

    State state = IDLE;
    const int MAX_ARR_LENGTH = 100;
    const double THRESHOLD = 15000;

    volatile bool flag = false;

    InterruptIn enterKey(USER_BUTTON);
    enterKey.fall([&flag]() { flag = true; });
    int remainingAttempts = 5;
    
    while (1) {
        if (flag) {
            switch (state) {
                case IDLE:
                    recordData.clear();
                    attemptData.clear();
                    printf("IDLE -> RECORDING\n");
                    lcdDisplay.displayText("IDLE", 5, LCD_COLOR_BLACK);
                    state = RECORDING;
                    break;
                case RECORDING:
                    printf("RECORDING -> WAITING\n");
                    lcdDisplay.displayText("Recording...", 5, LCD_COLOR_BLACK);
                    state = WAITING;
                    break;
                case WAITING:
                    printf("WAITING -> UNLOCKING\n");
                    lcdDisplay.displayText("Unlocking", 5, LCD_COLOR_BLACK);
                    state = UNLOCKING;
                    break;
                case UNLOCKING:
                    printf("UNLOCKING -> CHECKING\n");
                    state = CHECKING;
                    break;
                case CHECKING: {
                    printf("Checking sequences\n");
                    GyroSequenceMatcher sequenceMatcher(THRESHOLD);
                    bool correct = sequenceMatcher.compare(recordData, attemptData);

                    if (!correct) {
                        lcdDisplay.displayText("Incorrect!", 5, LCD_COLOR_RED);
                        lcdDisplay.displayText("-.-", 7, LCD_COLOR_RED);
                        char buffer[32];
                        sprintf(buffer, "Remaining: %d", remainingAttempts);
                        lcdDisplay.displayText(buffer, 6, LCD_COLOR_RED);
                        printf("Incorrect. %d attempts remaining...\n", remainingAttempts);

                        if (remainingAttempts == 0) {
                            state = LOCKED;
                        } else {
                            state = WAITING;
                            remainingAttempts--;
                        }
                    } else {
                        printf("Correct! Checking -> IDLE\n");
                        lcdDisplay.displayText("Unlocked", 4, LCD_COLOR_GREEN);
                        lcdDisplay.displayText("<3", 6, LCD_COLOR_GREEN);
                        state = IDLE;
                    }
                    break;
                }
                case LOCKED:
                    lcdDisplay.displayText("LOCKED.", 5, LCD_COLOR_BLACK);
                    state = LOCKED;
                    break;
                default:
                    state = IDLE;
                    break;
            }
            flag = false;
        } else if (state == RECORDING || state == UNLOCKING) {
            if (recordData.size() < MAX_ARR_LENGTH) {
                vector<GyroData>& data = (state == RECORDING) ? recordData : attemptData;
                data.push_back(gyroscope.readData());
                printf("%d, %d, %d\n", data.back().x, data.back().y, data.back().z);
                wait_us(50000);
            } else {
                printf("Exceeded max array length\n");
            }
        }
    }
}
