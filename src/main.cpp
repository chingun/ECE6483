#include <mbed.h>
#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"
#include <fft.h>  // Include the FFTW header file

using namespace std;

extern "C" void wait_ms(int ms) { wait_us(1000*ms); }

// Creativity Achievements:
// 1. LCD Screen based user feed back
// 2. Button debouncing for better lock performance
// 3. User unlocks are numbered until completely locked out

struct GyroData {
    int16_t x, y, z; //creating struct to store x, y, z data points
};

// Class representing a gyroscope module controlled via SPI communication.
// The class provides methods to initialize the gyroscope and read data from it.
class Gyroscope {
public:
    Gyroscope(SPI& spi, DigitalOut& cs) : spi(spi), cs(cs) {}

    void initialize() { //initializing the gyroscope
        cs = 1; //Set chip select high
        cs = 0; //Set chip select low
        spi.write(0x20);
        spi.write(0xCF); 
        cs = 1; //Set chip select high
        spi.format(8, 3); 
        spi.frequency(100000);
    }

    GyroData readData() {
        cs = 0; // Start transmission
        spi.write(0xE8);
        int xl = spi.write(0x00); //getting low byte for x data point
        int xh = spi.write(0x00); //getting high byte for x data point
        int yl = spi.write(0x00); //getting low byte for y data byte
        int yh = spi.write(0x00); //getting high byte for y data byte
        int zl = spi.write(0x00); //getting low byte for z data byte
        int zh = spi.write(0x00); //getting high byte for z data byte
        cs = 1; // end transmission

        GyroData data;
        data.x = (xh << 8) | (xl); //Combine high and low bytes of x data
        data.y = (yh << 8) | (yl); //Combine high and low bytes of y data
        data.z = (zh << 8) | (zl); //Combine high and low bytes of z data

        wait_us(50000);

        return data;
    }

private:
    SPI& spi; //creating spi object for the gyroscope
    DigitalOut& cs; //creating digital out object to set chip select high or low
};

// The GyroSequenceMatcher class is responsible for comparing sequences of GyroData
// and determining if they are similar based on a specified threshold.
// It calculates the cross-correlation between the x, y, and z measurements of two sequences,
// and if the cross-correlation values are below the threshold, it considers the attempt sequence as correct.
// The class also provides a method to compute the cross-correlation between two vectors.
// Overall, it offers functionality to compare and analyze gyroscopic data for similarity assessment.
class GyroSequenceMatcher {
public:
    GyroSequenceMatcher(double threshold) : threshold(threshold) {}

    bool compare(const vector<GyroData>& seq1, const vector<GyroData>& seq2) {
        int n = min(seq1.size(), seq2.size()); // minimum size of the recording and attempt sequences

        vector<int16_t> x1(n, 0), y1(n, 0), z1(n, 0); //constraining the size of each vector of measurements to the minimum between both sequences
        vector<int16_t> x2(n, 0), y2(n, 0), z2(n, 0); //constraining the size of each vector of measurements to the minimum between both sequences

        for (int i = 0; i < n; i++) {
            //populating the empty vectors with recording sequence measurements
            x1[i] = seq1[i].x;
            y1[i] = seq1[i].y;
            z1[i] = seq1[i].z;
            //populating the empty vectors with attempt sequence measurements
            x2[i] = seq2[i].x;
            y2[i] = seq2[i].y;
            z2[i] = seq2[i].z;
        }
        //computing cross correlation between x, y, z recording and attempt measurements
        // Determine level of relationship between both signals
        double distX = computeCrossCorrelation(x1, x2);
        double distY = computeCrossCorrelation(y1, y2);
        double distZ = computeCrossCorrelation(z1, z2);

        printf("Dist: %d, %d, %d\n", (int)distX, (int)distY, (int)distZ);
        // If cross correlation is under the threshold, the attempt sequence is correct
        bool correct = distX < threshold && distY < threshold && distZ < threshold;
        printf("Correct? %d\n", correct);

        return correct;
    }

private: 
    double computeCrossCorrelation(const vector<int16_t>& x, const vector<int16_t>& y) {
        int n = min(x.size(), y.size());
        double distance = 0;
        int N = 0;

        for (int i = 0; i < n; i++) {
            if (x[i] && y[i]) {
                distance += abs(x[i] - y[i]);
                N++;
            }
        }

        return N ? distance / N : 0;
    }

    double threshold;
};

// Initializes the LCD display and provides a method to display text on the screen. 
// The constructor initializes the LCD screen by initializing the necessary components 
// and clearing the screen. The displayText function clears the screen, sets the text 
// color, and displays the provided text at the specified line on the LCD screen.
class LCDDisplay {
public:
    LCDDisplay() {
        //Initializing the LCD screen
        HAL_Init();
        BSP_LCD_Init();
        BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);
        BSP_LCD_SelectLayer(0);
        BSP_LCD_DisplayOn(); //Turning display on
        BSP_LCD_Clear(LCD_COLOR_WHITE); //Clearing the screen
    }
    void displayText(const string& text, int line, uint32_t color) {
        BSP_LCD_SetTextColor(color); //Setting text color
        BSP_LCD_DisplayStringAt(0, LINE(line), (uint8_t*)text.c_str(), CENTER_MODE); //display text at center of the screen
    }
};

enum State { IDLE, RECORDING, WAITING, UNLOCKING, CHECKING, LOCKED };

int main() {
    SPI spi(PF_9, PF_8, PF_7); //Creating spi object for gyroscope
    DigitalOut cs(PC_1); //Creating cs object on PC1 for gyroscope
    Gyroscope gyroscope(spi, cs); //Initializing gyroscope object with spi and cs pins
    LCDDisplay lcdDisplay; //Creating LCD display object
    vector<GyroData> recordData; //Creating vector to store recorded data
    vector<GyroData> attemptData; //Creating vector to store attempted data

    State state = IDLE; //State when recording and attempting is not occuring
    const int MAX_ARR_LENGTH = 100; //Setting a maximum length for the amount of recorded data points
    const double THRESHOLD = 15000; //Setting threshold for cross correlation

    volatile bool flag = false; //Initialize flag as false
    bool debounceInProgress = false;
    InterruptIn enterKey(USER_BUTTON); //Setting interrupt to user button

    enterKey.fall([&flag, &debounceInProgress]() {
        if (!debounceInProgress) {
            debounceInProgress = true;
            // Set a flag after the debounce time has passed
            wait_ms(200);
            // ThisThread::sleep_for(200ms); // Adjust the debounce time as needed
            flag = true;
            debounceInProgress = false;
        }
    });
    // enterKey.fall([&flag]() { flag = true; }); //When button is pressed, interrupt flags
    int remainingAttempts = 3; // Initializing the number of attempts
    
    while (1) {
        if (flag) {
            switch (state) {
                case IDLE:
                    recordData.clear(); //Clearing recorded data vector
                    attemptData.clear(); //Clearing attempted data vector
                    printf("IDLE to RECORDING\n");
                    BSP_LCD_Clear(LCD_COLOR_WHITE); //Clearing the screen
                    lcdDisplay.displayText("INITIALIZING", 5, LCD_COLOR_BLACK);
                    lcdDisplay.displayText("LOCK", 6, LCD_COLOR_BLACK);
                    lcdDisplay.displayText("SEQUENCE", 7, LCD_COLOR_BLACK);
                    state = RECORDING;
                    break;
                case RECORDING:
                    printf("RECORDING to WAITING\n");            
                    BSP_LCD_Clear(LCD_COLOR_WHITE); //Clearing the screen
                    lcdDisplay.displayText("INITIAL", 5, LCD_COLOR_BLACK);
                    lcdDisplay.displayText("KEY", 6, LCD_COLOR_BLACK);
                    lcdDisplay.displayText("RECORDED", 7, LCD_COLOR_BLACK);
                    state = WAITING; //User is in the process of recording their attempt
                    break;
                case WAITING:
                    printf("WAITING to UNLOCKING\n");            
                    BSP_LCD_Clear(LCD_COLOR_WHITE); //Clearing the screen
                    lcdDisplay.displayText("RECORDING", 5, LCD_COLOR_BLACK);
                    lcdDisplay.displayText("UNLOCK", 6, LCD_COLOR_BLACK);
                    lcdDisplay.displayText("SEQUENCE", 6, LCD_COLOR_BLACK);
                    state = UNLOCKING;
                    break;
                case UNLOCKING:
                    printf("UNLOCKING to CHECKING\n");
                    BSP_LCD_Clear(LCD_COLOR_WHITE); //Clearing the screen
                    lcdDisplay.displayText("RECORDING", 4, LCD_COLOR_BLACK);
                    lcdDisplay.displayText("COMPLETED", 5, LCD_COLOR_BLACK);
                    lcdDisplay.displayText("CLICK 2 CHECK", 7, LCD_COLOR_BLACK); 
                    state = CHECKING; //Checking if recorded and attempt sequences match
                    break;
                case CHECKING: {
                    printf("Running Gyro Sequence Matcher\n");
                    GyroSequenceMatcher sequenceMatcher(THRESHOLD);
                    bool correct = sequenceMatcher.compare(recordData, attemptData);

                    if (!correct) {
                        //Attempted sequence is incorrect            
                        BSP_LCD_Clear(LCD_COLOR_WHITE); //Clearing the screen
                        lcdDisplay.displayText("INCORRECT KEY", 5, LCD_COLOR_RED);
                        char buffer[32];
                        sprintf(buffer, "Remaining: %d", remainingAttempts); 
                        BSP_LCD_DisplayStringAt(0, LINE(6), (uint8_t *)buffer, CENTER_MODE);                       

                        lcdDisplay.displayText("-.-", 9, LCD_COLOR_RED);
                        printf("Incorrect. %d attempts remaining...\n", remainingAttempts);
                        if (remainingAttempts == 0) {
                            state = LOCKED; //All attempts have been used
                        } else {
                            state = WAITING;
                            remainingAttempts--; //If wrong attempt, remaining attempts are decremented
                        }
                    } else {
                        printf("Correct! Checking -> IDLE\n"); // Attempted key is correct
                        BSP_LCD_Clear(LCD_COLOR_WHITE); //Clearing the screen
                        lcdDisplay.displayText("Welcome", 4, LCD_COLOR_GREEN);
                        lcdDisplay.displayText("<3", 6, LCD_COLOR_GREEN);
                        state = IDLE;
                    }
                    break;
                }
                case LOCKED:
                    lcdDisplay.displayText("BLOCKED", 5, LCD_COLOR_BLACK);
                    state = LOCKED; //State when all attempts have been used
                    break;
                default:
                    state = IDLE;
                    break;
            }
            flag = false;
        } else if (state == RECORDING || state == UNLOCKING) {
            if (recordData.size() < MAX_ARR_LENGTH) { //Record data points if under maximum array length
                vector<GyroData>& data = (state == RECORDING) ? recordData : attemptData;
                data.push_back(gyroscope.readData()); // Recording gyroscope data for recording and attempted keys
                printf("%d, %d, %d\n", data.back().x, data.back().y, data.back().z);
                wait_us(50000);
            } else {
                // No longer record data points if maximum array length has been exceeded
                printf("Exceeded max array length\n");
            }
        }
    }
}