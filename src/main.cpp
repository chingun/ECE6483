#include <mbed.h>
#include <math.h>
#include <vector>
#include <cmath>
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"
 
extern "C" void wait_ms(int ms) {
  for (int i = 0; i < ms; i++) {
    for (int j = 0; j < 1000; j++) {
      __asm("nop");
    }
  }
}

struct GyroData {
  int16_t x, y, z;
};

const int threshold = 100;
const int max_sequence_length = 300;
volatile bool flag = false;
SPI spi(PF_9, PF_8, PF_7);
DigitalOut cs(PC_1); 
InterruptIn enter_key(USER_BUTTON);

void set_flag();
void set_mode();
void setup_gyro();
void init_display();
bool compare_sequences(const std::vector<GyroData> &seq1, const std::vector<GyroData> &seq2, double threshold);
double compute_cross_correlation(const std::vector<double> &x, const std::vector<double> &y);
GyroData read_gyro();

// Define state variables
enum State {
  IDLE,
  RECORDING,
  WAITING,
  UNLOCKING,
  CHECKING,
  LOCKED
};

State state = IDLE;

void updateState() {
  flag = true;
}

void setMode() {
  cs = 0;
  spi.write(0x20);
  spi.write(0xCF);
  cs = 1;
}

GyroData read_gyro() {
  const int num_bytes = 6;
  // Initialize buffer for reading data from SPI bus
  std::array<uint8_t, num_bytes> buffer = {0};

  // Send command to start data read
  cs = 0;
  spi.write(0xE8);
  // Read data from SPI bus
  for (int i = 0; i < num_bytes; i++) {
    buffer[i] = spi.write(0x00);
  }

  cs = 1;
  // Parse raw data from buffer
  GyroData raw_data;
  raw_data.x = (buffer[1] << 8) | buffer[0];
  raw_data.y = (buffer[3] << 8) | buffer[2];
  raw_data.z = (buffer[5] << 8) | buffer[4];

  return raw_data;
}

void setup_gyro() {
  cs = 1;
  setMode();
  spi.format(8, 3);
  spi.frequency(100000);
} 

bool compare_sequences(const std::vector<GyroData>& seq1, const std::vector<GyroData>& seq2, double threshold) {
  int n = std::max(seq1.size(), seq2.size());

  std::vector<double> x1(n, 0), y1(n, 0), z1(n, 0), x2(n, 0), y2(n, 0), z2(n, 0);
  std::transform(seq1.begin(), seq1.end(), x1.begin(), [](const auto& d) { return d.x; });
  std::transform(seq1.begin(), seq1.end(), y1.begin(), [](const auto& d) { return d.y; });
  std::transform(seq1.begin(), seq1.end(), z1.begin(), [](const auto& d) { return d.z; });
  std::transform(seq2.begin(), seq2.end(), x2.begin(), [](const auto& d) { return d.x; });
  std::transform(seq2.begin(), seq2.end(), y2.begin(), [](const auto& d) { return d.y; });
  std::transform(seq2.begin(), seq2.end(), z2.begin(), [](const auto& d) { return d.z; });

  return (compute_cross_correlation(x1, x2) > threshold) && (compute_cross_correlation(y1, y2) > threshold) && 
         (compute_cross_correlation(z1, z2) > threshold);
}

double compute_cross_correlation(const std::vector<double>& x, const std::vector<double>& y) {
  int n = x.size();
  double max_corr = 0.0;
  for (int shift = -n + 1; shift < n; shift++) {
    double corr = 0.0;
    for (int i = 0; i < n; i++) {
      int j = i + shift;
      if (j >= 0 && j < n) {
        corr += x[i] * y[j];
      }
    }
    max_corr = std::max(max_corr, corr / n);
  }
  return max_corr;
}

void init_display() {
  HAL_Init();
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(0);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_WHITE);
}

int main() {
  setup_gyro();
  init_display();

  // Initialize arrays for recorded and attempted sequences
  std::vector<GyroData> record(max_sequence_length);
  std::vector<GyroData> attempt(max_sequence_length);
  int index = 0;
  int remaining_attempts = 5;
  bool correct = false;

  BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"No Record", CENTER_MODE);

  enter_key.fall(&updateState);

  while (1) {
    if (flag) {
      BSP_LCD_Clear(LCD_COLOR_WHITE);
      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

      switch (state) {
        case IDLE:
          record.clear();
          attempt.clear();
          state = RECORDING;
          BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Recording...", CENTER_MODE);
          break;
        case RECORDING:
          state = WAITING;
          BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Unlock?", CENTER_MODE);
          break;
        case WAITING:
          BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Unlock Attempt", CENTER_MODE);
          state = UNLOCKING;
          index = 0;
          break;
        case UNLOCKING:
          BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Checking..", CENTER_MODE);
          state = CHECKING;
          for (int i=0;i<max_sequence_length;i++){
            printf("%d;%d;%d\n",record[i].x,record[i].y,record[i].z);
          }
          printf("Running Sequence Comparison\n");
          for (int i=0;i<max_sequence_length;i++){
            printf("%d;%d;%d\n",attempt[i].x,attempt[i].y,attempt[i].z);
          }
          break;
        case CHECKING:
          correct = compare_sequences(record, attempt, 100);
          if (!correct) {
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
            BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Incorrect :(", CENTER_MODE);
            if (remaining_attempts == 0) {
              state = LOCKED;
              break;
            } else {
              state = WAITING;
              remaining_attempts--;
              printf("Incorrect. %d attempts remaining...\n", remaining_attempts);
              break;
            }
          } else {
            BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
            BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Unlocked :)", CENTER_MODE);
            state = IDLE;
            break;
          }
        case LOCKED: 
          BSP_LCD_SetTextColor(LCD_COLOR_RED);
          BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"PERMA-LOCKED.", CENTER_MODE);
          state = LOCKED;
          break;
        default:
          state = IDLE;
          break;
      }
      flag = false;
    } else if (state == RECORDING || state == UNLOCKING) {
      if (index < max_sequence_length) {
        if (state == RECORDING) {
          record[index] = read_gyro();
        } else {
          attempt[index] = read_gyro();
        }
        index++;
      }
    }
  }
}
