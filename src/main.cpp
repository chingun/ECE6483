#include <mbed.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"

#define SPI_FLAG 1
#define OUT_X_L 0x28
//register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
#define CTRL_REG1 0x20
//configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
//register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG4 0x23
//configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
//register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)
#define CTRL_REG5 0x24
//configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
#define CTRL_REG5_CONFIG 0b00'00'00'10

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

const int threshold = 1000;
const int max_sequence_length = 300;
volatile bool flag = false;

SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel);
DigitalOut cs(PC_1); 
InterruptIn enter_key(USER_BUTTON);
EventFlags flags;
//The spi.transfer() function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
}


void set_flag();
void set_mode();
void setup_gyro();
void init_display();
bool compare_sequences(const std::vector<GyroData> &seq1, const std::vector<GyroData> &seq2, double threshold);
double compute_cross_correlation(const std::vector<double> &x, const std::vector<double> &y);
GyroData read_gyro();
GyroData raw_data;

int8_t write_buf[32],buffer[32];
int16_t temp;

int8_t def[3] = {0};

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
  spi.format(8,3);
  spi.frequency(1'000'000);

  write_buf[0]=CTRL_REG1;
  write_buf[1]=CTRL_REG1_CONFIG;
  spi.transfer(write_buf,2,buffer,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  write_buf[0]=CTRL_REG4;
  write_buf[1]=CTRL_REG4_CONFIG;
  spi.transfer(write_buf,2,buffer,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  write_buf[0]=CTRL_REG5;
  write_buf[1]=CTRL_REG5_CONFIG;
  spi.transfer(write_buf,2,buffer,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);  
  printf("Configured, ready to start reading\n");
}

GyroData read_gyro() {
  // Initialize buffer for reading data from SPI bus
  //start sequential sample reading
  write_buf[0]=OUT_X_L|0x80|0x40;
  spi.transfer(write_buf,7,buffer,7,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);
  
  temp = ( ( (uint16_t)buffer[2] ) <<8 ) | ( (uint16_t)buffer[1] );
  raw_data.x = (int)((temp)*(17.5f*0.017453292519943295769236907684886f / 1000.0f));

  temp = ( ( (uint16_t)buffer[4] ) <<8 ) | ( (uint16_t)buffer[3] );
  raw_data.y = (int)((temp)*(17.5f*0.017453292519943295769236907684886f / 1000.0f)); 

  temp = ( ( (uint16_t)buffer[6] ) <<8 ) | ( (uint16_t)buffer[5] );
  raw_data.z = (int)((temp)*(17.5f*0.017453292519943295769236907684886f / 1000.0f));

  return raw_data;
}

void setup_gyro() {
  cs = 1;
  spi.format(8, 3);
  spi.frequency(100000);
  setMode();
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
  printf("Corr: %f", max_corr);
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
          BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Recording...", CENTER_MODE);
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
          BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Failed!", CENTER_MODE);
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
        thread_sleep_for(50);
      }
    }
  }
}
