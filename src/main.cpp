#include <mbed.h>
#include "MovingAverage.h"

#define rad_limit 1.0
#define reading_count 50

SPI spi(PF_9, PF_8, PF_7,PC_1,use_gpio_ssel); // mosi, miso, sclk, cs

//address of first register with gyro data
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

#define SPI_FLAG 1

uint8_t write_buf[32]; 
uint8_t read_buf[32];

MovingAverage <uint8_t, 4> filter_x;
MovingAverage <uint8_t, 4> filter_y;
MovingAverage <uint8_t, 4> filter_z;


float data_buffer[3][reading_count];
float verify_buffer[3][reading_count];

EventFlags flags;
//The spi.transfer() function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
}


int main() {
    
    int i = 0;
    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    spi.format(8,3);
    spi.frequency(1'000'000);

    write_buf[0]=CTRL_REG1;
    write_buf[1]=CTRL_REG1_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    write_buf[0]=CTRL_REG4;
    write_buf[1]=CTRL_REG4_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);

    write_buf[0]=CTRL_REG5;
    write_buf[1]=CTRL_REG5_CONFIG;
    spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
    flags.wait_all(SPI_FLAG);  
    printf("Configured, ready to start reading\n");
    while (1) {
    int16_t raw_gx,raw_gy,raw_gz;
    float gx, gy, gz;
      //prepare the write buffer to trigger a sequential read
      write_buf[0]=OUT_X_L|0x80|0x40;
      //start sequential sample reading
      spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
      flags.wait_all(SPI_FLAG);
      //read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
      //Put the high and low bytes in the correct order lowB,HighB -> HighB,LowB
      raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
      raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
      raw_gz=( ( (uint16_t)read_buf[6] ) <<8 ) | ( (uint16_t)read_buf[5] );

      //printf("RAW|\tgx: %d \t gy: %d \t gz: %d\t",raw_gx,raw_gy,raw_gz);
      
      //gx=filter_x.add((int)((raw_gx)*(17.5f*0.017453292519943295769236907684886f / 1000.0f)));
      //gy=filter_y.add((int)((raw_gy)*(17.5f*0.017453292519943295769236907684886f / 1000.0f)));
      //gz=filter_z.add((int)((raw_gz)*(17.5f*0.017453292519943295769236907684886f / 1000.0f)));

      gx=(int)((raw_gx)*(17.5f*0.017453292519943295769236907684886f / 1000.0f));
      gy=(int)((raw_gy)*(17.5f*0.017453292519943295769236907684886f / 1000.0f));
      gz=(int)((raw_gz)*(17.5f*0.017453292519943295769236907684886f / 1000.0f));

      if(gx<rad_limit && gx>=-rad_limit)
        gx = 0.0;
      if(gy<rad_limit && gy>=-rad_limit)
        gy = 0.0;
      if(gz<rad_limit && gz>=-rad_limit)
        gz = 0.0;
      if(i<reading_count){
        data_buffer[0][i] = gx;
        data_buffer[1][i] = gy;
        data_buffer[2][i] = gz;
        i++;
      }
      else{
        break;
      }
     thread_sleep_for(50);
    }
    printf("Completed reading\n");
    for(int i=0;i<reading_count;i++){
      printf("%4.5f;%4.5f;%4.5f\n",data_buffer[0][i],data_buffer[1][i],data_buffer[2][i]);
    }
    thread_sleep_for(1000);
    printf("Ready for new reading\n");
    i = 0 ;
    while (1) {
    int16_t raw_gx,raw_gy,raw_gz;
    float gx, gy, gz;
    
      //prepare the write buffer to trigger a sequential read
      write_buf[0]=OUT_X_L|0x80|0x40;
      //start sequential sample reading
      spi.transfer(write_buf,7,read_buf,7,spi_cb,SPI_EVENT_COMPLETE );
      flags.wait_all(SPI_FLAG);
      //read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
      //Put the high and low bytes in the correct order lowB,HighB -> HighB,LowB
      raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
      raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
      raw_gz=( ( (uint16_t)read_buf[6] ) <<8 ) | ( (uint16_t)read_buf[5] );

      //printf("RAW|\tgx: %d \t gy: %d \t gz: %d\t",raw_gx,raw_gy,raw_gz);
      
      //gx=filter_x.add((int)((raw_gx)*(17.5f*0.017453292519943295769236907684886f / 1000.0f)));
      //gy=filter_y.add((int)((raw_gy)*(17.5f*0.017453292519943295769236907684886f / 1000.0f)));
      //gz=filter_z.add((int)((raw_gz)*(17.5f*0.017453292519943295769236907684886f / 1000.0f)));

      gx=(int)((raw_gx)*(17.5f*0.017453292519943295769236907684886f / 1000.0f));
      gy=(int)((raw_gy)*(17.5f*0.017453292519943295769236907684886f / 1000.0f));
      gz=(int)((raw_gz)*(17.5f*0.017453292519943295769236907684886f / 1000.0f));

      if(gx<rad_limit && gx>=-rad_limit)
        gx = 0.0;
      if(gy<rad_limit && gy>=-rad_limit)
        gy = 0.0;
      if(gz<rad_limit && gz>=-rad_limit)
        gz = 0.0;
      if(i<reading_count){
        verify_buffer[0][i] = gx;
        verify_buffer[1][i] = gy;
        verify_buffer[2][i] = gz;
        i++;
      }
      else{
        break;
      }
     thread_sleep_for(50);
    }
    printf("Completed Reading\n");
    for(int i=0;i<reading_count;i++){
      printf("%4.5f;%4.5f;%4.5f\n",verify_buffer[0][i],verify_buffer[1][i],verify_buffer[2][i]);
    }
}
