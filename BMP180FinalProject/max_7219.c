#include <stdio.h>     
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>   
#include <time.h>       
#include <fcntl.h>                       
#include <sys/ioctl.h>          
#include <linux/types.h>        
#include <linux/spi/spidev.h>  
#include "MAX7219.h"

// 0 -> 9
uint8_t maledcc[10] = {0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B};
// -, _, a, b , C, d, E , F, H, P, n, U, S, A
uint8_t makitu[20] = {0x01, 0x08, 0x7D, 0x1F, 0x4E, 0x3D, 0x4F, 0x47, 0x37, 0x67, 0x15, 0x3E, 0x5B, 0x77, 0x5D};

// SPI parameters 
int fd;
int bpw = 8;
int max_speed = 10000000;

// Send Data SPI
void sendDataSPI(uint8_t address, uint8_t value) {
    struct spi_ioc_transfer spi;
    memset (&spi, 0, sizeof (spi));

    uint8_t data[2];
    data[0] = address;
    data[1] = value;

    spi.tx_buf        = (unsigned long)(data);
    spi.len           = 2;
    spi.delay_usecs   = 0;
    spi.speed_hz      = max_speed;
    spi.bits_per_word = bpw;
    ioctl (fd, SPI_IOC_MESSAGE(1), &spi);
}

// Load SPI
void loadSPI(void) {
    // Load SPI driver 
    fd = open("/dev/spidev0.0", O_RDWR);
    // Set SPI parameters
    int mode = 0;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
        printf("Can't set SPI mode\n");
        exit(1);
    }
    int lsb_first = 0;
    if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb_first) < 0) {
        printf("Can't set LSB_first\n");
        exit(1);
    }
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) {
        printf("Can't set Bit per word\n");
        exit(1);
    }
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed) < 0) {
        printf("Can't set Max Speed\n");
        exit(1);
    }
}

// Init Max7219 SPI
void initMax7219(void) {
    // Setup Max7219
    sendDataSPI(MAX7219_REG_DECODEMODE, 0x00);   // No decode mode
    sendDataSPI(MAX7219_REG_INTENSITY, 0x09);   // Intensity 80% = 0x0C, 50% = 0x08, 60% = 0x09
    sendDataSPI(MAX7219_REG_SCANLIMIT, 0x07);   // Scan Limit sang 8 Led
    sendDataSPI(MAX7219_REG_SHUTDOWN, 1);      // Normal Operation_Shutdown Mode
    sendDataSPI(MAX7219_REG_DISPLAYTEST, 0);   // Normal Operation_Display test    
}

void display3Number(float in1, float in2, float in3) {
    sendDataSPI(8, maledcc[((int)fabs(in1))/10]);
    if (in1 < 0) sendDataSPI(7, 0x80|maledcc[((int)fabs(in1))%10]);
    else sendDataSPI(7, maledcc[((int)fabs(in1))%10]);

    sendDataSPI(6, 0x01); // -

    sendDataSPI(5, maledcc[((int)fabs(in2))/10]);
    if (in2 < 0) sendDataSPI(4, 0x80|maledcc[((int)fabs(in2))%10]);
    else sendDataSPI(4, maledcc[((int)fabs(in2))%10]);

    sendDataSPI(3, 0x01); // -

    sendDataSPI(2, maledcc[((int)fabs(in3))/10]);
    if (in3 < 0) sendDataSPI(1, 0x80|maledcc[((int)fabs(in3))%10]);
    else sendDataSPI(1, maledcc[((int)fabs(in3))%10]);
}

void display1Angle(float in) {
    static float prevAngle = 0; // luu gia tri goc truoc do
    float diff = fabs(in - prevAngle); // do chenh lech giua goc hien tai va goc truoc do
    prevAngle = in;

    // Negative real numbers- gui ky hieu dau tru den hang 6
    if (in < 0) sendDataSPI(6, 0x01); // -
    else sendDataSPI(6, 0x00); //neu khong phai gia tri - thi khong hien gi 

    float n = fabs(in); // need to include stdlib.h
    int inPart= (int)n; // integer part - phan nguyen
    int decPart = (int)((n - inPart) * 10 );  // decimal part (1 number) lay 1 gia tri sua chua so thap phan

    // Display number
    sendDataSPI(5, maledcc[inPart/10]);
    sendDataSPI(4, 0x80|maledcc[inPart%10]);
    sendDataSPI(3, maledcc[decPart]);

    // Display format
    sendDataSPI(8, 0x00); // 
    sendDataSPI(1, 0x08); // _
    sendDataSPI(2, 0x63); // o
    // Hien thi P(xoay) hoac S(dung yen)
    if (diff > 0.1){ // do nhay cua mpu
        sendDataSPI(7, makitu[9]); // P
    } else{
        sendDataSPI(7, 0x5B); // S
    }

    // dieu chinh do sang intensity neu trong nguong [-60,60]
    static int warning = 0;  // co de theo doi 
    static int brightness = 0x00;
    static int dir = 1; // tang giam do sang
 
    if (in < -60 || in > 60) {
        warning = 1;
        sendDataSPI(MAX7219_REG_INTENSITY, brightness); // gui gia tri do sang
 
        brightness += dir;
        if (brightness >= 0x0F) dir = -1; // max do sang
        else if (brightness <= 0x01) dir = 1; // min do sang
    } else if (warning == 1) {
        warning = 0;
        sendDataSPI(MAX7219_REG_INTENSITY, 0x08); // reset về độ sáng bình thường 50%
    }
}

void displayTemp(float in) {
    int inPart= (int)in; // integer part 
    int decPart = (int)((in - inPart) * 100 );  // decimal part (2 number)

    // Display number
    // sendDataSPI(7, maledcc[inPart/10]);
    // sendDataSPI(6, 0x80|maledcc[inPart%10]);
    // sendDataSPI(5, maledcc[decPart/10]);
    // sendDataSPI(4, maledcc[decPart%10]);
    // // Display format
    // sendDataSPI(8, 0x08); // _
    // sendDataSPI(3, 0x63); // o
    // sendDataSPI(2, makitu[4]); // C
    // sendDataSPI(1, 0x08); // _

    //Display number
    sendDataSPI(8, maledcc[inPart/10]);
    sendDataSPI(7, 0x80|maledcc[inPart%10]);
    sendDataSPI(6, maledcc[decPart/10]);
    //sendDataSPI(5, maledcc[decPart%10]);
    sendDataSPI(5, 0x63); // o

    sendDataSPI(4, 0x7F); // 8
    sendDataSPI(3, 0x7E); // On sang dan, Off tat dan
    sendDataSPI(2, 0x77); // _
    sendDataSPI(1, 0x5D); // _

}
