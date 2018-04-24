#ifdef __MS5611_I2C_h__

#else

#define __MS5611_I2C_h__

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#define MS5611_ADDRESS_A (0x76)
#define MS5611_ADDRESS_B (0x77)

typedef enum
{
    MS5611_ADC_ADDR        = 0x00,
    MS5611_PROM_ADDR       = 0xA2
} ms5611_reg_t;

typedef enum
{
    MS5611_CMD_RESET       = 0x1E,        // ADC reset command 
    MS5611_CMD_ADC_CONV    = 0x40,        // ADC conversion command 
    MS5611_CMD_ADC_D1      = 0x00,        // ADC D1 conversion 
    MS5611_CMD_ADC_D2      = 0x10,        // ADC D2 conversion 
    MS5611_CMD_ADC_256     = 0x00,        // ADC OSR=256 
    MS5611_CMD_ADC_512     = 0x02,        // ADC OSR=512 
    MS5611_CMD_ADC_1024    = 0x04,        // ADC OSR=1024 
    MS5611_CMD_ADC_2048    = 0x06,        // ADC OSR=2048 
    MS5611_CMD_ADC_4096    = 0x08         // ADC OSR=4096 
} ms5611_cmd_t;

typedef struct
{
    int file;
    uint8_t address;
    uint16_t C_[6];
    uint32_t D_[2];
    struct timespec lastcalcat;
    int lastcalctarget;
    float pressure;
    float temperature;
} ms5611_conn_t;

extern int ms5611_open(ms5611_conn_t* conn, const char* i2cdev, uint8_t address, int timeout_10ms);
extern int ms5611_init(ms5611_conn_t* conn);
extern int ms5611_convpressure(ms5611_conn_t* conn);
extern int ms5611_convtemperature(ms5611_conn_t* conn);
extern int ms5611_readadc(ms5611_conn_t* conn);
extern float ms5611_getpressure(ms5611_conn_t* conn);
extern float ms5611_gettemperature(ms5611_conn_t* conn);
extern float ms5611_calcheight(float pressure, float temperature);

#endif
