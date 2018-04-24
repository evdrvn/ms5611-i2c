#include "ms5611-i2c.h"
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>

typedef uint8_t byte;

//#define MS5611_TRACE 
#ifdef MS5611_TRACE
#define TRACE(fmt, ...) printf("##TRACE## " fmt "\n", ##__VA_ARGS__); fflush(stdout)
#else
#define TRACE(fmt, ...) (void)sizeof(printf(fmt,##__VA_ARGS__))
#endif

int ms5611_writebytes(ms5611_conn_t* conn, uint8_t *data, uint8_t count){
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];
    int ret;

    messages[0].addr = conn->address;
    messages[0].flags = 0;
    messages[0].len = count;
    messages[0].buf = (unsigned char *)data;

    packets.msgs = messages;
    packets.nmsgs = 1;

    ret = ioctl(conn->file, I2C_RDWR, &packets);

#ifdef MS5611_TRACE
    int _trace_i;
    printf("write -> ");
    for(_trace_i = 0; _trace_i < count; _trace_i++){ 
        printf(" %02x", data[_trace_i]); 
    }
    printf("\n");
#endif

    return ret;
}

int ms5611_readbytes(ms5611_conn_t* conn, uint8_t reg, uint8_t *dest, uint8_t count){
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    int ret = 0;
    /* write the register we want to read from */
    messages[0].addr = conn->address; 
    messages[0].flags = 0;        
    messages[0].len = 1;         
    messages[0].buf = (unsigned char *)&reg;

    /* read */
    messages[1].addr = conn->address;  
    messages[1].flags = I2C_M_RD;
    messages[1].len = count;    
    messages[1].buf = (unsigned char *)dest;

    packets.msgs = messages;
    packets.nmsgs = 2;

    ret = ioctl(conn->file, I2C_RDWR, &packets);
   
#ifdef MS5611_TRACE
    int _trace_i;
    printf("read [%02x] -> ", reg);
    for(_trace_i = 0; _trace_i < count; _trace_i++){ 
        printf(" %02x", dest[_trace_i]); 
    }
    printf("\n");
#endif

    return ret;
}

static int write8(ms5611_conn_t* conn, byte value){
    return ms5611_writebytes(conn, &value, 1);
}

int ms5611_open(ms5611_conn_t* conn, const char* i2cdev, uint8_t address, int timeout_10ms){
    int fd, ret;

    if ((fd = open (i2cdev, O_RDWR)) < 0)
        return fd;
    if ((ret = ioctl (fd, I2C_SLAVE, address)) < 0)
        return ret;
    if ((ret = ioctl (fd, I2C_TIMEOUT, timeout_10ms)) < 0)
        return ret;
 
    conn->file = fd;
    conn->address = address;

    return fd;
}

static struct timespec timespec_add (struct timespec* a, struct timespec* b){
    struct timespec ret;
    ret = *a;
    ret.tv_sec  += b->tv_sec;
    ret.tv_nsec += b->tv_nsec;
    long nsec_is_1sec = 1000L * 1000L * 1000L;
    while(nsec_is_1sec <= ret.tv_nsec){
        ret.tv_sec++;
        ret.tv_nsec -= nsec_is_1sec;
    }
    return ret;
}

int ms5611_init(ms5611_conn_t* conn){
    uint8_t buffer[2];
    int i;
    int ret;

    conn->lastcalcat.tv_sec = 0;
    conn->lastcalcat.tv_nsec = 0;
    conn->lastcalctarget = -1;
    conn->D_[0] = 0xFFFFFFFF;
    conn->D_[1] = 0xFFFFFFFF;

    for(i = 0; i < 6; i++){ 
        ret = ms5611_readbytes(conn, MS5611_PROM_ADDR + 2 * i, buffer, 2);
        if(!ret) break;
        conn->C_[i] = (((int16_t)buffer[0]) << 8) | (int16_t)buffer[1];
#ifdef MS5611_TRACE
        printf("C_[%d]: %d\n", i, conn->C_[i]);
#endif
    }

    if ((ret = ms5611_convpressure(conn)) < 0) return ret;
    if ((ret = ms5611_readadc(conn)) < 0) return ret;
    if ((ret = ms5611_convtemperature(conn)) < 0) return ret;
    if ((ret = ms5611_readadc(conn)) < 0) return ret;

    return ret;
}

static struct timespec ms5611_getcalctimeout(ms5611_conn_t* conn){
    struct timespec timeout = {0, 10 * 1000 * 1000};
    timeout = timespec_add(&conn->lastcalcat, &timeout);
    return timeout;
}

static int waittimer(struct timespec *timeout){
    int ret;
    while(true){
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, timeout, NULL);
        if(ret == EINTR) continue;
        else if(ret != 0) return -1;
        else if(ret == 0) break;
    }
    return ret;
}

int ms5611_convpressure(ms5611_conn_t* conn){
    struct timespec timeout;
    int ret;
    timeout = ms5611_getcalctimeout(conn);
    if((ret = waittimer(&timeout)) == 0) 
        ret = write8(conn, MS5611_CMD_ADC_CONV + MS5611_CMD_ADC_D1 + MS5611_CMD_ADC_4096);
    
    if(ret >= 0){ 
        clock_gettime(CLOCK_MONOTONIC, &conn->lastcalcat); 
        conn->lastcalctarget = 0;
    }else conn->lastcalctarget = -1;
    return ret;
}

int ms5611_convtemperature(ms5611_conn_t* conn){
    struct timespec timeout;
    int ret;
    timeout = ms5611_getcalctimeout(conn);
    if((ret = waittimer(&timeout)) == 0) 
        ret = write8(conn, MS5611_CMD_ADC_CONV + MS5611_CMD_ADC_D2 + MS5611_CMD_ADC_4096);
    
    if(ret >= 0){ 
        clock_gettime(CLOCK_MONOTONIC, &conn->lastcalcat); 
        conn->lastcalctarget = 1;
    }else conn->lastcalctarget = -1;
    return ret;
}

static void calc(ms5611_conn_t* conn){
    int32_t dT,TEMP;
    int64_t OFF,SENS;
    int64_t T2,OFF2,SENS2;
    int32_t P;
    
    dT   = (int32_t)(conn->D_[1] - ((int32_t)conn->C_[4] << 8));
    TEMP = 2000 + ((dT * (int64_t)conn->C_[5]) >> 23);
    OFF  = (((int64_t)conn->C_[1]) << 16) + (((int64_t)conn->C_[3] * dT) >> 7);
    SENS = (((int64_t)conn->C_[0]) << 15) + (((int64_t)conn->C_[2] * dT) >> 8);
    P    = (((conn->D_[0] * SENS) >> 21) - OFF) >> 15;

    if (TEMP < 2000) {
        T2    = (dT * dT) >> 31;
        OFF2  = 5 * (TEMP - 2000) * (TEMP - 2000) >> 1;
        SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) >> 2;
        TEMP = TEMP - T2;
        OFF  = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    conn->pressure = P / 100.0;
    conn->temperature = TEMP / 100.0;
}

int ms5611_readadc(ms5611_conn_t* conn){
    int ret;
    uint8_t buffer[3];
    struct timespec timeout;
    if(conn->lastcalctarget == -1) return -1;
    timeout = ms5611_getcalctimeout(conn);
    if((ret = waittimer(&timeout)) == 0)
        ret = ms5611_readbytes(conn, MS5611_ADC_ADDR, buffer, 3);
    
    if(ret > 0){
        conn->D_[conn->lastcalctarget] = 0; 
        conn->D_[conn->lastcalctarget] = (((int32_t)buffer[0]) << 16) | (((int32_t)buffer[1]) << 8) | (int32_t)buffer[2];
        if(conn->D_[0] != 0xFFFFFFFF && conn->D_[1] != 0xFFFFFFFF) calc(conn);
        conn->lastcalctarget = -1;
    }
#ifdef MS5611_TRACE
    printf("ret = %d, buffer=%d\n", ret, buffer[0] * 65536 + buffer[1] * 256 + buffer[2]);
    printf("D_[0](%p) = %d\n", &conn->D_[0], conn->D_[0]);
    printf("D_[1](%p) = %d\n", &conn->D_[1], conn->D_[1]); 
#endif
    return ret;
}

float ms5611_getpressure(ms5611_conn_t* conn){
    return conn->pressure;
}

float ms5611_gettemperature(ms5611_conn_t* conn){
    return conn->temperature;
}

float ms5611_calcheight(float pressure, float temperature){
    return 153.8 * (temperature + 273.15) * (pow((1013.25 / pressure), 0.1902) - 1.0);
}
