#include <ms5611-i2c.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <evdsptc.h>
#include <math.h>

#define DEFAULT_I2C_DEVICE "/dev/i2c-1"
#define NS_AS_SEC (1000 * 1000 * 1000)
#define INFORM_INTERVAL_NS (1000 * 1000 * 1000)
#define READ_INTERVAL_HZ (50)
#define READ_INTERVAL_NS (NS_AS_SEC / READ_INTERVAL_HZ)

static int running = true;
static float p = 0.0;
static float t = 0.0;
static float h = 0.0;
static int initstep = 0;

static bool periodic_read(evdsptc_event_t* event){
    ms5611_conn_t* conn;
    conn = (ms5611_conn_t*)evdsptc_event_getparam(event);
    float h_; 
    
    if(conn->lastcalctarget == 0){
        ms5611_convtemperature(conn);
        ms5611_readadc(conn);
        if(initstep < 2){
            initstep++;
            t = ms5611_gettemperature(conn);
        } else t = t * 0.999 + 0.001 * ms5611_gettemperature(conn);
    }else{
        ms5611_convpressure(conn);
        ms5611_readadc(conn);
        p = ms5611_getpressure(conn);
    }
    h_ = ms5611_calcheight(p, t);
    if(initstep < 2){
        initstep++;
        h = h_;
    } else h = h * 0.95 + 0.05 * h_;
    
    return !running;
}

static bool periodic_inform(evdsptc_event_t* event){
    printf("p = %f, t = %f, h = %f\n", p, t, h);
    return false;
}

static void error(const char* message, const char* target){
    char buf[BUFSIZ];
    snprintf(buf, BUFSIZ - 1, "%s in \"%s\" ", message, target);
    perror(buf);
    exit(-1);
}

int main(int argc, char *argv[]){
    ms5611_conn_t conn;
    const char* dev = DEFAULT_I2C_DEVICE;
    pthread_t th;
    pthread_mutexattr_t mutexattr;
    struct sched_param param;
    evdsptc_context_t read_ctx, inform_ctx;
    evdsptc_event_t read_ev, inform_ev;
    struct timespec read_interval = { read_interval.tv_sec = 0, read_interval.tv_nsec = READ_INTERVAL_NS};
    struct timespec inform_interval = { inform_interval.tv_sec = 0, inform_interval.tv_nsec = INFORM_INTERVAL_NS};
    
    if(argc > 1) dev = argv[1];
    if(0 > ms5611_open(&conn, dev, MS5611_ADDRESS_B, 2)) error("open error", dev);
    if(0 > ms5611_init(&conn)) error("init error", dev);

    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
    evdsptc_setmutexattrinitializer(&mutexattr);

    evdsptc_create_periodic(&read_ctx, NULL, NULL, NULL, &read_interval);

    th = evdsptc_getthreads(&read_ctx)[0];
    param.sched_priority = 70;
    
    if(0 != pthread_setschedparam(th, SCHED_RR, &param)){
        printf("\nwarning : you get better performance to run this example as root via RT-Preempt.\n");
    }
    
    evdsptc_create_periodic(&inform_ctx, NULL, NULL, NULL, &inform_interval);
    th = evdsptc_getthreads(&inform_ctx)[0];
    param.sched_priority = 30;
    pthread_setschedparam(th, SCHED_RR, &param);

    evdsptc_event_init(&read_ev, periodic_read, &conn, false, NULL);
    evdsptc_post(&read_ctx, &read_ev);

    evdsptc_event_init(&inform_ev, periodic_inform, &conn, false, NULL);
    evdsptc_post(&inform_ctx, &inform_ev);

    evdsptc_event_waitdone(&read_ev);
    evdsptc_destory(&read_ctx, true);
    evdsptc_destory(&inform_ctx, true);

    return 0;
}
