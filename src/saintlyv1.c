/*  
 * saintly.c - The most holy of wireless speaker
 * author: David Smerkous
 * date: 9/08/2018
 * license: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define MINIMP3_ONLY_MP3
#define MINIMP3_IMPLEMENTATION
#include "minimp3/minimp3.h"
#include "b64.h"

/*
 * GPIOS
 */
#define WAN_LED 1UL //The first led

/*
 * GPIO BLOCK
 * 
 * This gpio memory block is specific to the Atheros AR7242 processor
 * I used the following spec sheet for all of the GPIO regs https://datasheetspdf.com/pdf-file/912138/Atheros/AR7242/1
 * The APB base is the bus bridge initial block and the GPIO falls under the bus block
 */
#define BIT(X)   (1UL << (X))
#define APB_BASE  0x18000000 //The bus memory block
#define GPIO_BASE (APB_BASE + 0x00040000) //The gpio memory reg block
#define GPIO_SIZE 0x100 //The end of the gpio memory reg block
#define GPIO_ENABLE 0x00 //The first address is the gpio enable registry
#define GPIO_SET 0x08 //The gpio set registry [sets bit]
#define GPIO_CLEAR 0x10 //The gpio clear registry [clears bit]
#define GPIO_ON(X) *gpio_set_addr = X //Set the specified bit
#define GPIO_OFF(X) *gpio_clear_addr = X //Clear the specified bit

/*
 * SELF EXPLANATORY
 */
#define LOG(X) printf(X "\n")
#define ERROR(X) fprint(stderr, "Error [%d]: %s", __LINE__, X)
#define FATAL(X) do { error(X); exit(1); } while(0)
#define TARG void *
#define CUL const unsigned long
#define CUD const long double
#define CULT CUL *

/*
 * UDP SERVER
 */
#define BUFFER_SIZE 30000
#define PORT_NUMBER 5555

/*
 * AUDIO THREADING
 */
#define AUDIO_HZ 8000
#define AUDIO_BUFFER_SIZE AUDIO_HZ * 60 //Max audio frame buffers
#define LSAMPLE pthread_mutex_lock(&sample_mutex)
#define USAMPLE pthread_mutex_unlock(&sample_mutex)

struct params_s {
  pthread_cond_t done;
  volatile bool enable;
  volatile unsigned long pin, low, high;
};

struct frame_s {
  mp3dec_frame_info_t frameinfo;
  short *data;
};

typedef struct params_s params_t;
typedef struct frame_s frame_t;

#define SINE_SIZE 120
const sineWave[SINE_SIZE] = {
  0x32,0x35,0x37,0x3a,0x3c,0x3f,0x41,0x44,
  0x46,0x49,0x4b,0x4d,0x4f,0x51,0x53,0x55,
  0x57,0x59,0x5a,0x5c,0x5d,0x5f,0x60,0x61,
  0x62,0x62,0x63,0x63,0x64,0x64,0x64,0x64,
  0x64,0x63,0x63,0x62,0x62,0x61,0x60,0x5f,
  0x5d,0x5c,0x5a,0x59,0x57,0x55,0x53,0x51,
  0x4f,0x4d,0x4b,0x49,0x46,0x44,0x41,0x3f,
  0x3c,0x3a,0x37,0x35,0x32,0x2f,0x2d,0x2a,
  0x28,0x25,0x23,0x20,0x1e,0x1b,0x19,0x17,
  0x15,0x13,0x11,0xf,0xd,0xb,0xa,0x8,
  0x7,0x5,0x4,0x3,0x2,0x2,0x1,0x1,
  0x0,0x0,0x0,0x0,0x0,0x1,0x1,0x2,
  0x2,0x3,0x4,0x5,0x7,0x8,0xa,0xb,
  0xd,0xf,0x11,0x13,0x15,0x17,0x19,0x1b,
  0x1e,0x20,0x23,0x25,0x28,0x2a,0x2d,0x2f
};

static int mem;
static unsigned int pin = WAN_LED;
static volatile void *gpio_addr;
static volatile short *sample_data[AUDIO_BUFFER_SIZE];
static pthread_mutex_t sample_mutex;
static volatile unsigned int sample_index = -1;
static volatile unsigned int sample_size, source_index, *gpio_enable_addr, *gpio_set_addr, *gpio_clear_addr;
static volatile params_t fake_pwm_params;
static pthread_t t_audio_thread;

/*static void sync_pwm(TARG data, CULT pin, CULT high, CULT low, CULT updatePeriod) {
  pthread_mutex_lock()
}*/

static inline CUD val_re_map(CUD value, CUD omin, CUD omax, CUD nmin, CUD nmax) {
    return ((value - omin) / (omax - omin)) * (nmax - nmin) + nmin;
}

static inline void play(CUL pin, CUD frequency, CUD duty) {
  unsigned long cycles = 1000;
  for(unsigned long i = 0; i < cycles; i++) {
    if(i % 100 < duty) {
      GPIO_ON(pin);  
    } else {
      GPIO_OFF(pin);
    }
  }
}

static void *audio_thread(TARG data) {
  const params_t *shared_pwm = (params_t *) data;
  
  LOG("started the audio thread");
  for(;;) {
    LSAMPLE;
    if(sample_index == -1 || sample_index >= source_index) { //sample_index < source_index || source_index == 0) {
      USAMPLE;
      printf("waiting...\n");
      usleep(300000);
      continue;
    }

    //Copy the frame data to unlock the current sample data
    //frame_t frame_data = sample_data[sample_index];
    
    //sample_data[sample_index].data = NULL;

    clock_t start, end;
    start = clock();

    for(sample_index = 0; sample_index < source_index; sample_index++) {
      //if(sample_index % AUDIO_HZ == 0) printf("progress %d\n", (int) (((double) sample_index / (double) source_index) * 100));
      play(pin, (unsigned long) sample_data[sample_index], (long) 1000 * ((double) (short) sample_data[sample_index] / AUDIO_HZ));
      /*if(sample_data[sample_index]) {
        GPIO_ON(pin);
      } else {
        GPIO_OFF(pin);
      }*/

      //usleep(1000); //); //Account for loop delays
      //play(pin, abs(frame_data.data[i]), frame_data.frameinfo.hz);
      //printf("%d", pcm[i]);
      //fake_write_frequency()
    }

    end = clock();
    int diff = (int) ((double) end - start) / (CLOCKS_PER_SEC * 1000);
    printf("played about %d milliseconds worth of audio\n", diff);

    sample_index++;
    if(sample_index == AUDIO_BUFFER_SIZE) sample_index = 0;
    USAMPLE;

    /*
    if(shared_pwm->enable) {
      GPIO_ON(shared_pwm->pin);
      usleep(shared_pwm->high);
      GPIO_OFF(shared_pwm->pin);
      usleep(shared_pwm->low);
    } else {
      GPIO_OFF(shared_pwm->pin);
      usleep(10000);
    }*/
  }
}

static inline void fake_write_period(CUL pin, CUD period, CUD duty) {
  //Remap the duty cycle (0 to 100 (no smaller granularity than 1)) to the high period value
  CUL high_delay = val_re_map(duty, 0UL, 1000UL, 0UL, period); //The nanosecond high delay
  CUL low_delay = period - high_delay; //The nanosecond low delay

  fake_pwm_params.pin = pin;
  fake_pwm_params.enable = true;
  fake_pwm_params.high = high_delay;
  fake_pwm_params.low = low_delay;
}

static inline void fake_write_stop(CUL pin) {
  fake_pwm_params.pin = pin;
  fake_pwm_params.enable = false;
}

static inline void fake_write_frequency(CUL pin, CUD freq, CUD duty) {
  if(freq > 0) fake_write_period(pin, 1000000UL / freq, duty); //You want an even square wave so the duty is always at (50 %)
  else fake_write_stop(pin);
}

int main(int argc, char **argv) {
  LOG("starting saintly the most holy wireless framework by David Smerkous");

  //Open up the userspace memory map
  LOG("opening memory map");
  if((mem = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL("failed to open memory map file");

  //Map the bus addresses to the virtual memory space
  LOG("mapping memory page");
  gpio_addr = mmap(0, GPIO_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, GPIO_BASE);
  if(gpio_addr == (void *) -1) FATAL("failed to map page size of 4096");
  
  //Map the virtual pointers
  gpio_enable_addr = gpio_addr + GPIO_ENABLE;
  gpio_set_addr = gpio_addr + GPIO_SET;
  gpio_clear_addr = gpio_addr + GPIO_CLEAR;

  //Enable the gpio output
  LOG("enabling the gpio");
  *gpio_enable_addr = pin;

  LOG("starting audio queue thread");
  pthread_mutex_init(&sample_mutex, NULL);
  pthread_create(&t_audio_thread, NULL, audio_thread, &fake_pwm_params);

  //Create the server socket
  int sockfd;
  if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) FATAL("couldn't create socket");

  //Set the socket options to allow rebinding of the same address
  const int optval = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &optval, sizeof(int));

  //Build the address
  struct sockaddr_in serveraddr;
  struct sockaddr_in clientaddr;
  bzero(&serveraddr, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
  serveraddr.sin_port = htons((unsigned short) PORT_NUMBER);

  //Bind to socket
  if(bind(sockfd, (struct sockaddr *) &serveraddr, sizeof(serveraddr)) < 0) FATAL("couldn't bind the server to port");

  //unsigned int sine = 0;
  int recvn, sendn;
  const unsigned int clientlen = sizeof(clientaddr);
  unsigned long recv_frames = 0;
  const char *next_chunk = "next";
  
  //Enable the mp3 decoder
  static mp3dec_t mp3d;
  mp3dec_init(&mp3d);

  //Set some static mp3 frame data
  mp3dec_frame_info_t info;
  info.channels = 1;
  info.hz = AUDIO_HZ;

  for(;;) {
    unsigned char *buf = malloc(BUFFER_SIZE);
    recvn = recvfrom(sockfd, buf, BUFFER_SIZE, 0, (struct sockaddr *) & clientaddr, &clientlen);
    
    if(strncmp(buf, "play", 4) == 0) {
      LOG("playing audio...");
      LSAMPLE;
      sample_index = 0;
      USAMPLE;
      free(buf);
      continue;
    }

    printf("got new frame data of size %d (total: %d)\n", recvn, source_index);

    int f_ind = 0;
    short frame_data[5000];
    char *pt = strtok(buf, ",");
    while (pt != NULL) {
      //frame_data[f_ind] = strtol(pt, NULL, 10);
      //double theta = (double) (f_ind * (strtod(pt, NULL) / (double) AUDIO_HZ));
      //printf("%.2f\n", theta);
      //frame_data[f_ind] = ((fmod(theta, 1) / AUDIO_HZ) < 50) ? 1 : 0; 
      frame_data[f_ind] = abs(strtol(pt, NULL, 10));
      f_ind++;
      //The sample buffer data
      /*int samples;
      short pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
      unsigned char *decoded_frame = b64_decode(pt, strlen(pt));
      samples = mp3dec_decode_frame(&mp3d, decoded_frame, strlen(decoded_frame), pcm, &info);
      free(decoded_frame);*/

      //printf("SAMPLES %d BYTES %d", samples, info.frame_bytes);
      pt = strtok(NULL, ",");
    }

    info.frame_bytes = f_ind;

    if(f_ind > 0) {
      printf("decoded frame! samples %d hertz %d\n", f_ind, info.hz);
      frame_t new_frame;
      //short frame_data_f[f_ind];
      
      LSAMPLE;
      memcpy(&sample_data[source_index], &frame_data, f_ind);
      new_frame.frameinfo = info;
      new_frame.frameinfo.frame_bytes = f_ind;
      //new_frame.data = frame_data_f;
      //printf("LOLOL %d", sizeof(new_frame.data));

      //LSAMPLE;
      //sample_data[source_index] = new_frame;


      source_index += f_ind;
      if(source_index == AUDIO_BUFFER_SIZE) source_index = 0;
      USAMPLE;
    } else {
      printf("failed to decode frame!\n");
    }

    /*
    int samples;
    short pcm[5000]; //MINIMP3_MAX_SAMPLES_PER_FRAME];
    samples = mp3dec_decode_frame(&mp3d, buf, recvn, pcm, &info);

    printf("samples %d bytes %d\n", samples, info.frame_bytes);
    if(samples > 0 && info.frame_bytes > 0) {
      printf("decoded frame! samples %d hertz %d\n", samples, info.hz);
      frame_t new_frame;
      short frame_data[samples];
      memcpy(&frame_data, pcm, samples);
      new_frame.frameinfo = info;
      new_frame.data = frame_data;

      LSAMPLE;
      sample_data[source_index] = new_frame;
      sample_size++;
      USAMPLE;

      source_index++;
      if(source_index == AUDIO_BUFFER_SIZE) source_index = 0;
    } else {
      printf("failed to decode frame!\n");
    }*/

    //free the allocated buffer
    free(buf);

    sendn = sendto(sockfd, next_chunk, sizeof(next_chunk), 0, (struct sockaddr *) &clientaddr, clientlen);
    if(sendn < 0) {
      fprintf(stderr, "failed to send the next chunk request");
    }

    /*for(int n = 0; n < (sizeof(melody) / sizeof(int)); n++) {
      fake_write_frequency(pin, melody[n]);
      long noteLength = (1000 / tempo[n]);
      usleep(1000 * noteLength);
      usleep(1000 * (noteLength * 1.3));
      fake_write_stop(pin);
    }*/

    //fake_write_frequency(pin, 100, 50); //sineWave[sine++ % SINE_SIZE]);
    //usleep(100000);
    //usleep(10000);
     //fake_write_frequency(pin, i);
    /*for(unsigned long i = 0; i < 2000; i += 100) {
        usleep(300000);
        fake_write_frequency(pin, i);
    }

    for(unsigned long i = 2000; i > 0; i -= 100) {
        usleep(300000);
        fake_write_frequency(pin, i);
    }*/

    //printf("running...\n");
  }

  return 0;
}