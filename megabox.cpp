{\rtf1\ansi\ansicpg1252\cocoartf2822
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\fnil\fcharset0 HelveticaNeue;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\paperw11900\paperh16840\margl1440\margr1440\vieww11520\viewh8400\viewkind0
\deftab560
\pard\pardeftab560\slleading20\pardirnatural\partightenfactor0

\f0\fs26 \cf0 \
\
#include <Bela.h>\
#include <cmath>\
#include <fcntl.h>\
#include <linux/i2c-dev.h>\
#include <sys/ioctl.h>\
#include <unistd.h>\
#include <cstdlib>\
#include <algorithm>\
\
#define MPU6050_ADDR 0x68\
#define NUM_BUFFERS 3\
#define RECORD_SECONDS 2.5\
#define MAX_SAMPLES_PER_BUFFER 110250\
\
int i2cFile = -1;\
float inverseSampleRate;\
int samplesPerBuffer;\
float buffers[NUM_BUFFERS][MAX_SAMPLES_PER_BUFFER];\
float playheads[NUM_BUFFERS] = \{0\};\
int recordBuffer = 0;\
int recordIndex = 0;\
\
int16_t ax, ay, az, gx, gy, gz;\
\
void setupMPU6050() \{\
    i2cFile = open("/dev/i2c-1", O_RDWR);\
    if (i2cFile < 0)\
        return;\
\
    if (ioctl(i2cFile, I2C_SLAVE, MPU6050_ADDR) < 0) \{\
        close(i2cFile);\
        i2cFile = -1;\
        return;\
    \}\
\
    unsigned char buf[2] = \{0x6B, 0\};\
    write(i2cFile, buf, 2);\
\}\
\
void readMPU6050() \{\
    if(i2cFile < 0)\
        return;\
\
    unsigned char reg = 0x3B;\
    write(i2cFile, &reg, 1);\
    unsigned char data[14];\
    if(read(i2cFile, data, 14) != 14)\
        return;\
\
    ax = (data[0] << 8) | data[1];\
    ay = (data[2] << 8) | data[3];\
    az = (data[4] << 8) | data[5];\
    gx = (data[8] << 8) | data[9];\
    gy = (data[10] << 8) | data[11];\
    gz = (data[12] << 8) | data[13];\
\}\
\
float mapf(float x, float inMin, float inMax, float outMin, float outMax) \{\
    return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);\
\}\
\
// Subtle exponential pitch control, same range as before\
float gyroToSpeed(int16_t g) \{\
    float norm = std::min((float)std::abs(g), 2000.0f) / 2000.0f;\
    float curved = pow(norm, 1.5f);  // subtle exponential mapping\
    return mapf(curved, 0.0f, 1.0f, 0.05f, 1.0f);  // same range as before\
\}\
\
float waveshape(float in, float amount) \{\
    return tanh(in * amount);\
\}\
\
bool setup(BelaContext *context, void *userData) \{\
    inverseSampleRate = 1.0 / context->audioSampleRate;\
    samplesPerBuffer = std::min((int)(RECORD_SECONDS * context->audioSampleRate), MAX_SAMPLES_PER_BUFFER);\
\
    usleep(500000);  // wait 0.5s at boot to give I2C time to settle\
    setupMPU6050();\
    return true;\
\}\
\
void render(BelaContext *context, void *userData) \{\
    static int sensorCounter = 0;\
    if (++sensorCounter >= 44) \{\
        readMPU6050();\
        sensorCounter = 0;\
    \}\
\
    float speeds[NUM_BUFFERS] = \{\
        gyroToSpeed(gx),\
        gyroToSpeed(gy),\
        gyroToSpeed(gz)\
    \};\
\
    float pans[NUM_BUFFERS] = \{\
        mapf(ax / 16384.0f, -1.5f, 1.5f, -1.0f, 1.0f),\
        mapf(ay / 16384.0f, -1.5f, 1.5f, -1.0f, 1.0f),\
        mapf(az / 16384.0f, -1.5f, 1.5f, -1.0f, 1.0f)\
    \};\
\
    for (unsigned int n = 0; n < context->audioFrames; ++n) \{\
        float in = audioRead(context, n, 0);\
\
        if(recordIndex < samplesPerBuffer)\
            buffers[recordBuffer][recordIndex] = in;\
        recordIndex++;\
        if (recordIndex >= samplesPerBuffer) \{\
            recordIndex = 0;\
            recordBuffer = (recordBuffer + 1) % NUM_BUFFERS;\
        \}\
\
        float outL = 0.0f;\
        float outR = 0.0f;\
\
        for (int i = 0; i < NUM_BUFFERS; ++i) \{\
            int idx = (int)playheads[i] % samplesPerBuffer;\
            int nextIdx = (idx + 1) % samplesPerBuffer;\
            float frac = playheads[i] - (int)playheads[i];\
            float raw = buffers[i][idx] * (1 - frac) + buffers[i][nextIdx] * frac;\
\
            int16_t gVal = (i == 0) ? gx : (i == 1) ? gy : gz;\
            float shapeAmt = mapf((float)std::abs(gVal), 0.0f, 2000.0f, 1.0f, 10.0f);\
            float samp = waveshape(raw, shapeAmt);\
\
            float pan = pans[i];\
            float left = samp * (1.0f - pan) * 0.5f;\
            float right = samp * (1.0f + pan) * 0.5f;\
\
            outL += left / NUM_BUFFERS;\
            outR += right / NUM_BUFFERS;\
\
            playheads[i] += speeds[i];\
            if (playheads[i] >= samplesPerBuffer)\
                playheads[i] -= samplesPerBuffer;\
        \}\
\
        float gain = 2.0f;\
        audioWrite(context, n, 0, outL * gain);\
        audioWrite(context, n, 1, outR * gain);\
    \}\
\}\
\
void cleanup(BelaContext *context, void *userData) \{\
    if (i2cFile >= 0)\
        close(i2cFile);\
\}\
}