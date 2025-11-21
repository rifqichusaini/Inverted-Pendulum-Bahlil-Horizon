// SERVO.h
#ifndef SERVO_h
#define SERVO_h

#include <Arduino.h>

class SERVO
{
private:
    int pin;
    int channel;
    int freq;
    int resolution;
    int minPulseUs;
    int maxPulseUs;

public:
    SERVO()
    {
        pin = -1;
        channel = 0;
        freq = 50;
        resolution = 16;
        minPulseUs = 500;
        maxPulseUs = 2500;
    }

    void attachPin(int p, int ch = 0)
    {
        pin = p;
        channel = ch;
        ledcSetup(channel, freq, resolution);
        ledcAttachPin(pin, channel);
    }

    void begin() {}

    void writeAngle(int angle)
    {
        if (angle < 0)
            angle = 0;
        if (angle > 180)
            angle = 180;
        int pulse = map(angle, 0, 180, minPulseUs, maxPulseUs);
        double periodUs = 1000000.0 / (double)freq;
        double dutyRatio = (double)pulse / periodUs;
        uint32_t maxDuty = ((1UL << resolution) - 1);
        uint32_t duty = (uint32_t)(dutyRatio * (double)maxDuty + 0.5);
        if (duty > maxDuty)
            duty = maxDuty;
        ledcWrite(channel, duty);
    }
};

#endif
