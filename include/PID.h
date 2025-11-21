// PID.h
#ifndef PID_h
#define PID_h

#include <Arduino.h>

class PID
{
private:
    float Kp, Ki, Kd;
    float integrator;
    float prev_error;
    float output_min, output_max;
    float integrator_min, integrator_max;
    bool first_run;

public:
    PID(float p = 0.0, float i = 0.0, float d = 0.0)
    {
        Kp = p;
        Ki = i;
        Kd = d;
        integrator = 0.0;
        prev_error = 0.0;
        output_min = -1000.0;
        output_max = 1000.0;
        integrator_min = -1000.0;
        integrator_max = 1000.0;
        first_run = true;
    }

    void setTunings(float p, float i, float d)
    {
        Kp = p;
        Ki = i;
        Kd = d;
    }

    void setOutputLimits(float minVal, float maxVal)
    {
        if (minVal >= maxVal)
            return;
        output_min = minVal;
        output_max = maxVal;
        // also constrain integrator bounds to be within output bounds by default
        integrator_min = minVal * 10.0; // allow integral to exceed output in some margin
        integrator_max = maxVal * 10.0;
    }

    void setIntegratorLimits(float minVal, float maxVal)
    {
        if (minVal >= maxVal)
            return;
        integrator_min = minVal;
        integrator_max = maxVal;
    }

    void reset()
    {
        integrator = 0.0;
        prev_error = 0.0;
        first_run = true;
    }

    // Compute PID output. dt in seconds.
    float compute(float setpoint, float measurement, float dt)
    {
        if (dt <= 0.0)
            return 0.0;

        float error = setpoint - measurement;

        // Proportional
        float P = Kp * error;

        // Integral (trapezoidal integration)
        integrator += 0.5f * (error + prev_error) * dt * Ki;

        // anti-windup clamp
        if (integrator > integrator_max)
            integrator = integrator_max;
        if (integrator < integrator_min)
            integrator = integrator_min;

        // Derivative (use derivative on measurement to reduce derivative kick? here on error)
        float D = 0.0f;
        if (first_run)
        {
            D = 0.0f;
            first_run = false;
        }
        else
        {
            D = Kd * (error - prev_error) / dt;
        }

        float output = P + integrator + D;

        // output clamp
        if (output > output_max)
            output = output_max;
        if (output < output_min)
            output = output_min;

        prev_error = error;
        return output;
    }

    // getters for debugging/tuning
    float getP() { return Kp; }
    float getI() { return Ki; }
    float getD() { return Kd; }
    float getIntegrator() { return integrator; }
};

#endif
