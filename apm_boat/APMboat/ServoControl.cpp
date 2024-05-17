#include "Rover.h"

/*
int16_t Rover::servoRamp(int16_t currentServoValue, int16_t targetServoValue, int16_t last_step) {
    int16_t distance = abs(currentServoValue - targetServoValue);
    int16_t smallDistance = 5;
    bool noNeedToMove = !(distance > smallDistance);
    if (noNeedToMove) {
        return targetServoValue;
    }

    int16_t maxIncrement = g.servo_increment;
    int16_t stepSize = 0;
    int16_t incrementShift = 0;

    //Stopping
    for (int16_t i = 0; i < 5; i++) {
        stepSize = maxIncrement >> incrementShift;
        if (stepSize > distance) {
            incrementShift++;
        }
    }
    if (stepSize > distance) {
        return targetServoValue;
    }

    //Starting
    if ((stepSize == maxIncrement)) {
        incrementShift = 0;
        for (int16_t i = 0; i < 5; i++) {
            stepSize = maxIncrement >> incrementShift;
            if (stepSize > (abs(last_step) + smallDistance)) {
                incrementShift++;
            }
        }
    }


    int16_t nextServoValue = currentServoValue;
    bool moving_up = currentServoValue < targetServoValue;
    bool moving_down = currentServoValue > targetServoValue;

    if (moving_up) {
        nextServoValue = currentServoValue + stepSize;
    } else if (moving_down) {
        nextServoValue = currentServoValue - stepSize;
    }

    return nextServoValue;
}*/

int16_t Rover::servoRamp(int16_t currentServoValue, int16_t targetServoValue, int16_t last_step)
{
    const int16_t distance = abs(currentServoValue - targetServoValue);
    const int16_t min_step = g.servo_increment >> 5;
    if (distance < min_step) {
        return targetServoValue;
    }

    int16_t stepSize = 0;
    const int16_t doubleLastStepSize = abs(last_step) << 1;

    if (last_step == 0) {
        stepSize = min_step;
    } else {
        stepSize = doubleLastStepSize;
        const bool isSmallDistance = distance < (g.servo_increment << 1);
        const bool lastStepWasBig = !((distance >> 1) > doubleLastStepSize);
        if (isSmallDistance && lastStepWasBig) {
            stepSize = distance >> 1;
        }
    }

    stepSize = limit_value(stepSize, g.servo_increment);

    int16_t nextServoValue = currentServoValue;

    const bool moving_up = currentServoValue < targetServoValue;
    if (moving_up) {
        nextServoValue = currentServoValue + stepSize;
    } else {
        nextServoValue = currentServoValue - stepSize;
    }

    return nextServoValue;
}

void Rover::init_servo_pids(void)
{
    sail_pid.kP(1);
    sail_pid.kI(0);
    sail_pid.kD(0.5f);
    sail_pid.imax(50);

    rudder_pid.kP(1);
    rudder_pid.kI(0);
    rudder_pid.kD(0.5f);
    rudder_pid.imax(50);
}