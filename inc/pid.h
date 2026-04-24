#ifndef PID_H
#define PID_H
#include "stm8s.h"
#include "imu.h"

// PID Values
#define PID_ROLL_P 90
#define PID_ROLL_I 30
#define PID_ROLL_D 20

#define PID_PITCH_P 90
#define PID_PITCH_I 30
#define PID_PITCH_D 20

#define PID_YAW_P 100
#define PID_YAW_I 20
#define PID_YAW_D 0
// Anti-windup limit (prevents I-term from growing too large)
#define I_LIMIT 400

int16_t axisPID[3];
static int32_t errorI[3] = {0, 0, 0};
static int16_t lastError[3] = {0, 0, 0};

void runPID(int16_t setpoint[3], int16_t currentAngle[3]);

#endif // !PID_H
