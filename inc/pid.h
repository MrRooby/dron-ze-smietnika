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

typedef union {
  struct {
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
  };
  int16_t axis[3];
} AxisPID;

extern AxisPID axisPID;
static int32_t errorI[3] = {0, 0, 0};
static int16_t lastError[3] = {0, 0, 0};
static const uint8_t kp[3] = {PID_ROLL_P, PID_PITCH_P, PID_YAW_P};
static const uint8_t ki[3] = {PID_ROLL_I, PID_PITCH_I, PID_YAW_I};
static const uint8_t kd[3] = {PID_ROLL_D, PID_PITCH_D, PID_YAW_D};

void runPID(int16_t setpoint[3], int16_t currentAngle[3], uint16_t throttle);

#endif // !PID_H
