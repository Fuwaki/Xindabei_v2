
#include "motor.h"
#include "pid.h"
#include "tim.h"
#include <stdint.h>

// 使用差分法扩展编码器值：记录上一原始计数并根据差值调整扩展计数，避免依赖更新中断
static int32_t encoder1_extended = 0;
static uint16_t encoder1_last_raw = 0;
static uint8_t encoder1_inited = 0;

static int32_t encoder2_extended = 0;
static uint16_t encoder2_last_raw = 0;
static uint8_t encoder2_inited = 0;

// 速度pid
static PIDController motor1_speed_pid;
static PIDController motor2_speed_pid;

// 电流pid
static PIDController motor1_current_pid;
static PIDController motor2_current_pid;

// 测速pll
static EncPLL motor1_pll;
static EncPLL motor2_pll;

static int32_t motor_1_speed_setpoint = 0;
static int32_t motor_2_speed_setpoint = 0;

static int32_t motor_1_current = 0;
static int32_t motor_1_current_setpoint = 0;

static int32_t motor_2_current = 0;
static int32_t motor_2_current_setpoint = 0;

// 电机模块初始化
void MotorInit() {

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 60000);
  EncPLL_Init(&motor1_pll, 10.0f, 2.0f, 0.01f);
  EncPLL_Init(&motor2_pll, 10.0f, 2.0f, 0.01f);

  PID_Init(&motor1_speed_pid, PID_MODE_POSITIONAL, 30.0f, 10.0f, 0.5f, 0.5f,
           0.01f);
  PID_SetOutputLimit(&motor1_speed_pid, 0, 10000.0f);

  PID_Init(&motor2_speed_pid, PID_MODE_POSITIONAL, 30.0f, 10.0f, 0.5f, 0.5f,
           0.01f);
  PID_SetOutputLimit(&motor1_speed_pid, 0, 10000.0f);

  PID_Init(&motor1_current_pid, PID_MODE_POSITIONAL, 30.0f, 10.0f, 0.5f, 0.5f,
           0.01f);
  PID_SetOutputLimit(&motor1_speed_pid, 0, 10000.0f);

  PID_Init(&motor2_current_pid, PID_MODE_POSITIONAL, 30.0f, 10.0f, 0.5f, 0.5f,
           0.01f);
  PID_SetOutputLimit(&motor1_speed_pid, 0, 10000.0f);
}

void SetTargetMotorSpeed(int32_t motor_1, int32_t motor_2) {
  motor_1_speed_setpoint = motor_1;
  motor_2_speed_setpoint = motor_2;
}
void SetTargetMotorCurrent(int32_t motor_1, int32_t motor_2) {
  motor_1_current_setpoint = motor_1;
  motor_2_current_setpoint = motor_2;
}
void CurrentLoopTimerHandler() {

  float output_1 = PID_Update_Positional(&motor1_current_pid, motor_1_current_setpoint,
                                         motor_1_current);
  // printf("%d.,%d\n", (int)(output), (int)filtered_current);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,
                        ((uint16_t)(65535 * output_1 / 10000.0)));
}
void SpeedLoopHandler() {
  // float speed = EncPLL_Update(&motor_pll, (uint32_t)Get_Encoder1_Count());
  // float output = PID_Update_Positional(&motor_pid, 3000 + 1500 * sinf(omega),
  //                                      speed); // 减小 setpoint
  //                                              // 
  // motor_a_current_setpoint = (int32_t)(output * 6.0f);

  // printf("%ld,%d,%d,%d\n", Get_Encoder1_Count(), (int)speed, (int)output,
  //        (int)(3000 + 1500 * sinf(omega)));
}
void CurrentLoopInit() {
  // PID_Init(&motor_pid, PID_MODE_POSITIONAL, 0., 0.3f, 0.0f, 0.01,
  //          0.001f); // 减小 Kf
  // PID_SetOutputLimit(&motor_pid, 0, 10000.0f);
}
// 处理单次读取，利用差分纠正溢出（前提：两次读取之间的真实增量 < 32768）
static inline int32_t DiffUpdate(uint16_t raw, uint16_t *last_raw,
                                 uint8_t *inited, int32_t *extended) {
  if (!*inited) {
    *last_raw = raw;
    *extended = 0;
    *inited = 1;
    return *extended;
  }
  int32_t diff = (int32_t)raw - (int32_t)(*last_raw);
  // 处理正向溢出 (raw 小, last 大) 或反向溢出
  if (diff > 32767) {
    diff -= 65536; // 说明发生了反向 wrap（raw 比 last 小很多）
  } else if (diff < -32768) {
    diff += 65536; // 说明发生了正向 wrap（raw 比 last 大很多为负大值）
  }
  *extended += diff;
  *last_raw = raw;
  return *extended;
}

int32_t Get_Encoder1_Count(void) {
  int32_t val;
  __disable_irq();
  uint16_t raw = (uint16_t)(htim1.Instance->CNT);
  val =
      DiffUpdate(raw, &encoder1_last_raw, &encoder1_inited, &encoder1_extended);
  __enable_irq();
  return val;
}

int32_t Get_Encoder2_Count(void) {
  int32_t val;
  __disable_irq();
  uint16_t raw = (uint16_t)(htim3.Instance->CNT);
  val =
      DiffUpdate(raw, &encoder2_last_raw, &encoder2_inited, &encoder2_extended);
  __enable_irq();
  return val;
}

void EncPLL_Init(EncPLL *pll, float kp, float ki, float dt) {
  pll->kp = kp;
  pll->ki = ki;
  pll->dt = dt;
  pll->theta = 0.0f;
  pll->omega = 0.0f;
  // 初始化内部 PID 为增量式，禁用 D
  PID_Init(&pll->pid, PID_MODE_INCREMENTAL, kp, ki, 0.0f, 0.0f, dt);
  // 限幅可选：这里设置一个较大的范围，用户可后续调整
  PID_SetOutputLimit(&pll->pid, -1e6f, 1e6f);
}
float EncPLL_Update(EncPLL *pll, int32_t now_theta) {
  // 预测角度（保持原风格，不进行角度 wrap）
  float theta_pred = pll->theta + pll->omega * pll->dt;
  // 使用增量式 PID：setpoint=now_theta, measurement=theta_pred
  float new_omega =
      PID_Update_Incremental(&pll->pid, (float)now_theta, theta_pred);
  pll->omega = new_omega;  // omega 与 PID 输出绑定
  pll->theta = theta_pred; // 仅推进，不额外相位比例修正
  return pll->omega;
}