#include <math.h>
#include <avr/eeprom.h>
#include <IRSendRev.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Encoder.h>


#define IR_LEN                4
#define K_UP                  0
#define K_DOWN                1
#define K_P                   2
#define K_M                   3
#define K_OK                  4
#define K_STOP                5
#define K_Q                   6
#define K_REV                 7
#define K_FWD                 8
#define K_PREV                9
#define K_NEXT                10
#define K_BACK                11
#define K_MUTE                12
#define K_MENU                13
#define K_LAST                14

// pins
#define PIN_IR                   A1
#define PIN_BUZZER               A2

#define PIN_EN1                  6
#define PIN_COM11                7
#define PIN_COM12                8
#define PIN_EN2                  9
#define PIN_COM21                10
#define PIN_COM22                11

#define PIN_ENC1_A               2
#define PIN_ENC1_B               5
#define PIN_ENC2_A               3
#define PIN_ENC2_B               4

// mode
#define LEFT                     0x1
#define RIGHT                    0x2
#define DUAL                     0x3

// others
#define FULL_SPEED              30
#define FULL_SPEED_THRES        500
#define DEBUG_SER Serial

struct PIDdata
{
  float lastPosition;
  float previousPIDTime;
  float integratedError;
  float lastDer;
  float lastErr;
};
struct PIDdata pid1 = { 0 };
struct PIDdata pid2 = { 0 };

float P = 15.0f;
float I = 10.0f;
float D = 0.0f;
float iLimit = 15.0f;


const char keys[K_LAST][IR_LEN] =
{
  { 0x0, 0x26, 0x80, 0x7f },
  { 0x0, 0x26, 0xe0, 0x1f },
  { 0x0, 0x26, 0xd0, 0x2f },
  { 0x0, 0x26, 0xb0, 0x4f },
  { 0x0, 0x26, 0x20, 0xdf },
  { 0x0, 0x26, 0x40, 0xbf },
  { 0x0, 0x26, 0x00, 0xff },
  { 0x0, 0x26, 0xc0, 0x3f },
  { 0x0, 0x26, 0xa0, 0x5f },
  { 0x0, 0x26, 0x60, 0x9f },
  { 0x0, 0x26, 0x10, 0xEf },
  { 0x0, 0x26, 0x90, 0x6f },
  { 0x0, 0x26, 0x50, 0xAf },
  { 0x0, 0x26, 0x30, 0xCf },
};

int key = -1;
int mode = DUAL;
bool calib_mode = false;
uint32_t last_pid_time;

struct info
{
  int32_t cur_pos, target_pos, total_len;
  int32_t cur_speed, target_speed;
  int32_t last_pos;
  bool first_stop_flag;
};

struct info info1 = { 0 };
struct info info2 = { 0 };



Encoder encoder1(PIN_ENC1_A, PIN_ENC1_B);
Encoder encoder2(PIN_ENC2_A, PIN_ENC2_B);


void setup()
{
  DEBUG_SER.begin(115200);
  IR.Init(PIN_IR);

  pinMode(PIN_EN1, OUTPUT);
  pinMode(PIN_EN2, OUTPUT);
  pinMode(PIN_COM11, OUTPUT);
  pinMode(PIN_COM12, OUTPUT);
  pinMode(PIN_COM21, OUTPUT);
  pinMode(PIN_COM22, OUTPUT);

  //save_pos1_to_eeprom(0);
  //save_len1_to_eeprom(10000);
  //save_pos2_to_eeprom(0);
  //save_len2_to_eeprom(10000);

  restore_len1_from_eeprom(&info1.total_len);
  restore_pos1_from_eeprom(&info1.cur_pos);
  restore_len2_from_eeprom(&info2.total_len);
  restore_pos2_from_eeprom(&info2.cur_pos);


  info1.target_pos = info1.cur_pos;
  encoder1.write(info1.cur_pos);
  info2.target_pos = info2.cur_pos;
  encoder2.write(info2.cur_pos);

  last_pid_time = millis();

  DEBUG_SER.print(F("---- init over----\r\n cur_pos1: "));
  DEBUG_SER.print(info1.cur_pos);
  DEBUG_SER.print(F(", total_len1: "));
  DEBUG_SER.print(info1.total_len);
  DEBUG_SER.print(F(", \tcur_pos2: "));
  DEBUG_SER.print(info2.cur_pos);
  DEBUG_SER.print(F(", total_len2: "));
  DEBUG_SER.println(info2.total_len);

  motor_power(1, 0);
  motor_power(2, 0);

}

void loop()
{
  //read position
  info1.cur_pos = encoder1.read();
  //DEBUG_SER.print("cur pos1: ");
  //DEBUG_SER.println(info1.cur_pos);
  info2.cur_pos = encoder2.read();
  //DEBUG_SER.print("cur pos2: ");
  //DEBUG_SER.println(info2.cur_pos);

  //handle key event
  int key = read_key();
  if (key >= 0)
  {
    DEBUG_SER.print("Key: ");
    DEBUG_SER.println(key);
    beep(20);
    handle_key(key);
  }

  //motor_speed(1, map(analogRead(A5), 0, 1023, -400, 400));
  //info1.target_pos = map(analogRead(A5), 0, 1023, 0, info1.total_len);

  uint32_t cur_time = millis();
  if (cur_time - last_pid_time > 50)
  {
    last_pid_time = cur_time;
    motor_power(1, calc_power(&info1, &pid1));
    motor_power(2, calc_power(&info2, &pid2));
  }

  check_second_beep();
}

int32_t calc_power(struct info *pInfo, struct PIDdata *pPID)
{
  pInfo->cur_speed = (pInfo->cur_pos - pInfo->last_pos); // r/s
  pInfo->last_pos = pInfo->cur_pos;

  // position ring
  int32_t dPos = pInfo->target_pos - pInfo->cur_pos;
  //DEBUG_SER.print("delta pos:");
  //DEBUG_SER.println(dPos);

  if (abs(dPos) > FULL_SPEED_THRES)
  {
    pInfo->target_speed = FULL_SPEED * (abs(dPos) / dPos);
    pInfo->first_stop_flag = true;
  } else if (dPos > 20)
  {
    pInfo->target_speed = map(dPos, 0, FULL_SPEED_THRES, 0, FULL_SPEED);
    pInfo->first_stop_flag = true;
  } else if (dPos < -20)
  {
    pInfo->target_speed = map(dPos, -FULL_SPEED_THRES, 0, -FULL_SPEED, 0);
    pInfo->first_stop_flag = true;
  } else
  {
    pInfo->target_speed = 0;
    pPID->integratedError = 0;
    pPID->lastPosition = pInfo->cur_speed;
    if (pInfo->first_stop_flag)
    {
      save_cur_pos();
      pInfo->first_stop_flag = false;
    }
  }
  //DEBUG_SER.print("target_speed:");
  //DEBUG_SER.println(pInfo->target_speed);
  //DEBUG_SER.print("cur speed:");
  //DEBUG_SER.println(pInfo->cur_speed);


  // velocity ring
  int32_t power = 0;
  if (pInfo->target_speed != 0)
  {
    power = updatePID(pInfo->target_speed, pInfo->cur_speed, pPID);
    //DEBUG_SER.print("power:");
    //DEBUG_SER.println(power);
  }

  //DEBUG_SER.println();

  return power;
}

/*******************************************************************************
* Function Name  : float updatePID
* Description    : ____
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters)
{
  unsigned long currentTime = millis();
  // AKA PID experiments
  const float deltaPIDTime = (currentTime - PIDparameters->previousPIDTime) / 1000.0f;

  PIDparameters->previousPIDTime = currentTime;  // AKA PID experiments
  float error = targetPosition - currentPosition;

  //if (abs(error) < 20)
  PIDparameters->integratedError += error * deltaPIDTime;
  PIDparameters->integratedError = constrain(PIDparameters->integratedError, -iLimit, iLimit);

  const float filter = 7.9577e-3f; //for 20Hz
  float der = 0;
  if (D != 0)
  {
    der = (error - PIDparameters->lastErr) / deltaPIDTime;
    der = PIDparameters->lastDer + (deltaPIDTime / (filter + deltaPIDTime)) * (der - PIDparameters->lastDer);
    PIDparameters->lastDer = der;
    PIDparameters->lastErr = error;
  }

  PIDparameters->lastPosition = currentPosition;
  float pTerm = P * error;
  float iTerm = I * PIDparameters->integratedError;
  float dTerm = D * der;
  float ret = pTerm + iTerm + dTerm;

  //DEBUG_SER.print(pTerm);
  //DEBUG_SER.print(" + ");
  //DEBUG_SER.print(iTerm);
  //DEBUG_SER.print(" + ");
  //DEBUG_SER.print(dTerm);
  //DEBUG_SER.print(" = ");
  //DEBUG_SER.println(ret);

  return ret;
}

void handle_key(int key)
{
  switch (key)
  {
    case K_UP:
      if ((mode & LEFT) > 0) info1.target_pos = 0;
      if ((mode & RIGHT) > 0) info2.target_pos = 0;
      break;
    case K_DOWN:
      if ((mode & LEFT) > 0) info1.target_pos = info1.total_len;
      if ((mode & RIGHT) > 0) info2.target_pos = info2.total_len;
      break;
    case K_OK:
      if ((mode & LEFT) > 0) info1.target_pos = info1.total_len/2;
      if ((mode & RIGHT) > 0) info2.target_pos = info2.total_len/2;
      break;
    case K_STOP:
      if ((mode & LEFT) > 0) info1.target_pos = info1.cur_pos + ((info1.cur_speed > 10) ? (info1.cur_speed * 4) : 0);
      if ((mode & RIGHT) > 0) info2.target_pos = info2.cur_pos + ((info2.cur_speed > 10) ? (info2.cur_speed * 4) : 0);
      break;
    case K_FWD:
      mode = RIGHT;
      break;
    case K_REV:
      mode = LEFT;
      break;
    case K_Q:
      mode = DUAL;
      double_beep(100,40);
      break;
    case K_NEXT:
      if ((mode & LEFT) > 0)
      {
        info1.target_pos += 450;
        if (!calib_mode && info1.target_pos > info1.total_len)
        {
          info1.target_pos = info1.total_len;
        }
      }
      if ((mode & RIGHT) > 0)
      {
        info2.target_pos += 450;
        if (!calib_mode && info2.target_pos > info2.total_len)
        {
          info2.target_pos = info2.total_len;
        }
      }
      break;
    case K_PREV:
      if ((mode & LEFT) > 0)
      {
        info1.target_pos -= 450;
        if (!calib_mode && info1.target_pos < 0)
        {
          info1.target_pos = 0;
        }
      }
      if ((mode & RIGHT) > 0)
      {
        info2.target_pos -= 450;
        if (!calib_mode && info2.target_pos < 0)
        {
          info2.target_pos = 0;
        }
      }
      break;

    case K_P:
      if ((mode & LEFT) > 0)
      {
        info1.target_pos = info1.cur_pos = 0;
        encoder1.write(0);
        save_pos1_to_eeprom(info1.cur_pos);
      }
      if ((mode & RIGHT) > 0)
      {
        info2.target_pos = info2.cur_pos = 0;
        encoder2.write(0);
        save_pos2_to_eeprom(info2.cur_pos);
      }
      break;
    case K_M:
      if ((mode & LEFT) > 0)
      {
        if (info1.cur_pos > 0)
        {
          info1.total_len = info1.cur_pos;
          save_pos1_to_eeprom(info1.cur_pos);
          save_len1_to_eeprom(info1.total_len);
          DEBUG_SER.print("total_len1: ");
          DEBUG_SER.println(info1.total_len);
        }
      }
      if ((mode & RIGHT) > 0)
      {
        if (info2.cur_pos > 0)
        {
          info2.total_len = info2.cur_pos;
          save_pos2_to_eeprom(info2.cur_pos);
          save_len2_to_eeprom(info2.total_len);
          DEBUG_SER.print("total_len2: ");
          DEBUG_SER.println(info2.total_len);
        }
      }
      break;
    case K_MENU:
      {
        calib_mode = !calib_mode;
        if (calib_mode)
        {
          double_beep(50,20);
        }
        break;
      }

  }
}

// IR
#define BIT_LEN         0
#define BIT_START_H     1
#define BIT_START_L     2
#define BIT_DATA_H      3
#define BIT_DATA_L      4
#define BIT_DATA_LEN    5
#define BIT_DATA        6

int read_key()
{
  unsigned char dta[20];

  if (IR.IsDta())
  {
    IR.Recv(dta);               // receive data to dta
    /*DEBUG_SER.println("+------------------------------------------------------+");
    DEBUG_SER.print("START_H: ");
    DEBUG_SER.print(dta[BIT_START_H]);
    DEBUG_SER.print("\tSTART_L: ");
    DEBUG_SER.println(dta[BIT_START_L]);

    DEBUG_SER.print("DATA_H: ");
    DEBUG_SER.print(dta[BIT_DATA_H]);
    DEBUG_SER.print("\tDATA_L: ");
    DEBUG_SER.println(dta[BIT_DATA_L]);

    DEBUG_SER.print("\r\nDATA_LEN = ");
    DEBUG_SER.println(dta[BIT_DATA_LEN]);

    for(int i=0; i<dta[BIT_DATA_LEN]; i++)
    {
        DEBUG_SER.print("0x");
        DEBUG_SER.print(dta[i+BIT_DATA], HEX);
        DEBUG_SER.print("\t");
    }

    DEBUG_SER.println();
    DEBUG_SER.println("+------------------------------------------------------+\r\n\r\n");
    */
    for (int i = 0; i < K_LAST; i++)
    {
      if (memcmp(dta + 6, keys[i], IR_LEN) == 0)
      {
        return i;
      }
    }
  }
  return -1;
}


void motor_power(int motor, int32_t power)
{
  int pwm = constrain(abs(power), 0, 255);
  //pwm = map(pwm, 0, 255, 20, 255);

  if (motor == 1)
  {
    if (power > 0)
    {
      analogWrite(PIN_EN1, pwm);
      digitalWrite(PIN_COM11, 1);
      digitalWrite(PIN_COM12, 0);
    }
    else if (power < 0)
    {
      analogWrite(PIN_EN1, pwm);
      digitalWrite(PIN_COM11, 0);
      digitalWrite(PIN_COM12, 1);
    } else
    {
      analogWrite(PIN_EN1, 255);
      digitalWrite(PIN_COM11, 1);
      digitalWrite(PIN_COM12, 1);
    }
  } else
  {
    if (power > 0)
    {
      analogWrite(PIN_EN2, pwm);
      digitalWrite(PIN_COM21, 1);
      digitalWrite(PIN_COM22, 0);
    } else if (power < 0)
    {
      analogWrite(PIN_EN2, pwm);
      digitalWrite(PIN_COM21, 0);
      digitalWrite(PIN_COM22, 1);
    } else
    {
      analogWrite(PIN_EN2, 255);
      digitalWrite(PIN_COM21, 1);
      digitalWrite(PIN_COM22, 1);
    }
  }
}


static uint32_t next_beep_time;
static int second_beep_length;
void beep(int t)
{
  digitalWrite(PIN_BUZZER, 1);
  delay(t);
  digitalWrite(PIN_BUZZER, 0);
}
void double_beep(uint32_t interval, int t2)
{
  next_beep_time = millis() + interval;
  second_beep_length = t2;
}
void check_second_beep()
{
  if((millis() > next_beep_time) && next_beep_time != 0)
  {
    beep(second_beep_length);
    next_beep_time = 0;
  }
}

void save_4byte(int addr, uint32_t value)
{
  eeprom_write_block((const void *)&value, (void *)addr, sizeof(uint32_t));
}

void restore_4byte(int addr, uint32_t *value)
{
  eeprom_read_block((void *)value, (void *)addr, sizeof(uint32_t));
}

void save_len1_to_eeprom(int32_t value)
{
  save_4byte(0, value);
}

void restore_len1_from_eeprom(int32_t *value)
{
  restore_4byte(0, (uint32_t *)value);
}

void save_pos1_to_eeprom(int32_t value)
{
  DEBUG_SER.println("save pos1 to eep");
  DEBUG_SER.print("cur pos1: ");
  DEBUG_SER.println(info1.cur_pos);
  save_4byte(4, value);
}

void restore_pos1_from_eeprom(int32_t *value)
{
  restore_4byte(4, (uint32_t *)value);
}

void save_len2_to_eeprom(int32_t value)
{
  save_4byte(8, value);
}

void restore_len2_from_eeprom(int32_t *value)
{
  restore_4byte(8, (uint32_t *)value);
}

void save_pos2_to_eeprom(int32_t value)
{
  DEBUG_SER.println("save pos2 to eep");
  DEBUG_SER.print("cur pos2: ");
  DEBUG_SER.println(info2.cur_pos);
  save_4byte(12, value);
}

void restore_pos2_from_eeprom(int32_t *value)
{
  restore_4byte(12, (uint32_t *)value);
}

void save_cur_pos()
{
  if (info1.first_stop_flag)
  {
    save_pos1_to_eeprom(info1.cur_pos);
  }
  if (info2.first_stop_flag)
  {
    save_pos2_to_eeprom(info2.cur_pos);
  }
}
