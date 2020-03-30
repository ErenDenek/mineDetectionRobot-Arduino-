#include <QTRSensors.h>

#define NUM_serialerialENSORS             8
#define NUM_serialerialAMPLES_PER_serialerialENSOR  4
#define EMITTER_PIN             QTR_NO_EMITTER_PIN

QTRSensorsAnalog qtra((unsigned char[]) {
  0, 1, 2, 3, 4, 5, 6, 7
},
NUM_serialerialENSORS, NUM_serialerialAMPLES_PER_serialerialENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_serialerialENSORS];
unsigned int position = 0;

#define in1 7
#define in2 6
#define stndby 8
#define in3 10
#define in4 9
#define led1 4
#define led2 13
#define led3 12
#define relayPin 3
#define mz80pin 2

inline void _forward(int sol_motor_max, int sag_motor_max ), _STEP(), _SENS();
void _Stop(), _rightTurn(int dly), _leftTurn(int dly, int sens1, int sens2), _north(), _south(), _east(), _west(), limit();

int pwm_right_pin = 5, pwm_left_pin = 11;

float Kp = 0.159, Ki = 0.0 , Kd = 0.430;
int hedef = 3500;
int pid, hata;
int toplam_hata, son_hata;
int pwm_right, pwm_left;

int check_west = 0;
int t = 500, t2 = 700;
int sV[8];

int buton_y = 0, buton_x = 0, buton_west, buton_limit;
int x1, x2, x3, x4;
int axis_x = 0, axis_y = -2, axis_west = 0;

int konum[12][8];
unsigned long lastTime = 0;

int input[10] = {A0, A1, A2, A3, A4, A5, A6, A7};
int output[10] = {pwm_right_pin, pwm_left_pin, in1, in2, in3, in4, stndby, led1, led2};


int road[30];

int r = 0;



void setup() {
  for (int i = 0; i < 8; i++)
    pinMode(input[i], INPUT);
  for (int k = 0; k < 9; k++)
    pinMode(output[k], OUTPUT);
  for (int p = 0; p < 12; p++)
  {
    for (int j = 0; j < 8; j++)
      konum[p][j] = 0;
  }

  delay(1000);

  digitalWrite(led1, HIGH);
  for (int i = 0; i < 90; i++)
  {
    qtra.calibrate();
  }
  digitalWrite(led1, LOW);

  delay(1500);
  Serial.begin(9600);
  
  /*road[0]= 1;
  road[1]= 1;
  road[2]= 2;
  road[3]= 2;
  road[4]= 1;
  road[5]= 1;
  road[6]= 4;
  road[7]= 1;
  road[8]= 1;
  road[9]= 2;
  road[10]= 2;
  road[11]= 0;
  road[12]= 1;
  road[13]=8;
  r=12;*/
     
}
///***************************************************************************************************//////////


void loop() {

  _north();
  _south();
  _east(0);
  if (check_west > 0)
    _west();


}


///*************************************************************************************************//////////

void _north() {

  while (true) {

    digitalWrite(led1, 1);
    _SENS();
    _forward(90, 90);

    if ((sV[0] < t && sV[1] < t && sV[2] < t) || (sV[0] < t && sV[1] < t && sV[2] < t && sV[3] < t && sV[4] < t && sV[5] < t && sV[6] < t) || (sV[7] < t && sV[5] < t && sV[6] < t)) {
      buton_y = 1;
    }
    else if ((sV[0] > t2 && sV[1] > t2 && sV[2] > t2) || (sV[0] > t2 && sV[1] > t2 && sV[2] > t2 && sV[3] > t2 && sV[4] > t2 && sV[5] > t2 && sV[6] > t2) || (sV[7] > t2 && sV[5] > t2 && sV[6] > t2)) {
      buton_y = 0;
    }
    if (buton_y == 1 && x1 == 0) {
      x1 = 1;
      axis_y = axis_y + 2;
      road[r] = 1; r++;
    }
    else if (buton_y == 0 && x1 == 1)x1 = 0;


    if ( sV[3] < t && sV[4] < t && sV[5] < t && sV[6] < t   && sV[2] < t  &&  digitalRead(relayPin) == HIGH) {
      while (true)
      {
        _leftTurn(900, 6, 7);
        _Stop();
        road[r + 1] = 1;
        _serial();
        for (int g = 0; g < r; g++)
        {
          Serial.print(road[g]);
        }
        Serial.println();
        _Stop();
        delay(3000);
        while (1)
        {
          _ezber();
        }

      }

    }


    if (sV[0] > t2 && sV[1] > t2 && sV[2] > t2 && sV[3] > t2 && sV[4] > t2 && sV[5] > t2 && sV[6] > t2 && sV[7] > t2)
    {
      _STEP();
      delay(100);
      _rightTurn(0, 3, 4);
      _east(1);

    }


    if (digitalRead(relayPin) == HIGH || digitalRead(mz80pin) == LOW)
    {
      konum[axis_y + 1][axis_x] = 1;
      digitalWrite(led1, 0);
      break;
    }
  }

}

void _south() {

  _leftTurn(100, 7, 6);
  while (true) {
    _SENS();
    _forward(70, 70);
    digitalWrite(led2, 1);

    if ((sV[0] < t && sV[1] < t && sV[2] < t) || (sV[0] < t && sV[1] < t && sV[2] < t && sV[3] < t && sV[4] < t && sV[5] < t && sV[6] < t) || (sV[7] < t && sV[5] < t && sV[6] < t) )
    {
      _STEP();
      delay(300);
      digitalWrite(led2, 0);
      break;
    }
  }
}

void _east(bool est) {

  if (est == 0) _leftTurn(100, 7, 6);

  while (true) {
    digitalWrite(led1, 1);
    digitalWrite(led2, 2);
    _forward(80, 80);
    _SENS();

    if (digitalRead(relayPin) == HIGH || digitalRead(mz80pin) == LOW)
    {

      _leftTurn(0, 3, 4);

      check_west++;
      konum[axis_y][axis_x + 1] = 1;
      digitalWrite(led1, 0);
      digitalWrite(led2, 0);
      break;
    }


    if (sV[0] > t2 && sV[1] > t2 && sV[2] > t2 && sV[3] > t2 && sV[4] > t2 && sV[5] > t2 && sV[6] > t2 && sV[7] > t2)
    {
      _STEP();
      delay(200);
      _leftTurn(100, 7, 6);
      axis_x = axis_x + 2;
      if (est == 0)
        road[r] = 2; r++;
      digitalWrite(led1, 0);
      digitalWrite(led2, 0);
      break;
    }


    if (est == 0) {
      if ((sV[0] < t && sV[1] < t && sV[2] < t) || (sV[0] < t && sV[1] < t && sV[2] < t && sV[3] < t && sV[4] < t && sV[5] < t && sV[6] < t) || (sV[7] < t && sV[5] < t && sV[6] < t))
      {
        _STEP();
        delay(300);
        _leftTurn(100, 6, 7);
        axis_x = axis_x + 2;
        road[r] = 2; r++;
        digitalWrite(led1, 0);
        digitalWrite(led2, 0);
        break;
      }
    }


    if (est == 1) {
      _serial();
      if ((sV[0] < t && sV[1] < t && sV[2] < t) || (sV[0] < t && sV[1] < t && sV[2] < t && sV[3] < t && sV[4] < t && sV[5] < t && sV[6] < t) || (sV[7] < t && sV[5] < t && sV[6] < t)) {
        buton_limit = 1;
      }
      else if ((sV[0] > t2 && sV[1] > t2 && sV[2] > t2) || (sV[0] > t2 && sV[1] > t2 && sV[2] > t2 && sV[3] > t2 && sV[4] > t2 && sV[5] > t2 && sV[6] > t2) || (sV[7] > t2 && sV[5] > t2 && sV[6] > t2)) {
        buton_limit = 0;
      }
      if (buton_limit == 1 && x4 == 0) {
        x4 = 1;
        axis_x = axis_x + 2;
        road[r] = 2; r++;
      }
      else if (buton_limit == 0 && x4 == 1)x4 = 0;
    }


  }
}

void _west() {

  _leftTurn(100, 7, 6);

  while (true) {

    _forward(80, 80);
    _SENS();


    if ((sV[0] < t && sV[1] < t && sV[2] < t) || (sV[0] < t && sV[1] < t && sV[2] < t && sV[3] < t && sV[4] < t && sV[5] < t && sV[6] < t) || (sV[7] < t && sV[5] < t && sV[6] < t)) {
      buton_west = 1;
    }
    else if ((sV[0] > t2 && sV[1] > t2 && sV[2] > t2) || (sV[0] > t2 && sV[1] > t2 && sV[2] > t2 && sV[3] > t2 && sV[4] > t2 && sV[5] > t2 && sV[6] > t2) || (sV[7] > t2 && sV[5] > t2 && sV[6] > t2)) {
      buton_west = 0;
    }
    if (buton_west == 1 && x3 == 0) {
      x3 = 1;
      axis_west++;
    }
    else if (buton_west == 0 && x3 == 1)x3 = 0;


    if (axis_west == 2)
    {
      _STEP();
      delay(200);
      _rightTurn(200, 0, 1);
      check_west = 0;
      axis_west = 0;
      axis_x = axis_x - 2;
      if (axis_x < 0)axis_x = 0;
      road[r] = 4; r++;
      break;
    }

  }
}


void _leftTurn(int dly, int sens1, int sens2) {

  while (true) {

    digitalWrite(in1, 1);
    digitalWrite(in2, 0);
    digitalWrite(in3, 0);
    digitalWrite(in4, 1);
    digitalWrite(stndby, 1);
    _SENS();

    if (sV[sens1] < t && sV[sens2] < t)
    {
      delay(dly);
      break;
    }


  }
}

void _rightTurn(int dly, int sens1, int sens2)
{
  while (true)
  {

    _SENS();
    digitalWrite(in1, 0);
    digitalWrite(in2, 1);
    digitalWrite(in3, 1);
    digitalWrite(in4, 0);
    digitalWrite(stndby, 1);
    if (sV[sens2] < t && sV[sens1] < t)
    {
      delay(dly);
      break;
    }

  }
}


inline void _forward(int sol_motor_max, int sag_motor_max) {

  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  digitalWrite(in3, 1);
  digitalWrite(in4, 0);
  digitalWrite(stndby, 1);

  _SENS();

  if (sV[3] < t && sV[4] < t && sV[5] < t )
    _STEP();


  hata = hedef - position;
  toplam_hata += hata;
  pid = hata * Kp + (hata - son_hata) * Kd + (toplam_hata * Ki);
  son_hata = hata;
  pwm_right = (sol_motor_max + pid);
  pwm_left = (sag_motor_max  - pid);


  if (pwm_left > sol_motor_max) {
    pwm_left = sol_motor_max;
  }

  if (pwm_left < 0) {
    pwm_left = 0;
  }

  if (pwm_right > sag_motor_max) {
    pwm_right = sag_motor_max;
  }

  if (pwm_right < 0) {
    pwm_right = 0;
  }

  if (pwm_left > 0) {
    analogWrite(pwm_left_pin, pwm_left);
  }
  if (pwm_right > 0) {
    analogWrite(pwm_right_pin, pwm_right);
  }

}
inline void _STEP()
{
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  digitalWrite(in3, 1);
  digitalWrite(in4, 0);

  analogWrite(pwm_right_pin, 66);
  analogWrite(pwm_left_pin, 70);
  digitalWrite(stndby, 1);
}
void _serial()
{
  //Serial.print(axis_y); Serial.print("       "); Serial.println(axis_x);
}
inline void _SENS()
{
  position = qtra.readLine(sensorValues);
  sV[0] = sensorValues[0];
  sV[1] = sensorValues[1];
  sV[2] = sensorValues[2];
  sV[3] = sensorValues[3];
  sV[4] = sensorValues[4];
  sV[5] = sensorValues[5];
  sV[6] = sensorValues[6];
  sV[7] = sensorValues[7];
}

void _Stop() {

  digitalWrite(stndby, 0);
  delay(400);

}
void _ezber()
{

  digitalWrite(stndby, 1);
  while (true)
  {
    _forward(80, 80);
    if (road[r] == 0) r--;

    position = qtra.readLine(sensorValues);
    if ((sensorValues[0] < 500 && sensorValues[1] < 500 && sensorValues[2] < 500) || (sensorValues[0] < 500 && sensorValues[1] < 500 && sensorValues[2] < 500 && sensorValues[3] < 500 && sensorValues[4] < 500 && sensorValues[5] < 500 && sensorValues[6] < 500) || (sensorValues[7] < 500 && sensorValues[5] < 500 && sensorValues[6] < 500)) {
      r--;
      while (r == -1)
        _Stop();
      digitalWrite(led1, 1);
      _STEP();
      delay(400);
      digitalWrite(led1, 0);
      _Stop();
      break;
    }
  }
  delay(0);
  if (road[r] == 0) r--;
  if (road[r] == 1 && road[r + 1] == 2)//SOL
  {
    digitalWrite(led2, 1);
    _leftTurn(100, 7, 6);
    _Stop();
    delay(0);
    digitalWrite(led2, 0);
  }
  if (road[r] == 0) r--;
  else if (road[r] == 1 && road[r + 1] == 4)//SAG
  {
    digitalWrite(led3, 1);
    _rightTurn(100, 0, 1);
    _Stop();
    delay(0);
    digitalWrite(led3, 0);

  }
  if (road[r] == 0) r--;
  else if (road[r] == 2 && road[r + 1] != 2)//SAG
  {
    digitalWrite(led3, 1);
    _rightTurn(100, 0, 1);
    _Stop();
    delay(0);
    digitalWrite(led3, 0);

  }
  if (road[r] == 0) r--;
  else if (road[r] == 4)//SOL
  {
    digitalWrite(led2, 1);
    _leftTurn(100, 7, 6);
    _Stop();
    delay(0);
    digitalWrite(led2, 0);
  }
  if (road[r] == 0) r--;
}
