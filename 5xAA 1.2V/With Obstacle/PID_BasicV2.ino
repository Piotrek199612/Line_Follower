/********************************************************
   PID Basic Example
   Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <NewPing.h>
int empty = 0;
const int ilosc_pustych = 600;
#define PROG 700
//Define Variables we'll be connecting to
double Setpoint, Input,   Output = float(510);
//prop  caĹ‚k  rĂłĹĽn
//Specify the links and initial tuning parameters
double Kp = 55, Ki = 10, Kd = 0.3f;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#define trig 1
#define echo 0
#define pwm_lewy 6
#define lewy_1 7
#define lewy_2 8
#define pwm_prawy 11
#define prawy_1 13
#define prawy_2 12
bool omin;


bool pomiar_odleglosci()
{
  int czas, dist;
  digitalWrite(trig, HIGH);
  delayMicroseconds(1000);
  digitalWrite(trig, LOW);
  czas = pulseIn(echo, HIGH);
  dist = (czas/2) / 29.1;
  if (dist <=20.0f && dist>=0.0f )
    return true;
  else return false;
}


void setup()
{
  //initialize the variables we're linked to
  //turn the PID on
  myPID.SetOutputLimits(0, 1020);
  myPID.SetMode(AUTOMATIC);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(pwm_prawy, OUTPUT);
  pinMode(prawy_2, OUTPUT);
  pinMode(prawy_1, OUTPUT);
  pinMode(pwm_lewy, OUTPUT);
  pinMode(lewy_1, OUTPUT);
  pinMode(lewy_2, OUTPUT);
    digitalWrite(prawy_1, HIGH);
    digitalWrite(prawy_2, LOW);
    digitalWrite(lewy_2, HIGH);
    digitalWrite(lewy_1, LOW);
        analogWrite(pwm_lewy, 0);
    analogWrite(pwm_prawy, 0);
    
  //Serial.begin(9600);
  Input = 0;
  Setpoint = float(0);
 // Serial.println("WITSAM");
 omin = false;
   pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);
}

/*const int MAX_DISTANCE = 200;
NewPing sonar(trig, echo, MAX_DISTANCE);*/
bool start = true;
void loop()
{
  if (start)
  {
    analogWrite(pwm_lewy,100);
    analogWrite(pwm_prawy,100);
    
  }
  omin = pomiar_odleglosci();
  if (omin && !start)
  {
     digitalWrite(9,HIGH);
    digitalWrite(prawy_1, HIGH);
    digitalWrite(prawy_2, LOW);
    digitalWrite(lewy_2, LOW);
    digitalWrite(lewy_1, HIGH);
    analogWrite(pwm_lewy, 255);
    analogWrite(pwm_prawy, 50);
    delay(1000);
    digitalWrite(prawy_1, HIGH);
    digitalWrite(prawy_2, LOW);
    digitalWrite(lewy_2, HIGH);
    digitalWrite(lewy_1, LOW);
    analogWrite(pwm_lewy, 50);
    analogWrite(pwm_prawy, 255);
    delay(1500);
    digitalWrite(prawy_1, LOW);
    digitalWrite(prawy_2, HIGH);
    digitalWrite(lewy_2, HIGH);
    digitalWrite(lewy_1, LOW);
    analogWrite(pwm_lewy, 255);
    analogWrite(pwm_prawy, 50);
    delay(1000);
    digitalWrite(prawy_1, HIGH);
    digitalWrite(prawy_2, LOW);
    digitalWrite(lewy_2, HIGH);
    digitalWrite(lewy_1, LOW);
    analogWrite(pwm_lewy, 50);
    analogWrite(pwm_prawy, 255);
    delay(2000);
    digitalWrite(prawy_1, LOW);
    digitalWrite(prawy_2, HIGH);
    digitalWrite(lewy_2, HIGH);
    digitalWrite(lewy_1, LOW);
    analogWrite(pwm_lewy, 255);
    analogWrite(pwm_prawy, 50);
    delay(3000);
    digitalWrite(prawy_1, HIGH);
    digitalWrite(prawy_2, LOW);
    digitalWrite(lewy_2, HIGH);
    digitalWrite(lewy_1, LOW);
    analogWrite(pwm_lewy, 50);
    analogWrite(pwm_prawy, 255);
    delay(1500);
    digitalWrite(prawy_1, HIGH);
    digitalWrite(prawy_2, LOW);
    digitalWrite(lewy_2, LOW);
    digitalWrite(lewy_1, HIGH);
    analogWrite(pwm_lewy, 255);
    analogWrite(pwm_prawy, 50);
    delay(3000);
    analogWrite(pwm_lewy, 100);
    analogWrite(pwm_prawy, 100);    
  }
  else
  {
    digitalWrite(9,LOW);

    Input = Polozenie();
    if (empty < ilosc_pustych)
    {
      myPID.Compute();
      if (Output < 255)
      {
        digitalWrite(prawy_1, HIGH);
        digitalWrite(prawy_2, LOW);
        digitalWrite(lewy_2, LOW);
        digitalWrite(lewy_1, HIGH);
        analogWrite(pwm_lewy, float(float(255) - Output));
        analogWrite(pwm_prawy, 255);
      }
      else if (Output >= 255 && Output < 510)
      {
        digitalWrite(prawy_1, HIGH);
        digitalWrite(prawy_2, LOW);
        digitalWrite(lewy_2, HIGH);
        digitalWrite(lewy_1, LOW);
        analogWrite(pwm_lewy, float(Output - float(255)));
        analogWrite(pwm_prawy, 255);
      }
      else if (Output >= 510 && Output < 765)
      {       
        //analogWrite(pwm_lewy, 255);
        if (start)delay(100);
        analogWrite(pwm_prawy, float(float(765) - Output));
        if (start)delay(1000);
        analogWrite(pwm_lewy, 255);
        delay(10);
        digitalWrite(prawy_1, HIGH);
        digitalWrite(prawy_2, LOW);
        digitalWrite(lewy_2, HIGH);
        digitalWrite(lewy_1, LOW);

      }
      else if (Output >= 765)
      {
        digitalWrite(prawy_1, LOW);
        digitalWrite(prawy_2, HIGH);
        digitalWrite(lewy_2, HIGH);
        digitalWrite(lewy_1, LOW);
        analogWrite(pwm_lewy, 255);
        analogWrite(pwm_prawy, float(Output - float(765) ));
      }
    }
        start = false;
  }
}

bool last_sensors [5];
double Polozenie()
{
  const int wagi [] = { -4, -2, 0, 2, 4};
  bool sensors [5];
  Measure(sensors);

  int ilosc = 0;
  int suma = 0;
  for (int i = 0; i < 5; i++)
  {
    if (sensors[i] == true)
    {
      suma += wagi[i];
      ilosc++;
    }
  }
  if (ilosc != 0)
  {
    empty = 0;
    for (int i = 0; i < 5; i++)
      last_sensors[i] = sensors[i];
    return float(suma) / float(ilosc);
  }
  else
  {
    empty++;
    if ((last_sensors[0] || last_sensors[1]) && !last_sensors[4] && empty >= ilosc_pustych)
    {
      digitalWrite(prawy_1, LOW);
      digitalWrite(prawy_2, HIGH);
      digitalWrite(lewy_2, HIGH);
      digitalWrite(lewy_1, LOW);
      analogWrite(pwm_lewy, 255);
      analogWrite(pwm_prawy, 255);
      return float(-8.0);
    }
    else if (!last_sensors[0] && (last_sensors[3] || last_sensors[4]) && empty >= ilosc_pustych)
    {
      digitalWrite(prawy_1, HIGH);
      digitalWrite(prawy_2, LOW);
      digitalWrite(lewy_2, LOW);
      digitalWrite(lewy_1, HIGH);
      analogWrite(pwm_lewy, 255);
      analogWrite(pwm_prawy, 255);
      return float(-8.0);
    }
    else if (last_sensors[0] && !last_sensors[1] && !last_sensors[3] && !last_sensors[4] && empty < ilosc_pustych)
    {
      return float(-8.0);
    }
    else if (!last_sensors[0] && last_sensors[1] && !last_sensors[3] && !last_sensors[4] && empty < ilosc_pustych )
    {
      return float(-1.0);
    }
    else if (last_sensors[0] && last_sensors[1] && !last_sensors[3] && !last_sensors[4] && empty < ilosc_pustych)
    {
      return float(-14.0);
    }
    else if (last_sensors[4] && !last_sensors[3] && !last_sensors[0] && !last_sensors[1] && empty < ilosc_pustych)
    {
      return float(8.0);
    }
    else if (!last_sensors[4] && last_sensors[3] && !last_sensors[0] && !last_sensors[1] && empty < ilosc_pustych)
    {
      // Serial.println("Sensor 4");
      return float(1.0);
    }
    else if (last_sensors[4] && last_sensors[3] && !last_sensors[0] && !last_sensors[1] && empty < ilosc_pustych)
    {
      return float(14.0);
    }
    else
    {
      int ilosc = 0;
      int suma = 0;
      for (int i = 0; i < 5; i++)
      {
        if (sensors[i] == true)
        {
          suma += wagi[i];
          ilosc++;
        }
      }
      if (ilosc != 0)
      {
        for (int i = 0; i < 5; i++)
          last_sensors[i] = sensors[i];
        return float(suma) / float(ilosc) * 2;
      }
    }
    return float(0.0);
  }
}



void Measure(bool *tab)
{
  tab [0] = Is_High(analogRead(A0));
  tab [1] = Is_High(analogRead(A1));
  tab [2] = Is_High(analogRead(A2));
  tab [3] = Is_High(analogRead(A3));
  tab [4] = Is_High(analogRead(A4));
}

bool Is_High(int input)
{
  if (input >= PROG)
    return true;
  else
    return false;
}


