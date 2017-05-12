/********************************************************
   PID Basic Example
   Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
int empty = 0;
const int ilosc_pustych =600;
#define PROG 700
//Define Variables we'll be connecting to
double Setpoint, Input,   Output = float(510);
//prop  całk  różn
//Specify the links and initial tuning parameters
double Kp = 55, Ki = 10, Kd = 0.3f;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  //Serial.begin(9600);
  Input = 0;
  Setpoint = float(0);
}

void loop()
{

  Input = Polozenie();
  /*Serial.println("---------------------");
    Serial.print("Input: ");
    Serial.print(Input);
    Serial.print(" Output: ");
    Serial.print(Output);
    Serial.println("");*/

  if (empty < ilosc_pustych)
{
  myPID.Compute();
  if (Output < 255)
  {
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, HIGH);
    analogWrite(6, float(float(255)-Output));
    /*Serial.print(Output);
    Serial.print("1  ");
    Serial.println(float(float(255)-Output));*/
    analogWrite(11, 255);
  }
  else if (Output >=255 && Output <510)
  { 
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
    analogWrite(6, float(Output-float(255)));
    /*Serial.print(Output);
    Serial.print("2  ");
    Serial.println( float(Output-float(255)));*/
    analogWrite(11, 255);
  }
  else if (Output >=510 && Output <765)
  {
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
    analogWrite(6, 255);
   /* Serial.print(Output);
    Serial.print("3  ");
    Serial.println((float(765) - Output));*/
    analogWrite(11, float(float(765) - Output));
  }
  else if (Output >=765)
  {
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
    analogWrite(6, 255);
    /*Serial.print(Output);
    Serial.print("4  ");
    Serial.println((Output-float(765) ));*/
    analogWrite(11, float(Output-float(765) ));
  }
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
    /*Serial.print("0-");
      Serial.print(last_sensors[0]);
      Serial.print(" 1-");
      Serial.print(last_sensors[1]);
      Serial.print(" 2-");
      Serial.print(last_sensors[2]);
      Serial.print(" 3-");
      Serial.print(last_sensors[3]);
      Serial.print(" 4-");
      Serial.print(last_sensors[4]);

      Serial.print("          0-");
      Serial.print(sensors[0]);
      Serial.print(" 1-");
      Serial.print(sensors[1]);
      Serial.print(" 2-");
      Serial.print(sensors[2]);
      Serial.print(" 3-");
      Serial.print(sensors[3]);
      Serial.print(" 4-");
      Serial.print(sensors[4]);
      Serial.println("");*/
  if ((last_sensors[0] ||last_sensors[1])&&!last_sensors[4] && empty >= ilosc_pustych)
    {
        digitalWrite(9, LOW);
      digitalWrite(10, HIGH);
      digitalWrite(8, HIGH);
      digitalWrite(7, LOW);
      analogWrite(6, 255);
      analogWrite(11, 255);
      return float(-8.0);
    }
    else if (!last_sensors[0] &&(last_sensors[3]||last_sensors[4]) && empty >= ilosc_pustych)
    {
      digitalWrite(9, HIGH);
      digitalWrite(10, LOW);
      digitalWrite(8, LOW);
      digitalWrite(7, HIGH);
      analogWrite(6, 255);
      analogWrite(11, 255);
      return float(-8.0);
    }
    else if (last_sensors[0] && !last_sensors[1]&& !last_sensors[3]&&!last_sensors[4] && empty < ilosc_pustych)
    {
    //  Serial.println("Sensor 0");
      return float(-8.0);
    }
    else if (!last_sensors[0] && last_sensors[1]&& !last_sensors[3]&&!last_sensors[4]&& empty < ilosc_pustych )
    {
   //   Serial.println("Sensor 1");
      return float(-1.0);
    }
    else if (last_sensors[0] && last_sensors[1]&& !last_sensors[3]&&!last_sensors[4] && empty < ilosc_pustych)
    {
   //   Serial.println("Sensor 2");
      return float(-14.0);
    }
    else if (last_sensors[4] && !last_sensors[3]&&!last_sensors[0] && !last_sensors[1]&& empty < ilosc_pustych)
    {
    //  Serial.println("Sensor 3");
      return float(8.0);
    }
    else if (!last_sensors[4] && last_sensors[3]&&!last_sensors[0] && !last_sensors[1]&& empty < ilosc_pustych)
    {
     // Serial.println("Sensor 4");
      return float(1.0);
    }
    else if (last_sensors[4] && last_sensors[3]&&!last_sensors[0] && !last_sensors[1]&& empty < ilosc_pustych)
    {
      //Serial.println("Sensor 5");
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
    return float(suma)/ float(ilosc)*2;
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

