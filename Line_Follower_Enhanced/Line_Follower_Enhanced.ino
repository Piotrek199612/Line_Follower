#include <PID_v1.h>

//Przypisanie pinów
#define trig 1
#define echo 0
#define lewy_2 10
#define lewy_1 9
#define pwm_lewy 11
#define prawy_2 8
#define prawy_1 7
#define pwm_prawy 6

#define PROG_ODCZYTU_CZARNEGO 700
#define OGRANICZENIE_MOCY 0.5
#define OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY 0.5
#define ODLEGLOSC_BRZEGOWA 30
#define ODSTEP_MIEDZY_POMIARAMI_ODLEGLOSCI 1000

//Ustawienia PID
double Setpoint, Input,   Output = float(510);
//prop  całk  różn
double Kp = 55, Ki = 10, Kd = 0.3f;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


int empty = 0; //ilosc odczytów czujników bez wykrycia linii


void setup()
{
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
  pinMode(lewy_2, OUTPUT);
  pinMode(lewy_1, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  Input = 0;
  Setpoint = float(0);
}


bool ominac = false;  //Wykycie przeszkody
int odczytaj = 0;  //odstęp między kolejnymi pomiarami odległości
int ilosc_wykrytych_odleglosci = 0;
bool przeszkoda =true;
void loop()
{
  if(odczytaj > ODSTEP_MIEDZY_POMIARAMI_ODLEGLOSCI && przeszkoda)
  {
    if( pomiar_odleglosci())
      {
        analogWrite(pwm_lewy, 0);
        analogWrite(pwm_prawy, 0);
        delay(300);
        ominac = pomiar_odleglosci();
        delay(100);
        przeszkoda = false;
      }
      odczytaj=0;
  }
  
  
  if (!ominac)
  {
    Input = Polozenie();
    myPID.Compute();

  if(Output > 500 && Output < 520) odczytaj +=5;
    
    if (Output < 255)
    {
      digitalWrite(prawy_1, HIGH);
      digitalWrite(prawy_2, LOW);
      digitalWrite(lewy_1, LOW);
      digitalWrite(lewy_2, HIGH);
      analogWrite(pwm_lewy, float(float(255) - Output)*OGRANICZENIE_MOCY);
      analogWrite(pwm_prawy, 255 * OGRANICZENIE_MOCY);
      odczytaj++;
    }
    else if (Output >= 255 && Output < 510)
    {
      digitalWrite(prawy_1, HIGH);
      digitalWrite(prawy_2, LOW);
      digitalWrite(lewy_1, HIGH);
      digitalWrite(lewy_2, LOW);
      analogWrite(pwm_lewy, float(Output - float(255))*OGRANICZENIE_MOCY);
      analogWrite(pwm_prawy, 255 * OGRANICZENIE_MOCY);
      odczytaj+=2;
    }
    else if (Output >= 510 && Output < 765)
    {
      digitalWrite(prawy_1, HIGH);
      digitalWrite(prawy_2, LOW);
      digitalWrite(lewy_1, HIGH);
      digitalWrite(lewy_2, LOW);
      analogWrite(pwm_lewy, 255 * OGRANICZENIE_MOCY);
      analogWrite(pwm_prawy, float(float(765) - Output)*OGRANICZENIE_MOCY);
      odczytaj+=2;
    }
    else if (Output >= 765)
    {
      digitalWrite(prawy_1, LOW);
      digitalWrite(prawy_2, HIGH);
      digitalWrite(lewy_1, HIGH);
      digitalWrite(lewy_2, LOW);
      analogWrite(pwm_lewy, 255 * OGRANICZENIE_MOCY);
      analogWrite(pwm_prawy, float(Output - float(765) )*OGRANICZENIE_MOCY);
      odczytaj++;
    }
  }
  else // Ominiecie przeszkody
 {
    przeszkoda=false;
    //Wycfanie po wykryciu przeszkody
    /*digitalWrite(prawy_1, LOW);
      digitalWrite(prawy_2, HIGH);
      digitalWrite(lewy_1, LOW);
      digitalWrite(lewy_2, HIGH);
      analogWrite(pwm_lewy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
      analogWrite(pwm_prawy, 255*OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
      delay(800);*/
    //Skret w prawo
    digitalWrite(prawy_1, HIGH);
    digitalWrite(prawy_2, LOW);
    digitalWrite(lewy_1, LOW);
    digitalWrite(lewy_2, HIGH);
    analogWrite(pwm_lewy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    analogWrite(pwm_prawy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    delay(800);
    //jazda prosto
    digitalWrite(prawy_1, HIGH);
    digitalWrite(prawy_2, LOW);
    digitalWrite(lewy_1, HIGH);
    digitalWrite(lewy_2, LOW);
    analogWrite(pwm_lewy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    analogWrite(pwm_prawy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    delay(1200);
    //skret w lewo
    digitalWrite(prawy_1, LOW);
    digitalWrite(prawy_2, HIGH);
    digitalWrite(lewy_1, HIGH);
    digitalWrite(lewy_2, LOW);
    analogWrite(pwm_lewy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    analogWrite(pwm_prawy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    delay(600);
    //jazda prosto
    digitalWrite(prawy_1, HIGH);
    digitalWrite(prawy_2, LOW);
    digitalWrite(lewy_1, HIGH);
    digitalWrite(lewy_2, LOW);
    analogWrite(pwm_lewy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    analogWrite(pwm_prawy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    delay(1400);
    //skret w lewo
    digitalWrite(prawy_1, LOW);
    digitalWrite(prawy_2, HIGH);
    digitalWrite(lewy_1, HIGH);
    digitalWrite(lewy_2, LOW);
    analogWrite(pwm_lewy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    analogWrite(pwm_prawy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    delay(500);
    //jazda prosto
    digitalWrite(prawy_1, HIGH);
    digitalWrite(prawy_2, LOW);
    digitalWrite(lewy_1, HIGH);
    digitalWrite(lewy_2, LOW);
    analogWrite(pwm_lewy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    analogWrite(pwm_prawy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    delay(1200);
    //skret w prawo
    digitalWrite(prawy_1, HIGH);
    digitalWrite(prawy_2, LOW);
    digitalWrite(lewy_1, LOW);
    digitalWrite(lewy_2, HIGH);
    analogWrite(pwm_lewy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    analogWrite(pwm_prawy, 255 * OGRANICZENIE_MOCY_OMINIECIE_PRZESZKODY);
    delay(500);
    ominac = false;
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
    {
      int ilosc = 0;
      int suma = 0;
      for (int i = 0; i < 5; i++)
      {
        if (last_sensors[i] == true)
        {
          suma += wagi[i];
          ilosc++;
        }
      }
      return float(suma) / float(ilosc) * 2;
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
  if (input >= PROG_ODCZYTU_CZARNEGO)
    return true;
  else
    return false;
}

bool pomiar_odleglosci()
{
  int czas, dist;
  digitalWrite(trig, HIGH);
  delayMicroseconds(2);
  digitalWrite(trig, LOW);
  czas = pulseIn(echo, HIGH, 10000);
  dist = (czas / 58);
  if (dist <= ODLEGLOSC_BRZEGOWA && dist >= 3.0f )
    return true;
  else return false;
}

