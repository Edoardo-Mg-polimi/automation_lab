#define Electromagnet_Pin 11
#define IR_LED 8
#define Status_LED 13
#define Status_LED_Colour_2 5
#define IR_Sensor_Pin 1
#define POT_PIN 2
#define PWR_SW 2
#define MIN_MAP 700//Change This
#define MAX_MAP 1023

const int A = 1; //I think its a number in the range of 0-10
int B = 60;//I think its a number in the range of 20-300
const int C = 30;
const int D = 1000;//Change this only if nothing else works
int maxPower = 255;//Change this if your magnet over heats. 255=Maximum power

int oldPosition_Of_Object;
int Ambient_IR_Light;
const int minpower = 0;
int Position_Of_Object;
int Velocity_Of_Object;
int Loops_Done_So_Far = 0;

void setup()

{
  pinMode(Status_LED, OUTPUT);
  pinMode(IR_LED, OUTPUT);
  pinMode(Electromagnet_Pin, OUTPUT);
  pinMode(PWR_SW, INPUT_PULLUP);
  pinMode(Status_LED_Colour_2, OUTPUT);
  int oldPower_SW = digitalRead(2);
  int Power_SW = oldPower_SW;
  Serial.begin(115200);
  Serial.println("Adjust Values until your system levitates. Typing a number here wil change B:");

}




void loop() {

  Loops_Done_So_Far ++;
  if (Loops_Done_So_Far > 999)

  {
    Ambient_IR_Light = Read_Ambient_IR_Light();
    Loops_Done_So_Far = 0;
  }
  int Raw_Reading_From_Sensor = analogRead(IR_Sensor_Pin);
  Raw_Reading_From_Sensor = map(Raw_Reading_From_Sensor, MIN_MAP, MAX_MAP, 100, 1024);
  Raw_Reading_From_Sensor = 1024 - Raw_Reading_From_Sensor;

  Position_Of_Object = Raw_Reading_From_Sensor - Ambient_IR_Light;
  Velocity_Of_Object = Position_Of_Object - oldPosition_Of_Object;
  oldPosition_Of_Object = Position_Of_Object;
  int power = Position_Of_Object / A + Velocity_Of_Object * B + C;

  if (power > maxPower) power = maxPower;
  if (power < minpower) power = minpower;

  analogWrite(Electromagnet_Pin, power );
  analogWrite(Status_LED_Colour_2, power );
    digitalWrite (Status_LED, LOW);

  delayMicroseconds(D);
  Check_Serial_Port_And_SW();
}




int Read_Ambient_IR_Light()

{
  digitalWrite(IR_LED, LOW);
  delayMicroseconds(100);
  int Ambient_IR_Light_Value = 1024 - analogRead(IR_Sensor_Pin);
  digitalWrite(IR_LED, HIGH);
  return Ambient_IR_Light_Value;
}




void Check_Serial_Port_And_SW()

{
  int Power_SW_Status = digitalRead(PWR_SW);
  if (Power_SW_Status == LOW) {
    Serial.println ("Power Off");
    delay (20);
    digitalWrite (Electromagnet_Pin, LOW);
    digitalWrite (Status_LED_Colour_2, LOW);
    digitalWrite (Electromagnet_Pin, LOW);
    digitalWrite (Status_LED, HIGH);
    while (Power_SW_Status == LOW) {
      Power_SW_Status = digitalRead(PWR_SW);
    }
    Serial.println ("Power On");
    delay (20);
  }

  


  if (Serial.available())
  {
    B = Serial.parseInt();
    Serial.print("B set to ");
    Serial.println (B);
  }

}

