// Dane  Slattery
// Nick Raal
// Arduino Nano Communication Interface Program

#define STP 2
#define DIR 3
#define MS1 4
#define MS2 5
#define MS3 6
#define EN  7

String incoming_command;
const float angle = 15.0; // degrees
float stepSize = 0.0;
int numSteps;

// the setup routine runs once when you press reset:
void setup() 
{
  // initialize serial communication with 9600 Baudrate
  Serial.begin(9600);

  // Setup pin-modes
  pinMode(STP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(EN,  OUTPUT);
  resetPins();

  // calculate the number of steps for a given angle
  stepSize = 1.8/16;
  numSteps = round(angle/stepSize);
//  Serial.println(numSteps);
}

void resetPins()
{
  digitalWrite(STP, LOW); // not stepping
  digitalWrite(DIR, LOW); // forward
  digitalWrite(MS1, LOW); // full step mode
  digitalWrite(MS2, LOW); // 
  digitalWrite(MS3, LOW); // 
  digitalWrite(EN, HIGH); // disabled
}

void runMotorTest()
{  
  // set the direction to counter clock-wise
  digitalWrite(DIR, LOW);

  digitalWrite(MS1, HIGH); // 1/16 step
  digitalWrite(MS2, HIGH); // 
  digitalWrite(MS3, HIGH); //

  digitalWrite(EN,  LOW); // enable the driver

  for (int x = 0; x < 3204; x++)
  {
    digitalWrite(STP, HIGH); //Trigger one step forward
    delayMicroseconds(500);
    digitalWrite(STP, LOW); //Pull step pin low so it can be triggered again
    delayMicroseconds(500);
  }
}
void runMotor()
{ 
  // set the direction to counter clock-wise
  digitalWrite(DIR, LOW);

  digitalWrite(MS1, HIGH); // 1/16 step
  digitalWrite(MS2, HIGH); // 
  digitalWrite(MS3, HIGH); //
  
  digitalWrite(EN,  LOW); // enable the driver
  
  for (int x = 0; x < numSteps; x++)
  {
    digitalWrite(STP, HIGH); //Trigger one step forward
    delayMicroseconds(750);
    digitalWrite(STP, LOW); //Pull step pin low so it can be triggered again
    delayMicroseconds(750);
  }

  // wait, to stop inertia
 delay(500);
}

// the loop routine runs over and over again forever:
void loop() {
  while(Serial.available())
  {
    // read the serial buffer
    incoming_command = Serial.readString();


    // INIT RESPONSE
    if (incoming_command.equals("AT0\r\n"))
    {
      Serial.println("OK");
    }
    // SPIN RESPONSE
    else if (incoming_command.equals("AT1\r\n"))
    {
      runMotor();
      Serial.println("SP");
//      resetPins();
    }
    // STOP RESPONSE
    else if (incoming_command.equals("AT2\r\n"))
    {
      resetPins();
      Serial.println("ST");
    }
    else if (incoming_command.equals("AT3\r\n"))
    {
      runMotorTest();
      Serial.println("TE");
    }
  }
}
