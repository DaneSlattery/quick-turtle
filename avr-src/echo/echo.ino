// Dane Slattery
// Nick Raal
// Arduino Nano Communication Interface Program

#define STP 2
#define DIR 3
#define MS1 4
#define MS2 5
#define MS3 6
#define EN  7

String incoming_command;

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

void runMotor()
{
  digitalWrite(DIR, LOW);
  
//  digitalWrite(MS1, LOW); // full step
//  digitalWrite(MS2, LOW); // 
//  digitalWrite(MS3, LOW); //
  
//  digitalWrite(MS1, HIGH);// half step
//  digitalWrite(MS2, LOW); // 
//  digitalWrite(MS3, LOW); //

//  digitalWrite(MS1, LOW); // 1/4 step
//  digitalWrite(MS2, HIGH);// 
//  digitalWrite(MS3, LOW); //
//
//  digitalWrite(MS1, HIGH); // 1/8 step
//  digitalWrite(MS2, HIGH); // 
//  digitalWrite(MS3,  LOW); //

  digitalWrite(MS1, HIGH); // 1/16 step
  digitalWrite(MS2, HIGH); // 
  digitalWrite(MS3, HIGH); //
  
  digitalWrite(EN,  LOW); // enable the driver
  
  for (int x = 0; x < 133; x++)
  {
    digitalWrite(STP, HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(STP, LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }

  // wait, to stop inertia
 delay(500);
// resetPins();
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
      resetPins();
    }
    // STOP RESPONSE
    else if (incoming_command.equals("AT2\r\n"))
    {
      Serial.println("ST");
    }
  }
}
