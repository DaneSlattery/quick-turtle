//***********************************
// Big Easy Stepper code            *
// Author: Nick Raal                *
//         : Dane Slattery          *
//***********************************


// Defining each pin
#define stp 2 //Logic input: trigger motor to move 1 step
#define dir 3 //Logic Input: direction
#define ms1 4 //Microstep determines step resolution + excitation mode
#define ms2 5
#define ms3 6
#define en  7 //Logic input: enables FET functionality
#define slp 8 //Logic input: enter sleep mode
         
void setup() {
  // put your setup code here, to run once:
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(ms1, OUTPUT);
  pinMode(ms2, OUTPUT);
  pinMode(ms3, OUTPUT);
  pinMode(en, OUTPUT);
  pinMode(slp, OUTPUT);

  resetPins(); // set pins to default state

  //Setup serial connection to begin motor
  Serial.begin(9600);
  Serial.println("Press 1 for standard forward stepping");
  Serial.println("Press 2 for sixteenth forward stepping");
  Serial.println();
}

void resetPins() {
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW); //Low = Forward
  digitalWrite(ms1, LOW);
  digitalWrite(ms2, LOW);
  digitalWrite(en, HIGH);
  digitalWrite(slp, HIGH);

}

// Steps the motor forward.
void StepForward() {
  digitalWrite(dir, LOW); //set direction to forward
  for(int x = 1; x < 10; x++){
    digitalWrite(stp, HIGH);
    delay(1);
    digitalWrite(stp, LOW); // set low to trigger again
    delay(1);
  }
}

void SixteenStep(){
  digitalWrite(dir, LOW);
  // For Sixteenth step 4W1-2 Phase
  digitalWrite(ms1, HIGH);
  digitalWrite(ms2, HIGH);
  digitalWrite(ms3, HIGH);

  for(int x = 1; x < 100; x++){
    digitalWrite(stp, HIGH);
    delay(1);
    digitalWrite(stp, LOW);
    delay(1);
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  while(Serial.available()){
    char input = Serial.read(); // read what user types
    digitalWrite(en, LOW); // allow motor to work
    if(input == '1'){
      StepForward();
      
    } else if(input == '2'){
      SixteenStep();
    } else {
      Serial.println("Invalid Option");
      
    }
    resetPins();
  }
  
}
