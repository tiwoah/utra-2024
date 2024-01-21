const int infraredSensorPinLeft = 11;
#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12
#define IR_SENSOR_RIGHT_ANALOG 0
#define IR_SENSOR_LEFT_ANALOG 1
#define MOTOR_SPEED 180

//Right motor
int enableRightMotor=6;
int rightMotorPin1=7;
int rightMotorPin2=8;

//Left motor
int enableLeftMotor=5;
int leftMotorPin1=7;
int leftMotorPin2=10;



void setup ( ) {
  Serial.print("test");
  Serial.begin (9600); //Starting the serial communication at 9600 baud rate
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

  // put your setup code here, to run once:
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  TCCR0B = TCCR0B & B11111000 | B00000010 ;



  rotateMotor(0,0);
}

void loop() {
  // // put your main code here, to run repeatedly:
  // // Serial.print("he he he ha");
  // int infraredValue = digitalRead(IR_SENSOR_RIGHT);
  // // Serial.println(infraredValue);

  //  int infraredValue2 = digitalRead(IR_SENSOR_LEFT);
  // // Serial.println(infraredValue2); 

  int rightIRSensorValue = analogRead(IR_SENSOR_RIGHT_ANALOG);
  int leftIRSensorValue = analogRead(IR_SENSOR_LEFT_ANALOG);

  // LOWER IS DARKER/CLOSER
  const int darkThreshold = 500;
  int rightDectectsBlackLine = rightIRSensorValue < darkThreshold;
  int leftDectectsBlackLine = leftIRSensorValue < darkThreshold;

  Serial.println(rightDectectsBlackLine);
  rotateMotor(180,-180);

  //If none of the sensors detects black line, then go straight
  // if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  // {
  //   rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  // }

  // int analogValue = analogRead(IR_SENSOR_LEFT_ANALOG);
  // Serial.println(analogValue);
  //   if (infraredValue == HIGH) {
  //   Serial.println("Line detected!");  // Print a message when the sensor detects a line
  // } else {
  //   Serial.println("No line detected.");
  // }

    // digitalWrite(rightMotorPin1,LOW);
    // digitalWrite(rightMotorPin2,LOW);     
 int motor_speed = map(200, 400, 0, 0, 255); 
  digitalWrite(rightMotorPin1, LOW);
digitalWrite(rightMotorPin2, HIGH);
analogWrite(enableRightMotor, motor_speed);

  delay(100);
}



void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  // analogWrite(enableRightMotor, abs(rightMotorSpeed));
  // analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}
