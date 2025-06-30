

// Motor pins
int rightPin1 = 4;
int rightPin2 = 5;
int leftPin1 = 6;
int leftPin2 = 7; 
int ERA = 9;
int ERB = 10;


void Back()
{
  digitalWrite(rightPin1, HIGH);
  digitalWrite(rightPin2, LOW);
  
  digitalWrite(leftPin1, LOW);
  digitalWrite(leftPin2, HIGH);

}

void Forward()
{
  digitalWrite(rightPin1, LOW);
  digitalWrite(rightPin2, HIGH);
  
  digitalWrite(leftPin1, HIGH);
  digitalWrite(leftPin2, LOW);

}


void setup() {

  // setting pins to recieve output
  pinMode(rightPin1, OUTPUT);
  pinMode(rightPin2, OUTPUT);
  pinMode(leftPin1, OUTPUT);
  pinMode(leftPin2, OUTPUT);

  pinMode(ERB, OUTPUT);
  pinMode(ERA, OUTPUT);

  Serial.begin(115200);

  delay(200);

  Serial.println("Initalization step complete");

  analogWrite(ERA, 100);
  analogWrite(ERB, 100);

}

void loop() {

  Forward();

  delay(5000);

  Back();

  delay(5000);

  Serial.println("Looping..");


}
