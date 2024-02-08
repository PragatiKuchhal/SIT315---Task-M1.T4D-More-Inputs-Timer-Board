// Define PIR sensor pins
const int pirSensorPin1 = 2;
const int pirSensorPin2 = 3;
const int pirSensorPin3 = 4;

// Define RGB LED pins
const int redPin = 9;
const int greenPin = 10;
const int bluePin = 11;
const int ledPin = 12;

// Initialize pirSensorStates array
volatile byte pirSensorStates[3] = {LOW, LOW, LOW};
 
void setup() 
{
  Serial.begin(9600);

  // Set up pins
  pinMode(pirSensorPin1, INPUT);
  pinMode(pirSensorPin2, INPUT);
  pinMode(pirSensorPin3, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // Set up pin change interrupts
  attachInterrupt(digitalPinToInterrupt(pirSensorPin1), pirSensorISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pirSensorPin2), pirSensorISR2, CHANGE);
  pciSetup(4); //initialize pin change interrupt for third pir sensor
  startTimer(0.5); //start timer with frequency of 0.5Hz
}

void loop() 
{
  // Main program loop
}

void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin)); 
  PCIFR |= bit (digitalPinToPCICRbit(pin));
  PCICR |= bit (digitalPinToPCICRbit(pin));
}
ISR (PCINT2_vect) //handle pin change interrupt
{
  pirSensorISR3();  //ISR for the pin change interrupt
}

void startTimer(double timerFrequency)
{
  noInterrupts();
  
  //Calculate the value for OCR1A based on the timer frequency
  uint16_t ocrValue = (uint16_t)(F_CPU / 1024.0 / timerFrequency - 1);
  
  //Set the Timer1 registers for CTC mode and set the OCR1A value
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = ocrValue;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  
  interrupts();
}

ISR(TIMER1_COMPA_vect)
{
  digitalWrite(ledPin, digitalRead(ledPin)^1);
}

void pirSensorISR1() {
  pirSensorStates[0] = digitalRead(pirSensorPin1);
  Serial.print("PIR Sensor 1: ");
  Serial.println(pirSensorStates[0] == HIGH ? "Motion Detected" : "No Motion");
  if (pirSensorStates[0] == HIGH) {
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
  } else {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
  }
}

void pirSensorISR2() {
  pirSensorStates[1] = digitalRead(pirSensorPin2);
  Serial.print("PIR Sensor 2: ");
  Serial.println(pirSensorStates[1] == HIGH ? "Motion Detected" : "No Motion");
  if (pirSensorStates[1] == HIGH) {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
    digitalWrite(bluePin, LOW);
  } else {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
  }
}

void pirSensorISR3() {
  pirSensorStates[2] = digitalRead(pirSensorPin3);
  Serial.print("PIR Sensor 3: ");
  Serial.println(pirSensorStates[2] == HIGH ? "Motion Detected" : "No Motion");
  if (pirSensorStates[2] == HIGH) {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, HIGH);
  } else {
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
  }
}