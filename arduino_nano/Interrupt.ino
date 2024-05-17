const int pinSpeed = 2;                             //pin connected to wind speed
const int pinDir = 3;                               //pin connected to wind direction
const float mTime = 0.1;                            //measuring time (s) for wind speed counting
const int offset = 7199;                            //900µs  25000/3.125*0.9-1  baseline offset for pwm 
const int dirOffset = 522;                           //offset for wind direction sensor boat 3
const float speedCoeff = 0.5;                       //coefficient to convert rpm in cm/s
const float speedOffset = 0.0;
unsigned long counter;
float speed;
int dir;
const int meanCount = 5;
int calcDirValues[meanCount];
int calcDirIndex = 0;
int calcDirSum = 0;


void setup() {
  //configure pins 9 and 10 as PWN output 
   for (int i = 0; i < meanCount; i++) {
    calcDirValues[i] = 0;
  }
  TCCR1A = (1<<COM1A1) + (1<<COM1B1) + (1<<WGM11);  //Phase Correct PWM
  TCCR1B = (1<<WGM13)  + (1<<CS10);                 //Phase Correct PWM
  ICR1 = 24999;                                     //320Hz  16000000/(320*2)-1 --> 25000 counts per 3.125 ms --> 8 counts per us
  OCR1A = offset;                                   //900µs  25000/3.125*0.9-1  baseline offset for speed 
  OCR1B = offset;                                   //900µs  25000/3.125*0.9-1  baseline offset for direction
  //set pins 9 and 10 as PWN output  
  DDRB |= (1<<PB1) | (1<<PB2);                      //Output: D9, D10
  Serial.begin(115200);                             //serial output --> uncomment below
}

void loop() {
  measure();                                        //count rotations
  noInterrupts();
  dir = pulseIn(pinDir, HIGH, 5000);                //wait 5000µs for pwm pulse
  interrupts();
  //Serial.println(speed);
  //Serial.println(dir);
  calcSpeed();
  calcDir();
  //Serial.println(speed);
  //Serial.println(dir);
  OCR1A = (offset + speed);    
  OCR1B = (offset + dir);
}

void calcDir() {
  // todo: calibrate and scale output to 1 deg / 3 µs (?), i.e., 
  // 0 .. 360 deg --> 0 .. 1080-1 us --> 0 .. 8640   
  dir = constrain(dir, 0, 1023);                    //Determine actual pulse width range first
  dir = (dir - dirOffset + 1023) % 1023;
  dir = map(dir, 0, 1023, 0, 8640-1);

  //Serial.print("dir before: ");
  //Serial.println(dir);
  calcDirValues[calcDirIndex] = dir;
  for (int i = 0; i < meanCount; i++) {
      calcDirSum += calcDirValues[i];
  }

  dir = calcDirSum / meanCount;
  //Serial.print("calcDirSum: ");
  //Serial.println(calcDirSum);
  calcDirSum = 0;


  calcDirIndex = (calcDirIndex + 1) % meanCount;
  //Serial.print("dir after: ");
  //Serial.println(dir);
  if (calcDirIndex == 0) {
    calcDirIndex = 0;
  }
          //900-1923µs including offset OCR1B
}  

void calcSpeed() {
  // todo: calibrate and scale output to 1 cm/s / µs (?), i.e., 
  // 9600 = 1200 µs = 12 m/s ~ 24kts = 6 Bft --> too much for our current boat anyway
  speed = speed * speedCoeff + speedOffset;  
  speed = constrain(speed, 0, 1200);               //speed in rpm --> app. 0 - 14.8m/s   needs calibration!
  speed = map(speed, 0, 1200, 0, 9600);             //900-2100µs including offset OCR1A
}  

void measure() {
  counter = 0;
  attachInterrupt(digitalPinToInterrupt(pinSpeed), countup, CHANGE);
  delay(1000 * mTime);
  detachInterrupt(digitalPinToInterrupt(pinSpeed));
  speed = (counter * 15) / (128 * mTime);           // (512 counts/revolution) / (60/s) --> check for overflow
}

void countup() {
  counter++;
}
