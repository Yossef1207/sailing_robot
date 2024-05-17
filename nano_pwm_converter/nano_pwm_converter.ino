int PWMin  = 5;       // Nano has PWM pins 3,5,6,9,10,11 (digital)
int PWMout = 6;       // only pin 5 and 6 allow 980 Hz PWM
int inDir  = 0;       // input direction
int outDir = 0;       // output direction
int offset = 900;     // offset to be added to PWM  

void setup() {
  // comment for 'production' 
  //Serial.begin(9600); // debug info on serial console
  // configure pins
  pinMode(PWMin, INPUT);    // define input pin as input
  pinMode(PWMout, OUTPUT);    // define output pin as output
}

void loop() {
  inDir = pulseIn(PWMin, HIGH);   // read PWM signal from AS5040
  outDir = inDir + offset;    // compensate for offset
  digitalWrite(PWMout,HIGH);    // start pulse
  delayMicroseconds(outDir);    
  digitalWrite(PWMout, LOW);    // end pulse
  delayMicroseconds(2000-outDir); // not sure, check for proper PWM and rate
  // comment for 'production' 
  //Serial.print("input wind direction: "); // debug info on serial console
  //Serial.println(inDir); // debug info on serial console
  //Serial.print("output wind direction:          "); // debug info on serial console
  //Serial.println(outDir); // debug info on serial console
}
