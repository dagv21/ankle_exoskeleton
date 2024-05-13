const int analogPin1 = A0;    
const int analogPin2 = A1; 

const int clockPin1 = 10;     
const int shiftOutPin1 = 9; 
const int signalPin1 = 8;    
const int clearPin1 = 13;
const int serialPin1 = 3;

const int clockPin2 = 7;    
const int shiftOutPin2 = 6; 
const int signalPin2 = 5;  
const int clearPin2 = 12;
const int serialPin2 = 2;

int iterations = 4; // Number of readings to average

int j = 1;

void setup() {
  pinMode(analogPin1, INPUT);
  pinMode(analogPin2, INPUT);
  
  pinMode(clockPin1, OUTPUT);
  pinMode(serialPin1, OUTPUT);
  pinMode(shiftOutPin1, INPUT);
  pinMode(clearPin1, OUTPUT);
  pinMode(signalPin1, OUTPUT);

  pinMode(clockPin2, OUTPUT);
  pinMode(serialPin2, OUTPUT);
  pinMode(shiftOutPin2, INPUT);
  pinMode(clearPin2, OUTPUT);
  pinMode(signalPin2, OUTPUT);

  Serial.begin(250000);

  // Enable Mux
  digitalWrite(signalPin1, HIGH);
  digitalWrite(signalPin2, HIGH);

  digitalWrite(clockPin1, LOW);
  digitalWrite(clockPin2, LOW);


}

void loop() {
  //Reset Mux
  digitalWrite(clearPin1, LOW);
  digitalWrite(clearPin2, LOW);

  digitalWrite(clearPin1, HIGH);
  digitalWrite(clearPin2, HIGH);

  int avg1[9] = {0}; 
  int avg2[9] = {0};

  //Activate each Mux output
    for (int i = 1; i <= 9; i++) {

    digitalWrite(serialPin1, i == 1); 
    digitalWrite(serialPin2, i == 1); 
      
    digitalWrite(clockPin1, HIGH);
    digitalWrite(clockPin2, HIGH);
    
    //Read and send sensor output
    int sum1 = 0;
    int sum2 = 0;

    delayMicroseconds(1);
    for (int j = 0; j < iterations; j++) {
      sum1 += analogRead(analogPin1);
      sum2 += analogRead(analogPin2);
      //delay(1); 
    }
    avg1[i-1] = sum1/iterations;
    avg2[i-1] = sum2/iterations;

    digitalWrite(clockPin1, LOW);
    digitalWrite(clockPin2, LOW);
  }
  for (int k = 1; k < 9; k++) {
    Serial.print(avg1[k]);
    Serial.print("\t");
    Serial.print(avg2[k]);
    Serial.print("\t");
  }
  Serial.println();


}