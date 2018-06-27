#include <NewPing.h>

#define TRIGGER_PIN1  10
#define ECHO_PIN1     10
#define TRIGGER_PIN2  11
#define ECHO_PIN2     11
#define TRIGGER_PIN3  12
#define ECHO_PIN3     12
#define TRIGGER_PIN4  13
#define ECHO_PIN4     13
#define MAX_DISTANCE 200

// Define number of devices connected
#define NUM_PINGS 4

// Add pins of each respective device
NewPing sonar[NUM_PINGS] = {
  NewPing(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE),
  NewPing(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE),
  NewPing(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE),
  NewPing(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE),
};


// Choose serial connection 
void setup() {
  Serial.begin(115200);
}


 
void loop() {
  
    unsigned int dists[NUM_PINGS];
  for (int i=0; i<NUM_PINGS; i++){
    delay(25);
    dists[i] = sonar[i].ping_cm();

    Serial.print("N");
    Serial.print(NUM_PINGS);
    Serial.print("I");  
    Serial.print(i);
    Serial.print("C");
    Serial.print(dists[i]);
    Serial.print("\n");

    /*
    Serial.print("Ping  ");
    Serial.print(i);  
    Serial.print(": ");
    Serial.print(dists[i]);
    Serial.print(" cm\n");

    */
  }
 
}

