#include <NewPing.h>

#define TRIGGER_PIN1  4
#define ECHO_PIN1     4
#define TRIGGER_PIN2  10
#define ECHO_PIN2     10
#define TRIGGER_PIN3  13
#define ECHO_PIN3     13
#define MAX_DISTANCE 200
#define NUM_PINGS 3

NewPing sonar[NUM_PINGS] = {
  NewPing(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE),
  NewPing(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE),
  NewPing(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE),
};


//NewPing sonar(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
 
void setup() {
  Serial.begin(115200);
}
 
void loop() {
  /*
  delay(50);
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm());
  Serial.println(" cm");
*/
    unsigned int dists[NUM_PINGS];
  for (int i=0; i<NUM_PINGS; i++){
    delay(1000);
    dists[i] = sonar[i].ping_cm();
    Serial.print("Ping  ");
    Serial.print(i);  
    Serial.print(": ");
    Serial.print(dists[i]);
    Serial.print(" cm\n");
  }
}

