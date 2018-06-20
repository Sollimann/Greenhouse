#include <NewPing.h>

#define NUM_PINGS 4
#define RLY_PIN 12
#define MAX_DISTANCE 200

#define DRIVE_PIN 8
#define DIR_PIN 9

#define DRIVE_BUTTON_PIN 6
#define MODE_BUTTON_PIN 7
#define LED_PIN 13

#define TOP_PIN 11
#define BOT_PIN 10

#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

uint8_t current_sensor = 0;          // Keeps track of which sensor is active.

unsigned long pingTimer[NUM_PINGS]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[NUM_PINGS];         // Where the ping distances are stored.


NewPing sonar[NUM_PINGS] = {     // Sensor object array.
  NewPing(2, 2, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(3, 3, MAX_DISTANCE),
  NewPing(4, 4, MAX_DISTANCE),
  NewPing(5, 5, MAX_DISTANCE)
};



unsigned long ser_dir_start = 0;
unsigned long ser_dir_time = 2000;

bool prev_drive_button = LOW;
bool prev_mode_button = LOW;

int low = 30;
int high = 40;
int stop_thres = 65;
int motor_command = 0;
int mid;


long wc;

String serial_command = "";
bool serial_ready = false;

int mode, prev_mode;

int latest_motor_command = -1;

enum MotorCommands
{
  GO_UP = 1,
  GO_DOWN = -1,
  STOP = 0
};

enum Modes
{
  M_MANUAL,
  M_AUTO,
  M_SERIAL,
  M_OFF,
  M_SEARCH,
  M_TO_TOP,
  M_TO_BOT,
  M_NUMBER_OF_MODES
};


// command data
String SD_M_AUTO = "auto";
String SD_M_SERIAL = "ser";
String SD_M_MANUAL = "man";
String SD_M_OFF = "off";
String SD_M_SEARCH = "srch";
String SD_M_TO_TOP = "top";
String SD_M_TO_BOT = "bot";
String SD_C_UP = "up";
String SD_C_DOWN = "down";
String SD_C_STOP = "stop";
String SD_C_LOFF = "loff";
String SD_C_LON = "lon";


// error messages
String SE_SYNTAX_LONG = "ER:lng";
String SE_SYNTAX_HEADER = "ER:hdr";
String SE_MAN_ON = "ER:manon";
String SE_UNKNOWN = "ER:unkn";
String SE_FAIL = "fail";

// reply messages
//String SR_NEW_MODE = "mset";
//String SR_NEW_DIR = "dset";
//String SR_LOFF = "loff";
//String SR_LON = "lon";




// message tail
char S_TAIL = ',';

// identifiers
String SI_COMMAND = "!";
String SI_REQUEST = "?";
String SI_RESPONSE = "~";

String SI_CMS = "c";
String SI_END_STOP = "e";
String SI_DIR = "d";
String SI_LIGHT = "l";
String SI_MODE = "m";
String SI_STOP_THRES = "t";
String SI_INTERVAL = "i";


String SI_FIRST = SI_COMMAND + SI_REQUEST + SI_RESPONSE;
String SI_SECOND = SI_CMS + SI_END_STOP + SI_DIR + SI_LIGHT + SI_MODE + SI_STOP_THRES + SI_INTERVAL;
String SI_THIRD = ":";
bool but_up;

void setup() {
  Serial.begin(115200);


  mid = (low + high)/2;
  mode = M_MANUAL;
  prev_mode = M_MANUAL;

  pinMode(DRIVE_BUTTON_PIN, INPUT);
  pinMode(MODE_BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DRIVE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(RLY_PIN, OUTPUT);
  pinMode(TOP_PIN, INPUT);
  pinMode(BOT_PIN, INPUT);
  

  prev_drive_button = LOW;
  but_up = true;


  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < NUM_PINGS; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

}



void loop() {

  bool mode_button = digitalRead(MODE_BUTTON_PIN);
  digitalWrite(LED_PIN, mode_button);
  
  if (mode_button)
  {
    mode = M_MANUAL;
    bool but = debounce(DRIVE_BUTTON_PIN, prev_drive_button);

    if (but && !prev_drive_button)
    {
      but_up = !but_up;
    }

    if (but)
    {
      if (but_up)
        motor_command = GO_UP;
      else
        motor_command = GO_DOWN;
    }
    else
    {
      motor_command = STOP;
    }
    prev_drive_button = but;
  }
  else if (prev_mode == M_MANUAL)
  {
    mode = M_OFF;
  }


  for (uint8_t i = 0; i < NUM_PINGS; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * NUM_PINGS;  // Set next time this sensor will be pinged.
      if (i == 0 && current_sensor == NUM_PINGS - 1)
      {
        wc = oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      }
      sonar[current_sensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      current_sensor = i;                          // Sensor being accessed.
      cm[current_sensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[current_sensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }

  


  if (mode != prev_mode)
  {
    motor_command = 0;
  }

  if (mode == M_OFF)
  {
    motor_command = 0;
  }

  else if (mode == M_SERIAL)
  {
    if (ser_dir_start + ser_dir_time < millis())
      motor_command = 0;

    
  }

  else if (mode == M_SEARCH)
  {

    bool success = false;
    
    if (wc > high)
    {
      motor_command = GO_DOWN;
    }
    else if (wc < low)
    {
      motor_command = GO_UP;
    }
    else if (motor_command == GO_UP && wc > mid)
    {
      success = true;
    }
    else if (motor_command == GO_DOWN && wc < mid)
    {
      success = true;
    }

    if (motor_command == GO_UP && digitalRead(TOP_PIN))
      success = true;
    else if (motor_command == GO_DOWN && digitalRead(BOT_PIN))
      success = true;

    if (success)
    {
      motor_command = STOP;
      mode = M_AUTO;
    }

  }

  else if (mode == M_TO_BOT)
  {

    bool success = false;
    
    if (wc > high)
    {
      motor_command = GO_DOWN;
    }
    else if (wc < low)
    {
      success = true;
    }
    else if (motor_command == GO_DOWN && wc < mid)
    {
      success = true;
    }

    if (motor_command == GO_DOWN && digitalRead(BOT_PIN))
      success = true;

    if (success)
    {
      motor_command = STOP;
      mode = M_OFF;
    }

  }

  else if (mode == M_TO_TOP)
  {

    bool success = false;

    if (!digitalRead(TOP_PIN))
      motor_command = GO_UP;
    else
      success = true;

    if (success)
    {
      motor_command = STOP;
      mode = M_OFF;
    }

  }



  else if (mode == M_AUTO)
  {

    if (wc >= stop_thres)
    {
      motor_command = STOP;
    }

    else if (wc > high)
    {
      motor_command = GO_DOWN;
    }
    else if (wc < low)
    {
      motor_command = GO_UP;
    }
    else if (motor_command == GO_UP && wc > mid)
    {
      motor_command = STOP;
    }
    else if (motor_command == GO_DOWN && wc < mid)
    {
      motor_command = STOP;
    }


  }

  if (digitalRead(TOP_PIN) && motor_command == GO_UP)
  {
    motor_command = STOP;
    //Serial.println("dont go UP!");
  }
  else if (digitalRead(BOT_PIN) && motor_command == GO_DOWN)
  {
    motor_command = STOP;
    //Serial.println("dont go DOWN!");

  }


  if (motor_command == STOP)
  {
    digitalWrite(DRIVE_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
  }
  else if (motor_command == GO_DOWN)
  {
    digitalWrite(DRIVE_PIN, HIGH);
    digitalWrite(DIR_PIN, LOW);
  }
  else if (motor_command == GO_UP)
  {
    digitalWrite(DRIVE_PIN, HIGH);
    digitalWrite(DIR_PIN, HIGH);
  }

  latest_motor_command = motor_command;

  prev_mode = mode;

 // Serial.println(mode);







  

  //unsigned int dists[NUM_PINGS];
  
  //for (int i=0; i<NUM_PINGS; i++)
  //{
  //  dists[i] = sonars[i].ping();  
  //  Serial.print(dists[i] / US_ROUNDTRIP_CM);
  //  Serial.print(" ");
  //  
  //}

  //Serial.println();
  
  //digitalWrite(RLY_PIN, HIGH);
  //delay(1000);
  //digitalWrite(RLY_PIN, LOW);
  //delay(1000);
  // put your main code here, to run repeatedly:

}




    



long oneSensorCycle()
{
  wc = stop_thres;
  for (int i=0; i<NUM_PINGS; i++)
  {
    //Serial.print(cm[i]);
    //Serial.print(" ");
    if (cm[i] < wc && cm[i] != 0)
    {
      wc = cm[i];
    }

  }
  //Serial.print(",\n");
  return wc;

}


long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}


void serialEvent()
{
 
  int max_bytes = 10;
  bool finished_reading = false;
  
  while (!finished_reading && Serial.available() && !serial_ready) // recheck serial is available
  {
    String response = "ok,";
    char inChar = (char)Serial.read(); // get the new byte:
    int current_byte = serial_command.length();
    //Serial.println(current_byte);
    
    bool ok = false;

    
    if (current_byte == 2)
    {

 
      if ((String)inChar == SI_THIRD)
      {
        ok = true;
      }

    }
    
    
    else if (current_byte == 1)
    {

      for (int i=0; i<SI_SECOND.length(); i++)
      {
        if (inChar == SI_SECOND[i])
        {
          ok = true;
        }
      }
    }
    else
    {
      ok = true;
    }
    
    if (!ok)
    {
      serial_command = "";
      current_byte = 0;
      ok = true;
    }
      
      
    
    
    if (current_byte == 0)
    {
      ok = false;
      for (int i=0; i<SI_FIRST.length(); i++)
      {
        if (inChar == SI_FIRST[i])
        {
          ok = true;
        }
      }
    }
    
    if (!ok)
    {

      finished_reading = true;
     // response = SE_SYNTAX_HEADER + S_TAIL;
     // serial_command = "";
    }
    
    else if (current_byte > max_bytes)
    {
      finished_reading = true;
     // response = SE_SYNTAX_LONG + S_TAIL;
     // serial_command = "";
    }
    
    else if (inChar == S_TAIL)
    {
      serial_ready = true;
      finished_reading = true;
      //response = SR_COMMAND + S_TAIL;
    }
    
    if(finished_reading)
    {

      String reply = "";
  
      if (!ok)
      {
        reply = SE_SYNTAX_HEADER;
      }
  
       else if (current_byte > max_bytes)
      {
        reply = SE_SYNTAX_LONG;
      }

      // REQUESTS
      else if (serial_command == (SI_REQUEST + SI_CMS + SI_THIRD))
      { 
        for (int i=0; i<NUM_PINGS; i++)
        {
          reply += (String)cm[i];
          
          if (i != NUM_PINGS - 1)
            reply += " ";
            
        }
      }
      else if (serial_command == (SI_REQUEST + SI_MODE + SI_THIRD))
      {
        if (mode == M_MANUAL)
          reply = SI_MODE + SI_THIRD + SD_M_MANUAL;
        else if (mode == M_AUTO)
          reply = SI_MODE + SI_THIRD + SD_M_AUTO;
        else if (mode == M_SERIAL)
          reply = SI_MODE + SI_THIRD + SD_M_SERIAL;
        else if (mode == M_OFF)
          reply = SI_MODE + SI_THIRD + SD_M_OFF;
        else if (mode == M_SEARCH)
          reply = SI_MODE + SI_THIRD + SD_M_SEARCH;
        else if (mode == M_TO_TOP)
          reply = SI_MODE + SI_THIRD + SD_M_TO_TOP;
        else if (mode == M_TO_BOT)
          reply = SI_MODE + SI_THIRD + SD_M_TO_BOT;
      }

      else if (serial_command == (SI_REQUEST + SI_DIR + SI_THIRD))
      {
        reply = SI_DIR + SI_THIRD + (String)latest_motor_command;
      }

      else if (serial_command == (SI_REQUEST + SI_END_STOP + SI_THIRD))
      {
        reply = SI_END_STOP + SI_THIRD + (String)digitalRead(TOP_PIN) + " " + digitalRead(BOT_PIN);
      }
      else if (serial_command == (SI_REQUEST + SI_LIGHT + SI_THIRD))
      {
        if (digitalRead(RLY_PIN))
        {
          reply = SI_LIGHT + SI_THIRD + SD_C_LON;
        }
        else
        {
          reply = SI_LIGHT + SI_THIRD + SD_C_LOFF;
        }
      }
      else if (serial_command == (SI_REQUEST + SI_STOP_THRES + SI_THIRD))
      {
        reply = SI_STOP_THRES + SI_THIRD + (String)stop_thres;   
      }
      else if (serial_command == (SI_REQUEST + SI_INTERVAL + SI_THIRD))
      {
        reply = SI_INTERVAL + SI_THIRD + (String)low + " " + (String) high;   
      }
    
      // MODE COMMANDS
      else if (serial_command == (SI_COMMAND + SI_MODE + SI_THIRD + SD_M_AUTO) )
      {
        if (mode != M_MANUAL)
        {
          mode = M_AUTO;
          reply = SI_MODE + SI_THIRD + SD_M_AUTO;
        }
        else
        {
          reply = SE_MAN_ON;
        }
      }
      else if (serial_command == (SI_COMMAND + SI_MODE + SI_THIRD + SD_M_SERIAL) )
      {
        if (mode != M_MANUAL)
        {
          mode = M_SERIAL;
          reply = SI_MODE + SI_THIRD + SD_M_SERIAL;
        }
        else
        {
          reply = SE_MAN_ON;
        }
      }
      else if (serial_command == (SI_COMMAND + SI_MODE + SI_THIRD + SD_M_OFF) )
      {
        if (mode != M_MANUAL)
        {
          mode = M_OFF;
          reply = SI_MODE + SI_THIRD + SD_M_OFF;
        }
        else
        {
          reply = SE_MAN_ON;
        }
      }
      else if (serial_command == (SI_COMMAND + SI_MODE + SI_THIRD + SD_M_SEARCH) )
      {
        if (mode != M_MANUAL)
        {
          mode = M_SEARCH;
          reply = SI_MODE + SI_THIRD + SD_M_SEARCH;
        }
        else
        {
          reply = SE_MAN_ON;
        }
      }
      else if (serial_command == (SI_COMMAND + SI_MODE + SI_THIRD + SD_M_TO_TOP) )
      {
        if (mode != M_MANUAL)
        {
          mode = M_TO_TOP;
          reply = SI_MODE + SI_THIRD + SD_M_TO_TOP;
        }
        else
        {
          reply = SE_MAN_ON;
        }
        
      }

      else if (serial_command == (SI_COMMAND + SI_MODE + SI_THIRD + SD_M_TO_BOT) )
      {
        if (mode != M_MANUAL)
        {
          mode = M_TO_BOT;
          reply = SI_MODE + SI_THIRD + SD_M_TO_BOT;
        }
        else
        {
          reply = SE_MAN_ON;
        }
      }
    
      // RUN COMMANDS
      else if (serial_command == (SI_COMMAND + SI_DIR + SI_THIRD + SD_C_UP) )
      {
        if (mode != M_MANUAL)
        {
          motor_command = GO_UP;
          ser_dir_start = millis();
          reply = SI_DIR + SI_THIRD + SD_C_UP;
        }
        else
        {
          reply = SE_MAN_ON;
        }
      }
      else if (serial_command == (SI_COMMAND + SI_DIR + SI_THIRD + SD_C_DOWN) )
      {
        if (mode != M_MANUAL)
        {
          motor_command = GO_DOWN;
          ser_dir_start = millis();
          reply = SI_DIR + SI_THIRD + SD_C_DOWN;
        }
        else
        {
          reply = SE_MAN_ON;
        }
      }
      else if (serial_command == (SI_COMMAND + SI_DIR + SI_THIRD + SD_C_STOP) )
      {
        if (mode != M_MANUAL)
        {
          motor_command = STOP;
          reply = SI_DIR + SI_THIRD + SD_C_STOP;
        }
        else
        {
          reply = SE_MAN_ON;
        }
      }
      else if (serial_command == (SI_COMMAND + SI_LIGHT + SI_THIRD + SD_C_LOFF) )
      {
        digitalWrite(RLY_PIN, LOW);
        reply = SI_LIGHT + SI_THIRD + SD_C_LOFF;
      }
      else if (serial_command == (SI_COMMAND + SI_LIGHT + SI_THIRD + SD_C_LON) )
      {
        digitalWrite(RLY_PIN, HIGH);
        reply = SI_LIGHT + SI_THIRD + SD_C_LON;
      }

      else if (serial_command.substring(0,3) == (SI_COMMAND + SI_STOP_THRES + SI_THIRD))
      {
        String data_string = serial_command.substring(3);
        
        bool valid_input = true;
        for (int i=0; i<data_string.length(); i++)
        {
          if (!isDigit(data_string.charAt(i)))
            valid_input = false;
        }

        if (valid_input)
        {
          stop_thres = data_string.toInt();
          reply = SI_STOP_THRES + SI_THIRD + data_string;
        }
        else
        {
          reply = SI_STOP_THRES + SI_THIRD + SE_FAIL;
        }

        
      }
      else if (serial_command.substring(0,3) == (SI_COMMAND + SI_INTERVAL + SI_THIRD))
      {
        int c_i = 3;
        String low_in, high_in;
        
        char c_c = serial_command.charAt(c_i);
        while (isDigit(c_c))
        {
          low_in += c_c;
          c_i++;
          c_c = serial_command.charAt(c_i);
        }
        
        c_i++;
        c_c = serial_command.charAt(c_i);
        while (isDigit(c_c))
        {
          high_in += c_c;
          c_i++;
          c_c = serial_command.charAt(c_i);
        }

        low = low_in.toInt();
        high = high_in.toInt();

        reply = serial_command.substring(1);
        

        
      }
      else
      {
        reply = SE_UNKNOWN;
      }
    
      reply = SI_RESPONSE + reply + S_TAIL;
      serial_command = "";
      serial_ready = false;

      Serial.println(reply);
    


    }
    else
    {
      serial_command += inChar;
    }
    
    
  }
}


bool debounce(int pin, bool prev)
{
  bool cur = digitalRead(pin);
  if (cur != prev)
  {
    delay(5);
    cur = digitalRead(pin);
  }

  return cur;
}


void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[current_sensor].check_timer())
    cm[current_sensor] = sonar[current_sensor].ping_result / US_ROUNDTRIP_CM;
}



