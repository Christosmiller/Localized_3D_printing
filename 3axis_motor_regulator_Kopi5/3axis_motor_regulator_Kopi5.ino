//Include encoder library
#include <Encoder.h>

//Serial variables
const int SS_PIN = 53;
volatile byte buffer[12];
volatile int byteCount = 0;
float positions[600][3];
int data_transfers = 0;

//Setup encoder
Encoder Enc1(20, 21);

//Define encoder and stepper resolutions
#define steps_per_rev 3200.0
#define pulses_per_rev 4000.0

//Define stepper control pins
#define steppin1 54
#define dirpin1 55
#define enpin1 38
#define steppin2 60
#define dirpin2 61
#define enpin2 56
#define steppin3 46
#define dirpin3 48
#define enpin3 62
#define steppin4 26
#define dirpin4 28
#define enpin4 24
#define steppin5 36
#define dirpin5 34
#define enpin5 30

//Loop variables
float velocity_interval1 = 1000.0;
long prev_micros_velocity1 = 0;
long target_interval = 20000;
int target_counter = 0;
int target_selector = 0;
bool run_flag = 0;
unsigned long prev_micros_target = 0;
unsigned long prev_micros_measure = 0;
float measure_interval = 10000;

//Stepper 1 variables
float angle1 = 0; //revs
float prev_angle1 = 0; //revs
float prev_prev_angle1 = 0; //revs
float target_angle1 = 0; //revs
float angular_velocity1 = 0; //revs/s
float max_angular_velocity1 = 5; //revs/s
unsigned long prev_micros_step1 = 0; //ms
unsigned int stepper_interval1 = 32767; //ms
float angular_acceleration1 = 0; //revs/s^2
float measured_angular_acceleration1 = 0; //revs/s^2
float target_aceleration1 = 0; //revs
float target_velocity1 = 0; //revs/s


//Stepper 2 variables
float angle2 = 0; //revs
float prev_angle2 = 0;//revs
float target_angle2 = 0; //revs
float angular_velocity2 = 0; //revs/s
float max_angular_velocity2 = 5; //revs/s
unsigned long prev_micros_step2 = 0; //ms
unsigned int stepper_interval2 = 32767; //ms
float angular_acceleration2 = 0; //revs/s^2
long stepper_position_2 = 0; //steps
float target_aceleration2 = 0; //revs
float target_velocity2 = 0; //revs/s

//Stepper 3 variables
float angle3 = 0; //revs
float prev_angle3 = 0; //revs
float target_angle3 = 0; //revs
float angular_velocity3 = 0; //revs/s
float max_angular_velocity3 = 5; //revs/s
unsigned long prev_micros_step3 = 0; //ms
unsigned int stepper_interval3 = 32767; //ms
float angular_acceleration3 = 0; //revs/s^2
long stepper_position_3 = 0; //steps
float target_aceleration3 = 0; //revs
float target_velocity3 = 0; //revs/s

//PD constants
float kp = 100.0;
float ki = 0.0;
float kd = 20.0;

//PD errors
float pe1 = 0.0;
float prev_pe1 = 0.0;
float de1 = 0.0;
float pe2 = 0.0;
float prev_pe2 = 0.0;
float de2 = 0.0;
float pe3 = 0.0;
float prev_pe3 = 0.0;
float de3 = 0.0;

void setup()
{
  //Setup stepper 1
  pinMode(steppin1,OUTPUT);
  pinMode(dirpin1,OUTPUT);
  pinMode(enpin1,OUTPUT);

  //Setup stepper 2
  pinMode(steppin2,OUTPUT);
  pinMode(dirpin2,OUTPUT);
  pinMode(enpin2,OUTPUT);

  //Setup stepper 3
  pinMode(steppin3,OUTPUT);
  pinMode(dirpin3,OUTPUT);
  pinMode(enpin3,OUTPUT);

  //Setup stepper 4
  pinMode(steppin4,OUTPUT);
  pinMode(dirpin4,OUTPUT);
  pinMode(enpin4,OUTPUT);

  //Setup stepper 5
  pinMode(steppin5,OUTPUT);
  pinMode(dirpin5,OUTPUT);
  pinMode(enpin5,OUTPUT);

  //Setup serial communications
  Serial.begin(115200);
}

void loop()
{
  //Serial handling
  if (Serial.available() > 0) 
  {
    // Buffer to store incoming data
    char buffer[32];
    int bytesRead = Serial.readBytesUntil('\n', buffer, 31);

    if (bytesRead > 0) 
    {
      buffer[bytesRead] = '\0';  // Null terminate the string

      // Parse the input string for 3 floats
      char* token = strtok(buffer, " ,");  // Split by space or comma

      if(strcmp(token, "get_data") == 0)
      {
        if(target_counter <= 599)
        {
          char buffer[32];
          int bytesRead = Serial.readBytesUntil('\n', buffer, 31);
          buffer[bytesRead] = '\0';  // Null terminate the string
          // Parse the input string for 3 floats
          char* token = strtok(buffer, " ,");  // Split by space or comma
          positions[target_counter][0] = atof(token);
          token = strtok(NULL, " ,");
          positions[target_counter][1] = atof(token);
          token = strtok(NULL, " ,");
          positions[target_counter][2] = atof(token);
          token = strtok(NULL, " ,");
          target_counter++;
        } 
        Serial.println("ack_data");
      }

      if(strcmp(token, "start_running") == 0)
      {
        run_flag = 1;
        target_selector = 0;
      }

      if(strcmp(token, "get_reg_const") == 0)
      {
        char buffer[32];
          int bytesRead = Serial.readBytesUntil('\n', buffer, 31);
          buffer[bytesRead] = '\0';  // Null terminate the string
          // Parse the input string for 3 floats
          char* token = strtok(buffer, " ,");  // Split by space or comma
          kp = atof(token);
          token = strtok(NULL, " ,");
          ki = atof(token);
          token = strtok(NULL, " ,");
          kd = atof(token);
          token = strtok(NULL, " ,");
      }

      if(strcmp(token, "move_to") == 0)
      {
        char buffer[32];
          int bytesRead = Serial.readBytesUntil('\n', buffer, 31);
          buffer[bytesRead] = '\0';  // Null terminate the string
          // Parse the input string for 3 floats
          char* token = strtok(buffer, " ,");  // Split by space or comma
          target_angle1 = atof(token);
          token = strtok(NULL, " ,");
          target_angle2 = atof(token);
          token = strtok(NULL, " ,");
          target_angle3 = atof(token);
          token = strtok(NULL, " ,");
      }

      if(strcmp(token, "return_data") == 0)
      {
        for(int g = 0; g <= target_counter; g++)
        {
          Serial.println(positions[g][0]);
          Serial.println(positions[g][1]);
          Serial.println(positions[g][2]);
        }
        target_angle1 = 0;
        target_angle2 = 0;
        target_angle3 = 0;
        target_counter = 0;
      }
    }
  }

  //Target handling
  if(micros()>=prev_micros_target+target_interval && run_flag==1 && target_selector <= target_counter)
  {
    target_angle1 = positions[target_selector][0];
    positions[target_selector][0] = angle1;
    target_angle2 = positions[target_selector][1];
    positions[target_selector][1] = angle2;
    target_angle3 = positions[target_selector][2];
    positions[target_selector][2] = angle3;
    if(target_selector <= target_counter-2)
    {
      //Calculate target velocities and accelerations
      target_velocity1 = (positions[target_selector+1][0]-positions[target_selector][0])/(target_interval);
      target_velocity2 = (positions[target_selector+1][1]-positions[target_selector][1])/(target_interval);
      target_velocity3 = (positions[target_selector+1][2]-positions[target_selector][2])/(target_interval);
      target_aceleration1 = (positions[target_selector][0]-2*positions[target_selector+1][0]+positions[target_selector+2][0])/(target_interval*target_interval);
      target_aceleration2 = (positions[target_selector][1]-2*positions[target_selector+1][1]+positions[target_selector+2][1])/(target_interval*target_interval);
      target_aceleration3 = (positions[target_selector][2]-2*positions[target_selector+1][2]+positions[target_selector+2][2])/(target_interval*target_interval);
    }
    else
    {
      //Reset target velocities and accelerations
      target_velocity1 = 0;
      target_velocity2 = 0;
      target_velocity3 = 0;
      target_aceleration1 = 0;
      target_aceleration2 = 0;
      target_aceleration3 = 0;
    }
    target_selector++;
    prev_micros_target = micros();
  }
  else if(target_selector == target_counter && run_flag==1)
  {
    Serial.println("process_finished");
    run_flag = 0;
  }

  //measure handling
  if(micros()>=prev_micros_measure+measure_interval)
  {
    //Calculate angle and angular velocity
    angle1 = Enc1.read()/pulses_per_rev; //Moved down here for timing reasons.
    angular_velocity1 = (angle1-prev_angle1)/(measure_interval/1000000);
    prev_angle1 = angle1;
    prev_micros_measure = micros();
  }

  //Velocity handling
  if(micros()>=prev_micros_velocity1+velocity_interval1)
  {
    //Calculate angles
    angle1 = Enc1.read()/pulses_per_rev;
    angle2 = stepper_position_2/steps_per_rev;
    angle3 = stepper_position_3/steps_per_rev;

    //Calculate errors for axis 3
    pe1 = target_angle1-angle1;
    de1 = (pe1-prev_pe1)/(velocity_interval1/1000000);
    prev_pe1 = pe1;

    //Calculate errors for axis 3
    pe2 = target_angle2-angle2;
    de2 = (pe2-prev_pe2)/(velocity_interval1/1000000);
    prev_pe2 = pe2;

    //Calculate errors for axis 3
    pe3 = target_angle3-(angle3+angle1);
    de3 = (pe3-prev_pe3)/(velocity_interval1/1000000);
    prev_pe3 = pe3;

    //Calculate new accelerations
    angular_acceleration1 = pe1*kp+de1*kd+target_aceleration1;
    angular_acceleration2 = pe2*kp+de2*kd+target_aceleration2;
    angular_acceleration3 = pe3*kp+de3*kd+target_aceleration3;

    //velocity handling
    angular_velocity1 += angular_acceleration1*(velocity_interval1/1000000);
    angular_velocity2 += angular_acceleration2*(velocity_interval1/1000000);
    angular_velocity3 += angular_acceleration3*(velocity_interval1/1000000);

    //Constrain angular velocities to max values
    angular_velocity1 = constrain(angular_velocity1,-5,5);
    angular_velocity2 = constrain(angular_velocity2,-5,5);
    angular_velocity3 = constrain(angular_velocity3,-5,5);

    //Set stepper direction pins
    digitalWrite(dirpin1, angular_velocity1 > 0);
    digitalWrite(dirpin2, angular_velocity2 > 0);
    digitalWrite(dirpin4, angular_velocity2 > 0);
    digitalWrite(dirpin3, angular_velocity3 > 0);
    digitalWrite(dirpin5, !(angular_velocity3 > 0));

    //Update stepper interval
    stepper_interval1 = 1.0/(abs(angular_velocity1)*(1.0/1000000.0)*steps_per_rev);
    stepper_interval2 = 1.0/(abs(angular_velocity2)*(1.0/1000000.0)*steps_per_rev);
    stepper_interval3 = 1.0/(abs(angular_velocity3)*(1.0/1000000.0)*steps_per_rev);

    //Save loop interval
    prev_micros_velocity1 = micros();
  }

  //Step handling, stepper 1
  if(micros()>=prev_micros_step1+stepper_interval1 && abs(pe1)>0)
  {
    digitalWrite(steppin1,HIGH);
    digitalWrite(steppin1,LOW);
    prev_micros_step1 = micros();
  }

  //Step handling, stepper 2 and 4
  if(micros()>=prev_micros_step2+stepper_interval2 && abs(pe2)>0)
  {
    digitalWrite(steppin2,HIGH);
    digitalWrite(steppin4,HIGH);
    digitalWrite(steppin2,LOW);
    digitalWrite(steppin4,LOW);
    if(angular_velocity2>0){stepper_position_2 = stepper_position_2 + 1;}
    if(angular_velocity2<0){stepper_position_2 = stepper_position_2 - 1;}
    prev_micros_step2 = micros();
  }

  //Step handling, stepper 3 and 5
  if(micros()>=prev_micros_step3+stepper_interval3 && abs(pe3)>0)
  {
    digitalWrite(steppin3,HIGH);
    digitalWrite(steppin5,HIGH);
    digitalWrite(steppin3,LOW);
    digitalWrite(steppin5,LOW);
    if(angular_velocity3>0){stepper_position_3 = stepper_position_3 + 1;}
    if(angular_velocity3<0){stepper_position_3 = stepper_position_3 - 1;}
    prev_micros_step3 = micros();
  }
}