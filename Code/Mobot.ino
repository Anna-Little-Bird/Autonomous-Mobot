// include libraries
#include <Wire.h>     // library for running I2C interface
#include <Servo.h>    // library for controlling the servo motor
#include <NewPing.h>  // library for ultrasonic sensor

// create servo objects to control the servo motors
Servo HS311;
Servo SG90;

// initialise constants
const byte SCAN=0,POSITION=1,COLOUR=2,GRAB=3,AVOID=4,STOP=5;   // states
const byte RED=1,GREEN=2,BLUE=3;  // colours
const byte SENSOR_1=0x29,SENSOR_2=0x39;   // sensors addresses: 0x29 - TCS34725, 0x39 - APDS-996
const unsigned int OPEN=1450,CLOSE=950;   // clamp's states
const byte inA1=7,inA2=8,inB1=12,inB2=13,pwmA=3,pwmB=11,PWM1=9,PWM2=6,INT=2,LED=10,ECHO=4,TRIG=5; // Arduino pins
const byte FORWARD=1,BACKWARD=2,RIGHT=3,LEFT=4,BRAKE=5;   // motors' states

// initialise global variables
unsigned int red_val,green_val,blue_val,maximum_distance=200,distance,distanceRight,distanceLeft;
byte i,col_buffer[2],state,angle,count=0;
bool object=false;
volatile bool border=false;

// ultrasonic sensor function
NewPing sonar(TRIG,ECHO,maximum_distance);

void setup()
{
  // motor driver pins' settings
  pinMode(inA1,OUTPUT); // in1 of L298N motor driver
  pinMode(inA2,OUTPUT); // in2 of L298N motor driver
  pinMode(pwmA,OUTPUT); // PWM for motor A
  pinMode(pwmB,OUTPUT); // PWM for motor B
  pinMode(inB1,OUTPUT); // in3 of L298N motor driver
  pinMode(inB2,OUTPUT); // in4 of L298N motor driver
  
  // interrupt settings
  pinMode(INT,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT),border_control,FALLING);
  
  // initialise servo motors
  pinMode(PWM1,OUTPUT); // PWM of servo motor HS-311
  pinMode(PWM2,OUTPUT); // PWM of servo motor SG90
  HS311.attach(PWM1,900,1800); // attach servo to pin 9, determine min & max pulse width (in microseconds)
  HS311.writeMicroseconds(OPEN);  // clamps are open
  SG90.attach(PWM2);    // attach servo to pin 6
  SG90.write(70);      // central position
  
  // initialise I2C protocol
  Wire.begin();         // start I2C interface in master mode
  Wire.setClock(400000);  // fast mode (400 kHz)
  
  // setting up the colour sensors
  setup_adc_time(SENSOR_1,100);   // in ms, from 2.4ms to 700ms
  setup_wait_time(SENSOR_1,500);  // in ms*10, from 2.4ms to 614ms
  setup_gain(SENSOR_1,16);        // gain (1x, 4x, 16x, 60x)
  enable_colour_sensor(SENSOR_1);
  setup_adc_time(SENSOR_2,50);    // in ms, from 2.78ms to 712ms
  setup_wait_time(SENSOR_2,500);  // in ms*100, from 2.78ms to 712ms
  setup_gain(SENSOR_2,16);        // gain (1x, 4x, 16x, 60x)
  setup_pulse(4,5);               // pulse width in microseconds (4,8,16,32), number of pulses (1-64)
  setup_pulse_gain(50,2);         // LED current in mA (12,25,50,100), proximity gain (1x,2x,4x,8x)
  setup_prox_persistence(2);      // interrupt persistence (0-15)
  setup_prox_limits(100,255);     // interrupr proximity limits, low & high
  Wire.beginTransmission(SENSOR_2);   // clear interrupt flags
  Wire.write(0xE5);
  Wire.write(0x00);
  Wire.endTransmission();
  enable_colour_sensor(SENSOR_2);
  pinMode(LED,OUTPUT);    // LED
  digitalWrite(LED,LOW);  // turn off the LED
  state=SCAN;             // initial state
}

void loop()
{
  // BORDER CONTROL //
  if (border==true)   // when interrupt occurs
  {
    if (object==true)   // if Mobot has the object
    {
      change_speed(70);
      move_mobot(FORWARD);  // go forward for 0.5 sec
      delay(500);
      state=STOP;           // go into STOP state
    }
    else                // if Mobot doesn't have the object
    {
      change_speed(70);   // avoid the border
      move_mobot(BACKWARD);
      delay(700);
      move_mobot(RIGHT);
      change_speed(250);
      delay(700);
      move_mobot(BRAKE);
      change_speed(70);
    }
    Wire.beginTransmission(SENSOR_2);   // clear interrupt flags
    Wire.write(0xE5);
    Wire.write(0x00);
    Wire.endTransmission();
    border=false;       // reset border variable
  }
  // STATE MACHINE //
  switch(state)
  {   
    case SCAN:
      for (i=10;i<=70;i+=30)    // scan the area from 30 to 90 degrees
      {
        SG90.write(i);
        delay(300);
        distance=readPing();    // double reading to exclude errors
        if (distance<=35)
        {
          distance = readPing();
          if (distance<=35)
          {
            if (object==false)  // if Mobot doesn't have the object
            {
              angle=i+20;       // store the angle value
              state=POSITION;   // go to POSITION state
            }
            else                // if Mobot has the object
            {state=AVOID;}      // go to AVOID state
          }
        }
      }
      for (i=130;i>=70;i-=30)   // scan the area from 150 to 90 degrees
      {
        SG90.write(i);
        delay(300);
        distance=readPing();    // double reading to exclude errors
        if (distance<=35)
        {
          distance = readPing();
          if (distance<=35)
          {
            if (object==false)  // if Mobot doesn't have the object
            {
              angle=i+20;       // store the angle value
              state=POSITION;   // go to POSITION state
            }
            else                // if Mobot has the object
            {state=AVOID;}      // go to AVOID state
          }
        }
      }
      if (state==SCAN)          // SCAN state movements
      {
        change_speed(70);
        move_mobot(FORWARD);
        delay(400);
        move_mobot(BRAKE);
      }
      break;
    case POSITION:
      count=0;              // reset counter
      move_mobot(BRAKE);
      change_speed(250);
      if (angle!=90)        // if the object isn't in front of Mobot
      {
        switch (angle)      // perform turns to align Mobot
        {
          case 30:
            move_mobot(RIGHT);
            delay(350);
            break;
          case 60:
            move_mobot(RIGHT);
            delay(175);
            break;
          case 120:
            move_mobot(LEFT);
            delay(175);
            break;
          case 150:
            move_mobot(LEFT);
            delay(350);
            break;             
        }
        move_mobot(BRAKE);
        delay(100);
        for (i=10;i<=70;i+=30)    // scan the area from 30 to 90 degrees
        {
          SG90.write(i);
          delay(300);
          distance=readPing();    // double reading to exclude errors
          if (distance<=35)
          {
            angle=i+20;           // store the angle value
            distance=readPing();
            if (distance<=35)
            {angle=i+20;}         // store the angle value
          }
          else if (distance>35)   // if Mobot loses the object
          {count=count+1;}        // increase counter
        }
        for (i=130;i>=70;i-=30)   // scan the area from 150 to 90 degrees
        {
          SG90.write(i);
          delay(300);
          distance=readPing();    // double reading to exclude errors
          if (distance<=35)
          {
            angle=i+20;           // store the angle value
            distance=readPing();
            if (distance<=35)
            {angle=i+20;}         // store the angle value
          }
          else if (distance>35)   // if Mobot loses the object
          {count=count+1;}        // increase counter
        }
      }
      else if (angle==90)         // if the object is in front of Mobot
      {state=COLOUR;}             // go to COLOUR state
      if (count==6)               // if there is no object in the scanning area
      {
        count=0;                  // reset the counter
        change_speed(70);         // move backwards and go to SCAN state
        move_mobot(BACKWARD);
        delay(500);
        move_mobot(BRAKE);
        state=SCAN;
      }
      break;
    case COLOUR:
      SG90.write(70);             // turn servo motor to central position
      digitalWrite(LED,HIGH);     // turn on LED
      change_speed(70);           // start approaching the object
      move_mobot(FORWARD);
      distance=readPing();        // double reading to exclude errors
      distance=readPing();
      // delay to stop in front of the object
      if (distance<30)
      {delay(600);}
      else if ((distance<34)&&(distance>=30))
      {delay(700);}
      else if ((distance>=34)&&(distance<38))
      {delay(800);}
      else if (distance>=38)
      {delay(900);}
      move_mobot(BRAKE);
      delay(1000);                // delay to check the colour
      // get colour data
      read_colour(SENSOR_1,BLUE);
      read_colour(SENSOR_1,RED);
      read_colour(SENSOR_1,GREEN);
      // decrease RED & GREEN values to increase detection accuracy
      green_val=green_val*8/10;
      red_val=red_val*8/10;
      if ((blue_val>=green_val)&&(blue_val>red_val))  // if the object is blue
      {
        state=GRAB;               // go to GRAB state
        object=true;
      }
      else                        // if the object isn't blue
      {
        HS311.writeMicroseconds(OPEN);  // clamps are open
        delay(100);
        digitalWrite(LED,LOW);    // turn off the LED
        state=AVOID;              // go to AVOID state
      }
      break;
    case GRAB:
      HS311.writeMicroseconds(CLOSE);  // clamps are closed
      delay(100);
      digitalWrite(LED,LOW);      // turn off the LED
      state=SCAN;                 // go to SCAN state
      break;
    case AVOID:
      move_mobot(BACKWARD);       // move Mobot backwards
      delay(700);
      move_mobot(BRAKE);
      distance = readPing();        // check the distance in front
      distanceRight = lookRight();  // check the distance to the right
      delay(300);
      distanceLeft = lookLeft();    // check the distance to the left
      delay(300);
      change_speed(250);
      if (distanceRight >= distanceLeft)  // if distance to the right is greater than to the left
      {
        move_mobot(RIGHT);                // turn right
        delay(500);
        move_mobot(BRAKE);
      }
      else                                // if distance to the left is greater than to the right
      {
        move_mobot(LEFT);                 // turn left
        delay(500);
        move_mobot(BRAKE);
      }
      state=SCAN;                         // go to SCAN state
      break;
    case STOP:
      move_mobot(BRAKE);                  // stop Mobot
      HS311.writeMicroseconds(OPEN);      // open the clamps
      delay(100);
      break;
  }
}

// ULTRASONIC SENSOR FUNCTIONS //
int readPing()
{
  int cm;
  delay(70);
  cm=sonar.ping_cm();
  if (cm==0)
  {cm=250;}
  return cm;
}
int lookRight()
{  
  SG90.write(10);     // turn the sensor to the right
  delay(150);
  int distance = readPing();  // measure the distance
  delay(150);
  SG90.write(70);     // turn the sensor to the central position
  return distance;
}

int lookLeft()
{
  SG90.write(130);    // turn the sensor to the left
  delay(150);
  int distance = readPing();  // measure the distance
  delay(150);
  SG90.write(70);     // turn the sensor to the central position
  return distance;
}

// MOTORS FUNCTIONS //
void move_mobot(byte direction_val)
 {
  switch (direction_val)
  {
    case FORWARD:
      digitalWrite(inA1,HIGH);  // motor A goes forward
      digitalWrite(inA2,LOW);
      digitalWrite(inB1,HIGH);  // motor B goes forward
      digitalWrite(inB2,LOW);
      break;
    case BACKWARD:
      digitalWrite(inA1, LOW);  // motor A goes backward
      digitalWrite(inA2, HIGH);
      digitalWrite(inB1, LOW);  // motor B goes backward
      digitalWrite(inB2, HIGH);
      break;
    case RIGHT:
      digitalWrite(inA1, HIGH);  // motor A goes forward
      digitalWrite(inA2, LOW);
      digitalWrite(inB1, LOW);   // motor B goes backward
      digitalWrite(inB2, HIGH);
      break;
    case LEFT:
      digitalWrite(inB1, HIGH);  // motor B goes forward
      digitalWrite(inB2, LOW);
      digitalWrite(inA1, LOW);   // motor A goes backward
      digitalWrite(inA2, HIGH);
      break;
    case BRAKE:
      digitalWrite(inA1, LOW);  // turn off motor A
      digitalWrite(inA2, LOW);
      digitalWrite(inB1, LOW);  // turn off motor B
      digitalWrite(inB2, LOW);
      break;
  }
}
void change_speed(byte speed_val)
{
  analogWrite(pwmA,speed_val);  // change the speed of motor A
  analogWrite(pwmB,speed_val);  // change the speed of motor B
}

// COLOUR SENSOR FUNCTIONS //
unsigned int read_colour(byte address, byte colour)
{
  switch (colour)
  {
    case RED:
      if (address==SENSOR_1)        // define the register address for red colour and read the data
      {read_data(address,0xB6);}
      else if (address==SENSOR_2)
      {read_data(address,0x96);}
      red_val=col_buffer[1];        // store received data into red_val
      red_val=red_val<<8;
      red_val=red_val+col_buffer[0];
      return red_val;
      break;
    case GREEN:
      if (address==SENSOR_1)        // define the register address for green colour and read the data
      {read_data(address,0xB8);}
      else if (address==SENSOR_2)
      {read_data(address,0x98);}
      green_val=col_buffer[1];      // store received data into green_val
      green_val=green_val<<8;
      green_val=green_val+col_buffer[0];
      return green_val;
      break;
    case BLUE:
      if (address==SENSOR_1)        // define the register address for blue colour and read the data
      {read_data(address,0xBA);}
      else if (address==SENSOR_2)
      {read_data(address,0x9A);}
      blue_val=col_buffer[1];       // store received data into blue_val
      blue_val=blue_val<<8;
      blue_val=blue_val+col_buffer[0];
      return blue_val;
      break;
  }
}
byte read_data(int address,int reg_address)
{
  Wire.beginTransmission(address);
  Wire.write(reg_address);      // start the command to read 2 bytes of colour data
  Wire.endTransmission();
  Wire.requestFrom(address,2);  // read 2 bytes
  col_buffer[0]=Wire.read();
  col_buffer[1]=Wire.read();
  return col_buffer;
}
void setup_adc_time(byte address,unsigned int adc_time)
{
  // calculate ADC time for writing into the register
  if (address==SENSOR_1)
  {adc_time=256-((adc_time*10)/24);}
  else if (address==SENSOR_2)
  {adc_time=256-((adc_time*100)/278);}
  Wire.beginTransmission(address);
  if (address==SENSOR_1)    // define the register address for ADC time
  {Wire.write(0x01);}
  else if (address==SENSOR_2)
  {Wire.write(0x81);}
  Wire.write(adc_time);     // write the ADC time into register
  Wire.endTransmission();
}
void setup_wait_time(byte address,unsigned int wait_time)
{
  // calculate wait time for writing into the register
  if (address==SENSOR_1)
  {wait_time=256-(wait_time/24);}
  else if (address==SENSOR_2)
  {wait_time=256-(wait_time/278);}
  Wire.beginTransmission(address);
  if (address==SENSOR_1)    // define the register address for wait time
  {Wire.write(0x03);}
  else if (address==SENSOR_2)
  {Wire.write(0x83);}
  Wire.write(wait_time);    // write the wait time into register
  Wire.endTransmission();
}
void setup_gain(byte address,byte gain)
{
  Wire.beginTransmission(address);
  if (address==SENSOR_1)    // define the register address for gain value
  {Wire.write(0x0F);}
  else if (address==SENSOR_2)
  {Wire.write(0x8F);}
  switch (gain)             // write gain value into register
  {
    case 1:
      Wire.write(0x00);   // 1x gain
      break;
    case 4:
      Wire.write(0x01);   // 4x gain
      break;
    case 16:
      Wire.write(0x02);   // 16x gain
      break;
    case 60:
      Wire.write(0x03);   // 60x or 64x gain
      break;
  }
  Wire.endTransmission();
}
void setup_pulse(byte p_length,byte p_count)   // only for APDS
{
  byte value;
  Wire.beginTransmission(SENSOR_2);
  Wire.write(0x8E);
  switch (p_length)   // calculate value of pulse length for writing into register
  {
    case 4:
      value=0x00;
      break;
    case 8:
      value=0x40;
      break;
    case 16:
      value=0x80;
      break;
    case 32:
      value=0xC0;
      break;
  }
  value=value+(p_count-1);  // calculate value of both pulse length and number of pulse for writing
  Wire.write(value);
  Wire.endTransmission();
}
void setup_pulse_gain(byte led_drive,byte p_gain)
{
  byte value;
  Wire.beginTransmission(SENSOR_2);   // read the value from Control Register One of APDS
  Wire.write(0x8F);
  Wire.endTransmission();
  Wire.requestFrom(SENSOR_2,1);
  value=Wire.read();
  Wire.beginTransmission(SENSOR_2);   // define the register address
  Wire.write(0x8F);
  switch (led_drive)                  // define LED drive value
  {
    case 12:
      value=value+0xC0;
      break;
    case 25:
      value=value+0x80;
      break;
    case 50:
      value=value+0x40;
      break;
    case 100:
      value=value+0x00;
      break;
  }
  switch (p_gain)                     // combine pulse gain value and LED drive value
  {
    case 1:
      value=value+0x00;
      break;
    case 2:
      value=value+0x04;
      break;
    case 4:
      value=value+0x08;
      break;
    case 8:
      value=value+0x0C;
      break;
  }
  Wire.write(value);
  Wire.endTransmission();
}
// FOR INTERRUPTS //
void setup_prox_limits(byte low_limit,byte high_limit) // only for APDS
{
  Wire.beginTransmission(SENSOR_2);     // write lower interrupt limit
  Wire.write(0x89);
  Wire.write(low_limit);
  Wire.endTransmission();
  Wire.beginTransmission(SENSOR_2);     // write higher interrupt limit
  Wire.write(0x8B);
  Wire.write(high_limit);
  Wire.endTransmission();
}
void setup_prox_persistence(byte persistence)    // only for APDS
{
  persistence=persistence<<4;           // calculate persistence for writing into register
  Wire.beginTransmission(SENSOR_2);     // write persistence into register
  Wire.write(0x8C);
  Wire.write(persistence);
  Wire.endTransmission();
}
void enable_colour_sensor(byte address)
{
  Wire.beginTransmission(address);    // define the register address to enable sensors
  if (address==SENSOR_1)
  {
    Wire.write(0x00);
    Wire.write(0x0B);   // PON=1 (power on), AEN=1 (light sensing), WEN=1 (wait)
  }
  else if (address==SENSOR_2)
  {
    Wire.write(0x80);
    Wire.write(0x2D);   // PON=1 (power on), AEN=0 (light sensing), WEN=1 (wait), PEN=1 (proximity sensing), PIEN=1 (proximity interrupts)
  }
  Wire.endTransmission();
}

// Interrupt Service Routine
void border_control(void)
{border=true;}