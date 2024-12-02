// 12/2/24 Chuck Staton

// this is written for RP2040 (MKS THR42 board) in Arduino IDE. You must use https://github.com/earlephilhower/arduino-pico/
// setup instructions for installing the github package: https://www.tomshardware.com/how-to/program-raspberry-pi-pico-with-arduino-ide

// I'm shit at programming anything but Python. This is a collection of other people's code that I found online, copied, pasted, and modified to fit my needs. 
// I don't remember where some of it came from, but here is a list of citations that should cover most of the important stuff:
// - Arduino PID temperature implementation (was kinda wack, needed help, but "worked"): https://electronoobs.com/eng_arduino_tut24_code2.php#google_vignette
// - Serial input basics. I think it's an excellent tutorial and serial in Arduino just sucks. I could be wrong; the tutorial could suck. https://forum.arduino.cc/t/serial-input-basics-updated/382007
// - Abducar's thermistor lookup table code. https://forum.arduino.cc/t/how-to-use-ntc-100k-thermistor-to-measure-high-temperature/472821/13
// - How to EEPROM https://docs.arduino.cc/learn/programming/eeprom-guide/

// The EEPROM stuff is Earle F. Philhower's simulation of EEPROM which is actually stored on Pico's flash, so try not to write too often to it.
// Commands which can be typed into the serial monitor:
// Sxxx - Set temperature (C). Ex: "S235" - sets setpoint to 235C
// Pxxx.xxx - Set PID Proportional constant. Ex: P2.25
// Ixxx.xxx - Set PID Integral constant. Ex: I0.25
// Dxxx.xxx - Set PID Derivative constant. Ex: D1.525
// PID and temperature setpoints survive power cycle, so just set it initially and it should work whenever powered on.

// TODO: 
// - Make a way to attach a potentiometer or something so you don't have to connect to PC to change the temp setpoint
// - Utilize the THR42's onboard TMC2209 to control a stepper in quasi-analog style in response to PWM input from finger trigger controller (emulate a DC motor). 

#include <EEPROM.h>
#define THERMISTOR_PIN A0
#define NUMTEMPS 20
// The following lookup table was found online and is not an exact match for the Peopoly thermistor but close enough. 
// I did try the converted lookup table from the peopoly 
short temptable[NUMTEMPS][2] = {
   {1, 821},
   {54, 252},
   {107, 207},
   {160, 182},
   {213, 165},
   {266, 152},
   {319, 141},
   {372, 131},
   {425, 123},
   {478, 115},
   {531, 107},
   {584, 100},
   {637, 93},
   {690, 86},
   {743, 78},
   {796, 70},
   {849, 60},
   {902, 49},
   {955, 34},
   {1008, 3}
};
int PWM_pin = 0;
float temperature_read = 0.0;
float set_temperature;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
//int kp = 9.1;   int ki = 0.3;   int kd = 1.8;
//float kp = 9.1;   float ki = 0.3;   float kd = 1.8;
float kp;
float ki;
float kd;
float PID_p = 0;    
float PID_i = 0;    
float PID_d = 0;
float PID_value_unlimited;
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
char firstLetter = '0';
boolean newData = false;
bool startup = true;
byte addr10;

void setup()
{
   pinMode(PWM_pin,OUTPUT); //HE0 output
   Time = millis();
   Serial.begin(9600);
   Serial.println("Starting temperature exerciser.");
   pinMode(1, OUTPUT); //fan0 output
   EEPROM.begin(512);
}

void loop()
{
  if (startup == true){
    addr10 = EEPROM.read(10);
    if (addr10 != 2){ // first run ever
      set_temperature = 215;
      saveTempSetting();
      kp = 1.0;
      ki = 0.2;
      kd = 1.0;
      savePIDSettings();
      EEPROM.write(10, 2);
      if (EEPROM.commit()) {
        Serial.println("EEPROM successfully committed");
      }else {
        Serial.println("ERROR! EEPROM commit failed");
      }
    }else{
      byte temp1 = EEPROM.read(0); // address 0 = temperature setpoint
      byte temp2 = EEPROM.read(1);
      //if (temp2 == 255)
      if (temp2 == 1){
        set_temperature = temp1+255;
      }else{
        set_temperature = temp1;
      }
      EEPROM.get(20,kp);    
      EEPROM.get(30,ki); 
      EEPROM.get(40,kd); 
    }
    startup = false;
  }
   int rawvalue = analogRead(THERMISTOR_PIN);
   int celsius = read_temp();
   int fahrenheit = (((celsius * 9) / 5) + 32);
   if (celsius < 17){
    digitalWrite(1, LOW); 
   }
   if (celsius >= 18){
    digitalWrite(1, HIGH); 
   }
   // First we read the real value of temperature
  temperature_read = celsius;
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read;
  //Calculate the P value
  PID_p = kp * PID_error;
  //Calculate the I value in a range on +-3
  if(-3 < PID_error <3)
  {
    PID_i = PID_i + (ki * PID_error);
  }
  else{
    PID_i = 0; //added to prevent integral windup
  }
  if (PID_i > 127){
    PID_i = 127; //added integral limit
  }
  if (PID_i < -127){
    PID_i = -127; //added integral limit
  }

  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;
  PID_value_unlimited = PID_value;
  //We define PWM range between 0 and 255
  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 255)  
  {    PID_value = 255;  }
  if (celsius > set_temperature){
    PID_value = 0;
  }
  //Now we can write the PWM signal to the mosfet on digital pin D3
  analogWrite(PWM_pin,PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.
  //int kp = 9.1;   int ki = 0.3;   int kd = 1.8;
//  Serial.print("Addr10: ");
//  Serial.print(addr10);
//  Serial.print(", Addr20: ");
//  Serial.print(EEPROM.read(20));
//  Serial.print(", Addr21: ");
//  Serial.print(EEPROM.read(21));
//  Serial.print(", Addr22: ");
//  Serial.print(EEPROM.read(22));
//  Serial.print(", Addr23: ");
//  Serial.print(EEPROM.read(23));
  Serial.print("PID_p: ");
  Serial.print(PID_p);
  Serial.print(", PID_i: ");
  Serial.print(PID_i);
  Serial.print(", PID_d: ");
  Serial.print(PID_d);
  Serial.print(", P: ");
  Serial.print(kp);
  Serial.print(", I: ");
  Serial.print(ki);
  Serial.print(", D: ");
  Serial.print(kd);
  Serial.print(", setP: ");
  Serial.print(set_temperature);
  Serial.print(", temp: ");
  Serial.print(celsius);
  Serial.print("C, error: ");
  Serial.print(PID_error);
  Serial.print(", output: ");
  Serial.print(PID_value);
  Serial.print(", outputUnlim: ");
  Serial.println(PID_value_unlimited);
  
  recvWithEndMarker();
  showNewData();
  
  delay(300);

}

int read_temp()
{
   int rawtemp = analogRead(THERMISTOR_PIN);
   int current_celsius = 0;

   byte i;
   for (i=1; i<NUMTEMPS; i++)
   {
      if (temptable[i][0] > rawtemp)
      {
         int realtemp  = temptable[i-1][1] + (rawtemp - temptable[i-1][0]) * (temptable[i][1] - temptable[i-1][1]) / (temptable[i][0] - temptable[i-1][0]);

         if (realtemp > 500)
            realtemp = 500; 

         current_celsius = realtemp;

         break;
      }
   }

   // Overflow: We just clamp to 0 degrees celsius
   if (i == NUMTEMPS)
   current_celsius = 0;

   return current_celsius;
}
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void showNewData() {
  if (newData == true) {
    char Sflag = 'S';
    char Pflag = 'P';
    char Iflag = 'I';
    char Dflag = 'D';
    char firstLetter = receivedChars[0];
    char DataChars[10];
    for (int i=0; i<11; i++) {
      DataChars[i] = receivedChars[i+1];
    }
    if (Sflag == firstLetter){     
      Serial.print("Temperature Setpoint change sent: ");
      Serial.println(DataChars);
      set_temperature = atof(DataChars);
      saveTempSetting();
    }
    if (Pflag == firstLetter){     
      Serial.print("Proportional factor change sent: ");
      Serial.println(DataChars);
      kp = atof(DataChars);
      savePIDSettings();
    }
    if (Iflag == firstLetter){     
      Serial.print("Integral factor change sent: ");
      Serial.println(DataChars);
      ki = atof(DataChars);
      savePIDSettings();
    }
    if (Dflag == firstLetter){     
      Serial.print("Derivative factor change sent: ");
      Serial.println(DataChars);
      kd = atof(DataChars);
      savePIDSettings();
    }
//    Serial.print("This just in ... ");
//    Serial.println(receivedChars);
    newData = false;
  }
}

void savePIDSettings() {
    //  uint8_t bytes[4] = { 0, 1, 2, 3 }; // fill this array with the four bytes you received
    //static_assert(sizeof(float) == 4, "float size is expected to be 4 bytes");
    //float f;
    //memcpy (&f, bytes, 4);
//  byte *ptr;   
//  ptr = (byte*)&kp;  //ptr holds beginning address of 4-byte memory space containing value of x 
//  for (int i = 0, address = 20; i < 4; i++, address++, ptr++)
//  {
//    EEPROM.write(address, *ptr);
//  }
  EEPROM.put(20,kp);
  EEPROM.put(30,ki);
  EEPROM.put(40,kd);
  if (EEPROM.commit()) {
    Serial.println("EEPROM successfully committed");
  }else {
    Serial.println("ERROR! EEPROM commit failed");
  }
}

void saveTempSetting() {
    if (set_temperature >310){
    set_temperature=310;
  }
  // EEprom can only hold up to 255
  if (set_temperature >254){
    EEPROM.write(0, set_temperature-255);
    EEPROM.write(1, 1);
  }else{
    EEPROM.write(0, set_temperature);
    EEPROM.write(1, 0);
  }
  
  if (EEPROM.commit()) {
    Serial.println("EEPROM successfully committed");
  }else {
    Serial.println("ERROR! EEPROM commit failed");
  }
}

//
//int sensorPin = A0;    // 100k thermistor
//int sensorValue = 0;  // variable to store the value coming from the sensor
//
//// the setup function runs once when you press reset or power the board
//void setup() {
//  // initialize serial communication:
//  Serial.begin(9600);
//  // initialize digital pin LED_BUILTIN as an output.
//  pinMode(1, OUTPUT);
//}
//
//// the loop function runs over and over again forever
//void loop() {
//  sensorValue = analogRead(sensorPin);
//  Serial.println(sensorValue);
//  // convert the value to resistance
//  float reading;
//  reading = (1023 / sensorValue)  - 1;     // (1023/ADC - 1) 
//  reading = 100000 * reading;  // 100K / (1023/ADC - 1)
//  Serial.print("Thermistor resistance "); 
//  Serial.println(reading);
//  digitalWrite(1, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(1000);                       // wait for a second
//  digitalWrite(1, LOW);    // turn the LED off by making the voltage LOW
//  delay(1000);                       // wait for a second
//}
