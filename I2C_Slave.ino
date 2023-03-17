//
// Serial debuging
//
#define SERIAL_DEBUG    true
#define Serial_BAUDRATE 115200

#include <PZEM004Tv30.h>
#include <Wire.h>
#include <avr/wdt.h>

//D1, D2 use to i2c address
#define I2C_ADDRESS 0x08

#define SYN 0x16
#define STX 0x02

#define mS_TO_S_FACTOR 1000

//define sensor pin
#define WIND_PIN A1
#define PZ_TX 4
#define PZ_RX 5

unsigned long read_sensor_peroid = 4 * mS_TO_S_FACTOR; // 4 minutes
unsigned long last_time = 0;

int seccount = 0;
int mincount = 0;
int hourcount = 0;
float wattpermin = 0.0;
float wattperh = 0.0;
float wattperday = 0.0;

float voltage = 0.0;
float current = 0.0;
float frequency = 0.0;
float power = 0.0;
float energy = 0.0;
float wind_speed = 0.0;

float vin;
float vout;
float voltagesensor;
//5.128
const float factor = 5.128;
const float vcc = 5.00;

//request command menu
int command_type = 0;
#define REQ_COMMAND 1
#define REQ_SENSOR_DATA 2

#define COMMAND_NONE 0
#define COMMAND_STATE 0x01
#define COMMAND_FAN_SPEED1 0x02
#define COMMAND_FAN_SPEED2 0x03
#define COMMAND_FAN_SPEED3 0x04

#define COMMAND_LINE_NOTIFY1 0x05
#define COMMAND_LINE_NOTIFY2 0x06

int request_type = 0;

PZEM004Tv30 pzem(PZ_TX, PZ_RX);

void read_voltage(void){
  float v = pzem.voltage();
    if(v != NAN){
      voltage = v;
      Serial.print("Voltage: "); Serial.print(voltage); Serial.println("V");
    }else{
        Serial.println("Error reading voltage");
    }
}

void read_current(void){
  float i = pzem.current();
    if(i != NAN){
      current = i;
      Serial.print("Current: "); Serial.print(current); Serial.println("A");
    }else{
      Serial.println("Error reading current");
    }
}

void read_frequency(void){
  float f = pzem.frequency();
    if(f != NAN){
      frequency = f;
      Serial.print("Frequency: "); Serial.print(frequency, 1); Serial.println("Hz");
    }else{
      Serial.println("Error reading frequency");
    }
}

void read_power(void){
   float p = pzem.power();
    if(p != NAN){
      power = p;
      Serial.print("Power: "); Serial.print(power); Serial.println("W");
      wattpermin += power;
    }else{
      Serial.println("Error reading power");
    }
}

void read_energy(void){
  float e = pzem.energy();
    if(e != NAN){
        energy = e;
        Serial.print("Energy: "); Serial.print(energy,3); Serial.println("kWh");
    }else{
        Serial.println("Error reading energy");
    }
}

void read_power_factor(void){
  float pf = pzem.pf();
    if(pzem.current() != NAN){
        Serial.print("PF: "); Serial.println(pf);
    }else{
        Serial.println("Error reading power factor");
    }
    Serial.println();
}

void wattperdays(void){
  if(hourcount == 60){
    Serial.print("watt per day: ");
    Serial.print(wattperday);
    Serial.println();
  }
  hourcount = 0;
  Serial.print("watt per day: ");
  Serial.print(wattperday);
  Serial.println();
}

void read_wind_speed(){
  if(power > 10.0){
    voltagesensor = analogRead(WIND_PIN);
    vout = (voltagesensor/1024)*vcc;
    vin = vout * factor;
    wind_speed = (vin - 0.7) * 1.63;
//    if(wind_speed < 0){
//      wind_speed = 0.0;
//    }
  }else{
    wind_speed = 0.0;
  }
  Serial.println(wind_speed);
}

void sendTomaster(int command){
  Wire.write(byte(SYN));
  Wire.write(byte(SYN));
  Wire.write(byte(STX));
  Wire.write(byte(command));

  command_type = 0;
}
//
// Get request type from I2C master
//
void receiveEvent(int howMany) {
  Serial.print("Got message from I2C master: ");
  Serial.println(howMany);
  int header = 0;
  wdt_enable(WDTO_15MS);
  while (Wire.available()) {
    char c = Wire.read();
    if(header == 0 && c == SYN){
      header++;
    }else if(header == 1 && c == SYN){
      header++;
    }else if(header == 2 && c == STX){
      Serial.print("Recived request type: ");
      request_type = Wire.read();
      Serial.println(request_type);
    }
  }
}
//
// Request callback from I2C master to sent back with previous request type
//
void requestEvent(){
  Serial.println("Got request event from I2C master");
  if(request_type == REQ_COMMAND){
    // I2C Master requests command from I2C Slave
    // Sent frame header (SYN, SYN, STX)
    if(command_type == 0){
      sendTomaster(COMMAND_NONE);
    }
    if(command_type == 1){
      sendTomaster(COMMAND_STATE);
    }
    if(command_type == 2){
      sendTomaster(COMMAND_LINE_NOTIFY1);
    }
    Serial.println("Send command success..");

  }else if(request_type == REQ_SENSOR_DATA){
    // I2C Master requests sensors data from I2C Slave
    
    byte bytes[4];

    // Send frame header (SYN, SYN, STX)
    Wire.write(byte(SYN));
    Wire.write(byte(SYN));
    Wire.write(byte(STX));
      
    // Voltage (Volt)
    memcpy(bytes, &voltage, sizeof(voltage));
    Wire.write(bytes, sizeof(voltage));
    // Current (Ampare)
    memcpy(bytes, &current, sizeof(current));
    Wire.write(bytes, sizeof(current));
    // Freequency (Hz)
    memcpy(bytes, &frequency, sizeof(frequency));
    Wire.write(bytes, sizeof(frequency));
    // Power (Watt)
    memcpy(bytes, &power, sizeof(power));
    Wire.write(bytes, sizeof(power));
    // Power (Watt Hour)
    memcpy(bytes, &energy, sizeof(energy));
    Wire.write(bytes, sizeof(energy));
    // Wind Speed ()
    memcpy(bytes, &wind_speed, sizeof(wind_speed));
    Wire.write(bytes, sizeof(wind_speed));
    
    Serial.print("Voltage: ");
    Serial.println(voltage);
    Serial.print("Current: ");
    Serial.println(current);
    Serial.print("Frequency: ");
    Serial.println(frequency);
    Serial.print("Power: ");
    Serial.println(power);
    Serial.print("Energy: ");
    Serial.println(energy);
    Serial.print("Wind speed: ");
    Serial.println(wind_speed);

    Serial.println("Send sensors data success..");
  }else{
    Wire.write(byte(SYN));
    Wire.write(byte(SYN));
    Wire.write(byte(STX));
    Wire.write(byte(0));
  }

  request_type = 0;
}

void check_condition(void){
   if(power > 10.0){
     if(voltage > 250.0){
        command_type = 1;
     }else if(wind_speed < 3.4 && power < 100.0){
        command_type = 2;
     }else {
        command_type = 0;
     }
   }
}

void setup() {
  Serial.begin(Serial_BAUDRATE);
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  Serial.println("Sensor is running");
  Serial.print("Buard rate : ");
  Serial.println(Serial_BAUDRATE);
  Serial.print("Debug Mode : ");
  Serial.println(SERIAL_DEBUG);
  Serial.print("I2C Address : ");
  Serial.println(I2C_ADDRESS);
}

void loop() {
  check_condition();
  if( millis() - last_time > read_sensor_peroid) {
    last_time = millis();
    read_wind_speed();
    if(!isnan(pzem.current())){
      read_voltage();
      read_current();
      read_power();
      read_energy();
      read_frequency();
      read_power_factor();
      
      /*
      if(seccount == 30){
        wattpermin /= 30.0;
        wattperh = wattperh + wattpermin;
        Serial.print("watt per min: ");
        Serial.print(wattpermin);
        Serial.println();
        wattpermin = 0.0;
        seccount = 0;
        mincount += 1;
      }
      
      if(mincount == 60){
        wattperh /= 60.0;
        wattperday += wattperh;
        Serial.print("watt per hour: ");
        Serial.print(wattperh);
        Serial.println();
        wattperh = 0.0;
        mincount = 0;
        hourcount += 1;
      }
      wattperdays();
      */
      
      seccount += 1;
    }else{
      Serial.println("PZEM module is offline..");
    }
    //Serial.print("watt per hour: ");
    //Serial.print(wattperh);
    //Serial.println();
    //Serial.print("watt per min: ");
    //Serial.print(wattpermin);
    //Serial.println();
    //Serial.println();
  }
}
