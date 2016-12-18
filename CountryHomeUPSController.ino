#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <DigitalIO.h>
#include <Bounce2.h>
#include <avr/wdt.h>

// Идентификатор ноды
#define NODE_ID 90

#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay

#define CHILD_ID_VOLTMETER 1
#define CHILD_ID_AMPERMETER 2
#define CHILD_ID_WATTMETER 3
#define CHILD_ID_INTVCC 4

#define CHILD_ID_TEMP_ONBOARD         10  // Temp sensor on relay
#define CHILD_ID_TEMP_LOWER           11 // Temp sensor on top 
#define CHILD_ID_TEMP_UPPER           12 //Onboard temp sensor

#define CHILD_ID_LIGHT_RED            20            
#define CHILD_ID_LIGHT_GREEN          21             
#define CHILD_ID_LIGHT_RELAY_POWER    22            
#define CHILD_ID_LIGHT_RELAY_STATUS   23   

#define CHILD_ID_RELAY                30 

#define CHILD_ID_BATTERY              40 

#define CHILD_ID_COMMONSTATUS         90 

#define REBOOT_CHILD_ID                 100
#define DISABLE_TEMPDIFF_CHECK_CHILD_ID 101
#define RECHECK_SENSOR_VALUES           102

#define VOLTMETER_PIN                     A0
#define AMPERMETER_PIN                    A1
#define TEMPERATURE_PIN                   6
#define LIGHT_SENSOR_RED_PIN              A2
#define LIGHT_SENSOR_GREEN_PIN            A3
#define LIGHT_RELAY_POWER_PIN             A6            
#define LIGHT_RELAY_STATUS_PIN            A7   
#define RELAY_PIN                         4

#define GREEN_LED_PIN  8
#define RED_LED_PIN    7

#define COUNT 50

float oldValue_voltmeter = -1;
float oldValue_ampermeter = -1;
float oldValue_wattmeter = -1;
int oldValue_batteryLevel = -1;

float R1 = 99200.0; // resistance of R1 (100K) -see text!
float R2 = 9930.0; // resistance of R2 (10K) - see text!

// эту константу (typVbg) необходимо откалибровать индивидуально
const float typVbg = 1.106; //1.119; // 1.0 -- 1.2 1.106 1.114 //1.128
float Vcc = 0.0;
float MaxVoltage = 0.0;
int i;
float curVoltage;

int mVperAmp = 100; // Current sensor: use 185 for 5A Module, 100 for 20A Module and 66 for 30A Module
int RawValue= 0;
int ACSoffset = 2500; 
double Voltage = 0;
double Amps = 0;
int VQ;


boolean metric = true;          // Celcius or fahrenheid




float lastTemp1 = -1;
float lastTemp2 = -1;
float lastTemp3 = -1;
long previousTempMillis = 0;        // last time the sensors are updated
long TempsensorInterval = 20000;     // interval at which we will take a measurement ( 30 seconds)

long previousVoltageMillis = 0;      
long VoltagesensorInterval = 60000;    
long VoltagesensorIntervalOnPower = 60000;

long previousLightMillis = 0;        // last time the sensors are updated
long LightsensorInterval = 1000;     // interval at which we will take a measurement ( 30 seconds)
int lastLightLevel1;  // Holds last light level
   // Holds last light level
int lastLightLevel2;             // Holds last light level

long previousRelayStatusMillis = 0; 
long RelayStatussensorInterval = 60000;     
int lastRelayStatusLightLevel1;             // Holds last light level
int lastRelayStatusLightLevel2;             // Holds last light level

long previousTDCMillis=0;
long TDCsensorInterval=300000;

boolean boolTempDiffCheckDisabled = false;

boolean boolOnBattery = false;
boolean boolRecheckRelayStatus=false;
boolean boolRecheckPowerControllerStatus=false;
boolean boolRecheckSensorValues = false;

OneWire oneWire(TEMPERATURE_PIN); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

 
MySensor gw;

MyMessage VoltmeterMsg(CHILD_ID_VOLTMETER, V_VOLTAGE);
MyMessage AmpermeterMsg(CHILD_ID_AMPERMETER, V_CURRENT);
MyMessage WattmeterMsg(CHILD_ID_WATTMETER, V_WATT);

MyMessage IntVCCMsg(CHILD_ID_INTVCC, V_WATT);

MyMessage Temp1Msg(CHILD_ID_TEMP_ONBOARD, V_TEMP);
MyMessage Temp2Msg(CHILD_ID_TEMP_LOWER, V_TEMP);
MyMessage Temp3Msg(CHILD_ID_TEMP_UPPER, V_TEMP);
MyMessage BatteryLightMsg(CHILD_ID_LIGHT_RED, V_LIGHT_LEVEL);
MyMessage InputPowerLight(CHILD_ID_LIGHT_GREEN, V_LIGHT_LEVEL);
MyMessage RelayPowerLightMsg(CHILD_ID_LIGHT_RELAY_POWER, V_LIGHT_LEVEL);
MyMessage RelayStatusLightMsg(CHILD_ID_LIGHT_RELAY_STATUS, V_LIGHT_LEVEL);
MyMessage msgRelay1(CHILD_ID_RELAY, V_LIGHT);
MyMessage msgBatteryLevel(CHILD_ID_BATTERY, V_VAR1);
MyMessage CriticalAlarmMsg(CHILD_ID_COMMONSTATUS, V_TRIPPED);
MyMessage TempDiffCheckStateMsg(DISABLE_TEMPDIFF_CHECK_CHILD_ID, V_TRIPPED);

void setup() {

      pinMode(GREEN_LED_PIN, OUTPUT);
      pinMode(RED_LED_PIN, OUTPUT);
     digitalWrite(GREEN_LED_PIN,LOW);
     digitalWrite(RED_LED_PIN,LOW);
   

    Serial.begin(115200);
    Serial.println("Begin setup");
    // Initialize library and add callback for incoming messages
    
    gw.begin(incomingMessage, NODE_ID, false);

    // Send the sketch version information to the gateway and Controller
    gw.sendSketchInfo("UPS controller", "1.1");  

    //controller status light sensors
    pinMode(LIGHT_SENSOR_RED_PIN, INPUT);
    pinMode(LIGHT_SENSOR_GREEN_PIN, INPUT);
    gw.present(CHILD_ID_LIGHT_RED, S_LIGHT_LEVEL);
    gw.present(CHILD_ID_LIGHT_GREEN, S_LIGHT_LEVEL);  
    boolRecheckPowerControllerStatus=true;
    checkPowerControllerStatus();
    
    metric = gw.getConfig().isMetric;

    //present voltmeter sensor
    pinMode(VOLTMETER_PIN, INPUT);
    gw.present(CHILD_ID_VOLTMETER, S_MULTIMETER); 

    //current sensor
    pinMode(AMPERMETER_PIN, INPUT);
    gw.present(CHILD_ID_AMPERMETER, S_MULTIMETER);

    //power sensor
    gw.present(CHILD_ID_WATTMETER, S_POWER);
    gw.present(CHILD_ID_INTVCC, S_POWER);
    
    
    //temperature sensors
    gw.present(CHILD_ID_TEMP_ONBOARD, S_TEMP);
    gw.present(CHILD_ID_TEMP_LOWER, S_TEMP);
    gw.present(CHILD_ID_TEMP_UPPER, S_TEMP);




    //relay
    pinMode(LIGHT_RELAY_POWER_PIN, INPUT);
    pinMode(LIGHT_RELAY_STATUS_PIN, INPUT);    
    gw.present(CHILD_ID_LIGHT_RELAY_POWER, S_LIGHT_LEVEL);
    gw.present(CHILD_ID_LIGHT_RELAY_STATUS, S_LIGHT_LEVEL);  
    pinMode(RELAY_PIN, OUTPUT);     
    gw.present(CHILD_ID_RELAY, S_LIGHT);
    
    //reboot sensor command
    gw.present(REBOOT_CHILD_ID, S_BINARY); 
 

//reread temp sensors
          sensors.requestTemperatures();
    float temperature = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(0) * 10.)) / 10.; 
          temperature = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(1) * 10.)) / 10.;    
          temperature = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(2) * 10.)) / 10.; 

//battery level sensor
    gw.present(CHILD_ID_BATTERY, S_MULTIMETER);

//critical alarm message sensor
  gw.present(CHILD_ID_COMMONSTATUS, S_DOOR);
 

//disable temp difference check
   gw.present(DISABLE_TEMPDIFF_CHECK_CHILD_ID, S_LIGHT); 

//reget sensor values
  gw.present(RECHECK_SENSOR_VALUES, S_LIGHT);    

    //Enable watchdog timer
    wdt_enable(WDTO_8S);
    
    Serial.println("End setup");  
  
//VQ = determineVQ(AMPERMETER_PIN); //Quiscent output voltage - the average voltage ACS712 shows with no load (0 A)
VQ=509;//508
        // Serial.print("VQ2: ");
        // Serial.println(VQ);    
//delay(1000);
  
}



void checkAmpsVoltage()
{
  
float vout = 0.0;
float vin = 0.0;

int value = 0;  
 
  unsigned long currentVoltageMillis = millis();
  
  if((currentVoltageMillis - previousVoltageMillis > VoltagesensorInterval) || boolRecheckSensorValues) {
  // Save the current millis
  previousVoltageMillis = currentVoltageMillis;
  
  
 


  Vcc = readVcc();   
  
  // считываем точное напряжение с A0, где будет находиться наш вольтметр с делителем напряжения
  curVoltage = 0.0;
  for (i = 0; i < COUNT; i++) {
      curVoltage = curVoltage + analogRead(VOLTMETER_PIN);
      delay(10);
  }
  curVoltage = curVoltage / COUNT;
  float v  = (curVoltage * Vcc) / 1024.0;
  float v2 = v / (R2 / (R1 + R2)); 


float amps=abs(readCurrent(AMPERMETER_PIN));

float watts = amps*v2;

v2 = round(v2*100)/100.0;
amps = round(amps*100)/100.0;
watts = round(watts*100)/100.0;

          int batteryLevel=0;
    
          //report battery level

             batteryLevel = 100 - ((13-v2)/3*100);

              if ( batteryLevel > 100 ) batteryLevel=100;
                
           if (batteryLevel != oldValue_batteryLevel || boolRecheckSensorValues) 
           {
               gw.send(msgBatteryLevel.set(batteryLevel), true);     
               oldValue_batteryLevel = batteryLevel; 
           }

 if (v2 != oldValue_voltmeter || boolRecheckSensorValues) 
  {
     // Send in the new value
     gw.send(VoltmeterMsg.set(v2,2),true);   
     oldValue_voltmeter = v2;  
         Serial.print("Voltage: ");
         Serial.print(v2);  
         Serial.println(" V");          
  }

if (amps != oldValue_ampermeter || boolRecheckSensorValues) 
{
     gw.send(AmpermeterMsg.set(amps,2),true);   
     oldValue_ampermeter = amps;   
     Serial.print("Current:");
     Serial.print(amps,2);
     Serial.println(" A");     
}


if (watts != oldValue_wattmeter || boolRecheckSensorValues) 
{
     gw.send(WattmeterMsg.set(watts,2),true);   
     oldValue_wattmeter = watts;   
     Serial.print("Power:");
     Serial.print(watts,2);
     Serial.println(" W");     
}



 }

  
}



void checkTemp()
{

    unsigned long currentTempMillis = millis();
    if((currentTempMillis - previousTempMillis > TempsensorInterval) || boolRecheckSensorValues) {
        // Save the current millis
        previousTempMillis = currentTempMillis;
        // take action here:

          // Fetch temperatures from Dallas sensors
          sensors.requestTemperatures();
    //float temperature = static_cast<float>(static_cast<int>(sensors.getTempCByIndex(0))) / 10.;
       // float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempCByIndex(0):sensors.getTempFByIndex(0)) * 10.)) / 10.;
       
        float relayTemp =0, topTemp = 0, onboardTemp = 0;

        relayTemp = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(0) * 10.)) / 10.; // Temp sensor relay

        Serial.print("Temp1: ");
        Serial.println(relayTemp);
        if (relayTemp != lastTemp1 || boolRecheckSensorValues) {
            gw.send(Temp1Msg.set(relayTemp,1),true);
            lastTemp1 = relayTemp;
        } 


        // Если температура на реле больше 70 градусов - отключить ввод 220в
        if ( relayTemp > 70 && !boolTempDiffCheckDisabled)
        {
          //Disconnect 220v input
          digitalWrite(RELAY_PIN, RELAY_ON);
          gw.send(CriticalAlarmMsg.set("1"),true);
          
        }




        topTemp = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(1) * 10.)) / 10.; // Temp sensor on top

        Serial.print("Temp2: ");
        Serial.println(topTemp);
        if (topTemp != lastTemp2 || boolRecheckSensorValues) {
            gw.send(Temp2Msg.set(topTemp,1),true);
            lastTemp2 = topTemp;
        } 
        
        
        onboardTemp = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(2) * 10.)) / 10.; //Onboard temp sensor

        Serial.print("Temp3: ");
        Serial.println(onboardTemp);
        if (onboardTemp != lastTemp3 || boolRecheckSensorValues) {
            gw.send(Temp3Msg.set(onboardTemp,1),true);
            lastTemp3 = onboardTemp;
        }         


        
        // Если разница температур между двумя датчиками более 20 градусов - отключить ввод 220в
        if ( ((abs(relayTemp - topTemp) > 20) || (abs(relayTemp - onboardTemp) > 20) || (abs(topTemp - onboardTemp) > 20)) && !boolTempDiffCheckDisabled )
        {
                   //Disconnect 220v input
          digitalWrite(RELAY_PIN, RELAY_ON);
          gw.send(CriticalAlarmMsg.set("1"),true); 
        }
        
    }    

  
}



void checkPowerControllerStatus()
{
  
int outputValue= 0; //value output to the terminal D9 (analog out)
int invertedValue; //analog value that we will send to the analog panel meter
    unsigned long currentLightMillis = millis();
 
//  if( (currentLightMillis - previousLightMillis > LightsensorInterval) || boolRecheckPowerControllerStatus ) {
    // Save the current millis 
//    previousLightMillis = currentLightMillis;   
    // take action here:

  int  lightLevel=0;
  
  boolRecheckPowerControllerStatus = false;

     lightLevel = (1023-analogRead(LIGHT_SENSOR_RED_PIN))/10.23; //LIGHT_SENSOR_GREEN_PIN //LIGHT_SENSOR_RED_PIN


            
      if (lightLevel > 0)
      {    

//          if (lightLevel != lastLightLevel1) {
          if ((abs(lightLevel - lastLightLevel1) > 15 ) || boolRecheckSensorValues) {            
           Serial.print("Lightlevel red: ");
            Serial.println(lightLevel);
            gw.send(BatteryLightMsg.set(lightLevel),true);
            
              if ( lightLevel > 80 )
              {
                //Battery is ok, output power is present
                     digitalWrite(RED_LED_PIN,LOW);
              }
              else
              {
                //Battery fault
                     digitalWrite(RED_LED_PIN,HIGH);
                    gw.send(BatteryLightMsg.set(lightLevel),true);
              }
              
            lastLightLevel1 = lightLevel;
          }
    
    
          lightLevel=0;
      }    

    
     lightLevel = (1023-analogRead(LIGHT_SENSOR_GREEN_PIN))/10.23; //LIGHT_SENSOR_GREEN_PIN //LIGHT_SENSOR_RED_PIN


            
      if (lightLevel > 0)
      {    

//          if (lightLevel != lastLightLevel2) {
          if ((abs(lightLevel - lastLightLevel2) > 15 ) || boolRecheckSensorValues) {             
           Serial.print("Lightlevel green: ");
            Serial.println(lightLevel);
            gw.send(InputPowerLight.set(lightLevel),true);
            
              if ( lightLevel > 80 )
              {
                //220V power is present
                digitalWrite(GREEN_LED_PIN,LOW);
                VoltagesensorInterval = VoltagesensorIntervalOnPower;
                boolOnBattery = false;
              }
              else
              {
                //UPS on battery
                VoltagesensorIntervalOnPower = VoltagesensorInterval;
                VoltagesensorInterval=30000; // set volt/amper update to 30 sec
                digitalWrite(GREEN_LED_PIN,HIGH);
                gw.send(InputPowerLight.set(lightLevel),true);    
                boolOnBattery = true;            
              }
              
            lastLightLevel2 = lightLevel;
          }
    
    
          lightLevel=0;
      }    



//  }  
  
}



void checkRelayStatus()
{
  
int outputValue= 0; //value output to the terminal D9 (analog out)
int invertedValue; //analog value that we will send to the analog panel meter
    unsigned long currentRelayStatusMillis = millis();
 
  if((currentRelayStatusMillis - previousRelayStatusMillis > RelayStatussensorInterval) || boolRecheckRelayStatus || boolRecheckSensorValues) {
    // Save the current millis 
    previousRelayStatusMillis = currentRelayStatusMillis;   
    // take action here:

  int  lightLevel=0;
  
  if (boolRecheckRelayStatus)
  {
      boolRecheckRelayStatus = false;
      delay(500);
  }

     lightLevel = (1023-analogRead(LIGHT_RELAY_POWER_PIN))/10.23; //LIGHT_SENSOR_GREEN_PIN //LIGHT_SENSOR_RED_PIN


            
      if (lightLevel > 0)
      {    

          if (lightLevel != lastRelayStatusLightLevel1 || boolRecheckSensorValues) {
           Serial.print("Relay power Lightlevel: ");
            Serial.println(lightLevel);
            gw.send(RelayPowerLightMsg.set(lightLevel),true);
            
              
            lastRelayStatusLightLevel1 = lightLevel;
          }
    
    
          lightLevel=0;
      }    

    
     lightLevel = (1023-analogRead(LIGHT_RELAY_STATUS_PIN))/10.23; //LIGHT_SENSOR_GREEN_PIN //LIGHT_SENSOR_RED_PIN


            
      if (lightLevel > 0)
      {    

          if (lightLevel != lastRelayStatusLightLevel2 || boolRecheckSensorValues) {
           Serial.print("Relay status Lightlevel: ");
            Serial.println(lightLevel);
            gw.send(RelayStatusLightMsg.set(lightLevel),true);
            
              
            lastRelayStatusLightLevel2 = lightLevel;
          }
    
    
          lightLevel=0;
      }    



  }  
  
}



void reportTempDiffCheckState()
{

    unsigned long currentTDCMillis = millis();
    if(currentTDCMillis - previousTDCMillis > TDCsensorInterval || boolRecheckSensorValues) {
        // Save the current millis
        previousTDCMillis = currentTDCMillis;
        // take action here:

       gw.send(TempDiffCheckStateMsg.set(boolTempDiffCheckDisabled ? "1" : "0" ));  
        
    }    

  
}


void loop() {


    checkPowerControllerStatus();  
    checkAmpsVoltage();
    checkTemp();
    checkRelayStatus();
    reportTempDiffCheckState();

    if (boolRecheckSensorValues)
      {
       boolRecheckSensorValues = false;
      }
//Serial.println(rvcc ());
  //report int vcc
    //   gw.send(IntVCCMsg.set(rvcc(),2));  
    //   delay(4000);
    // Alway process incoming messages whenever possible
    gw.process();
    
    //reset watchdog timer
    wdt_reset();  

}







void incomingMessage(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.

    if ( message.sensor == REBOOT_CHILD_ID ) {
             wdt_enable(WDTO_30MS);
              while(1) {};

     }

    if ( message.sensor == DISABLE_TEMPDIFF_CHECK_CHILD_ID ) {
         
         if (message.getBool() == true)
         {
            boolTempDiffCheckDisabled = true;
         }
         else
         {
            boolTempDiffCheckDisabled = false;
         }

     }
     
     
     if (message.type==V_LIGHT && strlen(message.getString())>0 ) 
     {
         if ( message.sensor == CHILD_ID_RELAY ) 
         {
                digitalWrite(RELAY_PIN, message.getBool()?RELAY_ON:RELAY_OFF);
                
                boolRecheckRelayStatus = true;

         }
     }

    if ( message.sensor == RECHECK_SENSOR_VALUES ) {
         
         if (message.getBool() == true)
         {
            boolRecheckSensorValues = true;
         }

     }     

        return;      
} 



float readVcc() {
  byte i;
  float result = 0.0;
  float tmp = 0.0;

  for (i = 0; i < 20; i++) {
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
        ADMUX = _BV(MUX5) | _BV(MUX0);
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
        ADMUX = _BV(MUX3) | _BV(MUX2);
    #else
        // works on an Arduino 168 or 328
        ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif

    delay(3); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    tmp = (high<<8) | low;
    tmp = (typVbg * 1023.0) / tmp;
    result = result + tmp;
    delay(5);
  }

  result = result / 20;
  return result;
}





int determineVQ(int PIN) {
Serial.print("estimating avg. quiscent voltage:");
long VQ = 0;
//read 5000 samples to stabilise value
for (int i=0; i<5000; i++) {
    VQ += analogRead(PIN);
    delay(1);//depends on sampling (on filter capacitor), can be 1/80000 (80kHz) max.
}
VQ /= 5000;
Serial.print(map(VQ, 0, 1023, 0, 5000)); //5000
Serial.println(" mV");
return int(VQ);
}


float readCurrent(int PIN) {
int current = 0;
int sensitivity = 100.0;//change this to 100 for ACS712-20A or to 66 for ACS712-30A
//read 5 samples to stabilise value
for (int i=0; i<20; i++) {
    current += analogRead(PIN) - VQ;
    delay(1);
}
current = map(current/20, 0, 1023, 0, 5000); //5000
return float(current)/sensitivity;
}



float rvcc ()
{

ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
ADCSRA |= _BV(ADSC); // начало преобразований
while (bit_is_set(ADCSRA, ADSC)); // измерение
uint8_t low = ADCL; // сначала нужно прочесть ADCL - это запирает ADCH
uint8_t high = ADCH; // разлочить оба
float result = (high<<8) | low;
result = (1.1 * 1023.0 * 1000) / result; // Результат Vcc в милливольтах  

  return result;
  
}


/*
float readCurrent(int PIN) {
int current = 0;
int sensitivity = 100.0;//change this to 100 for ACS712-20A or to 66 for ACS712-30A
//read 5 samples to stabilise value
for (int i=0; i<20; i++) {
    current += analogRead(PIN) - VQ;
    delay(1);
}
current = map(current/20, 0, 1023, 0, 5000); //5000
return float(current)/sensitivity;
}



int determineVQ(int PIN) {
Serial.print("estimating avg. quiscent voltage:");
long VQ = 0;
//read 5000 samples to stabilise value
for (int i=0; i<5000; i++) {
    VQ += analogRead(PIN);
    delay(1);//depends on sampling (on filter capacitor), can be 1/80000 (80kHz) max.
}
VQ /= 5000;
Serial.print(map(VQ, 0, 1023, 0, 5000)); //5000
Serial.println(" mV");
return int(VQ);
}

*/
