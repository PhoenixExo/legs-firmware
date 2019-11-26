#include <VL53L0X.h>

#include <Wire.h>

#include <ArduinoBLE.h>

BLEService legService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create service

// create switch characteristic and allow remote device to read and write
BLECharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite| BLENotify, "01234567890123456789");



 #define HALL1_PIN  A0
 #define HALL2_PIN  A1
 #define HALL3_PIN  A2
 #define HALL4_PIN  A3
 #define PRESURE_PIN   A7
 #define AIR_PIN   4
  #define FALL_SIGNAL_PIN   5
 #define EXHAUST_PIN   3
 #define LASER_PIN 12
 #define TRESHHOLD  400L
 #define MIN_NORMAL_PRESSURE  50
 #define MAX_NORMAL_PRESSURE  100
 #define MAX_PRESSURE  200
 #define STABILIZING_PRESSURE  200
 const byte interruptPin = 2;
 
 #define MODE_OFF   0
 #define MODE_WALK  100
 #define MODE_FALL  200
 #define MODE_SEAT  300
VL53L0X sensor;
int printAsked  = LOW;
int rotateAsked = LOW;
int lastPosition = 0;
float pressure = 0;
int presureSensor = 0;
int timesOnSamePosition; 
int mode = MODE_WALK;
int stabilizing = false;
int distance;

void setup(void)
{
  Serial.begin(57600);
 // while (!Serial);
   pinMode(AIR_PIN, OUTPUT);
    pinMode(EXHAUST_PIN, OUTPUT);
        pinMode(FALL_SIGNAL_PIN, OUTPUT);

   pinMode(interruptPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), askPrintState, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rotateState, CHANGE);
  pinMode(LASER_PIN,INPUT_PULLUP);
  digitalWrite(LASER_PIN,HIGH);
 //   Wire.begin();

  //sensor.init();
  //sensor.setTimeout(500);
  //sensor.startContinuous();
  //ads.setGain(GAIN_ONE); 
  //  ads.setGain(GAIN_TWOTHIRDS); 
    //ads.setGain(GAIN_TWO);
 // ads.begin();

     if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }


    BLE.setLocalName("BobsRightLeg");
  BLE.setDeviceName("BobsRightLeg");

  // set the local name peripheral advertises
 
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedService(legService);

  // add the characteristic to the service
  legService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(legService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);
  // set an initial value for the characteristic
 // switchCharacteristic.setValue(3);

  // start advertising
  BLE.advertise();
}

void rotateState(){
  rotateAsked = HIGH;
}

void askPrintState(){
  printAsked = HIGH;
}


void printState(){
  if ( !printAsked) return;
  printAsked=LOW;

  
      Serial.print("Leg position:");Serial.println(lastPosition);
       Serial.print("laser distanse:");Serial.println(distance);
       Serial.print("timesOnSamePosition:");Serial.println(timesOnSamePosition);
      Serial.print("MODE:"); 
       switch(mode){
     case MODE_WALK:
           Serial.println("WALK");
     break;
      case MODE_FALL:
        Serial.println("FALL");
     break;
      case MODE_SEAT:
        Serial.println("SEAT");
     break;
   default:
   Serial.println("OTHER");
   break;
 }      
 
 Serial.print("Pressure:");Serial.println(pressure);
 Serial.print("Presure Raw:");Serial.println(presureSensor);
 Serial.print("Stabilizing:");Serial.println(stabilizing);    

}

void addPressure(bool iterative){
  digitalWrite(AIR_PIN, HIGH);
  digitalWrite(EXHAUST_PIN, LOW);

  if( iterative ){
    delay(100);
    digitalWrite(AIR_PIN, LOW);
   
  }
}

void dropPressure(bool iterative){
  digitalWrite(AIR_PIN, LOW);
    digitalWrite(EXHAUST_PIN, HIGH);
    
  if( iterative ){
    delay(10);
    digitalWrite(EXHAUST_PIN, LOW);
    
  }
}

void readPressure(){
  int sensorVal=analogRead(PRESURE_PIN) +165;
  presureSensor = sensorVal;
//Serial.print("Sensor Value: ");
//Serial.println(sensorVal);

float voltage = (sensorVal*5.0)/1024.0;
  //  Serial.print("Volts: ");
    //Serial.print(voltage);
   
  float pressure_pascal = (3.0*((float)voltage-0.47))*1000000.0;
  float pressure_bar = pressure_pascal/10e5;
  pressure = pressure_bar *14.5038;
}

void readLegPosition(){
   int adc0, adc1, adc2, adc3;
  int sensorNo = lastPosition;
  adc0 = analogRead(HALL1_PIN);
  adc1 = analogRead(HALL2_PIN);
  adc2 = analogRead(HALL3_PIN);
  adc3 = analogRead(HALL4_PIN);
  if ( adc0< TRESHHOLD )
      sensorNo = 1;
   if ( adc1< TRESHHOLD )
      sensorNo = 2;
    if ( adc2< TRESHHOLD )
      sensorNo = 3;
    if ( adc3< TRESHHOLD )
      sensorNo = 4;

    if ( sensorNo == 4 && timesOnSamePosition<50 ){
         Serial.println("fall detected");
      mode= MODE_FALL;
      
      notifyMode(mode);
      //printState();
      //Serial.println("FALLLLLLL!!!!!!!!!!!!! ");
      stabilizing = false;
    }
        if ( lastPosition == sensorNo )
          timesOnSamePosition +=1;
        else {
           
           timesOnSamePosition = 0;
      //  Serial.print("sensor activated");   Serial.println(sensorNo);
    
 
             }
        
   lastPosition = sensorNo;
   // Serial.println(adc0);  
   //Serial.println(adc1);       
   //Serial.println(adc2);
   //Serial.println(adc3);    

//Serial.println("====");   
}
 
void loop(void)
{
  int startMode = mode;
if ( rotateAsked ){

switch(mode){
    case MODE_OFF:
      mode = MODE_WALK;
      break;
    case MODE_WALK:
        mode = MODE_SEAT;
      break;
    case MODE_SEAT:
        mode = MODE_FALL;
      break;
    case MODE_FALL:
       mode = MODE_OFF;
      break;

  }
  rotateAsked=  LOW;
}
  
 // Serial.println("Starting!");
 readLegPosition();
 readPressure();

//distance =sensor.readRangeContinuousMillimeters();
//Serial.println(lastPosition);
 digitalWrite(FALL_SIGNAL_PIN, mode == MODE_FALL );
 
 switch(mode){
     case MODE_WALK:
     if (pressure< MIN_NORMAL_PRESSURE)
        addPressure(true);
        if (pressure> MAX_NORMAL_PRESSURE)
        dropPressure(true);
     break;
      case MODE_FALL:
        addPressure(false);
       // Serial.println(lastPosition);
        if ( stabilizing && (lastPosition==1 ||lastPosition ==2) ){
         // Serial.println(timesOnSamePosition);
          if (timesOnSamePosition>5000){
            stabilizing = false;
            mode= MODE_WALK;
            notifyMode(mode);
          }
        }
        if ( pressure >=STABILIZING_PRESSURE){
          stabilizing = true;
        }
        
     break;
      case MODE_SEAT:
        dropPressure(false);
     break;
   default:
   break;
 }
 
    //  Serial.print("SensorNo: "); Serial.println(lastPosition);
     // Serial.print("timesOnSamePosition: "); Serial.println(timesOnSamePosition);
//    
//if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  //delay(1000);
  printState();
  // Serial.println(".!");
    BLE.poll();


    if ( mode != startMode ){
      Serial.println(mode);
      notifyMode(mode);
    }
}


void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}



void notifyMode(int mode){
  Serial.print("new mode= "); Serial.println(mode);
  switch(mode){
    case MODE_OFF:
      sendBLE("mode:off");
      break;
    case MODE_FALL:
      sendBLE("mode:fall");
      break;
    case MODE_WALK:
      sendBLE("mode:walk");
      break;
    case MODE_SEAT:
      sendBLE("mode:seat");
      break;
  }
    
}


void sendBLE( String data){

String toSend = data +="*";
   
 Serial.print("write to BLE:");Serial.println(toSend);
   int size = sizeof(toSend);
   char buffer[size];
   toSend.toCharArray( buffer, size);
   switchCharacteristic.writeValue(   buffer,size);
}


void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");
  
  int size =switchCharacteristic.valueLength();
  int i=0;
     Serial.println(size);

   char buffer[size];
  
//   while (i<size) { 
//       Serial.println(".");
//      String c = *(char*)switchCharacteristic.value(); //(Característica del interruptor VALOR)guardado en la variable estado
//      buffer[i]=c;
//      i++;
//      Serial.println("+");
//   }


switchCharacteristic.readValue( buffer, size );
     String readString = String(buffer);

     Serial.println(readString);

     int ind1 = readString.indexOf(':');  //finds location of first ,
     String command = readString.substring(0, ind1);
     int ind2 = readString.indexOf('*');  //finds location of first ,
     String argument = readString.substring(ind1+1, ind2);

     //Serial.print("command= "); Serial.println(command);
     //Serial.print("argument= '"); Serial.print(argument);Serial.println("'");
     if (command == "M"){
           if ( argument == "walk") {
            // Serial.println("set to walk");
                   mode = MODE_WALK;
           }
           else if ( argument ==  "off")
           {
               // Serial.println("set to off");
                   mode = MODE_OFF;
           }
           else if ( argument ==  "seat"){
              //  Serial.println("set to seat");
                 mode = MODE_SEAT;
           }
           else
           {
             //Serial.println("not matched");
             //     Serial.print("argument= '"); Serial.print(argument); Serial.print("'"); Serial.println(sizeof(argument));
             //     Serial.print("argument= '"); Serial.print("seat"); Serial.print("'"); Serial.println(sizeof("seat"));
            }
              
               //Serial.print("new mode= "); Serial.println(mode);
          notifyMode(mode);
     }
  
}
