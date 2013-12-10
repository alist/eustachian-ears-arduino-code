/*
 * ET Diag Sketch for Fio V3 + BLEBee
 * v0.0.1 2013-11-19
 * Copyright (c) 2013, Ears Team
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *^love, alex & jacob
*/

// TEST COMMIT LINE FROM JACOB

//#include <SoftwareSerial.h>
#include "types.h"
/*
1: setup
+ setup pins
+ setup coms
+ check for faults
+ sessionStatus = idle
2: Control loop

*/


String sysVersion = "0.1";

//sensor values
unsigned int pressureSensorValue = 0;
unsigned int sonotubometryAmplitudeValue = 0;

//interval between actions
unsigned long articulationIntervalMillis = 5;//happening 200x a second 
unsigned long measurementIntervalMillis = 10;
unsigned long activeStateDataAcqIntervalMillis = 50; //20hz
unsigned long idleStateDataAcqIntervalMillis = 200; //5hz

//need to speicify these
int solenoidD2APin = 6; //unfortunately, this is a PWM, not a DAC
int motorRelayPin = 5;
int pressureSensorA2DPin = A0;
int sonotubometryAmplitudeSensorA2DPin = A1;

//control parameters
unsigned int pressureSetpoint = 500;
unsigned int pressureError = 0; // Error used to set the control input //variable not param
unsigned int pressureFiltError = 0; // Error used to turn on the control//variable not param
double controlGain = 0.1;
unsigned int controlBias = 174;
unsigned int solenoidControlPWMValue = 0;

//control logic
//ESTOP envelope?
unsigned int outerEnvelope = 300;
unsigned int innerEnvelope = 100;
byte pumpDisabled = LOW; //LOW= motor on , HIGH= off
boolean controlON = false; //whether valve is enabled or not (false=not enabled)

//filtering pressure
unsigned int pressureFiltered = 0;
unsigned long filterTimeConstant = 1000; //in millis


SystemStatus systemStatus = unknownStatus;
SessionState sessionState = pendingState;


//the loop
unsigned long lastArticulationMillis = 0;
unsigned long lastMeasurementMillis = 0;
unsigned long lastDataAcqMillis = 0;
void loop(){
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastMeasurementMillis > measurementIntervalMillis){
    takeMeasurements();
    processData();   
    lastMeasurementMillis = currentMillis;
  }
  
  if (currentMillis - lastArticulationMillis > articulationIntervalMillis){
    articulateActuators();
    lastArticulationMillis = currentMillis;
  }
  
  //active vs idle state data acq
  if (sessionState == activeState){
    if (currentMillis - lastDataAcqMillis > activeStateDataAcqIntervalMillis){
      updateDataAcquisition(); 
      lastDataAcqMillis = currentMillis;
    }
  }else{
    if (currentMillis - lastDataAcqMillis > idleStateDataAcqIntervalMillis){
      updateDataAcquisition(); 
      lastDataAcqMillis = currentMillis;
    }
  }
  
  respondToRequests();
  transitionStates();
}
  

//the loop's helpers
void transitionStates(){
  if (sessionState == beginState){
    sessionState = activeState;
    debugLog("began session");
  }else if (sessionState == endState){
    sessionState = idleState;
    debugLog("ended session");
  }else if (sessionState == estopState){
    //perhaps not the best place to put this...
    systemStatus =  faultStatus; 
    debugLog("system went fault via session");
  } 
}


//see whether pending requests exist on bluetooth control channel
//(control commnads<ESTOP, controlActive, endSession, newSession>)
String message = "";
void respondToRequests(){
  //for now, just debug incoming commands
  if (Serial1.available()) {

    // #potential problem of hang here?
    while(Serial1.available()){
      message += char(Serial1.read());
      // debugLog("BT read: " +  message);
    }

    int commandIndex = message.indexOf(";");
    if (commandIndex == -1){
      return;
    }

    String command = message.substring(0,commandIndex);
    message = message.substring(commandIndex +1);
    debugLog("BT Control: " +  command);
    
    //we'll compare command to available commands
    if (command == "startSession"){
      sessionState = beginState; 
    }else if (command == "endSession"){
      sessionState = endState; 
    }else if (command == "estop"){
      sessionState = estopState;
    }else if (command == "version"){
      sendControlJSON(&String("version"), &String(sysVersion));
    }
    
  }
}

//save the data somewhere
void updateDataAcquisition(){
 String nextCSVEntry = String(sessionState) + "," + String(lastMeasurementMillis) + "," + String(pressureSensorValue) + "," + String(solenoidControlPWMValue) + "," + String(pumpDisabled) + "\n";
 // String csvLabels = "sessionState , dataMillis ,pressure, solenoidControlPWMValue,pumpDisabled \n"
 // ="{ \"sessionState\": "  + String(sessionState) + ", \"dataMillis\":" + String(lastMeasurementMillis) + ", \"pressure\":" + String(pressureSensorValue) + , \"sonotubometryAmplitude\":" + String(sonotubometryAmplitudeValue) + "}"; //in JSON
 //we save to an SD card if we have one, but we'll write to serial1(BT) & log for now!
 // debugLog (nextCSVEntry);
 //bluetooth!
 Serial1.print(nextCSVEntry); 
}

void sendControlJSON(String* property, String* value){
   String json = "{\""  + *property + "\":\""  + *value + "\"}\n";

   debugLog (json);
   //bluetooth!
   Serial1.print(json);
}


//actuate
void articulateActuators(){
  //Update Filtered Error - used for turning ON control
  if (pressureFiltered > pressureSetpoint) {
  pressureFiltError = pressureFiltered-pressureSetpoint;
  }else{
  pressureFiltError = pressureSetpoint-pressureFiltered;
  }
  
  //Update Pressure Sensor Error - used as the control variable
  if (pressureSensorValue > pressureSetpoint) {
  pressureError = pressureSensorValue-pressureSetpoint;
  pumpDisabled = HIGH;
  }else{
  pressureError = pressureSetpoint-pressureSensorValue;
  pumpDisabled = LOW;
  }
  
  
  if (sessionState == activeState && systemStatus == goodStatus){
    //motor runs pump coninuously
    //Determine if control is necessary
    if (pressureFiltError > outerEnvelope){controlON = true;}
    if (pressureError < innerEnvelope){controlON = false;}
    
    if (controlON){
      //Run control loop to pull pressure back to setpoint
    int controlAction = (double)controlBias+controlGain*pressureError;
    
    solenoidControlPWMValue = min(255 - controlAction, 255);

    analogWrite(solenoidD2APin, solenoidControlPWMValue); //What type to use for the PWM port?

    debugLog ("solenoid, pumpDisabled, pressureError, pressureFiltered: " + String(solenoidControlPWMValue) + "," + String(pumpDisabled) + "," + String(pressureError) + "," + String(pressureFiltered));

    digitalWrite(motorRelayPin, pumpDisabled);
    pressureFiltered = pressureSensorValue; // Reset filtered value to follow exact signal
    
    }else{
      debugLog ("nocntr-solenoid, pumpDisabled, pressureError, pressureFiltered: " + String(solenoidControlPWMValue) + "," + String(pumpDisabled) + "," + String(pressureError) + "," + String(pressureFiltered));
      //Run without control, taking best data with motor off and valve closed
    digitalWrite(motorRelayPin, HIGH); //this is disabling motor
    analogWrite(solenoidD2APin, 255); //this closes the valve
    }  

  }else{ //bad status
    //stopMotor
    digitalWrite(motorRelayPin, HIGH);
    
    //close solenoid
      debugLog ("nc-solenoid, pressureFiltered: " + String(255) + "," + String(pressureFiltered));
    analogWrite(solenoidD2APin, 255);  //this closes the valve

    if (sessionState == estopState || systemStatus == faultStatus){
        debugLog("waiting-- estopped or faulted"); 
    }
  
  }

};

//read sesnor data
void takeMeasurements() {
  //lastMeasurementMillis = millis(); Moving to processData function, need it for filtering (JI)
  
  pressureSensorValue = analogRead(pressureSensorA2DPin);//sin((1.0) * millis()/1000.0)*127 + 127; //or analogRead(pressureSensorA2DPin) //a pin #
  sonotubometryAmplitudeValue = sin((1.0) * millis()/5000.0)*127 + 127; //or analogRead(sonotubometryAmplitudeSensorA2DPin) //a pin #
  
};

//process data 
void processData() {
  int dt = millis() - lastMeasurementMillis;
  pressureFiltered = (pressureSensorValue*dt + pressureFiltered*filterTimeConstant)/(filterTimeConstant+dt);  
  
  //process data from takeMeasurements
  //detemrine whether an ESTOP is needed, or fault is occuring
  //make it easy for articulateActuators function, and *separate* the processing logic from the control logic as much as possible
  
  
}
  
//setup and startup
void setup()   {
  //pins 
  pinMode(motorRelayPin, OUTPUT);
  pinMode(solenoidD2APin, OUTPUT);
  pinMode(pressureSensorA2DPin, INPUT);
  pinMode(sonotubometryAmplitudeSensorA2DPin, INPUT);

  // pinMode(5, OUTPUT);
  // analogWrite(5, 127);  //this closes the valve
  
  // Set the baudrate of the Arduino
  Serial.begin(9600);
  delay(1000);
  debugLog("Beginning setup.");

  Serial1.begin(9600);
  debugLog("..done setup.");
  
  if (checkSystemStatus() == goodStatus){
     debugLog("Began control loop.");
     sessionState = idleState;
  }else{
      debugLog("Faulted and aborted before entering control loop.");
      while(1){
      };
  }
}

SystemStatus checkSystemStatus(){
  
  boolean neverAFault = true;
  if (neverAFault == false){
    systemStatus = faultStatus;
  }
  
  //making sure status wasn't previously modified
  //if fault status, this ensures a reset is needed
  if (systemStatus == unknownStatus){
    //we checked to make sure stop button isn't on
    //we checked to make sure whatever was whatever
    systemStatus = goodStatus;
  }
  
  return systemStatus;
}

void debugLog(String logPiece){\
  //milisecondsSinceProgramStart: "log piece"
  //nice logging, printing to com port attached to computer
  const String seperator = String(": ");
  String toSend = String(millis(), DEC) + seperator + logPiece;
  
  
  //debug in the way most preferred 
  Serial.println(toSend);
} 




