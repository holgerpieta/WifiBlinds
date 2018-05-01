#include <pins_arduino.h>
#include <Bounce2.h>
#include <Basecamp.hpp>

// Basecamp
Basecamp iot( Basecamp::SetupModeWifiEncryption::secured, Basecamp::ConfigurationUI::accessPoint );

// Wiring
const int stepPin = 16; // Step pin of driver is connected here.
const int dirPin = 17; // Direction pin of driver is connected here.
const int notEnablePin = 18; // Enable pin of driver is connected here. Logic is inverted: HIGH = Motor off
const int notSleepPin = 19; // Sleep pin of driver is connected here. Logic is inverted: LOW = Driver sleeping
const int buttonPin = 23; // Button is connected here to trigger actions.

// Configuration
Configuration memConf;
const unsigned long minPulseHigh = 8; // Minimum HIGH time for step signal in micros
const unsigned long minPulseLow = 8; // Minimum LOW time for step signal in micros
float maxV = fmin( 1.0e5, 1.0e6 / ( minPulseHigh + minPulseLow ) ); // Maximum speed in (micro)steps/second. Cannot be higher than defined by step signal width.
float accel = 1e5; // Acceleration in (micro)steps/second^2
float minV = sqrt( 2 * accel ); // Minimum speed, i.e. speed in first or last step
unsigned long maxPulseWidth = ceil( 1e6 / minV ); // Longest possible step signal width at lowest speed
long maxP = 1e5;
long minP = 0;

// Global variables...Urgs!
// TODO: Put everything into a class/object
// Motor control
long curPos = 0; // Current position in steps
long targetPos = 0; // Target position in steps
float curV = 0; // Current speed in (micro)steps/second
bool parked = true; // Driver disabled, motor not powered
int lastFlankState = LOW;
unsigned long lastFlankTime = 0; // Time of last step pin flank in micros
unsigned long pulseWidth = 0; // Time to next step
bool slowToHalt = false;
bool configChanged = false;

// Sensing
Bounce debouncer = Bounce();
unsigned int curState = 0;

// Misc
unsigned long i = 0;


// Functions

// Setup basecamp
void setupBasecamp() {
  Serial.println( "Setup basecamp..." );
  // Short delay, otherwise WiFi turn-on messes up serial output. Maybe insufficient power supply.
  delay(500);
  iot.begin();

  if( iot.wifi.getOperationMode() == WifiControl::Mode::client ) {
    Serial.println( "WiFi in client mode, setting up custom web server..." );
    //Reset the webserver to remove basic configuration interface
    iot.web.reset();
    
    // Add a webinterface element for the h1 that contains the device name. It is a child of the #wrapper-element.
    iot.web.addInterfaceElement("heading", "h1", "","#wrapper");
    iot.web.setInterfaceElementAttribute("heading", "class", "fat-border");
    iot.web.addInterfaceElement("logo", "img", "", "#heading");
    iot.web.setInterfaceElementAttribute("logo", "src", "/logo.svg");
    String DeviceName = iot.configuration.get("DeviceName");
    if (DeviceName == "") {
      DeviceName = "Unconfigured Basecamp Device";
    }
    iot.web.addInterfaceElement("title", "title", DeviceName,"head");
    iot.web.addInterfaceElement("devicename", "span", DeviceName,"#heading");
    // Set the class attribute of the element to fat-border.
    iot.web.setInterfaceElementAttribute("heading", "class", "fat-border");

    // Add the configuration form, that will include all inputs for config data
    iot.web.addInterfaceElement("configform", "form", "","#wrapper");
    iot.web.setInterfaceElementAttribute("configform", "action", "saveConfig");
    
    iot.web.addInterfaceElement("active", "input", "Aktiviert:", "#configform", "active");
    iot.web.setInterfaceElementAttribute("active", "type", "number");
    iot.web.addInterfaceElement("openFrac", "input", "Öffnung in Prozent:", "#configform", "openFrac");
    iot.web.setInterfaceElementAttribute("openFrac", "type", "number");
    iot.web.setInterfaceElementAttribute("openFrac", "min", "0");
    iot.web.setInterfaceElementAttribute("openFrac", "max", "100");
    iot.web.addInterfaceElement("maxP", "input", "Obere Position in Schritten:", "#configform", "maxP");
    iot.web.setInterfaceElementAttribute("maxP", "type", "number");
    iot.web.addInterfaceElement("minP", "input", "Untere Position in Schritten:", "#configform", "minP");
    iot.web.setInterfaceElementAttribute("minP", "type", "number");
    iot.web.addInterfaceElement("maxV", "input", "Maximale Geschwindigkeit in Schritten/Sekunde:", "#configform", "maxV");
    iot.web.setInterfaceElementAttribute("maxV", "type", "number");
    iot.web.addInterfaceElement("accel", "input", "Beschleunigung in Schritten/Sekunde^2:", "#configform", "accel");
    iot.web.setInterfaceElementAttribute("accel", "type", "number");
    
    iot.web.addInterfaceElement("WifiConfigured", "input", "", "#configform", "WifiConfigured");
    iot.web.setInterfaceElementAttribute("WifiConfigured", "type", "hidden");
    iot.web.setInterfaceElementAttribute("WifiConfigured", "value", "true");

    // Add a save button that calls the JavaScript function collectConfiguration() on click
    iot.web.addInterfaceElement("saveform", "input", " ","#configform");
    iot.web.setInterfaceElementAttribute("saveform", "type", "button");
    iot.web.setInterfaceElementAttribute("saveform", "value", "Save");
    iot.web.setInterfaceElementAttribute("saveform", "onclick", "collectConfiguration()");
    
    // Start webserver with new interface and pass a lambda-function that indicates a changed config
    iot.web.begin(memConf, [&](){
      configChanged = true;
      Serial.println( "Config change detected." );
    });
    Serial.println( "Web server done." );
  }
  Serial.println( "Basecamp done." );
}

void setup() {
  // Open serial output
  Serial.begin(115200);
  Serial.println( "Serial output online." );

  pinMode(LED_BUILTIN, OUTPUT);

  // Setup output and motor
  Serial.println( "Setup motor..." );
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(notEnablePin, OUTPUT);
  pinMode(notSleepPin, OUTPUT);
  parkMotor();
  Serial.println( "Motor done." );

  // Setup button
  Serial.println( "Setup button..." );
  pinMode(buttonPin, INPUT);
  debouncer.attach(buttonPin);
  debouncer.interval(5); // Debounce interval in ms
  Serial.println( "Button done." );

  Serial.println( "Setup config..." );
  char printBuf[64];
  memConf.set("active", "1");
  memConf.set("openFrac", "0");
  sprintf(printBuf, "%li", maxP);
  memConf.set("maxP", printBuf);
  sprintf(printBuf, "%li", minP);
  memConf.set("minP", printBuf);
  sprintf(printBuf, "%f", maxV);
  memConf.set("maxV", printBuf);
  sprintf(printBuf, "%f", accel);
  memConf.set("accel", printBuf);
  Serial.println( "Config done." );

  setupBasecamp();
}


// Disables the driver, unpowers the motor (no holding force), sets speed to zero and sets the parked bit.
void parkMotor() {
  //Serial.println( "Parking motor..." );
  digitalWrite(notEnablePin, HIGH);
  digitalWrite(notSleepPin, LOW);
  digitalWrite(dirPin, HIGH);
  digitalWrite(stepPin, LOW);
  curV = 0;
  parked = true;
  lastFlankState = LOW;
  lastFlankTime = 0;
  pulseWidth = 0;
  slowToHalt = false;
  digitalWrite(LED_BUILTIN, LOW);
  //Serial.println( "Park done." );
}

// Immediately stop moving and unpower the motor
void emergencyStop() {
  //Serial.println( "Emergency stopping..." );
  targetPos = curPos;
  parkMotor();
  //Serial.println( "Emergency stop done." );
}

// Switch the state of the step signal, then store the current time and the time until something needs to be done.
void switchStepSignal() {
  // Choose correct direction
  digitalWrite(dirPin, curV > 0 ? HIGH : LOW );
  // Switch state
  if( lastFlankState == HIGH ) {
    // Switching to LOW, so configure dynamic wait time
    lastFlankState = LOW;
    pulseWidth = fmax( minPulseLow, ceil( 1e6 / abs( curV ) ) - minPulseHigh );
  } else {
    // Switching to HIGH, so configure constant wait time and update current position
    lastFlankState = HIGH;
    pulseWidth = minPulseHigh;
    curPos += curV > 0 ? 1 : -1;
  }
  digitalWrite(stepPin, lastFlankState);
  // Store time
  lastFlankTime = micros();
}

// Start motor and do first step
void startMotor() {
  //Serial.println( "Starting motor..." );
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(notEnablePin, LOW);
  digitalWrite(notSleepPin, HIGH);
  curV = targetPos > curPos ? minV : -minV;
  parked = false;
  slowToHalt = false;
  switchStepSignal();
  //Serial.println( "Starting done." );
}

// Update speed, then send next signal
void doStep() {
  unsigned long now = micros();
  if( now - lastFlankTime >= pulseWidth ) {
    // Time's up, we have to do something.
    if( lastFlankState == LOW ) {
      // Last flank was LOW, so update speed
      if( slowToHalt ) {
        // No target, just slowing down => Reset target to current position
        targetPos = curPos;
      }
      // Elapsed time since last LOW to HIGH flank
      float dt = 1.0e-6 * ( now - lastFlankTime + minPulseHigh );
      // if( i++%1000 == 0 ) Serial.printf("t: %lu | pw: %lu | tp: %lu | cp: %lu | dt: %f | v: %f\n", now, pulseWidth, targetPos, curPos, dt, curV);
      if( targetPos == curPos ){
        // We're at target, but we're still moving => Slow down
        curV += ( curV > 0 ? -accel : accel ) * dt;
      } else {
        float curMaxV = targetPos > curPos ? fmin( maxV, sqrt( 2 * accel * (targetPos - curPos) ) ) : maxV;
        float curMinV = targetPos < curPos ? fmax( -maxV, -sqrt( 2 * accel * (curPos - targetPos) ) ) : -maxV;
        if( curV > curMaxV || curV < curMinV ) {
          // We're going to fast for the remaining distance => Slow down
          curV += ( curV > 0 ? -accel : accel ) * dt;
        } else {
          // Nothing special, just not at target => Accelerate in correct direction
          curV += ( targetPos > curPos ? accel : -accel ) * dt;
          // No speeding or we get a ticket
          curV = fmax( -maxV, fmin( curV, maxV ) );
        }
      }
      // Maybe we got some rounding errors somewhere and did not hit zero => Correct that
      curV = abs( curV ) < minV ? 0 : curV;
      if( curV != 0 ) {
        // Movement requested => Do something
        switchStepSignal();
      } else if( targetPos != curPos ){
        // We're standing, but want to move => No need to wait, just start in the correct direction
        startMotor();
      } else {
        // At target and standing => Park motor
        parkMotor();
      }
    } else {
      // Last flank was HIGH => Just switch to LOW.
      switchStepSignal();
    }
  }
}

// Checks if we need to do something and does it
void motorLoop() {
  if(  curPos == targetPos && curV == 0 ) {
    //Nothing to do
    if( !parked ) {
      // Not parked, but nothing to do => Park.
      parkMotor();
    }
  } else {
    // Current position unequal target position or remaining speed => We have to do something.
    if( parked ) {
      // Driver and motor parked => Start it up and do first step.
      startMotor();
    } else {
      doStep();
    }
  }
}

//Reads sensors and updates targets, if needed
void sensorLoop() {
  debouncer.update();
  if( debouncer.read() ) {
    emergencyStop();
  }
}

void configLoop() {
  // Only do something when config has changed, parsing strings is expensive
  if( configChanged ) {
    char buf[64];
    memConf.get("active").toCharArray(buf, sizeof(buf));
    int active = atoi(buf);
    if( active != 1 && !parked ) {
      emergencyStop();
    }
    memConf.get("maxP").toCharArray(buf, sizeof(buf));
    maxP = atol(buf);
    memConf.get("minP").toCharArray(buf, sizeof(buf));
    minP = atol(buf);
    memConf.get("maxV").toCharArray(buf, sizeof(buf));
    maxV = atof(buf);
    memConf.get("accel").toCharArray(buf, sizeof(buf));
    accel = atof(buf);
    memConf.get("openFrac").toCharArray(buf, sizeof(buf));
    float openFrac = atof(buf);
    // Calculate target position based on desired opening percent
    targetPos = fmax( minP, fmin( ceil( openFrac / 100 * (maxP - minP) ) + minP, maxP ) );

    // Changed config has been parsed. Do nothing until further changes comes in.
    configChanged = false;

    // Debug output of received config
    Serial.print("active: ");
    Serial.print(memConf.get("active"));
    Serial.print(" | openFrac: ");
    Serial.print(memConf.get("openFrac"));
    Serial.print(" | maxP: ");
    Serial.print(memConf.get("maxP"));
    Serial.print(" | minP: ");
    Serial.print(memConf.get("minP"));
    Serial.print(" | maxV: ");
    Serial.print(memConf.get("maxV"));
    Serial.print(" | accel: ");
    Serial.print(memConf.get("accel"));
    Serial.print("\n");
  }
}

void loop() {
  configLoop();
  sensorLoop();
  motorLoop();
}

