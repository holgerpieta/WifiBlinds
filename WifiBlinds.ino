#include <pins_arduino.h>
#include <Bounce2.h>

// Wiring
const int stepPin = 16; // Step pin of driver is connected here.
const int dirPin = 17; // Direction pin of driver is connected here.
const int notEnablePin = 18; // Enable pin of driver is connected here. Logic is inverted: HIGH = Motor off
const int notSleepPin = 19; // Sleep pin of driver is connected here. Logic is inverted: LOW = Driver sleeping
const int buttonPin = 23; // Button is connected here to trigger actions.

// Configuration
const unsigned long minPulseHigh = 8; // Minimum HIGH time for step signal in micros
const unsigned long minPulseLow = 8; // Minimum LOW time for step signal in micros
const float maxV = min( 1.0e5, 1.0e6 / ( minPulseHigh + minPulseLow ) ); // Maximum speed in (micro)steps/second. Cannot be higher than defined by step signal width.
const float accel = 1e5; // Acceleration in (micro)steps/second^2
const float minV = sqrt( 2 * accel ); // Minimum speed, i.e. speed in first or last step
const unsigned long maxPulseWidth = ceil( 1e6 / minV ); // Longest possible step signal width at lowest speed

// Global variables...Urgs!
// Motor control
long curPos = 0; // Current position in steps
long targetPos = 0; // Target position in steps
float curV = 0; // Current speed in (micro)steps/second
bool parked = true; // Driver disabled, motor not powered
int lastFlankState = LOW;
unsigned long lastFlankTime = 0; // Time of last step pin flank in micros
unsigned long pulseWidth = 0; // Time to next step
bool slowToHalt = false;

// Sensing
Bounce debouncer = Bounce();
unsigned int curState = 0;

// Misc
unsigned long i = 0;

// Functions

// Disables the driver, unpowers the motor (no holding force), sets speed to zero and sets the parked bit.
void parkMotor() {
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
}

// Immediately stop moving and unpower the motor
void emergencyStop() {
  targetPos = curPos;
  parkMotor();
}

// Switch the state of the step signal, then store the current time and the time until something needs to be done.
void switchStepSignal() {
  // Choose correct direction
  digitalWrite(dirPin, curV > 0 ? HIGH : LOW );
  // Switch state
  if( lastFlankState == HIGH ) {
    // Switching to LOW, so configure dynamic wait time
    lastFlankState = LOW;
    pulseWidth = max( minPulseLow, ceil( 1e6 / abs( curV ) ) - minPulseHigh );
  } else {
    // Switching to HIGH, so configure constant wait time and update current position
    lastFlankState = HIGH;
    pulseWidth = minPulseHigh;
    curPos += curV > 0 ? 1 : -1;
  }
  digitalWrite(stepPin, lastFlankState);
  digitalWrite(LED_BUILTIN, lastFlankState);
  // Store time
  lastFlankTime = micros();
}

// Start motor and do first step
void startMotor() {
  digitalWrite(notEnablePin, LOW);
  digitalWrite(notSleepPin, HIGH);
  curV = targetPos > curPos ? minV : -minV;
  parked = false;
  slowToHalt = false;
  switchStepSignal();
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
      //if( i++%1000 == 0 ) Serial.printf("t: %lu | pw: %lu | tp: %lu | cp: %lu | dt: %f | v: %f\n", now, pulseWidth, targetPos, curPos, dt, curV);
      if( targetPos == curPos ){
        // We're at target, but we're still moving => Slow down
        curV += ( curV > 0 ? -accel : accel ) * dt;
      } else {
        float curMaxV = targetPos > curPos ? min( maxV, sqrt( 2 * accel * (targetPos - curPos) ) ) : maxV;
        float curMinV = targetPos < curPos ? max( -maxV, -sqrt( 2 * accel * (curPos - targetPos) ) ) : -maxV;
        if( curV > curMaxV || curV < curMinV ) {
          // We're going to fast for the remaining distance => Slow down
          curV += ( curV > 0 ? -accel : accel ) * dt;
        } else {
          // Nothing special, just not at target => Accelerate in correct direction
          curV += ( targetPos > curPos ? accel : -accel ) * dt;
          // No speeding or we get a ticket
          curV = max( -maxV, min( curV, maxV ) );
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
  if( debouncer.rose() ) {
    // Button clicked
    Serial.println( "New state" );
    switch( curState ) {
      case 0:
        curState = 1;
        targetPos = 2 * maxV;
        break;
      case 1:
        curState = 2;
        targetPos = 0;
        break;
      case 2:
        curState = 3;
        targetPos = 10 * maxV;
        break;
      case 3:
        curState = 4;
        targetPos = 0;
        break;
      case 4:
        curState = 5;
        targetPos = 10 * maxV;
        break;
      case 5:
        curState = 6;
        slowToHalt = true;
        break;
      case 6:
        curState = 7;
        targetPos = 10 * maxV;
        break;
      case 7:
        curState = 8;
        emergencyStop();
        break;
      default:
        // Finished => Do nothing
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(notEnablePin, OUTPUT);
  pinMode(notSleepPin, OUTPUT);
  parkMotor();
  pinMode(buttonPin, INPUT);
  debouncer.attach(buttonPin);
  debouncer.interval(5); // Interval in ms
}

void loop() {
  sensorLoop();
  motorLoop();
}

