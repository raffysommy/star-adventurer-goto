#include <AccelStepper.h>
// This is the arduino code for controlling the NEMA17 motor, connect the easy driver https://www.schmalzhaus.com/EasyDriver/ to the pin 2 and 3 of the arduino.
// Optionally, you can also connect an st4 wires to the 4 and 5 pin, this togheter with the Star Adventurer guiding port will enable you to guide from the camera itself (not really needed here).

// Define the stepper motor connections and interface type
#define STEP_PIN 2
#define DIR_PIN 3
#define DEC_PLUS_PIN 4
#define DEC_MINUS_PIN 5

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Steps for one full rotation
const long stepsPerRevolution = 585600;

// Sidereal rate in steps per second
const float siderealStepsPerSecond = 6.796; // Calculated earlier

// DEC motion states
bool dec_plus = false;
bool dec_minus = false;
bool meade = false;
bool movingToTarget = false;

// LX200 variables
long targetDecPosition = 292800;
long currentDecPosition = 292800;
float slewSpeed = siderealStepsPerSecond * 128ul; // Slew speed multiplier
float st4Speed = siderealStepsPerSecond * 8ul; // ST4 movement speed
float guideSpeed = siderealStepsPerSecond; // ST4 movement speed

// Serial input buffer
char inputBuffer[20];
int inputIndex = 0;

void setup() {
	Serial.begin(115200);
	pinMode(DEC_PLUS_PIN, INPUT_PULLUP);
	pinMode(DEC_MINUS_PIN, INPUT_PULLUP);
	pinMode(LED_BUILTIN, OUTPUT);

	// Initialize the stepper
	stepper.setMaxSpeed(slewSpeed);
	stepper.setSpeed(0);
	stepper.setCurrentPosition(currentDecPosition);
	digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
	// Update the current DEC position
	currentDecPosition = stepper.currentPosition();

	// Check for manual ST4 inputs if not in meade mode
	if(!meade){
	handleSt4Inputs();
	}

	// Process LX200 commands from serial input
	processSerialInput();

	stepper.runSpeed();
	// Run the stepper to the target position if needed
	if (movingToTarget) {
	  if (stepper.distanceToGo()==0) {
	      movingToTarget = false;
	      //Serial.println("DONE");
	      stepper.setSpeed(0);
	  }
	}
	}
	void stop_everything(){
	  stepper.stop();
	  movingToTarget = false;
	  targetDecPosition = currentDecPosition;
	  //stepper.moveTo(currentDecPosition);
	  stepper.setSpeed(0);  
	}
	void handleSt4Inputs() {
	if (!digitalRead(DEC_PLUS_PIN)) {
	  if (!dec_plus) {
	      stepper.setSpeed(st4Speed);
	  }
	  dec_plus = true;
	  dec_minus = false;
	} else if (!digitalRead(DEC_MINUS_PIN)) {
	  if (!dec_minus) {
	      stepper.setSpeed(-st4Speed);
	  }
	  dec_plus = false;
	  dec_minus = true;
	} else {
	  if (dec_plus || dec_minus) {
	      stepper.stop();
	      stepper.setSpeed(0);
	  }
	  dec_plus = false;
	  dec_minus = false;
	}
}

void processSerialInput() {
	while (Serial.available() > 0) {
	  char c = Serial.read();
	  if (c == '#') {
	      handleCommand(String(inputBuffer));
	      memset(inputBuffer, 0, sizeof(inputBuffer));
	      inputIndex = 0;
	  } else {
	      inputBuffer[inputIndex++] = c;
	      if (inputIndex >= sizeof(inputBuffer)) {
		  inputIndex = 0; // Prevent buffer overflow
	      }
	  }
	}
}

void handleCommand(String command) {
	if (command.startsWith(":Q")) {
	  stop_everything();
	  Serial.println("#");
	} else if (command.startsWith(":Mn")) {
	  stop_everything();
	  stepper.setSpeed(guideSpeed);
	  Serial.println("#");
	} else if (command.startsWith(":Ms")) {
	  stop_everything();
	  stepper.setSpeed(-guideSpeed);
	  Serial.println("#");
	} else if (command.startsWith(":Gd")) {
	  Serial.println(currentDecPosition);
	  //Serial.println("#");
	} else if (command.startsWith(":Y")) {
	  meade = true;
	  Serial.println("#");
	  digitalWrite(LED_BUILTIN, HIGH);
	} else if (command.startsWith(":X")) {
	  meade = false;
	  stop_everything();
	  Serial.println("#");
	  digitalWrite(LED_BUILTIN, LOW);
	} else if (command.startsWith(":CP")) {
	  Serial.println("#");
	  stepper.setPinsInverted(true, false, false);
	} else if (command.startsWith(":MS")) {
	  if(targetDecPosition != currentDecPosition){
	    stepper.moveTo(targetDecPosition);
	    if (targetDecPosition>currentDecPosition){
	      stepper.setSpeed(slewSpeed);
	    }else{
	      stepper.setSpeed(-slewSpeed);
	    }
	    movingToTarget = true;
	  }
	  Serial.println("#");
	} else if (command.startsWith(":Sd")) {
	  targetDecPosition = command.substring(3).toInt(); //position in steps
	  Serial.println("#"); // Acknowledge
	} else if (command.startsWith(":CM")) { //sync
	  stepper.setCurrentPosition(targetDecPosition);
	} else if (command.startsWith(":D")){
	  if(movingToTarget){
	      Serial.println("1");
	  } else {
	      Serial.println("0");
	  }
	} else {
	  Serial.println("Got "+ command + " unknown"); // Unknown command
	}
}
