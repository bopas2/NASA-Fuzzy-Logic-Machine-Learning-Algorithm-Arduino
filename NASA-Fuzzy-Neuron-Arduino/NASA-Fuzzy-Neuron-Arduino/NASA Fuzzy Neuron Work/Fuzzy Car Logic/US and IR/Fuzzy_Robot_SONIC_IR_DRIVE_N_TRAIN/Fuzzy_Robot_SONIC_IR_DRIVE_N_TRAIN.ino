#include <math.h>
#include <Servo.h>                          

Servo servoRight;                           // Right servo is motor 2
Servo servoLeft;                            // Left servo is motor 1

const int inputRange = 255;    // range of values inputed from user/sensors, minimum of zero
const int outputRange = 1;     // range of values that represent how much of a member a specific input is to a member function, minimum of zero
const int numOfFuzzifiers = 4; // How many member functions exist. Inputs can only map to two member functions. 

// Arduino Pin definition
const int triggerL = 11; // left ultrasonic sensor trigger
const int triggerR = 2;  // right ultrasonic sensor trigger
const int echoL = 10;    // left ultrasonic sensor reciever pin
const int echoR = 3;     // right ultrasonic sensor reciever pin
const int ledOut = 5;    // trigger IR sensor
const int IR_IN = 7;     // read IR sensor

// Used to determine what percentage a input relates to a member function. 
// Triangle illustration helps visualize this. 
float slope = outputRange / (float)((inputRange / numOfFuzzifiers));   

const int input = 2;  // Number of values we may input - VARIABLE
const int output = 2; // we have two motors we must control, requiring two sets of weights, HARDCODED

// Weight matrix for each permutation of member functions.
// Multiplied by two because of the infrared sensor, which doesn't require a full set of member function weights because of it's 1 or 0 binary output. 
// Organized by member functions: 1A to 2A with IR = 0, 1A to 2B with IR = 0, 1A to 2C with IR = 0 ... 1B to 2A with IR = 0, 1B to 2B with IR = 0 ... 1Y to 2A with IR = 0, 1Y to 2B with IR = 0 etc
// Second of half of weight matrix is for when the IR sensor outputs a 1: 1A to 2A with IR = 1, 1A to 2B with IR = 1, 1A to 2C with IR = 1 ... 1B to 2A with IR = 1 etc. etc.
const int numberOfWeightArrays = 2 * (int)pow(numOfFuzzifiers, input);
double M1weights[numberOfWeightArrays][input + 2]; // for motor 1
double M2weights[numberOfWeightArrays][input + 2]; // for motor 2

const double alpha = .00001;    // Controls how fast the algorithm learns. Will overshoot minimum error if made larger -- don't know why.

double midPts[numOfFuzzifiers]; // Location of center of member functions. Again, see triangle diagram for better visualization. 

void setup() {
  randomSeed(analogRead(4));  // Randomize the random number generator
  Serial.begin(9600);   
  defineMidpoints();          // Determine where each member function exists on the x-y universe
  createWeights(*M1weights);  // Give us random weights to break symmetry
  createWeights(*M2weights);
  servoRight.attach(13);      // Define the arduino pins 
  servoLeft.attach(12);
  pinMode(IR_IN, INPUT);    
  pinMode(ledOut, OUTPUT);
  pinMode(triggerL, OUTPUT); pinMode(triggerR, OUTPUT); 
  pinMode(echoL, INPUT); pinMode(echoR, INPUT);
}

char currentState = 'q';        // aka the 'off' state for user controls

int inputs[input + 1];          // Array of all input values
double expectedOutputs[output]; // "correct" answer, used for training, based on user input.

void loop() {
  detect();                               // Read sensors
  if (inputs[0] != 0 || inputs[1] != 0) { // This just makes sure the sensors aren't being wonky
    fuzzy(drive());                       // Train, or self-drive
    // Displays operator's actions and sensor readings, while driving, so we can train off the arduino if needed
    if (currentState != 'q' && currentState != 'T' && currentState != 'I' && currentState != 'p' && currentState != 'q') {
      Serial.print(inputs[0]);
      Serial.print(" ");
      Serial.print(inputs[1]);  
      Serial.print(" ");
      Serial.print(inputs[2]);
      Serial.print(" ");
      Serial.print(expectedOutputs[0]);
      Serial.print(" ");
      Serial.println(expectedOutputs[1]);
    }
  }
  delay(15);
}

// Gathers information from sensors, distance is how far we want the ultrasonic sensors to detect (centimeters).
// Map recorded values to the input range that we defined.
int viewDistance = 30; // in centimeters, can be changed to affect learning
void detect() {
  inputs[0] = map(USDetect(triggerL, echoL, viewDistance), 0, viewDistance, 0, inputRange);
  delay(25); // Makes sure we don't read unwanted ultrasonic waves
  inputs[1] = map(USDetect(triggerR, echoR, viewDistance), 0, viewDistance, 0, inputRange);
  // 38,000 is a predetermined efficient blinking efficency for the IR sensor - don't change unless your sure
  inputs[2] = map(1 - IRDetect(ledOut, IR_IN, 38000), 0, 1, 0, inputRange);                 
}

// Reads distance from ultrasonic sensors
int USDetect(int trigger, int echo, int maxi)
{
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  double distance = pulseIn(echo, HIGH) / 2 * .0343; // Translate to centimeters
  if (distance > maxi)                               // Check max range
    return maxi;
  return distance;
}

//reads from infrared sensor - only returns 1 or 0.
int IRDetect(int irLedPin, int irReceiverPin, long freq) {
  tone(irLedPin, freq, 8);
  delay(1);
  int r = digitalRead(irReceiverPin);
  delay(1);
  return r;
}

//Allows the user to drive the arduino car. Return value definitions:
// -1 -> self drive
// 3 -> print weights
// 2 -> pause everything
// 0 -> driving and train
int drive() {
  if (Serial.available() > 0) {
    char tester = Serial.read(); 
    if (tester != '\n')       // Don't need to read return characters.
      currentState = tester;
    if (tester == 'q')        // Used to help define changes in program states.
      Serial.println("+++++++++++");
  }
  if (currentState == 'w') { // forwards
    servoRight.writeMicroseconds(1200);
    servoLeft.writeMicroseconds(1700);
    expectedOutputs[0] = 1700;
    expectedOutputs[1] = 1200;
  }
  else if (currentState == 'a') { // left
    servoRight.writeMicroseconds(1200);
    servoLeft.writeMicroseconds(1200);
    expectedOutputs[0] = 1200;
    expectedOutputs[1] = 1200;
  }
  else if (currentState == 's') { // backwards
    servoRight.writeMicroseconds(1700);
    servoLeft.writeMicroseconds(1200);
    expectedOutputs[0] = 1200;
    expectedOutputs[1] = 1700;
  }
  else if (currentState == 'd') { // right
    servoRight.writeMicroseconds(1700);
    servoLeft.writeMicroseconds(1700);
    expectedOutputs[0] = 1700;
    expectedOutputs[1] = 1700;
  }
  else if (currentState == 'T') { // self drive
    return -1;
  }
  else if (currentState == 'R') { // reset Weights
    createWeights(*M1weights); 
    createWeights(*M2weights);
    Serial.println("RESET");
    currentState = 'q';
  }
  else if (currentState == 'I') { // read off sensor readings 
    Serial.println("---");
    Serial.println("Left Sensor:");
    Serial.println(inputs[0]);
    Serial.println("Right Sensor:");
    Serial.println(inputs[1]);
    Serial.println("Infrared Sensor:");
    Serial.println(inputs[2]);
    Serial.println("---");
    currentState = 'q';
  }
  else if (currentState == 'p') { // print current weights
    return 3;
  }
  else if (currentState == 'q') { // pause
    expectedOutputs[0] = 1500;
    expectedOutputs[1] = 1500;
    servoRight.writeMicroseconds(1500);
    servoLeft.writeMicroseconds(1500);
    return 2;
  }
  else { // stop
    servoRight.writeMicroseconds(1500);
    servoLeft.writeMicroseconds(1500);
    expectedOutputs[0] = 1500;
    expectedOutputs[1] = 1500;
  }
  Serial.flush();
  return 0;
}

const int numberOfCouplets = pow(2, input); // How many combinations we have based on one set of X input variables

// Training and decision making 
void fuzzy(int orders) {
  if (orders != 2) { 
    
    // [Lower Bound1, Upper Bound1, Ydown, Yup] aka [Membership A (left midpoint), Membership B (right midpoint), DegreeOfMembership of A, DegreeOfMembership of B]
    double rangeAndOutputs[input][4];
    for (int i = 0; i < input; i++) {
      returnOutputAndInterval(inputs[i], (*rangeAndOutputs) + 4 * i);
    }

    // This function creates a matrix of size 2 x input where each value contains the index value for its membership function in the rangeAndOutputs array
    double arrayOfCouplets[2][input];
    for (int i = 0; i < input; i++) {            // e.g: [Input1 LowerBound, Input2 LowerBound ... etc ]
      for (int j = 0; j < 2; j++) {              //      [Input1 UpperBound, Input2 UpperBound ... etc ]
        arrayOfCouplets[j][i] = (i * 4) + j;
      }
    }

    // Creates an array of EVERY couplet permutation, using current inputs
    // [Minimum membership value, Specific Member Functions[input], Degrees of Membership[input], Actual Output Motor 1, Actual Output Motor 2] 
    double couplets[numberOfCouplets][3 + (2 * input)]; 
    for (int x = 0; x < numberOfCouplets; x++) {                                                                                    // Go until we reach the max permutations for the given inputs
      for (int L = 0; L < input; L++) {                                                                                             // x is current permutation number, in this case, translate it from base 10 to binary
        couplets[x][1 + L] = *(*(rangeAndOutputs) + (int)arrayOfCouplets[(x >> L) & 1][L]);                                         // (x>>L) does a bitwise shift to the right L indecies.
        couplets[x][1 + input + L] = *(*(rangeAndOutputs) + (int)arrayOfCouplets[(x >> L) & 1][L] + 2);                             // &1 determines if the binary value at that point is a 1 or a 0
      }                                                                                                                             // this essentially goes through the arrayOfCouplets array and parses out every 
      couplets[x][0] = smallestVal(*(couplets) + (1 + input) + x * (3 + (2 * input)), input);                                       // couplet permutation. 
      couplets[x][1 + (2 * input)] = couplets[x][0] * determineOutput(*couplets + x * (3 + (2 * input)) + 1, inputs, *M1weights);   // to really understand it, write it out
      couplets[x][2 + (2 * input)] = couplets[x][0] * determineOutput(*couplets + x * (3 + (2 * input)) + 1, inputs, *M2weights);   
    }

    // Sum the minimum degree of membership of each couplet
    double sumOfMins = 0.0;
    for (int i = 0; i < numberOfCouplets; i++) {
      sumOfMins += (couplets[i][0]);
    }

    // Take the sum of each couplet's output
    double outputGuessM2 = 0.0;
    double outputGuessM1 = 0.0;
    for (int i = 0; i < numberOfCouplets; i++) {
      outputGuessM1 += couplets[i][1 + (2 * input)];
      outputGuessM2 += couplets[i][2 + (2 * input)];
    }
    
    // Divide it by the sum of the mins to produce the algorithim's output
    outputGuessM1 /= sumOfMins;
    outputGuessM2 /= sumOfMins;

    // Calculuate Errors
    double errorM1 = (expectedOutputs[0] - outputGuessM1);
    double errorM2 = (expectedOutputs[1] - outputGuessM2);

    // Print, weights if told to do so
    if (orders == 3) {
      printWeights(*M1weights);
      printWeights(*M2weights);
      currentState = 'q';
    }

    // Train, if told to do so
    else if (orders == 0) {
      // Adjust the weights of each couplet permutation
      for (int i = 0; i < numberOfCouplets; i++) {
        train(sumOfMins, errorM1, *(couplets) + i * (3 + (2 * input)), inputs, *M1weights);
        train(sumOfMins, errorM2, *(couplets) + i * (3 + (2 * input)), inputs, *M2weights);
      }
    }

    // Self driving
    else if (orders == -1) {
      Serial.println(outputGuessM2);
      Serial.println(outputGuessM1);
      servoRight.writeMicroseconds(outputGuessM2);
      servoLeft.writeMicroseconds(outputGuessM1);
    }
  }
}

// Prints out weight matrix, output can be copied and used when we just want to drive and not train etc.
void printWeights(double * waits) {
  Serial.println("{");
  for (int i = 0; i < numberOfWeightArrays; i++) {
    Serial.print("  {");
    for (int j = 0; j < input + 2; j++) {
      Serial.print(*(waits + i * (input + 2) + j), 5);
      if (j != input + 1)
        Serial.print(", ");
    }
    Serial.println("},");
  }
  Serial.println("}");
  Serial.println();
}

// Use weights to do some calculations for each couplet's output
// note, it doesn't complete a full calculation based on inputs, it does part of a calculation, couplet per couplet
// pointer starts at index 1
double determineOutput(double * couplet, int * inputs, double * whichWeights) {
  // Determines which weight matrix based on couplet permutation
  double index = 0;
  for (int i = 0; i < input; i++) {
    index += *(couplet + i) * pow(numOfFuzzifiers, input - i - 1);
  }
  double ans = 0;
  int inx = (int)index;
  if (inputs[2] == 255) { // If infrared is activated, adjust weight index
    index += numberOfWeightArrays / 2;
    inx = (int)index;
    // Begin calculations using the weight matrix and input variables
    ans += *(inputs + 2) * *(whichWeights + input +  inx * (input + 2)); 
  }
  // Non-IR calculations
  for (int i = 0; i < input; i++) {
    ans += *(inputs + i) * *(whichWeights + i + inx * (input + 2));
  }
  // Add the constant 
  ans += *(whichWeights + input + 1 + inx * (input + 2));
  return ans;
}

// Apply Gradient Decent
// Similar to determineOutput, except we adjust the weights now.
void train(double sumOfMins, double error, double * couplet, int * inputs, double * whichWeights) {
  double constant = *(couplet) * alpha * error / sumOfMins;
  double index = 0;
  for (int i = 0; i < input; i++) {
    index += *(couplet + i + 1) * pow(numOfFuzzifiers, input - i - 1);
  }
  int inx = (int)index;
  if (inputs[2] == 255) { 
    index += numberOfWeightArrays / 2;
    inx = (int)index;
    *(whichWeights + inx * (input + 2) + input) = constant * *(inputs + 2);
  }
  for (int i = 0; i < input; i++) {
    *(whichWeights + inx * (input + 2) + i) += constant * *(inputs + i);
  }
  // I found that giving the constant an input value allows the algorithim train a bit faster / better 
  // Probably because the learning rate is so small. 
  *(whichWeights + inx * (input + 2) + input + 1) += inputRange / 2 * constant;
}

// Based on an input, determine which fuzzy member groups it belongs to, and its degree of membership to each member function
void returnOutputAndInterval(int input, double * ans) {
  int radius = ceil((double)inputRange / (double)numOfFuzzifiers);
  for (int i = 0; i < numOfFuzzifiers; i++) {
    if (input >= midPts[i] && input <= midPts[i + 1]) {  //if in between two points
      *(ans) = i;
      *(ans + 1) = i + 1;   //record the points its inbetween
      *(ans + 2) = outputRange - slope * (input % radius);
      *(ans + 3) = slope * (input % radius);
    }
  }
}

// Based on how many fuzzifiers we have, we need to determine the range of each one
void defineMidpoints() {
  int width = ceil((double)inputRange / (double)(numOfFuzzifiers - 1));
  for (int i = 0; i < numOfFuzzifiers; i++) {
    midPts[i] = i * width;
  }
}

// Fills the weight array with random value in the +- 10 range
void createWeights(double * ans) {
  for (int i = 0; i < numberOfWeightArrays; i++) {
    for (int j = 0; j < (input + 2); j++) {
      double r = ((double)random(-100, 100) / 10);
      *(ans + (i * (input + 2)) + j) = r;
    }
  }
}

// Determines the smallest value out of an array of value
double smallestVal(double * a, int num) {
  double smallest = *a;
  for (int i = 0; i < num; i++) {
    if (*(a + i) < smallest)
      smallest = *(a + i);
  }
  return smallest;
}
