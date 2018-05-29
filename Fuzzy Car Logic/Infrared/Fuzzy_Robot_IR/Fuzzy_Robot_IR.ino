#include <math.h>
#include <Servo.h>                           // Include servo library

Servo servoRight;                            // Right is motor 2
Servo servoLeft;                            // Left is motor 1

const int inputRange = 255; // X axis of values inputed from user/sensors
const int outputRange = 1;  // Y axis of values that represent how much of a member a specific input is
const int numOfFuzzifiers = 4; // Number of members that an input can belong to

float slope = outputRange / (float)((inputRange / numOfFuzzifiers));   // Slope of triangles based on our graph layout

const int input = 2; // Number of values we may input - VARIABLE
const int output = 2; // we have two motors we must control, requiring two sets of weights

const int numberOfWeightArrays = (int)pow(numOfFuzzifiers, input);
double M1weights[numberOfWeightArrays][input + 1]; // for motor 1
double M2weights[numberOfWeightArrays][input + 1]; // for motor 2

const double alpha = .00005; // Controls how fast the algorthim learns...largest value we can have without overshooting the cost min

double midPts[numOfFuzzifiers]; // X value where the tip of each triangle ( or center of member function is )

void setup() {
  randomSeed(analogRead(4)); //randomize the random number
  Serial.begin(9600);
  defineMidpoints(); // Determine where each member function exists on the x-y universe
  createWeights(*M1weights); // Give us random weights to break symetry
  createWeights(*M2weights);
  servoRight.attach(13);  // define the arduino pins the servos are attached to
  servoLeft.attach(12);
  pinMode(10, INPUT);  pinMode(9, OUTPUT);   // Left IR LED & Receiver
  pinMode(2, INPUT);  pinMode(3, OUTPUT);   // Right IR LED & Receiver
}

const int numberOfCouplets = pow(2, input); // How many combinations we have based on one set of X input variables
double counter = 0;

void loop() {
  detect();
  fuzzy(counter, drive());
  counter++;
}

int inputs[input];
//Gathers information from sensors
void detect() {
  inputs[0] = map(irDetect(9, 10, 38000), 0, 1, 0, 250);
  inputs[1] = map(irDetect(3, 2, 38000), 0, 1, 0, 250);
}

//reads distance from IR sensors
int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
  tone(irLedPin, frequency, 8);              // IRLED 38 kHz for at least 1 ms
  delay(1);                                  // Wait 1 ms
  double ir = digitalRead(irReceiverPin);       // IR receiver -> ir variable
  delay(1);                                  // Down time before recheck
  return ir;                                 // Return 1 no detect, 0 detect
}

double expectedOutputs[2];
//Allows the user to drive the arduino car
char currentState = 'x';
boolean drive() {
  if (Serial.available() > 0) {
    currentState = Serial.read();
    Serial.println(currentState);
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
  else if (currentState == 'T') { // SELF DRIVING?!
    return false;
  }
  else if (currentState == 'R') { // Reset Weights
    createWeights(*M1weights); // Give us random weights to break symetry
    createWeights(*M2weights);
  }
  else { // STOP
    servoRight.writeMicroseconds(1500);
    servoLeft.writeMicroseconds(1500);
    expectedOutputs[0] = 1500;
    expectedOutputs[1] = 1500;
  }
  return true;
}

//training and decision part of the arduino
void fuzzy(double counter, boolean driveOrTrain) {
  // [Lower Bound1, Upper Bound1, Ydown, Yup] aka [Membership A (left midpoint), Membership B (right midpoint), DegreeOfMembership of A, DegreeOfMembership of B]
  double rangeAndOutputs[input][4];
  for (int i = 0; i < input; i++) {
    returnOutputAndInterval(inputs[i], (*rangeAndOutputs) + 4 * i);
  }

  // This function creates a matrix of size 2xinput WHERE each value contains the index value for its membership function in the rangeAndOutputs array
  double arrayOfCouplets[2][input];
  for (int i = 0; i < input; i++) {         // e.g: [In1 LowerBound, In2 LowerBound ... etc ]
    for (int j = 0; j < 2; j++) {              //   [In1 UpperBound, In2 UpperBound ... etc ]
      arrayOfCouplets[j][i] = (i * 4) + j;
    }
  }

  //creates an array of EVERY couplet permutation, actually implemented using black magic, call the code wizards if you run into trouble
  double couplets[numberOfCouplets][3 + (2 * input)]; // [Minimum membership value, Groups[], Outputs[], Actual Output M1, Actual Output M2] both arrays are of size input
  for (int x = 0; x < numberOfCouplets; x++) {
    for (int L = 0; L < input; L++) {                                                                                   // Go until we reach the max permutations for the given inputs
      couplets[x][1 + L] = *(*(rangeAndOutputs) + (int)arrayOfCouplets[(x >> L) & 1][L]);                                // (x>>L)&1 -> x is current permutation number, (x>>L) does a bitwise shift to the right L indecies
      couplets[x][1 + input + L] = *(*(rangeAndOutputs) + (int)arrayOfCouplets[(x >> L) & 1][L] + 2);                     // &1 determines if the binary value at that point is a 1 or a 0
    }                                                                                                                     // we use this to determine which member function we use for a certain input and for all inputs to
    couplets[x][0] = smallestVal(*(couplets) + (1 + input) + x * (3 + (2 * input)), input);                             // create the permutations without repeat or stupid, ugly, 100X nested for loops
    couplets[x][1 + (2 * input)] = couplets[x][0] * determineOutput(*couplets + x * (3 + (2 * input)) + 1, inputs, *M1weights);   // after getting the index from arrayOfCouplets, we go into rangeAndOutputs, get the actual value and put it in our array
    couplets[x][2 + (2 * input)] = couplets[x][0] * determineOutput(*couplets + x * (3 + (2 * input)) + 1, inputs, *M2weights);
  }

  //Sum the minimum of the smallest output of each couplet
  double sumOfMins = 0.0;
  for (int i = 0; i < numberOfCouplets; i++) {
    sumOfMins += (couplets[i][0]);
  }

  //Take the sum of each couplet's output
  double outputGuessM2 = 0.0;
  double outputGuessM1 = 0.0;
  for (int i = 0; i < numberOfCouplets; i++) {
    outputGuessM1 += couplets[i][1 + (2 * input)];
    outputGuessM2 += couplets[i][2 + (2 * input)];
  }
  //Divide it by the sum of the mins to produce the algorithim's output
  outputGuessM1 /= sumOfMins;
  outputGuessM2 /= sumOfMins;

  //cost function is the typical square error calculation
  //.5 * pow(expected - outputGuess, 2);
  //To train the algorithim, we use the derivative of the cost function to find the absolute minimum of the cost function
  double errorM1 = (expectedOutputs[0] - outputGuessM1);
  double errorM2 = (expectedOutputs[1] - outputGuessM2);



  if (driveOrTrain) {
    //Prints variable weights and average error occationally
    printWeights(counter, *M1weights);
    printWeights(counter, *M2weights);
    //Adjust the weights based on each couplet, its contribution to the error, and the membership functions it utilizes
    for (int i = 0; i < numberOfCouplets; i++) {
      train(sumOfMins, errorM1, *(couplets) + i * (3 + (2 * input)), inputs, *M1weights);
      train(sumOfMins, errorM2, *(couplets) + i * (3 + (2 * input)), inputs, *M2weights);
    }
  }
  else {
    Serial.println(outputGuessM2);
    servoRight.writeMicroseconds(outputGuessM2);
    servoLeft.writeMicroseconds(outputGuessM1);
  }
}

//prints out weight matrix, output can be copied and pasted into feedforward for output, no train, usage
void printWeights(int r, double * waits) {
  int printEveryNtrainingExamples = 1000;
  if (r % printEveryNtrainingExamples == 0) {
    Serial.println("{");
    for (int i = 0; i < numberOfWeightArrays; i++) {
      Serial.print("  {");
      for (int j = 0; j < input + 1; j++) {
        Serial.print(*(waits + i * (input + 1) + j), 5);
        if (j != input)
          Serial.print(", ");
      }
      Serial.println("},");
    }
    Serial.println("}");
    Serial.println();
  }
}

//use weights to do some calculations for each couplet's output
//note, it doesn't complete a full calculation based on inputs, it does part of a calculation, couplet per couplet
//pointer starts at index 1
double determineOutput(double * couplet, int * inputs, double * whichWeights) {
  double index = 0;
  for (int i = 0; i < input; i++) {
    index += *(couplet + i) * pow(numOfFuzzifiers, input - i - 1);
  }
  int inx = (int)index;
  double ans = 0;
  for (int i = 0; i < input; i++) {
    ans += *(inputs + i) * *(whichWeights + i + inx * (input + 1));
  }
  ans += *(whichWeights + input +  inx * (input + 1));
  return ans;
}

//compute gradient decent, adjust weights, reduce error
void train(double sumOfMins, double error, double * couplet, int * inputs, double * whichWeights) {
  double constant = *(couplet) * alpha * error / sumOfMins;
  double index = 0;
  for (int i = 0; i < input; i++) {
    index += *(couplet + i + 1) * pow(numOfFuzzifiers, input - i - 1);
  }
  int inx = (int)index;
  for (int i = 0; i < input; i++) {
    *(whichWeights + inx * (input + 1) + i) += constant * *(inputs + i);
  }
  *(whichWeights + inx * (input + 1) + input) += inputRange / 2 * constant;
}

//Based on an input, determine which fuzzy member groups it belongs to, and its membership % to each group
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

//Based on how many fuzzifiers we have, we need to determine the range of each one
void defineMidpoints() {
  int width = ceil((double)inputRange / (double)(numOfFuzzifiers - 1));
  for (int i = 0; i < numOfFuzzifiers; i++) {
    midPts[i] = i * width;
  }
}

//Fills the weight array with random value in the +- 1 range ish, perhaps expand it to lower learning time or rate?
void createWeights(double * ans) {
  for (int i = 0; i < numberOfWeightArrays; i++) {
    for (int j = 0; j < (input + 1); j++) {
      double r = ((double)random(-100, 100) / 10);
      *(ans + (i * (input + 1)) + j) = r;
    }
  }
}

//Determines the smallest value out of an array of value
double smallestVal(double * a, int num) {
  double smallest = *a;
  for (int i = 0; i < num; i++) {
    if (*(a + i) < smallest)
      smallest = *(a + i);
  }
  return smallest;
}
