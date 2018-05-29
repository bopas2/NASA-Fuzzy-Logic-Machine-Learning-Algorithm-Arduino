#include <math.h>
#include <Servo.h>                          // Include servo library
#include "specific.h"

Servo servoRight;                           // Right is motor 2
Servo servoLeft;                            // Left is motor 1

float slope = outputRange / (float)((inputRange / numOfFuzzifiers));   // Slope of triangles based on our graph layout

const double alpha = .00001; // Controls how fast the algorthim learns...largest value we can have without overshooting the cost min

double midPts[numOfFuzzifiers]; // X value where the tip of each triangle ( or center of member function is )

void setup() {
  randomSeed(analogRead(4));  //randomize the random number
  Serial.begin(9600);
  defineMidpoints();          // Determine where each member function exists on the x-y universe
  servoRight.attach(13);      // define the arduino pins the servos are attached to
  servoLeft.attach(12);
  pinMode(IR_IN, INPUT);      // same for IR sensor
  pinMode(ledOut, OUTPUT);
  pinMode(triggerL, OUTPUT); pinMode(triggerR, OUTPUT); // same for ultrasonic sensors
  pinMode(echoL, INPUT); pinMode(echoR, INPUT);
}

char currentState = 'q'; // start in 'off' state

int inputs[input + 1]; // +1 for IR sensor
double expectedOutputs[output];
void loop() {
  detect();
  if (inputs[0] != 0 || inputs[1] != 0) {
    fuzzy();
  }
}

//Gathers information from sensors
int viewDistance = 30;
void detect() {
  inputs[0] = map(USDetect(triggerL, echoL, viewDistance), 0, viewDistance, 0, inputRange);
  delay(20);
  inputs[1] = map(USDetect(triggerR, echoR, viewDistance), 0, viewDistance, 0, inputRange);
  inputs[2] = map(1 - IRDetect(ledOut, IR_IN, 38000), 0, 1, 0, inputRange);
}

//reads distance from ultrasonic sensors
int USDetect(int trigger, int echo, int maxi)
{
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  double distance = pulseIn(echo, HIGH) / 2 * .0343;
  if (distance > maxi)
    return maxi;
  return distance;
}

//reads from infrared sensor
int IRDetect(int irLedPin, int irReceiverPin, long freq) {
  tone(irLedPin, freq, 8);
  delay(1);
  int r = digitalRead(irReceiverPin);
  delay(1);
  return r;
}

const int numberOfCouplets = pow(2, input); // How many combinations we have based on one set of X input variables
//training and decision part of the arduino
void fuzzy() {
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
    for (int L = 0; L < input; L++) {                                                                                             // Go until we reach the max permutations for the given inputs
      couplets[x][1 + L] = *(*(rangeAndOutputs) + (int)arrayOfCouplets[(x >> L) & 1][L]);                                         // (x>>L)&1 -> x is current permutation number, (x>>L) does a bitwise shift to the right L indecies
      couplets[x][1 + input + L] = *(*(rangeAndOutputs) + (int)arrayOfCouplets[(x >> L) & 1][L] + 2);                             // &1 determines if the binary value at that point is a 1 or a 0
    }                                                                                                                             // we use this to determine which member function we use for a certain input and for all inputs to
    couplets[x][0] = smallestVal(*(couplets) + (1 + input) + x * (3 + (2 * input)), input);                                       // create the permutations without repeat or stupid, ugly, 100X nested for loops
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
  Serial.print(inputs[0]); Serial.print(" "); Serial.print(inputs[1]); Serial.print(" "); Serial.print(inputs[2]); Serial.print(" "); 
  Serial.print(outputGuessM2); Serial.print(" ");
  Serial.println(outputGuessM1);
  servoRight.writeMicroseconds(outputGuessM2);
  servoLeft.writeMicroseconds(outputGuessM1);
}

//prints out weight matrix, output can be copied and pasted into feedforward for output, no train, usage
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

//use weights to do some calculations for each couplet's output
//note, it doesn't complete a full calculation based on inputs, it does part of a calculation, couplet per couplet
//pointer starts at index 1
double determineOutput(double * couplet, int * inputs, double * whichWeights) {
  double index = 0;
  for (int i = 0; i < input; i++) {
    index += *(couplet + i) * pow(numOfFuzzifiers, input - i - 1);
  }
  double ans = 0;
  int inx = (int)index;
  if (inputs[2] == 255) { // account for infrared sensor
    index += numberOfWeightArrays / 2;
    inx = (int)index;
    ans += *(inputs + 2) * *(whichWeights + input +  inx * (input + 2));
  }
  for (int i = 0; i < input; i++) {
    ans += *(inputs + i) * *(whichWeights + i + inx * (input + 2));
  }
  ans += *(whichWeights + input + 1 + inx * (input + 2));
  return ans;
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
    for (int j = 0; j < (input + 2); j++) {
      double r = ((double)random(-100, 100) / 10);
      *(ans + (i * (input + 2)) + j) = r;
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
