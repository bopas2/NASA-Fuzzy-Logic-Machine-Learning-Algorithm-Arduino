#include <math.h>
#include "specific.h"

float slope = outputRange / (float)((inputRange / numOfFuzzifiers));
const int output = 1;

double midPts[numOfFuzzifiers]; // X value where the tip of each triangle ( or center of member function is )

void setup() {
  Serial.begin(9600);
  defineMidpoints(); // Determine where each member function exists on the x-y universe
}

int numberOfCouplets = pow(2, input); // How many combinations we have based on one set of X input variables

void loop() {
  double inputs[input];
  for(int i = 0; i < input; i++) {
    Serial.print("Enter input ");
    Serial.print(i);
    Serial.println(": ");
    delay(3000);    
    inputs[i] = Serial.parseInt();
    Serial.println(inputs[i]);
  }

  double rangeAndOutputs[input][4]; // [Lower Bound1, Upper Bound1, Ydown, Yup] aka [Membership A (left midpoint), Membership B (right midpoint), DegreeOfMembership of A, DegreeOfMembership of B]
  for (int i = 0; i < input; i++) {
    returnOutputAndInterval(inputs[i], (*rangeAndOutputs) + 4 * i);
  }

  double arrayOfCouplets[2][input];  // This function creates a matrix of size 2xinput WHERE each value contains the index value for its membership function in the rangeAndOutputs array
  for (int i = 0; i < input; i++) {         // e.g: [In1 LowerBound, In2 LowerBound ... etc ]
    for (int j = 0; j < 2; j++) {              //   [In1 UpperBound, In2 UpperBound ... etc ]
      arrayOfCouplets[j][i] = (i * 4) + j;
    }
  }

  //creates an array of EVERY couplet permutation, implemented using black magic, call the wizards if you run into trouble
  double couplets[numberOfCouplets][2 + (2 * input)]; // [Minimum membership value, Groups[], Outputs[], Actual Output using weights] both arrays are of size input
  for (int x = 0; x < numberOfCouplets; x++) {
    for (int L = 0; L < input; L++) {                                                                                   // Go until we reach the max permutations for the given inputs
      couplets[x][1 + L] = *(*(rangeAndOutputs) + (int)arrayOfCouplets[(x >> L) & 1][L]);                                // (x>>L)&1 -> x is current permutation number, (x>>L) does a bitwise shift to the right L indecies
      couplets[x][1 + input + L] = *(*(rangeAndOutputs) + (int)arrayOfCouplets[(x >> L) & 1][L] + 2);                     // &1 determines if the binary value at that point is a 1 or a 0
    }                                                                                                                     // we use this to determine which member function we use for a certain input and for all inputs to
    couplets[x][0] = smallestVal(*(couplets) + (1 + input) + x * (2 + (2 * input)), input);                             // create the permutations without repeat or stupid, ugly, 100X nested for loops
    couplets[x][1 + (2 * input)] = couplets[x][0] * determineOutput(*couplets + x * (2 + (2 * input)) + 1, inputs);   // after getting the index from arrayOfCouplets, we go into rangeAndOutputs, get the actual value and put it in our array
  }

  //Sum the minimum of the smallest output of each couplet
  double sumOfMins = 0.0;
  for (int i = 0; i < numberOfCouplets; i++) {
    sumOfMins += (couplets[i][0]);
  }

  //Take the sum of each couplet's output
  double outputGuess = 0.0;
  for (int i = 0; i < numberOfCouplets; i++) {
    outputGuess += couplets[i][1 + (2 * input)];
  }
  //Divide it by the sum of the mins
  outputGuess /= sumOfMins;
  Serial.println(outputGuess);
  Serial.println();
}

//use weights to do some calculations for each couplet's output
//note, it doesn't complete a full calculation based on inputs, it does part of a calculation, couplet per couplet
double determineOutput(double * couplet, double * inputs) {
  double index = 0;
  for (int i = 0; i < input; i++) {
    index += *(couplet + i) * pow(numOfFuzzifiers, input - i - 1);
  }
  int inx = (int)index;
  double ans = 0;
  for (int i = 0; i < input; i++) {
    ans += *(inputs + i) * weights[inx][i];
  }
  ans += weights[inx][input];
  return ans;
}

//Based on an input, determine which fuzzy member groups it belongs to, and its membership % to each group
void returnOutputAndInterval(double input, double * ans) {
  int radius = ceil((double)inputRange / (double)numOfFuzzifiers);
  for (int i = 0; i < numOfFuzzifiers; i++) {
    if (input > midPts[i] && input < midPts[i + 1]) {  //if in between two points
      *(ans) = i;
      *(ans + 1) = i + 1;   //record the points its inbetween
      *(ans + 2) = outputRange - slope * (fmod(input,radius));
      *(ans + 3) = slope * (fmod(input,radius));
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

//Determines the smallest value out of an array of value
double smallestVal(double * a, int num) {
  double smallest = *a;
  for (int i = 0; i < num; i++) {
    if (*(a + i) < smallest)
      smallest = *(a + i);
  }
  return smallest;
}
