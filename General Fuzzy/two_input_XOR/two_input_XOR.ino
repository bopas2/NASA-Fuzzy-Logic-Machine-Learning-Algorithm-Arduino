#include <math.h>

const int inputRange = 255; // X axis of values inputed from user/sensors
const int outputRange = 1;  // Y axis of values that represent how much of a member a specific input is
const int numOfFuzzifiers = 5; // Number of members that an input can belong to

float slope = outputRange / (float)((inputRange / numOfFuzzifiers));   // Slope of triangles based on our graph layout

const int input = 2; // Number of values we may input - VARIABLE
const int output = 1; // Number of values outputed - LIMITED TO ONE
const int numberOfWeightArrays = (int)pow(numOfFuzzifiers, input);
double weights[numberOfWeightArrays][input + 1]; // Weights for each combination of input member functions
// 1A to 2A, 1A to 2B, 1A to 2C ... 1B to 2A, 1B to 2B ... 1Y to 2A, 1Y to 2B  etc

const double alpha = .00001; // Controls how fast the algorthim learns...largest value we can have without overshooting the cost min

double midPts[numOfFuzzifiers]; // X value where the tip of each triangle ( or center of member function is )

void setup() {
  randomSeed(analogRead(4)); //randomize the random number
  Serial.begin(9600);
  defineMidpoints(); // Determine where each member function exists on the x-y universe
  createWeights(*weights); // Give us random weights to break symetry
}

const int numberOfCouplets = pow(2, input); // How many combinations we have based on one set of X input variables
double counter = 0;
int numCorrect = 0;
void loop() {
  int inputs[input]; // What inputs we have for this iter
  double expected = 100; // Expected output
  inputs[0] = random(255); inputs[1] = random(255);
  if (inputs[0] <= 128 && inputs[1] <= 128)
    expected = 0;
  else if (inputs[0] > 128 && inputs[1] > 128)
    expected = 0;

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

  //Divide it by the sum of the mins to produce the algorithim's output
  outputGuess /= sumOfMins;

  //cost function is the typical square error calculation
  //.5 * pow(expected - outputGuess, 2);
  //To train the algorithim, we use the derivative of the cost function to find the absolute minimum of the cost function
  double error =  (expected - outputGuess);

  if (abs(error) < 50) {
    numCorrect++;
  }
  if (fmod(counter, 200) == 0) {
    Serial.println((double)numCorrect / 200 * 100);
    numCorrect = 0;
  }
  //Prints variable weights and average error occationally
  printWeights(counter);

  //Adjust the weights based on each couplet, its contribution to the error, and the membership functions it utilizes
  for (int i = 0; i < numberOfCouplets; i++) {
    train(sumOfMins, error, *(couplets) + i * (2 + (2 * input)), inputs);
  }
  counter++;
}

//prints out weight matrix, output can be copied and pasted into feedforward for output, no train, usage
void printWeights(int r) {
  int printEveryNtrainingExamples = 10000;
  if (r % printEveryNtrainingExamples == 0) {
    Serial.println("Weights:");
    Serial.println("{");
    for (int i = 0; i < numberOfWeightArrays; i++) {
      Serial.print("  {");
      for (int j = 0; j < input + 1; j++) {
        Serial.print(weights[i][j], 5);
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
double determineOutput(double * couplet, int * inputs) {
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

//compute gradient decent, adjust weights, reduce error
void train(double sumOfMins, double error, double * couplet, int * inputs) {
  double constant = *(couplet) * alpha * error / sumOfMins;
  double index = 0;
  for (int i = 0; i < input; i++) {
    index += *(couplet + i + 1) * pow(numOfFuzzifiers, input - i - 1);
  }
  int inx = (int)index;
  for (int i = 0; i < input; i++) {
    weights[inx][i] += constant * *(inputs + i);
  }
  weights[inx][input] += inputRange / 2 * constant;
}

//Based on an input, determine which fuzzy member groups it belongs to, and its membership % to each group
void returnOutputAndInterval(int input, double * ans) {
  int radius = ceil((double)inputRange / (double)numOfFuzzifiers);
  for (int i = 0; i < numOfFuzzifiers; i++) {
    if (input > midPts[i] && input < midPts[i + 1]) {  //if in between two points
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
      double r = ((double)random(-100, 100) / 100);
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
