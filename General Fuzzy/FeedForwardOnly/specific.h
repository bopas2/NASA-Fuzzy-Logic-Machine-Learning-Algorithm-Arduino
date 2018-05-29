const int input = 2;
const int numOfFuzzifiers = 5;
const int inputRange = 255;
const int outputRange = 1;
const int numberOfWeightArrays = (int)pow(numOfFuzzifiers, input);
const double alpha = .00005;

double weights[numberOfWeightArrays][input + 1] =
{
  {0.23807, -0.02078, -0.33041},
  {0.55942, -0.42059, -0.31432},
  { -0.48828, 0.40888, -0.17328},
  {0.36020, 0.54501, 0.89385},
  { -0.30579, 0.47265, 0.91670},
  { -0.46543, 0.58884, 0.22608},
  { -0.38183, -0.09225, -0.14728},
  { -0.62500, 0.81263, -0.00587},
  {0.75390, 0.24769, 0.35993},
  {0.04433, 0.35517, 0.43745},
  {0.63192, -0.39617, -0.27570},
  {1.16302, -0.63540, -1.21800},
  {0.30150, -0.12290, 0.87647},
  { -1.27583, 0.94321, 0.17756},
  { -0.71385, 0.75644, -0.01835},
  {0.72617, 0.17770, 0.15566},
  {0.11850, 0.73632, -0.61131},
  {0.84375, -0.85862, 0.91720},
  {0.20111, -0.29383, 0.41547},
  {0.85640, -0.58706, 0.16122},
  {0.38127, -0.28037, 0.52988},
  {0.40391, 0.14739, -0.38802},
  {0.33441, -1.22187, -0.55640},
  {0.19627, 0.08897, 0.91389},
  { -0.66696, -0.38168, -0.30317},
};
