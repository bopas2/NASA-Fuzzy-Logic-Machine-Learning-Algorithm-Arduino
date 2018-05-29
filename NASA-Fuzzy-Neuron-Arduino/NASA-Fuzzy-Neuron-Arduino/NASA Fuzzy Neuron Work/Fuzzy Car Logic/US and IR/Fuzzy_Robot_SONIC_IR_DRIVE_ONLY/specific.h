const int input = 2;
const int output = 2;
const int numOfFuzzifiers = 4;
const int inputRange = 255;
const int outputRange = 1;
//including infrared essentially doubles the amount of weights needed (close vs far), 1 or 0, no fuzzy needed
const int numberOfWeightArrays = 2 * (int)pow(numOfFuzzifiers, input);
const int sensorRange = 30; //MUST BE SAME AS WHEN TRAIN
const int triggerL = 11; //left ultrasonic sensor trigger
const int triggerR = 2; //right US sensor trig
const int echoL = 10; // left ultrasonic sensor reciever pin
const int echoR = 3; // right US reciver
const int ledOut = 5;    // trigger IR sensor
const int IR_IN = 7;     // read IR sensor

double M1weights[numberOfWeightArrays][input + 2] =
{
  {9.5, 3.5, -8.6, 3},
  {3, 8.9, 1.7, -6},
  {0.24628, 8.52376, -8.7, 2.90148},
  {0.0190026, 6.66603, 5.1, -1.00511},
  {6.7, -8.1, -5.2, -6.4},
  {6.4, -9.8, 0.5, 6.9},
  {-2.65199, 9.32476, -6.3, 11.996},
  {-0.00410975, 6.72828, 8.7, -15.158},
  {1.6, 2.8, 8.1, -2.2},
  {5, -0.7, 2, 2},
  {-14.4878, 23.6126, -3.1, 24.8066},
  {0.00401563, 6.58016, 5.1, 21.5417},
  {8.1473, -5.96739, -0.6, 6.00944},
  {8.92126, -3.65111, -6.5, 0.950586},
  {4.94292, 1.79162, 0.9, 152.953},
  {0.00966749, 6.44915, -6.5, 53.7326},
  {32.6026, 33.7026, -1.91068e-005, 63.0241},
  {23.5939, 4.34593, -1.91068e-005, 41.7808},
  {19.3835, 2.46343, 0, 8.89519},
  {-0.485793, 5.87851, 0.00419607, 237.499},
  {23.5827, 33.6004, -1.91068e-005, 66.8249},
  {2.79915, 8.06745, -8.12041e-005, 495.164},
  {-4.40261, 7.31474, 0, 868.668},
  {1.66689, 3.22911, 0.00491953, 789.5},
  {29.2062, 20.4902, -0.0263842, 35.0015},
  {-8.62902, 14.7933, -0.0527684, 297.298},
  {17.3491, -6.09561, -0, 774.444},
  {1.59395, 2.73002, -0.00213142, 772.858},
  {6.57502, 0.276938, 0.000198882, 4.91501},
  {6.73744, -0.389803, 0.000233172, 2.59708},
  {-1.7, 2.6, -0, -8.9},
  {0.431522, 6.21014, -0.00231434, 5.97799},
};

double M2weights[numberOfWeightArrays][input + 2] =
{
  {9.5, 3.5, -8.6, 3},
  {3, 8.9, 1.7, -6},
  {4.83162, 6.26291, -8.7, -13.8101},
  {-9.49672, 7.43366, 5.1, 0.669289},
  {6.7, -8.1, -5.2, -6.4},
  {6.4, -9.8, 0.5, 6.9},
  {-13.4605, 14.8501, -6.3, -1.58334},
  {-4.3858, 7.28774, 8.7, -10.9766},
  {1.6, 2.8, 8.1, -2.2},
  {5, -0.7, 2, 2},
  {-1.45881, 10.5307, -3.1, 6.28548},
  {0.224037, 4.48806, 5.1, -0.384092},
  {6.35207, -6.44611, -0.6, 5.11535},
  {6.07888, -2.22635, -6.5, -0.46503},
  {3.83036, 1.1118, 0.9, 47.328},
  {-0.163432, 4.52299, -6.5, 70.2401},
  {32.2502, 33.3502, -0.000156135, 62.1467},
  {19.9355, -4.82241, -0.000156135, 32.6708},
  {22.1496, 1.27873, -0, 12.7804},
  {-6.95918, 5.80168, -0.00879108, 403.908},
  {24.1959, 34.7926, -0.000156135, 68.8141},
  {0.31062, 10.4305, -0.000663573, 645.669},
  {-11.0067, 7.2024, -0, 1250.88},
  {-4.00559, 4.18291, -0.0103068, 835.971},
  {21.7699, 19.0691, -0.0269592, 28.9007},
  {-1.42562, 3.19622, -0.0539185, 402.181},
  {33.7175, -21.3672, -0, 1125.1},
  {2.29046, 1.91112, -0.0758849, 334.308},
  {13.3133, -32.1838, 0.0612716, 8.27096},
  {4.96409, -5.45268, 0.0718357, 1.71388},
  {-1.7, 2.6, 0, -8.9},
  {5.43928, 0.689702, 0.626892, 3.2286},
};
