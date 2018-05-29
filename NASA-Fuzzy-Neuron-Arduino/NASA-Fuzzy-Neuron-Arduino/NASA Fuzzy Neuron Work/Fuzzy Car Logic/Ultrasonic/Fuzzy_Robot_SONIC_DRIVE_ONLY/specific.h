const int input = 2;
const int numOfFuzzifiers = 4;
const int inputRange = 255;
const int outputRange = 1;
const int numberOfWeightArrays = (int)pow(numOfFuzzifiers, input);
const int sensorRange = 30; //MUST BE SAME AS WHEN TRAIN
const int triggerL = 11; //left ultrasonic sensor trigger
const int triggerR = 2; //right US sensor trig
const int echoL = 10; // left ultrasonic sensor reciever pin
const int echoR = 3; // right US reciver

double M1weights[numberOfWeightArrays][input + 1] =
{
  {-2, 0.8, 7.4},
  {4.8, 7, 1.7},
  {-8.59432, -4.96291, 10.9185},
  {-0.487877, 7.08648, 7.99324},
  {4.6, -0.2, -2.7},
  {3.39482, 10.2315, 15.1712},
  {9.47265, 1.59817, 35.6624},
  {0.905082, 6.03487, 180.315},
  {4.23041, 13.7438, 4.88136},
  {4.55615, 7.46483, 66.998},
  {5.63458, 4.32844, 142.388},
  {2.33387, 5.0528, 129.335},
  {1.77575, 16.6778, -34.6136},
  {5.70151, 0.626985, 139.271},
  {6.28038, -0.0273414, 113.539},
  {3.18434, 2.07275, 344.049},
};

double M2weights[numberOfWeightArrays][input + 1] =
{
  {-2, 0.8, 7.4},
  {4.8, 7, 1.7},
  {-8.8915, -0.680957, 13.0595},
  {-31.0015, 13.9125, 11.4062},
  {4.6, -0.2, -2.7},
  {1.93879, 7.4924, 8.32367},
  {7.70026, 0.376233, 29.13},
  {-1.01351, 5.53962, 226.288},
  {1.07195, 15.4633, 9.38485},
  {3.83432, 4.17322, 60.7991},
  {3.25967, 3.76111, 118.855},
  {2.79546, 2.80718, 110.483},
  {3.94477, 1.10051, 116.53},
  {4.27624, -0.0572488, 112.861},
  {4.23142, 0.35014, 79.3057},
  {2.11842, 1.56252, 247.643},
};
