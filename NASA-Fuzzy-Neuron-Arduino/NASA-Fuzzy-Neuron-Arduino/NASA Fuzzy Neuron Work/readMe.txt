Thomas Lang's implementation of NASA's, Mike K.'s and Norm P.'s fuzzy logic machine learning algorithim for the Arduino Microcontroller. 5/25/2018

Best documented code to understand implementation: "Fuzzy_Robot_SONIC_IR_DRIVE_N_TRAIN.ino"

Uses fuzzification to 'sort' inputs into certain member functions, where each input belongs to two member functions by some percentage (totaling 100%).

e.g. input one may map 34% to member A and 66% to member B.
     input two may may 10% to member C and 90% to member D.

Each permutation of member function has a respective weight matrix which we use for calculations.

e.g. we have a weight matrix for all permutations of member functions:
     AC = [...] AD = [...]
     BC = [...] BD = [...]

Where the weight matrix has a variable for input one, input two and a constant variable that does seperate from inputs.

e.g. basic calculation:
	
     InputOne * weights[0] + InputTwo * weights[1] + weights[2];

After retreiving the weight matricies we require, we complete the calculation shown above for each input and each weight matrix.

e.g. if we have two inputs, this results in 4 numbers.

From there, we must normalize out outputs. This is done by utilizing the % membership calculated in step 1. 
We take one of the outputs from the previous step, and multiply the result by the smallest membership percentage from step one.

e.g. for AC. the result is calculated to be Z. the minimum from step 1 is 10% or .1. so the result is .1 * Z.

We repeat this step for all outputs, then add those results together.

Finally, we divide this number by the sum of all the smallest membership percentages for each membership permutations. This gives us our result.

e.g. total min sum = .1 + .34 + .1 + .66

This essentially ensures that no one output dominates the total output and to normalize the values to ensure that they are all in similar numeric ranges. 

With a dataset of inputs and outputs we can use gradient decent to adjust the weights to minimize the error. We use the general squared error formula to calculate error:

C = .5 * (outputGuess - expected)^2 

We use the derivative of the cost function to adjust the weights in the correction direction to reach the error minimum.

C' = expected - outputGuess

To adjust these weights we loop through each permutation's weight matrix and calculate an unique constant. The constant is calculated as follows:

const = (smallest membership %) * learningRate * error / (sum of smallest membership % of all current member permutations).

Where the learning constant is the rate at which the algorithm learns. It is currently capped at a very small number, as the weights will explode otherwise.
(We don't know why). We, again, normalize the amount we adjust the weights by, like we did when calculating the output.

After calculating this constant, for a certain weight matrix, it is time to adjust the matrix. 
We do this by adding to each index the respective input for that iteration times the constant. 
The final value, which is independent of inputs, has just the constant added to it. This is repeated for all member permutations.

e.g. adjusting AC's weight matrix:
     AC[0] += inputOne * const;
     AC[1] += inputTwo * const;
     AC[2] += const; 

This process is how we train and use the learning algorithim. 
Of course, using a microcontroller, and in general, the methods of completing this task are harder than described.

Some downfalls with the current implementation:
	The small size of the learning constant. It works, but really shouldn't need to be that small to function.
	Using doubles instead of integers uses alot of memory on the small Arduino microcomputer. 	

Interesting numbers: 

# of sets of weights = (# of member functions)^(# of inputs)
# of combination of member functions = 2^(# of inputs)

See patent for further technical explanation.

Thomas Lang's implementation of NASA's, Mike K.'s and Norm P.'s fuzzy logic machine learning algorithim for the Arduino Microcontroller. 5/25/2018