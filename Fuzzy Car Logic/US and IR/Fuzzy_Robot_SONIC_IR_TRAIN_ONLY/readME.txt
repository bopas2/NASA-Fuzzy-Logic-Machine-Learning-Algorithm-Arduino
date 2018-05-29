The idea of this program is to train the algorithm on a desktop -- not the Arduino -- to maximize efficency. 

Simply train the robot per usual, and have sensor readings and user input printed to serial screen.

Copy and paste this data into trainingInfo.txt.

Use trainer.cpp to train the data for however many interations you want. 
Once done it will print out the trained weights to the screen, which can be copied and utilized.

If one changes the amount of inputs or record format, the fileRead functionality must also be adjusted to avoid errors.