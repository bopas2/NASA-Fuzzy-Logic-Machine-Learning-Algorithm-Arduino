#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

int main() {
	string String;
	ifstream infile;
	infile.open("trainingData.txt");
	while(!infile.eof()) {
		getline(infile,String);
		int counter = 0; 
		int indecies[3]; 
		for(int i = 0; i < String.length(); i++) {
			if(String.substr(i,1).compare(" ") == 0) {
				indecies[counter] = i;
				counter++;
			}
		}
		int data[4];
		for(int i = 0; i < 4; i++) {
			if(i == 0) 
				data[i] = atoi(String.substr(0,indecies[1]).c_str());
			else if(i == 4) 
				data[i] = atoi(String.substr(indecies[i-1]+1,String.length() - indecies[i-1]).c_str());
			else 
				data[i] = atoi(String.substr(indecies[i-1]+1,indecies[i]-indecies[i-1]).c_str());
		}
		cout << data[1] << endl;
	}
	infile.close();
	system ("pause");
	return 0;
}
