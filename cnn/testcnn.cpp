//
// testcnn     minimal program to ensure the neural network code works.
// 

#include <iostream>
#include <vector>
#include "cnn.h"

int main(int argc, char *argv[])
{

	// typical exclusive-or test for NN: 
	struct {
		std::vector<float> input,labels;
	} trainset[4] = {  // a,b, a^b
		{{ 0, 0},{ 0} },
		{{ 1, 0},{ 1} },
		{{ 0, 1},{ 1} },
		{{ 1, 1},{ 0} },
	};
	CNN nn({ 2, 2, 1 });  // simple constructor takes input, hidden(s), and output.  So builds two fully interconnected layers with tanh activation layers following them
	std::cout << "epoch   mse\n--------------------\n";
	for (int i = 0; i <= (1 << 16); i++)
	{
		float mse = 0.0f;
		for (auto s : trainset)
			mse += nn.Train(s.input, s.labels); 
		if (0 == (i&(i - 1))) std::cout << i << "   " << mse << "\n";
	}
	std::cout << "\n";
}

