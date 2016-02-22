//
// testcnn     minimal program to ensure the neural network code works.
// 
//  I didn't want to bloat this repo with data files, so please download the four mnist data files and place in this directory.
//  look for names "train-images-idx3-ubyte".  Easy to find.  
//  There are many copies on the internet including a handful of other github repos. 

#include <iostream>
#include <fstream>
#include <vector>
#include "cnn.h"
#include <geometric.h>
#include <Windows.h>  // for messagebox if an error is thrown

// the mnist dataset is the standard nn benchmark and extensively studied.  
// A good test to ensure any cnn code is behaving as expected.
// There's not much point writing a general purpose parser that properly 
// decodes the other-byte-ordered header part to extract count,width,height 
// for the idx{1,3} format when there are only 2 files on the planet using this format that even matter.
// so i just seek to the start position, and read the well known data 28x28x60000 block in uchar format.
// 
std::vector<std::vector<float>> mnist_read_images(std::string filename,int count)
{
	std::ifstream in(filename, std::istream::binary);
	if (!in.is_open()) throw("unable to open mnist dataset, please download this if you haven't already");
	in.seekg(16);
	std::vector<unsigned char> buf(count * 28 * 28);
	in.read((char*) buf.data(), buf.size());
	auto buff = Transform(buf, [](unsigned char s) {return s / 255.0f;});
	std::vector<std::vector<float>> images(count);
	for (int i = 0;i < count;i++)
		images[i] = std::vector<float>(buff.data() + i * 28 * 28, buff.data() + (i + 1) * 28 * 28);
	return images;
}
std::vector<std::vector<float>> mnist_read_labels(std::string filename, int count)
{
	std::ifstream in(filename, std::istream::binary);
	if (!in.is_open()) throw("unable to open mnist dataset, please download this if you haven't already");
	in.seekg(8);
	std::vector<unsigned char> buf(count );
	in.read((char*)buf.data(), buf.size());
	std::vector<std::vector<float>> labels(count);
	for (int i = 0;i < count;i++)
	{
		labels[i] = std::vector<float>(10, 0.0f);
		labels[i][buf[i]] = 1.0f;
	}
	return labels;
}
int minst_best(std::vector<float> &v) { assert(v.size() == 10); int best = 0; for (int j = 0;j < 10;j++) if (v[j]>v[best]) best = j; return best; }


void mnist()
{
	std::cout << "mnist\n";
	std::cout << "be patient this may take minutes.  suggest you use 'release' mode.\n";
	std::cout << "should get close to 99% correctness\n"; 
	auto train_in = mnist_read_images("train-images-idx3-ubyte", 60000);
	auto train_lb = mnist_read_labels("train-labels-idx1-ubyte", 60000);
	auto test_in  = mnist_read_images("t10k-images-idx3-ubyte" , 10000);
	auto test_lb  = mnist_read_labels("t10k-labels-idx1-ubyte" , 10000);

	// I just used a typical cnn setup here.
	// Feel free to try other configurations
	CNN cnn({});
	cnn.layers.push_back(new CNN::LConv({ 28,28,1 }, { 5,5,1,16 }, { 24,24,16 }));
	cnn.layers.push_back(new CNN::LActivation<TanH>(24 * 24 * 16));
	cnn.layers.push_back(new CNN::LMaxPool({ 24,24,16 }));
	cnn.layers.push_back(new CNN::LMaxPool({ 12,12,16 }));
	cnn.layers.push_back(new CNN::LConv({ 6,6,16 }, { 3,3,16,64 }, { 4,4,64 }));
	cnn.layers.push_back(new CNN::LActivation<TanH>(4 * 4 * 64));
	//cnn.layers.push_back(new CNN::LMaxPool({ 8,8,256 }));
	cnn.layers.push_back(new CNN::LFull(4 * 4 * 64, 64));
	cnn.layers.push_back(new CNN::LActivation<TanH>(64));
	cnn.layers.push_back(new CNN::LFull(64, 10));


	for (int e = 0; e < 20;e++) // each training epoch does an initial test followed by backprop on all 60K samples.
	{
		int correct = 0;
		for (int i = 0;i < 10000;i++)
			correct += (minst_best(cnn.Eval(test_in[i])) == minst_best(test_lb[i]));
		std::cout << correct << " of 10000 correct\n";
		for (int i = 0;i < 60000;i++)
			cnn.Train(train_in[i], train_lb[i]);
	}
}

void xor()
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
	for (int i = 0; i <= (1 << 20); i++)
	{
		float sse = 0.0f;  // sum of square error in this case
		for (auto s : trainset)
			sse += nn.Train(s.input, s.labels); 
		if (0 == (i&(i - 1))) // if power of 2
			std::cout << i << "   " << sse << "\n";
	}
	std::cout << "\n";
}

void junk()
{
	CNN::LConv cl({ 5,5,1 }, { 3,3,1,2 }, { 3,3,2 });
	CNN cnn({});
	cnn.layers.push_back(&cl);
	std::ifstream("deleteme.txt") >> cl;
	std::cout << "weights initial: " << cl;
	std::cout << "\n\n";
	std::vector<float> in(25);
	for (int i = 0; i < 25; i++)in[i] = (float)i;
	std::vector<float> expected(18, 0.0f);
	auto out = cl.forward(in);
	std::cout << "output \n";
	for (auto y : out)
		std::cout << y << " ";
	std::cout << "\n\n";
	cnn.Train(in, expected);
	std::cout << "weights now: " << cl;
	std::cout << "\n\n";

	for (auto v : vol_iteration({ 2,2,2 }))
		std::cout << v << "\n";
	std::cout << "\n";
}

int main(int argc, char *argv[]) try
{
	//xor();
	mnist();

	std::cout << "\n";
	return 0;


}
catch (const char *c)
{
	std::cerr << "Program aborted: " << c << "\n";
	MessageBox(GetActiveWindow(), c, "FAIL", 0);
}
catch (std::exception e)
{
	std::cerr << "Program aborted: " << e.what() << "\n";
	MessageBox(GetActiveWindow(), e.what(), "FAIL", 0);
}
