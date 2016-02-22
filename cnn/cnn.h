// 
// nothing to see here.  just rewrote this for my own educational benefit.
// small and inefficient implementation of conv neural network  
// (uhm  conv and pooling layer types yet to be implemented)
// 
// inspired by layer design from torch and from compact c++11 implementation goals from tiny cnn
// 

#ifndef MINI_CNN_H
#define MINI_CNN_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <assert.h>

struct Sigmoid
{
	static float f (float t  )  { return 1.0f / (1 + exp(-t)); }
	static float df(float f_t)  { return f_t*(1 - f_t); }
};
struct TanH
{
	static float f(float t)  	{ auto e = std::exp(2 * t); return (e - 1) / (e + 1); }      // { return std::tanh(t); }  // even compiling with sse and fast math, tanh is slow , but there is a sse2 optimized fast path for exp()
	static float df(float f_t)  { return 1.0f - f_t*f_t; }
};
struct ReLU
{
	static float f(float t)     { return std::max(0.0f,t); }
	static float df(float f_t)  { return (f_t > 0.0f) ? 1.0f : 0.0f; }
};


struct CNN
{
	

	struct LBase // base class implemented as identity layer
	{
		mutable std::vector<float> X; // cached input            <- optimization todo:  we dont need to duplicate many of these and hence do extra copying, construction, etc
		mutable std::vector<float> Y; // cached output
		mutable std::vector<float> D; // cached error at input
		mutable std::vector<float> E; // cached error at output
		LBase(){};
		LBase(int n) :X(n), Y(n), D(n), E(n) {}
		LBase(int input_size, int output_size) :X(input_size), Y(output_size), D(input_size), E(output_size) {}
		const std::vector<float> &forward(const float* x) { return forward(std::vector<float>(x,x+X.size())); }
		virtual const std::vector<float> &forward(std::vector<float> x) { X = x; Y = x; return Y; }
		virtual const std::vector<float> &backward(std::vector<float> e) { E = e; D = e; return D; }
		virtual void  update(float alpha) {}
	};
	struct LFull : public LBase
	{
		std::vector<float> W;
		LFull(int input_size, int output_size) :LBase(input_size, output_size) , W((input_size + 1) * output_size)
		{
			std::default_random_engine random_number_generator;
			float range = 1.0f / sqrtf((float)input_size);
			for (auto &w : W)
				w = std::uniform_real<float>(-2.0f*range, 2.0f*range)(random_number_generator);
		}
		virtual std::vector<float> &forward(std::vector<float> x)
		{
			X = x;
			auto M = X.size();
			for (unsigned int i = 0; i < Y.size(); i++)
			{
				Y[i] = W[M + i*(M + 1)];
				for (unsigned int j = 0; j < M; j++)
					Y[i] += x[j] * W[j + i*(M + 1)];
			}
			return Y;
		}
		virtual const std::vector<float> &backward(std::vector<float> e)
		{
			E = e;
			for (auto &d : D)
				d = 0.0f;
			for (unsigned int i = 0; i < Y.size(); i++)
				for (unsigned int j = 0; j < X.size(); j++)
					D[j] += W[j + i*(X.size() - 1)] * E[i];
			return D;
		}
		void update(float alpha)
		{
			auto M = X.size(); // input_size
			for (unsigned int i = 0; i < Y.size(); i++)
			{
				for (unsigned int j = 0; j < M; j++)
					W[i*(M + 1) + j] -= X[j] * E[i] * alpha;
				W[i*(M + 1) + M] -= E[i] * alpha;
			}
		}
	};
	template<class F>
	struct LActivation :public LBase // F is the activation function
	{
		LActivation(int n) :LBase(n){}
		virtual const std::vector<float> &forward(std::vector<float> x)
		{
			X = x;
			std::transform(X.begin(), X.end(), Y.begin(), F::f);
			return Y;
		}
		virtual const std::vector<float> &backward(std::vector<float> e)
		{
			E = e;
			std::transform(E.begin(), E.end(), Y.begin(), D.begin(), [](float g, float y){ return F::df(y) * g; });    // for example sigmoid would be d = e * y*(1-y)
			return D;
		}
	};
	std::vector<LBase*> layers;

	std::vector<float> Eval(std::vector<float> x)
	{
		auto y = std::move(x);
		for (auto &layer : layers)
			y = layer->forward(std::move(y));
		return y;
	}

	float Train(const std::vector<float> &x, const std::vector<float> &t, float alpha = 0.1f)
	{
		auto y = x;
		for (auto &layer : layers)
			y = layer->forward(std::move(y));
		assert(y.size() == t.size());

		float mse = 0;                                             // mean square error 
		std::vector<float> e(y.size());
		std::transform(y.begin(), y.end(), t.begin(), e.begin(), [&mse](float y, float t)->float { float e = y - t; mse += e*e ; return e; });
		mse /= y.size();

		for (auto rit = layers.rbegin(); rit != layers.rend(); ++rit)   // gradient backprop
			e = (*rit)->backward(e);

		for (auto &layer:layers)
			layer->update( alpha);

		static volatile bool trace_here = 0;  
//		if (trace_here) printf("%f %f\n", t[0],y[0],e[0]);
		return   mse; 
	}

	CNN(const std::vector<int> &s)  // quick test for simple NNs
	{
		for (unsigned int i = 1; i < s.size(); i++)
		{
			layers.push_back(new LFull(s[i - 1], s[i]));
			layers.push_back(new LActivation<TanH>(s[i]));
		}
	
	}
};

#endif     // MINI_CNN_H

