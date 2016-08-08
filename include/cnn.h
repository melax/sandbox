// 
// just a minimalist implementation of conv neural network  
// 
// inspired by layer and tensor design from torch and from compact c++11 implementation goals from tiny cnn
// 

#ifndef MINI_CNN_H
#define MINI_CNN_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <istream>
#include <assert.h>

#include "linalg.h"
#include "geometric.h"  // for the multi dimensional iterators

#include <immintrin.h>
static bool simd_enable=true;

struct Sigmoid
{
	static float f(float t) { return 1.0f / (1 + exp(-t)); }
	static float df(float f_t) { return f_t*(1 - f_t); }
};
struct TanH
{
	static float f(float t) { auto e = std::exp(2 * t); return (e - 1) / (e + 1); }      // { return std::tanh(t); }  // even compiling with sse and fast math, tanh is slow , but there is a sse2 optimized fast path for exp()
	static float df(float f_t) { return 1.0f - f_t*f_t; }
};
struct ReLU
{
	static float f(float t) { return std::max(0.0f, t); }
	static float df(float f_t) { return (f_t > 0.0f) ? 1.0f : 0.0f; }
};
struct LeakyReLU
{
	static float f(float t) { return std::max(0.01f*t, t); }
	static float df(float f_t) { return (f_t > 0.0f) ? 1.0f : 0.01f; }
};

int2 make_packed_stride(const int2 & dims) { return {1, dims.x}; }
int3 make_packed_stride(const int3 & dims) { return {1, dims.x, dims.x*dims.y}; }
int4 make_packed_stride(const int4 & dims) { return {1, dims.x, dims.x*dims.y, dims.x*dims.y*dims.z}; }

template<class T, int K> struct tensorview // Note: Works for K in {2,3,4}
{
	using               intK = linalg::vec<int,K>;
	T *                 data;
	intK                dims, stride;

	                    tensorview(T * data, intK dims, intK stride)    : data(data), dims(dims), stride(stride) {}
	                    tensorview(T * data, intK dims)                 : tensorview(data, dims, make_packed_stride(dims)) {}
	template<class U>   tensorview(const tensorview<U,K> & view)        : tensorview(view.data, view.dims, view.stride) {} // Allows for T -> const T and other such conversions

	T &                 operator[] (intK i) const                       { return data[dot(stride,i)]; }
	tensorview<T,K-1>   operator[] (int i) const                        { return {data + stride[K-1]*i, (const linalg::vec<int,K-1> &)dims, (const linalg::vec<int,K-1> &)stride}; }

	tensorview          subview(intK woffset, intK wdims) const         { return {data + dot(stride,woffset), wdims, stride}; }
};

template<class T, int K> tensorview<      T,K> make_tensorview(      std::vector<T> & vec, linalg::vec<int,K> dims) { return {vec.data(), dims}; }
template<class T, int K> tensorview<const T,K> make_tensorview(const std::vector<T> & vec, linalg::vec<int,K> dims) { return {vec.data(), dims}; }

float dot(const tensorview<float,3> & a, const tensorview<      float,3> & b) {float s=0; for(auto i:vol_iteration(a.dims))s+=a[i]*b[i]; return s;}

template<class T> void madd(const tensorview<T,3> & d, const tensorview<const T,3> & a, T s) 
{ 
	if (a.stride.x != 1 || d.stride.x != 1)  // general case, no assumptions about stride in data layout 
	{
		for (auto i : vol_iteration(a.dims))
			d[i] += a[i] * s;
	}
	else  // a.stride.x == 1 && d.stride.x == 1   // the math is the same, we just write the code such that the compiler can better optimize this common case
	{
		for (int z = 0; z < a.dims.z; z++)  
		{
			auto az = a.data + z*a.stride.z;
			auto dz = d.data + z*d.stride.z;
			for (int y = 0; y < a.dims.y; y++)
			{
				auto aa = az + y*a.stride.y;
				auto dd = dz + y*d.stride.y;
				for (int x = 0; x < a.dims.x; x++)
				{
					*dd++ += *aa++ *s; //  d[{x, y, z}] += a[{x, y, z}] * s;
				}
			}
		}
	}
}
template<class T> void madd(const tensorview<T,3> & d, const tensorview<T,3> & a, T s) { return madd(d, tensorview<const T,3>(a), s); }

inline void loadvb(std::istream &s,      std::vector<float> &a) { s.read ((char*)a.data(), a.size()*sizeof(float)); }
inline void savevb(std::ostream &s,const std::vector<float> &a) { s.write((char*)a.data(), a.size()*sizeof(float)); }

struct CNN
{
	struct LBase
	{
		virtual std::vector<float> forward(const std::vector<float> & x) = 0;
		virtual std::vector<float> backward(const std::vector<float> & X, const std::vector<float> & Y, const std::vector<float> & E) = 0;
		virtual void update(const std::vector<float> & X, const std::vector<float> & Y, const std::vector<float> & E, float alpha) {}
		virtual void loada(std::istream & s)       {}
		virtual void savea(std::ostream & s) const {}
		virtual void loadb(std::istream & s)       {}
		virtual void saveb(std::ostream & s) const {}
		virtual void init(std::default_random_engine &rng) {};
	};
	struct LAvgPool final : public LBase  // 2x2
	{
		int3 indims;
		LAvgPool(int3 indims) : indims(indims) {}
		int3 outdims() const { return {indims.x/2, indims.y/2, indims.z}; }
		std::vector<float> forward(const std::vector<float> &input) override
		{
			std::vector<float> out(outdims().x*outdims().y*outdims().z);
			auto in = make_tensorview(input, indims); auto ot = make_tensorview(out, outdims());
			for(auto i: vol_iteration(outdims()))
				ot[i] = (in[{i.x*2+0,i.y*2+0,i.z}]+in[{i.x*2+1,i.y*2+0,i.z}]+in[{i.x*2+0,i.y*2+1,i.z}]+in[{i.x*2+1,i.y*2+1,i.z}])/4.0f; 
			return out;
		}
		std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E)  override  // assumes E is up to date
		{
			std::vector<float> D(indims.x*indims.y* indims.z);
			auto d = make_tensorview(D,indims);
			auto er = make_tensorview(E, outdims());
			for (auto i : vol_iteration(indims))
				d[i] = er[int3(i.x / 2, i.y / 2, i.z)] / 4.0f;
			return D;
		}
	};
	struct LMaxPool final : public LBase  // 2x2  
	{
		int3 indims;
		LMaxPool(int3 indims) : indims(indims) {}
		int3 outdims() const { return {indims.x/2, indims.y/2, indims.z}; }
		std::vector<float> forward(const std::vector<float> &input) override
		{
			std::vector<float> out(outdims().x*outdims().y*outdims().z);
			auto in = make_tensorview(input, indims); auto ot = make_tensorview(out, outdims());
			for (auto i : vol_iteration(outdims()))
				ot[i] = std::max(std::max(std::max(in[{i.x * 2 + 0, i.y * 2 + 0, i.z}], in[{i.x * 2 + 1, i.y * 2 + 0, i.z}]), in[{i.x * 2 + 0, i.y * 2 + 1, i.z}]), in[{i.x * 2 + 1, i.y * 2 + 1, i.z}]);
			return out;
		}
		std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E)  override  // assumes E is up to date
		{
			std::vector<float> D(indims.x*indims.y*indims.z);
			auto d  = make_tensorview(D, indims);
			auto in = make_tensorview(X, indims);
			auto er = make_tensorview(E, outdims());
			for (auto i : vol_iteration(outdims()))
			{
				int3 offset(i.x * 2, i.y * 2, i.z), mx = offset;
				for (auto v : vol_iteration({ 2,2,1 }))
					if (in[offset + v] > in[mx])
						mx = offset + v;
				d[mx] = er[i];
			}
			return D;
		}
	};
	struct LConv final : public LBase
	{
		int3 indims;
		int4 dims;
		int3 outdims;
		std::vector<float> W;
		std::vector<float> B;
		tensorview<float,4> weights() { return make_tensorview(W, dims); }

		LConv(int3 indims, int4 dims, int3 outdims) : indims(indims), dims(dims), outdims(outdims), W(dims.x*dims.y*dims.z*dims.w), B(dims.w, 0.0f) {}

		std::vector<float> forward(const std::vector<float> &input) override
		{
			std::vector<float> output(outdims.x*outdims.y*outdims.z);
			auto in = make_tensorview(input, indims);
			auto ot = make_tensorview(output, outdims);
			auto wt = weights();
			// following two lines work elegantly but a bit slow:
			//for (auto i : vol_iteration(outdims))
			//	ot[i] = dot(in.subview({ i.x,i.y,0 }, wt.dims.xyz()), wt[i.z]) + B[i.z];

			// following implementation with conv kernal loop on the outside so far beats tinycnn's perf
			// this removes any instruction stalls in the addition, and provides a larger range to the innermost loop.  eg 320 instead of 5 probably enables better throughput
			if (0)for (auto v : vol_iteration(outdims))
				ot[v] = B[v.z];
			for (int z = 0; z < outdims.z; z++)
				for (int y = 0; y < outdims.y; y++)
					for (int x = 0; x < outdims.x; x++)
						ot[{x, y, z}] = B[z];
			for (auto p : rect_iteration(dims.xy()))
			{
				for (int iz = 0; iz < indims.z; iz ++) for (int oz = 0; oz < outdims.z; oz++)
				{
					float w = wt[{p.x, p.y, iz, oz}];
					float *op = ot.data + oz*ot.stride.z;
					for (int y = 0; y < outdims.y; y++)// , ip += in.stride.y - outdims.x*in.stride.x)
					{
						const float *ip = in.data + dot(p, in.stride.xy()) + iz*in.stride.z + in.stride.y*y;
						if (simd_enable)
						{
							int x_ = 0;
							__m128 wwww = _mm_load_ps1(&w);

							//_mm_store_ps()
							for (; ((uintptr_t)(op)& 15) && x_ < outdims.x; x_++)
								*op++ += *ip++ *w;
							__m128 *y = (__m128*)(op);
							for (; x_ < outdims.x - 3; x_ += 4, ip += 4, y++,op+=4)
								*y = _mm_add_ps(*y, _mm_mul_ps(_mm_loadu_ps(ip), wwww));
							for (; x_ < outdims.x; x_++)
							{
								*op++ += *ip++ *w;
							}

						}
						else for (int x_ = 0; x_ < outdims.x; x_++)
						{
							*op++ += *ip++ *w;
						}
					}
				}
			}
			return output;
		}
		std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E) override  // assumes E is up to date
		{
			std::vector<float> D(indims.x*indims.y*indims.z);
			auto dt = make_tensorview(D, indims);
			auto in = make_tensorview(X, indims);
			auto er = make_tensorview(E, outdims);
			auto wt = weights();
			for(auto i : vol_iteration(outdims))
				madd(dt.subview({ i.x,i.y,0 }, wt.dims.xyz()), wt[i.z], er[i]);
			return D;
		}
		void update(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E, float alpha) override
		{
			auto in = make_tensorview(X, indims);
			auto er = make_tensorview(E, outdims);
			auto wt = weights();
			for (auto i : vol_iteration(outdims))
			{
				madd(wt[i.z], in.subview({ i.x,i.y,0 }, wt.dims.xyz()), -alpha*er[i]);  // W -= X * E * alpha;
				B[i.z] -= er[i] * alpha;
			}
		}
		virtual void init(std::default_random_engine &rng) override
		{
			float range = sqrtf(6.0f / (dims.x*dims.y*dims.z + dims.x*dims.y*dims.w));  // fan_in + fan_out
			for (auto &w : W)
				w = std::uniform_real<float>(-range, range)(rng);
		}
		void loada(std::istream &s)       override { for(auto & w : W) s >> w       ; for (auto & w : B) s >> w       ; }
		void savea(std::ostream &s) const override { for(auto & w : W) s << w << ' '; for (auto & w : B) s << w << ' '; }
		void loadb(std::istream &s)       override { loadvb(s, W); loadvb(s, B); }
		void saveb(std::ostream &s) const override { savevb(s, W); savevb(s, B); }
	};
	struct LFull final : public LBase
	{
		int M, N;
		std::vector<float> W;
		std::vector<float> B;
		LFull(int input_size, int output_size) : M(input_size), N(output_size), W(input_size  * output_size), B(output_size, 0.0f) {}

		std::vector<float> forward(const std::vector<float> &input) override
		{
			std::vector<float> Y = B;
			assert(Y.size() == N);

			const float *w = W.data();
			if (simd_enable) for (int i = 0; i < M; i++)
			{
				__m128 ri = _mm_load_ps1(&input[i]);
				int j = 0;
				//_mm_store_ps()
				for (; ((uintptr_t)(Y.data() + j) & 15) && j < (int)Y.size(); j++)
					Y[j] += input[i] * *w++;//W[j + i*Y.size()];
				__m128 *y = (__m128*)(Y.data() + j);
				for (; j < (int)Y.size() - 3; j += 4, w += 4, y++)
					*y = _mm_add_ps(*y, _mm_mul_ps(_mm_loadu_ps(w), ri));
				for (; j < (int)Y.size(); j++)
					Y[j] += input[i] * *w++;//W[j + i*Y.size()];
			}
			else for (int i = 0; i < M; i++)
				for (unsigned int j = 0; j < Y.size(); j++)
					Y[j] += input[i] * *w++;//W[j + i*Y.size()];

			return Y;
		}
		std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E)  override  // assumes E is up to date
		{
			std::vector<float> D(M, 0.0f);   // initialize to 0
			for (int i = 0; i < M; i++)
				for (unsigned int j = 0; j < Y.size(); j++)  // still need to A-B test the ordering of these two for loops,  instruction throughput vs locality of reference in this case
					D[i] += W[j + i*Y.size()] * E[j];
			return D;
		}
		void update(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E, float alpha) override
		{
			for (unsigned int j = 0; j < Y.size(); j++)
				B[j] -= E[j] * alpha;
			for (int i = 0; i < M; i++)
				for(unsigned int j = 0; j < Y.size(); j++)
					W[i*Y.size() + j] -= X[i] * E[j] * alpha;
		}
		virtual void init(std::default_random_engine &rng) override
		{
			float range = sqrtf(6.0f / (M + N)); // (input_size + output_size)); // xavier vs lecunn // = 1.0f / sqrtf((float)input_size);
			for (auto &w : W)
				w = std::uniform_real<float>(-range, range)(rng);
		}
		void loada(std::istream &s)       override { for(auto & w : W) s >> w       ; for (auto & w : B) s >> w       ; }
		void savea(std::ostream &s) const override { for(auto & w : W) s << w << ' '; for (auto & w : B) s << w << ' '; }
		void loadb(std::istream &s)       override { loadvb(s, W); loadvb(s, B); }
		void saveb(std::ostream &s) const override { savevb(s, W); savevb(s, B); }
	};
	template<class F> struct LActivation final : public LBase // F is the activation function
	{
		LActivation(int n) {}
		std::vector<float> forward(const std::vector<float> &input) override
		{
			return Transform(input,F::f);
		}
		std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E) override
		{
			std::vector<float> D(Y.size());
			std::transform(E.begin(), E.end(), Y.begin(), D.begin(), [](float g, float y) { return F::df(y) * g; });    // for example sigmoid would be d = e * y*(1-y)
			return D;
		}
	};
	struct LSoftMax final : public LBase
	{
		LSoftMax(int n) {}
		std::vector<float> forward(const std::vector<float> &input) override
		{
			float sum = 0.0f;
			auto out = Transform(input, [&sum](float x) {float y = std::expf(x); sum += y; return y; });
			for (auto &y : out)
				y /= sum;
			return out;
		}
		std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E) override
		{
			float dp = 0;  // sum or dot product of error with output  (note that changing one input upward pushes everybody else down) 
			for (unsigned int i = 0; i < Y.size(); i++)
				dp += E[i] * Y[i];
			std::vector<float> D(Y.size());
			std::transform(E.begin(), E.end(), Y.begin(), D.begin(), [dp](float e, float y) { return y*(e-dp); });    // yup, after cancelling and substituting the calculus derivatives you end up with just this
			return D;
		}

	};
	struct LCrossEntropy final : public LBase
	{
		LCrossEntropy(int n) {}
		std::vector<float> forward(const std::vector<float> &input) override
		{
			float sum = 0.0f;
			const auto max_value = *std::max_element(input.begin(), input.end());
			auto out = Transform(input, [&sum, &max_value](float x) {float y = std::expf(x - max_value); sum += y; return y; });
			for (auto &y : out)
				y /= sum;
			return out;
		}
		std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E) override
		{
			std::vector<float> D = E;
			return D;
		}

	};
	std::vector<LBase*> layers;

	std::vector<float> Eval(const std::vector<float> &x)
	{
		std::vector<std::vector<float>> outputs;
		for (auto &layer : layers)
			outputs.push_back(layer->forward(outputs.size() ? outputs.back() : x));
		return std::move(outputs.back());
	}

	float Train(const std::vector<float> &x, const std::vector<float> &t, float alpha = 0.01f)
	{
		std::vector<std::vector<float>> outputs;
		for (auto &layer : layers)
			outputs.push_back(layer->forward(outputs.size() ? outputs.back() : x));

		std::vector<std::vector<float>> errors(layers.size());

		float mse = 0;                                             // mean square error 
		errors.back().resize(outputs.back().size());
		std::transform(outputs.back().begin(), outputs.back().end(), t.begin(), errors.back().begin(), [&mse](float y, float t)->float { float e = y - t; mse += e*e; return e; });
		mse /= errors.back().size();

		for (unsigned int i = layers.size() - 1; i > 0; i--)
			errors[i-1] = layers[i]->backward(outputs[i-1], outputs[i], errors[i]);

		for (int i = 0; i < (int)layers.size(); i++)
			layers[i]->update(i ? outputs[i-1] : x, outputs[i], errors[i], alpha);

		static volatile bool trace_here = 0;
		//		if (trace_here) printf("%f %f\n", t[0],y[0],e[0]);
		return   mse;
	}
	void Init()
	{
		std::default_random_engine rng;
		for (auto &layer : layers)
			layer->init(rng);
	}

	void loada(std::istream &s   )       { for (auto layer:layers) layer->loada(s); } 
	void savea(std::ostream &s   ) const { for (auto layer:layers) layer->savea(s); } 
	void loadb(std::istream &s   )       { for (auto layer:layers) layer->loadb(s); } 
	void saveb(std::ostream &s   ) const { for (auto layer:layers) layer->saveb(s); } 
	void loadb(std::string  fname)       { loadb(std::ifstream(fname,std::istream::binary)); } 
	void saveb(std::string  fname) const { saveb(std::ofstream(fname,std::ostream::binary)); } 

	CNN(const std::vector<int> &s)  // quick test for simple NNs
	{
		std::default_random_engine rng;
		for (unsigned int i=1; i<s.size(); i++)
		{
			layers.push_back(new LFull(s[i-1], s[i]));
			layers.push_back(new LActivation<TanH>(s[i]));
		}
		Init();
	}
};
inline std::istream &operator >>(std::istream &in,       CNN::LConv &cl) { cl.loada(in); return in; }
inline std::ostream &operator <<(std::ostream &ot, const CNN::LConv &cl) { cl.savea(ot); return ot; }
inline std::istream &operator >>(std::istream &in,       CNN::LFull &cl) { cl.loada(in); return in; }
inline std::ostream &operator <<(std::ostream &ot, const CNN::LFull &cl) { cl.savea(ot); return ot; }
inline std::istream &operator >>(std::istream &in,       CNN &nn       ) { nn.loada(in); return in; }
inline std::ostream &operator <<(std::ostream &ot, const CNN &nn       ) { nn.savea(ot); return ot; }

#endif     // MINI_CNN_H

