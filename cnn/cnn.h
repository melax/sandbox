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

#include <vecmatquat.h>
#include <geometric.h>  // for the multi dimensional iterators

struct rect_iteration
{
	struct iterator
	{
		int2 coord; int dims_x;
		int2 operator * () const { return coord; }
		bool operator != (const iterator & r) const { return coord.y != r.coord.y; }
		void operator ++ () { coord.y += !(++coord.x %= dims_x); }
	};

	int2 dims;
	rect_iteration(const int2 & dims) : dims(dims) {}
	iterator begin() const { return{ { 0,0 }, dims.x }; }
	iterator end() const { return{ { 0,dims.y }, dims.x }; }
};
struct vol_iteration
{
	struct iterator
	{
		int3 coord; int2 dims;
		int3 operator * () const { return coord; }
		bool operator != (const iterator & r) const { return coord.z != r.coord.z; }
		void operator ++ () { coord.z += ((coord.y += !(++coord.x %= dims.x)) == dims.y); coord.y %= dims.y; }
	};

	int3 dims;
	vol_iteration(const int3 & dims) : dims(dims) {}
	iterator begin() const { return{ { 0,0,0 }, dims.xy() }; }
	iterator end() const { return{ { 0,0,dims.z }, dims.xy() }; }
};

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

struct tensorview1
{
	float* data;
	int    dim;
	int    stride;
	float &operator[](int i) { return data[i*stride]; }
};
struct tensorview2
{
	float* data;
	int2   dims;
	int2   stride;
	tensorview1 operator[](int  i) { return{ data + stride.y*i,dims.x,stride.x }; }
	float &     operator[](int2 i) { return  (*this)[i.y][i.x]; }
};
struct tensorview3
{
	float* data;
	int3 dims;
	int3 stride;
	tensorview2 operator[](int  i) { return{ data + stride.z*i, dims.xy(), stride.xy() }; }
	tensorview1 operator[](int2 i) { return  (*this)[i.y][i.x]; }
	float &     operator[](int3 i) { return  (*this)[i.z][i.y][i.x]; }      // or just data[dot(i,stride)];
	tensorview3 subview(int3 woffset, int3 wdims) { return {data+dot(stride,woffset),wdims,stride}; }
	tensorview3(float *data, int3 dims, int3 stride) :data(data), dims(dims), stride(stride) {}
	tensorview3(float *data, int3 dims) :data(data), dims(dims), stride(1, dims.x, dims.x*dims.y) {}
	tensorview3(const std::vector<float> &vec, int3 dims) :data((float*)vec.data()), dims(dims), stride(1, dims.x, dims.x*dims.y) { assert(dims.x*dims.y*dims.z == vec.size()); }
};
float dot(tensorview3 &a, tensorview3 &b) {float s=0; for(auto i:vol_iteration(a.dims))s+=a[i]*b[i]; return s;}

void madd(tensorview3 &d, tensorview3 &a,float s) { 
	//for (auto i:vol_iteration(a.dims)) d[i] += a[i]*s; 
	assert(a.stride.x == 1);
	assert(d.stride.x == 1);
	for (int z = 0; z < a.dims.z; z++)
	{
		float* az = a.data + z*a.stride.z;
		float* dz = d.data + z*d.stride.z;
		for (int y = 0; y < a.dims.y; y++)
		{
			float* aa = az + y*a.stride.y;
			float* dd = dz + y*d.stride.y;
			for (int x = 0; x < a.dims.x; x++)
			{
				*dd++ += *aa++ *s; //  d[{x, y, z}] += a[{x, y, z}] * s;
			}
		}
	}
}

struct tensorview4
{
	float* data;
	int4 dims;
	int4 stride;
	tensorview3 operator[](int  i) { return{ data + stride.w*i, dims.xyz(), stride.xyz() }; }
	float &     operator[](int4 i) { return  data[dot(i,stride)]; }
};

static std::default_random_engine random_number_generator;  // fixme: sorry about this global here.


inline std::istream &loadvb(std::istream &s,      std::vector<float> &a) { s.read ((char*)a.data(),a.size()*sizeof(float)); return s; }
inline std::ostream &savevb(std::ostream &s,const std::vector<float> &a) { s.write((char*)a.data(),a.size()*sizeof(float)); return s; }

struct CNN
{
	struct LBase // base class implemented as identity layer
	{
		int M, N;
		LBase() {};
		LBase(int n) :M(n), N(n) {}
		LBase(int input_size, int output_size) :M(input_size),N(output_size) {}
		virtual std::vector<float> forward(const std::vector<float> &x) { return x; }
		virtual std::vector<float> backward(const std::vector<float> &X,const std::vector<float> &Y,const std::vector<float> &E) = 0;
		virtual void  update(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E,float alpha) {}
		virtual std::istream &loada(std::istream &s)       { return s;}
		virtual std::ostream &savea(std::ostream &s) const { return s;}
		virtual std::istream &loadb(std::istream &s)       { return s;}
		virtual std::ostream &saveb(std::ostream &s) const { return s;}
	};
	struct LAvgPool : public LBase  // 2x2  
	{
		int3  indims;
		int3  outdims()      { return{ indims.x / 2,indims.y / 2,indims.z }; }
		virtual std::vector<float> forward(const std::vector<float> &input) override
		{
			assert(N == outdims().x*outdims().y*outdims().z);
			std::vector<float> out(N);
			auto in = tensorview3(input, indims); auto ot = tensorview3(out, outdims());
			for(auto i: vol_iteration(outdims()))
				ot[i] = (in[{i.x*2+0,i.y*2+0,i.z}]+in[{i.x*2+1,i.y*2+0,i.z}]+in[{i.x*2+0,i.y*2+1,i.z}]+in[{i.x*2+1,i.y*2+1,i.z}])/4.0f; 
			return out;
		}
		virtual std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E)  override  // assumes E is up to date
		{
			assert(M == indims.x*indims.y* indims.z);
			std::vector<float> D(M);
			auto d = tensorview3(D.data(),indims);
			auto er = tensorview3(E, outdims());
			for (auto i : vol_iteration(indims))
				d[i] = er[int3(i.x / 2, i.y / 2, i.z)] / 4.0f;
			return D;
		}
		void update(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E,float alpha) override {}
		LAvgPool(int3 indims) :LBase(indims.x*indims.y*indims.z, indims.x*indims.y*indims.z/4),indims(indims) {}
	};
	struct LMaxPool : public LBase  // 2x2  
	{
		int3  indims;
		int3  outdims() { return{ indims.x / 2,indims.y / 2,indims.z }; }
		virtual std::vector<float> forward(const std::vector<float> &input) override
		{
			assert(N == outdims().x*outdims().y*outdims().z);
			std::vector<float> out(N);
			auto in = tensorview3(input, indims); auto ot = tensorview3(out, outdims());
			for (auto i : vol_iteration(outdims()))
				ot[i] = std::max(std::max(std::max(in[{i.x * 2 + 0, i.y * 2 + 0, i.z}], in[{i.x * 2 + 1, i.y * 2 + 0, i.z}]), in[{i.x * 2 + 0, i.y * 2 + 1, i.z}]), in[{i.x * 2 + 1, i.y * 2 + 1, i.z}]);
			return out;
		}
		virtual std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E)  override  // assumes E is up to date
		{
			assert(M == indims.x*indims.y* indims.z);
			std::vector<float> D(M,0.0f);   // initialize to 0
			auto d  = tensorview3(D, indims);
			auto in = tensorview3(X, indims);
			auto er = tensorview3(E, outdims());
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
		void update(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E,float alpha) override {}
		LMaxPool(int3 indims) :LBase(indims.x*indims.y*indims.z, indims.x*indims.y*indims.z / 4), indims(indims) {}
	};
	struct LConv : public LBase
	{
		int3 indims;
		int4 dims;
		int3 outdims;
		std::vector<float> W;
		std::vector<float> B;
		tensorview4 weights(){ return{ W.data() ,dims   ,{1,dims.x,dims.y*dims.x,dims.z*dims.y*dims.x } }; }
		LConv(int3 indims, int4 dims, int3 outdims) :LBase(indims.x*indims.y*indims.z, outdims.x*outdims.y*outdims.z), indims(indims), dims(dims), outdims(outdims), W(dims.x*dims.y*dims.z*dims.w), B(dims.w,0.0f) 
		{
			float range = sqrtf(6.0f / (dims.x*dims.y*dims.z + dims.x*dims.y*dims.w));  // fan_in + fan_out
			for (auto &w : W)
				w = std::uniform_real<float>(-range, range)(random_number_generator);
		}
		virtual std::vector<float> forward(const std::vector<float> &input) override
		{
			assert(N == outdims.x*outdims.y*outdims.z);
			std::vector<float> output(N);
			auto in = tensorview3(input, indims); 
			auto ot = tensorview3(output, outdims);
			auto wt = weights();
			// following two lines work elegantly but a bit slow:
			//for (auto i : vol_iteration(outdims))
			//	ot[i] = dot(in.subview({ i.x,i.y,0 }, wt.dims.xyz()), wt[i.z]) + B[i.z];

			// following implementation with conv kernal loop on the outside so far beats tinycnn's perf
			// this removes any instruction stalls in the addition, and provides a larger range to the innermost loop.  eg 320 instead of 5 probably enables better throughput
			if(0)for (auto v : vol_iteration(outdims))
				ot[v] = B[v.z];
			for (int z = 0; z < outdims.z; z++)
				for (int y = 0; y < outdims.y; y++)
					for (int x = 0; x < outdims.x; x++)
						ot[{x, y, z}] = B[z];
			for (auto p : rect_iteration(dims.xy()))
			{
				for (int iz = 0; iz < indims.z; iz ++)
				for (int oz = 0; oz < outdims.z; oz++)
				{
					float w = wt[{p.x, p.y, iz, oz}];
					float *op = ot.data + oz*ot.stride.z;
					for (int y = 0; y < outdims.y; y++)// , ip += in.stride.y - outdims.x*in.stride.x)
					{
						float *ip = in.data + dot(p, in.stride.xy()) + iz*in.stride.z  + in.stride.y*y;
						for (int x_ = 0; x_ < outdims.x; x_++)
						{
							*op++ += *ip++ *w;
						}
					}
				}
			}
			return output;
		}
		virtual std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E)  override  // assumes E is up to date
		{
			assert(M == indims.x*indims.y* indims.z);
			std::vector<float> D(M, 0.0f);   // initialize to 0
			auto dt = tensorview3(D, indims);
			auto in = tensorview3(X, indims);
			auto er = tensorview3(E, outdims);
			auto wt = weights();
			for(auto i : vol_iteration(outdims))
				madd(dt.subview({ i.x,i.y,0 }, wt.dims.xyz()), wt[i.z], er[i]);
			return D;
		}
		void update(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E,float alpha) override
		{
			auto in = tensorview3(X, indims);
			auto er = tensorview3(E, outdims);
			auto wt = weights();
			for (auto i : vol_iteration(outdims))
			{
				madd(wt[i.z], in.subview({ i.x,i.y,0 }, wt.dims.xyz()), -alpha*er[i]);  // W -= X * E * alpha;
				B[i.z] -= er[i] * alpha;
			}
		}
		virtual std::istream &loada(std::istream &s)       override {for (auto &w : W) s >> w     ; for (auto &w : B) s >> w       ; return s; }
		virtual std::ostream &savea(std::ostream &s) const override {for (auto &w : W) s << w<<" "; for (auto &w : B) s << w <<" " ; return s; }
		virtual std::istream &loadb(std::istream &s)       override { loadvb(s, W); loadvb(s, B); return s; }
		virtual std::ostream &saveb(std::ostream &s) const override { savevb(s, W); savevb(s, B); return s; }
	};
	struct LFull : public LBase
	{
		std::vector<float> W;
		std::vector<float> B;
		LFull(int input_size, int output_size) :LBase(input_size, output_size), W(input_size  * output_size), B(output_size,0.0f)
		{
			float range = sqrtf(6.0f / (input_size + output_size)); // xavier vs lecunn // = 1.0f / sqrtf((float)input_size);
			for (auto &w : W)
				w = std::uniform_real<float>(-range, range)(random_number_generator);
			//for (auto &b : B)
			//	b = 0;// std::uniform_real<float>(-1.0f*range, 1.0f*range)(random_number_generator);
		}
		virtual std::vector<float> forward(const std::vector<float> &input) override
		{
			std::vector<float> Y = B;
			assert(Y.size() == N);

			const float *w = W.data();
			for (int i = 0; i < M; i++)
				for (unsigned int j = 0; j < Y.size(); j++)
					Y[j] += input[i] * *w++;//W[j + i*Y.size()];

			return Y;
		}
		virtual std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E)  override  // assumes E is up to date
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
				for (unsigned int j = 0; j < Y.size(); j++)
					W[i*Y.size() + j] -= X[i] * E[j] * alpha;
		}
		virtual std::istream &loada(std::istream &s)       override {for (auto &w : W) s >> w     ; for (auto &w : B) s >> w       ; return s; }
		virtual std::ostream &savea(std::ostream &s) const override {for (auto &w : W) s << w<<" "; for (auto &w : B) s << w <<" " ; return s; }
		virtual std::istream &loadb(std::istream &s)       override { loadvb(s, W); loadvb(s, B); return s; }
		virtual std::ostream &saveb(std::ostream &s) const override { savevb(s, W); savevb(s, B); return s; }
	};
	template<class F>
	struct LActivation :public LBase // F is the activation function
	{
		LActivation(int n) :LBase(n) {}
		virtual std::vector<float> forward(const std::vector<float> &input) override
		{
			return Transform(input,F::f);
		}
		virtual std::vector<float> backward(const std::vector<float> &X, const std::vector<float> &Y, const std::vector<float> &E) override
		{
			std::vector<float> D(M);
			std::transform(E.begin(), E.end(), Y.begin(), D.begin(), [](float g, float y) { return F::df(y) * g; });    // for example sigmoid would be d = e * y*(1-y)
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
		errors.back().resize(layers.back()->N);
		std::transform(outputs.back().begin(), outputs.back().end(), t.begin(), errors.back().begin(), [&mse](float y, float t)->float { float e = y - t; mse += e*e; return e; });
		mse /= errors.back().size();

		for (unsigned int i = layers.size() - 1; i > 0; i--)
			errors[i-1]=layers[i]->backward(outputs[i-1],outputs[i],errors[i]);

		for (int i = 0; i < (int)layers.size();i++)
			layers[i]->update(i?outputs[i - 1]:x, outputs[i], errors[i],alpha);

		static volatile bool trace_here = 0;
		//		if (trace_here) printf("%f %f\n", t[0],y[0],e[0]);
		return   mse;
	}

	std::istream &loada(std::istream &s   )       { for (auto layer:layers) layer->loada(s); return s; } 
	std::ostream &savea(std::ostream &s   ) const { for (auto layer:layers) layer->savea(s); return s; } 
	std::istream &loadb(std::istream &s   )       { for (auto layer:layers) layer->loadb(s); return s; } 
	std::ostream &saveb(std::ostream &s   ) const { for (auto layer:layers) layer->saveb(s); return s; } 
	void          loadb(std::string  fname)       { loadb(std::ifstream(fname,std::istream::binary));  } 
	void          saveb(std::string  fname) const { saveb(std::ofstream(fname,std::ostream::binary));  } 

	CNN(const std::vector<int> &s)  // quick test for simple NNs
	{
		for (unsigned int i = 1; i < s.size(); i++)
		{
			layers.push_back(new LFull(s[i - 1], s[i]));
			layers.push_back(new LActivation<TanH>(s[i]));
		}
	}
};
inline std::istream &operator >>(std::istream &in,       CNN::LConv &cl) { return cl.loada(in); }
inline std::ostream &operator <<(std::ostream &ot, const CNN::LConv &cl) { return cl.savea(ot); }
inline std::istream &operator >>(std::istream &in,       CNN::LFull &cl) { return cl.loada(in); }
inline std::ostream &operator <<(std::ostream &ot, const CNN::LFull &cl) { return cl.savea(ot); }
inline std::istream &operator >>(std::istream &in,       CNN &nn       ) { return nn.loada(in); }
inline std::ostream &operator <<(std::ostream &ot, const CNN &nn       ) { return nn.savea(ot); }

#endif     // MINI_CNN_H

