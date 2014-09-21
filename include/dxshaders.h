//
// inlined hlsl shaders.  see dxwin.h
//

const char *dxshaders = R"EFFECTFILE(

float4 qconj(float4 q)
{
	return float4(-q.x,-q.y,-q.z,q.w);
}

float4 qmul(float4 a, float4 b)
{
	float4 c;
	c.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z; 
	c.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y; 
	c.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x; 
	c.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w; 
	return c;
}


float3 qrot(  float4 q,  float3 v )
{
	return qmul(qmul(q,float4(v,0)), float4(-q.x,-q.y,-q.z,q.w)).xyz; 
}

struct Vertex 
{
    float3 position    : POSITION;
    float4 orientation : TEXCOORD1;
    float2 texcoord	   : TEXCOORD0;
};

struct Fragment
{
    float4 screenpos   : SV_POSITION;
    float2 texcoord    : TEXCOORD0;
	float3 position    : TEXCOORD6;
//	float4 orientation : TEXCOORD7; 
	float3 tangent     : TEXCOORD3; 
	float3 binormal    : TEXCOORD4; 
	float3 normal      : TEXCOORD5; 
};

cbuffer ConstantBuffer : register( b0 )
{
	float4 hack;
	matrix Projection;
	float3 camerap; float unusedc;  // extra float for 128bit padding
	float4 cameraq;
	float3 meshp;   float unusedm;
	float4 meshq;
}

Texture2D    txDiffuse : register( t0 );
SamplerState samLinear : register( s0 );


//--------------------------------------------------------------------------------------
// Vertex Shader
//--------------------------------------------------------------------------------------
//Fragment VS(Vertex v)
Fragment VS( Vertex v ) //: SV_POSITION
{
	Fragment foo = (Fragment)0;
	//out.position = meshp + qrot(meshq,v.position);
	//out.screenpos = mul(float4( mul(qrot(qconj(cameraq,out.position))-camerap,1),Projection);
	//	out.screenpos = mul(float4(v.position,1),Projection);
	//	return out.screenpos;
	float3 pw = meshp.xyz + qrot(meshq,v.position.xyz) ;
	foo.position = pw; // world space position
	float4 pc = float4(qrot(qconj(cameraq),pw-camerap) ,1);
	foo.screenpos =  mul(pc,Projection);
	foo.texcoord = v.texcoord;
	float4 sq = qmul(meshq,v.orientation);
	foo.normal   = qrot(sq,float3(0,0,1));
	foo.tangent  = qrot(sq,float3(1,0,0));
	foo.binormal = qrot(sq,float3(0,1,0));
	return foo;
	//   return mul(p,Projection);
}

//--------------------------------------------------------------------------------------
// Pixel Shader
//--------------------------------------------------------------------------------------
float4 PS( float4 Pos : SV_POSITION , float2 texcoord : TEXCOORD0 ) : SV_Target
{
    return hack * txDiffuse.Sample( samLinear,texcoord) ; // float4( 1.0f, 1.0f, 0.0f, 1.0f );    // Yellow, with Alpha = 1
}

)EFFECTFILE";
