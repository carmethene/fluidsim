#include "FluidSim.h"
#include "Profiler.h"
#include <algorithm>

//------------------------------------------------------------------------------
const static uint  SOLVER_ITERATIONS		= 10;

//------------------------------------------------------------------------------
#define SWAP(x0,x) {float* tmp = x0; x0 = x; x = tmp;}

//------------------------------------------------------------------------------
FluidSim::FluidSim( uint size_x, uint size_y, float viscosity, float diffusion, float decay )
	:	mSizeX( size_x )
	,	mSizeY( size_y )
	,	mNumPoints( size_x * size_y )
	,	mViscosity( viscosity )
	,	mDiffusion( diffusion )
	,	mDecay( decay )
	,	mGravityU( 0.0f )
	,	mGravityV( 0.0f )
{
	mDensitiesR		= new float[ mNumPoints ];
	mDensitiesG		= new float[ mNumPoints ];
	mDensitiesB		= new float[ mNumPoints ];
	mDensitiesR0	= new float[ mNumPoints ];
	mDensitiesG0	= new float[ mNumPoints ];
	mDensitiesB0	= new float[ mNumPoints ];
	mVelocitiesU	= new float[ mNumPoints ];
	mVelocitiesV	= new float[ mNumPoints ];
	mVelocitiesU0	= new float[ mNumPoints ];
	mVelocitiesV0	= new float[ mNumPoints ];
	mSourcesR		= new float[ mNumPoints ];
	mSourcesG		= new float[ mNumPoints ];
	mSourcesB		= new float[ mNumPoints ];

	memset( mDensitiesR, 0, mNumPoints * sizeof(float) );
	memset( mDensitiesG, 0, mNumPoints * sizeof(float) );
	memset( mDensitiesB, 0, mNumPoints * sizeof(float) );
	memset( mDensitiesR0, 0, mNumPoints * sizeof(float) );
	memset( mDensitiesG0, 0, mNumPoints * sizeof(float) );
	memset( mDensitiesB0, 0, mNumPoints * sizeof(float) );
	memset( mVelocitiesU, 0, mNumPoints * sizeof(float) );
	memset( mVelocitiesV, 0, mNumPoints * sizeof(float) );
	memset( mVelocitiesU0, 0, mNumPoints * sizeof(float) );
	memset( mVelocitiesV0, 0, mNumPoints * sizeof(float) );
	memset( mSourcesR, 0, mNumPoints * sizeof(float) );
	memset( mSourcesG, 0, mNumPoints * sizeof(float) );
	memset( mSourcesB, 0, mNumPoints * sizeof(float) );
}

//------------------------------------------------------------------------------
FluidSim::~FluidSim()
{
	delete [] mDensitiesR;		mDensitiesR = NULL;
	delete [] mDensitiesG;		mDensitiesG = NULL;
	delete [] mDensitiesB;		mDensitiesB = NULL;
	delete [] mDensitiesR0;		mDensitiesR0 = NULL;
	delete [] mDensitiesG0;		mDensitiesG0 = NULL;
	delete [] mDensitiesB0;		mDensitiesB0 = NULL;
	delete [] mVelocitiesU;		mVelocitiesU = NULL;
	delete [] mVelocitiesV;		mVelocitiesV = NULL;
	delete [] mVelocitiesU0;	mVelocitiesU0 = NULL;
	delete [] mVelocitiesV0;	mVelocitiesV0 = NULL;
	delete [] mSourcesR;		mSourcesR = NULL;
	delete [] mSourcesG;		mSourcesG = NULL;
	delete [] mSourcesB;		mSourcesB = NULL;
}

//------------------------------------------------------------------------------
void FluidSim::Update( float dt )
{
	DensityStep( mSourcesR, mDensitiesR, mDensitiesR0, mVelocitiesU, mVelocitiesV, mDiffusion, dt );
	DensityStep( mSourcesG, mDensitiesG, mDensitiesG0, mVelocitiesU, mVelocitiesV, mDiffusion, dt );
	DensityStep( mSourcesB, mDensitiesB, mDensitiesB0, mVelocitiesU, mVelocitiesV, mDiffusion, dt );
	VelocityStep( mVelocitiesU, mVelocitiesV, mVelocitiesU0, mVelocitiesV0, mViscosity, dt );
	Decay( mDensitiesR, mDecay, dt );
	Decay( mDensitiesG, mDecay, dt );
	Decay( mDensitiesB, mDecay, dt );
}

//------------------------------------------------------------------------------
void FluidSim::PlaceSource( uint x, uint y, float r, float g, float b )
{
	if( x == 0 || x >= (mSizeX-1) ||
		y == 0 || y >= (mSizeY-1) )
	{
		//We don't allow manipulation of the edge regions
		return;
	}

	const uint index = IDX( x, y );

	//Create a source
	mSourcesR[ index ]			= r;
	mSourcesR[ index-1 ]		= r;
	mSourcesR[ index+1 ]		= r;
	mSourcesR[ index-mSizeX ]	= r;
	mSourcesR[ index+mSizeX ]	= r;
	mSourcesG[ index ]			= g;
	mSourcesG[ index-1 ]		= g;
	mSourcesG[ index+1 ]		= g;
	mSourcesG[ index-mSizeX ]	= g;
	mSourcesG[ index+mSizeX ]	= g;
	mSourcesB[ index ]			= b;
	mSourcesB[ index-1 ]		= b;
	mSourcesB[ index+1 ]		= b;
	mSourcesB[ index-mSizeX ]	= b;
	mSourcesB[ index+mSizeX ]	= b;
}

//------------------------------------------------------------------------------
void FluidSim::EraseSource( uint x, uint y )
{
	if( x == 0 || x >= (mSizeX-1) ||
		y == 0 || y >= (mSizeY-1) )
	{
		//We don't allow manipulation of the edge regions
		return;
	}

	const uint index = IDX( x, y );

	//Erase nearby sources
	mSourcesR[ index ]			= 0.0f;
	mSourcesR[ index-1 ]		= 0.0f;
	mSourcesR[ index+1 ]		= 0.0f;
	mSourcesR[ index-mSizeX ]	= 0.0f;
	mSourcesR[ index+mSizeX ]	= 0.0f;
	mSourcesG[ index ]			= 0.0f;
	mSourcesG[ index-1 ]		= 0.0f;
	mSourcesG[ index+1 ]		= 0.0f;
	mSourcesG[ index-mSizeX ]	= 0.0f;
	mSourcesG[ index+mSizeX ]	= 0.0f;
	mSourcesB[ index ]			= 0.0f;
	mSourcesB[ index-1 ]		= 0.0f;
	mSourcesB[ index+1 ]		= 0.0f;
	mSourcesB[ index-mSizeX ]	= 0.0f;
	mSourcesB[ index+mSizeX ]	= 0.0f;
}

//------------------------------------------------------------------------------
void FluidSim::ClearSources()
{
	for( uint i = 0; i < mNumPoints; ++i )
	{
		mSourcesR[ i ] = 0.0f;
		mSourcesG[ i ] = 0.0f;
		mSourcesB[ i ] = 0.0f;
	}
}

//------------------------------------------------------------------------------
void FluidSim::ClearDensity()
{
	for( uint i = 0; i < mNumPoints; ++i )
	{
		mDensitiesR[ i ] = 0.0f;
		mDensitiesG[ i ] = 0.0f;
		mDensitiesB[ i ] = 0.0f;
		mDensitiesR0[ i ] = 0.0f;
		mDensitiesG0[ i ] = 0.0f;
		mDensitiesB0[ i ] = 0.0f;
	}
}

//------------------------------------------------------------------------------
void FluidSim::ApplyForce( uint x, uint y, float amount )
{
	if( x == 0 || x >= (mSizeX-1) ||
		y == 0 || y >= (mSizeY-1) )
	{
		//We don't allow manipulation of the edge regions
		return;
	}

	//Create a splash velocity
	mVelocitiesU[IDX(x-1,y-1)]	-= amount;
	mVelocitiesU[IDX(x-1,y  )]	-= amount;
	mVelocitiesU[IDX(x-1,y+1)]	-= amount;
	mVelocitiesU[IDX(x+1,y-1)]	+= amount;
	mVelocitiesU[IDX(x+1,y  )]	+= amount;
	mVelocitiesU[IDX(x+1,y+1)]	+= amount;
	mVelocitiesV[IDX(x-1,y-1)]	-= amount;
	mVelocitiesV[IDX(x  ,y-1)]	-= amount;
	mVelocitiesV[IDX(x+1,y-1)]	-= amount;
	mVelocitiesV[IDX(x-1,y+1)]	+= amount;
	mVelocitiesV[IDX(x  ,y+1)]	+= amount;
	mVelocitiesV[IDX(x+1,y+1)]	+= amount;
}

//------------------------------------------------------------------------------
void FluidSim::SetGravity( float gu, float gv )
{
	mGravityU = gu;
	mGravityV = -gv;
}

//------------------------------------------------------------------------------
void FluidSim::Draw( PixelToaster::vector<PixelToaster::Pixel>& out_pixels, bool clamp_colours, bool show_sources, bool show_velocity ) const
{
	if( out_pixels.size() != mNumPoints )
	{
		out_pixels.resize( mNumPoints );
	}

	for( uint i = 0; i < mNumPoints; ++i )
	{
		float cr = mDensitiesR[ i ];
		float cg = mDensitiesG[ i ];
		float cb = mDensitiesB[ i ];

		if( clamp_colours )
		{
			const float cmax = std::max( cr, std::max( cg, cb ) );

			if( cmax > 1.0f )
			{
				cr /= cmax;
				cb /= cmax;
				cg /= cmax;
			}
		}

		out_pixels[ i ].r = cr;
		out_pixels[ i ].g = cg;
		out_pixels[ i ].b = cb;

		if( show_velocity )
		{
			const float v = abs( mVelocitiesU[ i ] ) + abs( mVelocitiesV[ i ] ) / 2.0f;
			out_pixels[ i ].r = v;
			out_pixels[ i ].g = v;
			out_pixels[ i ].b = v;
		}

		if( show_sources )
		{
			float r = mSourcesR[ i ];
			float g = mSourcesG[ i ];
			float b = mSourcesB[ i ];

			const float max = std::max( r, std::max( g, b ) );
			
			if( max > 0.0f )
			{
				//Scale back to 0..1
				r /= max;
				g /= max;
				b /= max;

				out_pixels[ i ].r = r;
				out_pixels[ i ].g = g;
				out_pixels[ i ].b = b;
			}
		}
	}
}

//------------------------------------------------------------------------------
void FluidSim::DensityStep( float* s, float* x, float* x0, float* u, float* v, float diff, float dt )
{
	AddSources( x, s, dt );
	SWAP( x0, x ); Diffuse( 0, x, x0, diff, dt );
	SWAP( x0, x ); Advect( 0, x, x0, u, v, dt );
}

//------------------------------------------------------------------------------
void FluidSim::VelocityStep( float* u, float* v, float* u0, float* v0, float visc, float dt )
{
	AddSources( u, u0, dt );
	AddSources( v, v0, dt );
	ApplyGravity( dt );
	SWAP( u0, u ); Diffuse( 1, u, u0, visc, dt );
	SWAP( v0, v ); Diffuse( 2, v, v0, visc, dt );
	Project( u, v, u0, v0 );
	SWAP( u0, u ); SWAP( v0, v );
	Advect( 1, u, u0, u0, v0, dt ); Advect( 2, v, v0, u0, v0, dt );
	Project( u, v, u0, v0 );
}

//------------------------------------------------------------------------------
void FluidSim::AddSources( float* x, float* s, float dt )
{
	for( uint i = 0; i < mNumPoints; ++i )
	{
		x[ i ] += dt * s[ i ];
	}
}

//------------------------------------------------------------------------------
void FluidSim::ApplyGravity( float dt )
{
	const float gu = mGravityU * dt;
	const float gv = mGravityV * dt;

	for( uint y = 1; y < (mSizeY-1); ++y )
	{
		for( uint x = 1; x < (mSizeX-1); ++x )
		{
			const uint i = IDX(x,y);

			float d = ( mDensitiesR[ i ] + mDensitiesG[ i ] + mDensitiesB[ i ] ) / 3.0f;

			mVelocitiesU[ i ] += d * gu;
			mVelocitiesV[ i ] += d * gv;
		}
	}
}

//------------------------------------------------------------------------------
void FluidSim::Decay( float* d, float rate, float dt )
{
	const float amount = rate * dt;

	for( uint y = 1; y < (mSizeY-1); ++y )
	{
		for( uint x = 1; x < (mSizeX-1); ++x )
		{
			const uint index = IDX(x,y);

			d[index] -= amount;
			if( d[index] < 0.0f )
			{
				d[index] = 0.0f;
			}
		}
	}
}

//------------------------------------------------------------------------------
void FluidSim::Diffuse( int b, float* d, float* d0, float diff, float dt )
{
	const float a = dt * diff * mSizeX * mSizeY;

	for( uint k = 0; k < SOLVER_ITERATIONS; ++k )
	{
		for( uint y = 1; y < (mSizeY-1); ++y )
		{
			for( uint x = 1; x < (mSizeX-1); ++x )
			{
				d[IDX(x,y)] = (d0[IDX(x,y)] + a*(d[IDX(x-1,y)]+d[IDX(x+1,y)]+d[IDX(x,y-1)]+d[IDX(x,y+1)]))/(1+4.0f*a);
			}
		}

		SetBnd( b, d );
	}
}

//------------------------------------------------------------------------------
void FluidSim::Advect( int b, float* d, float* d0, float* u, float* v, float dt )
{
	const float dt0 = dt * mSizeX;

	for( uint y = 1; y < (mSizeY-1); ++y )
	{
		for( uint x = 1; x < (mSizeX-1); ++x )
		{
			float x1 = x - dt0 * u[IDX(x,y)];
			float y1 = y - dt0 * v[IDX(x,y)];

			x1 = std::min( std::max( x1, 0.5f ), mSizeX - 1.501f );
			y1 = std::min( std::max( y1, 0.5f ), mSizeY - 1.501f );

			const int i0 = (int)x1;
			const int i1 = i0+1;
			const int j0 = (int)y1;
			const int j1 = j0+1;

			const float s1 = x1-i0;
			const float s0 = 1-s1;
			const float t1 = y1-j0;
			const float t0 = 1-t1;

			d[IDX(x,y)] = s0*(t0*d0[IDX(i0,j0)]+t1*d0[IDX(i0,j1)])+s1*(t0*d0[IDX(i1,j0)]+t1*d0[IDX(i1,j1)]);
		}
	}

	SetBnd( b, d );
}

//------------------------------------------------------------------------------
void FluidSim::Project( float* u, float* v, float* p, float* div )
{
	const float h = 1.0f / mSizeX;

	for( uint y = 1; y < (mSizeY-1); ++y )
	{
		for( uint x = 1; x < (mSizeX-1); ++x )
		{
			div[IDX(x,y)] = -0.5f * h * ( u[IDX(x+1,y)] - u[IDX(x-1,y)] + v[IDX(x,y+1)] - v[IDX(x, y-1)] );
			p[IDX(x,y)] = 0;
		}
	}

	SetBnd( 0, div );
	SetBnd( 0, p );

	for( uint k = 0; k < SOLVER_ITERATIONS; ++k )
	{
		for( uint y = 1; y < (mSizeY-1); ++y )
		{
			for( uint x = 1; x < (mSizeX-1); ++x )
			{
				p[IDX(x,y)] = (div[IDX(x,y)]+p[IDX(x-1,y)]+p[IDX(x+1,y)]+p[IDX(x,y-1)]+p[IDX(x,y+1)])/4;
			}
		}

		SetBnd( 0, p );
	}

	for( uint y = 1; y < (mSizeY-1); ++y )
	{
		for( uint x = 1; x < (mSizeX-1); ++x )
		{
			u[IDX(x,y)] -= 0.5f*(p[IDX(x+1,y)]-p[IDX(x-1,y)])/h;
			v[IDX(x,y)] -= 0.5f*(p[IDX(x,y+1)]-p[IDX(x,y-1)])/h;
		}
	}

	SetBnd( 1, u );
	SetBnd( 2, v );
}

//------------------------------------------------------------------------------
void FluidSim::SetBnd( int b, float* d )
{
	for( uint x = 1; x < (mSizeX-1); ++x )
	{
		d[IDX(x,0 )]		= b==2 ? -d[IDX(x,1)]			: d[IDX(x,1)];
		d[IDX(x,mSizeY-1)]	= b==2 ? -d[IDX(x,mSizeY-2)]	: d[IDX(x,mSizeY-2)];
	}

	for( uint y = 1; y < (mSizeY-1); ++y )
	{
		d[IDX(0 ,y)]		= b==1 ? -d[IDX(1,y)]			: d[IDX(1,y)];
		d[IDX(mSizeX-1,y)]	= b==1 ? -d[IDX(mSizeX-2,y)]	: d[IDX(mSizeX-2,y)];
	}

	d[IDX(0,0 )]				= 0.5f*(d[IDX(1,0)]					+ d[IDX(0,1)]);
	d[IDX(0,mSizeY-1)]			= 0.5f*(d[IDX(1,mSizeY-1)]			+ d[IDX(0,mSizeY-2)]);
	d[IDX(mSizeX-1,0)]			= 0.5f*(d[IDX(mSizeX-2,0)]			+ d[IDX(mSizeX-1,1)]);
	d[IDX(mSizeX-1,mSizeY-1)]	= 0.5f*(d[IDX(mSizeX-2,mSizeY-1)]	+ d[IDX(mSizeX-1,mSizeY-2)]);
}
