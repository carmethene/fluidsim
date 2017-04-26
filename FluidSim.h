#ifndef FLUIDSIM_H
#define FLUIDSIM_H


#include <vector>
#include <cassert>
#include "PixelToaster.h"
#include "types.h"


class FluidSim
{
public:
	FluidSim( uint size_x, uint size_y, float viscosity, float diffusion, float decay );
	~FluidSim();

	void Update( float dt );
	void PlaceSource( uint x, uint y, float r, float g, float b );
	void EraseSource( uint x, uint y );
	void ClearSources();
	void ClearDensity();
	void ApplyForce( uint x, uint y, float amount );
	void SetGravity( float gu, float gv );
	void Draw( PixelToaster::vector<PixelToaster::Pixel>& out_pixels, bool clamp_colours, bool show_sources, bool show_velocity ) const;

private:
	//Array index helper
	inline uint IDX( uint x, uint y ) const
	{
		assert( x >= 0 && x < mSizeX && y >= 0 && y < mSizeY );

		return (y * mSizeX) + x;
	}

	void DensityStep( float* s, float* x, float* x0, float* u, float* v, float diff, float dt );
	void VelocityStep( float* u, float* v, float* u0, float* v0, float visc, float dt );

	void AddSources( float* x, float* s, float dt );
	void ApplyGravity( float dt );
	void Decay( float* d, float rate, float dt );
	void Diffuse( int b, float* x, float* x0, float diff, float dt );
	void Advect( int b, float* d, float* d0, float* u, float* v, float dt );
	void Project( float* u, float* v, float* p, float* div );
	void SetBnd( int b, float* d );

private:
	const uint mSizeX;
	const uint mSizeY;
	const uint mNumPoints;

	const float mViscosity;
	const float mDiffusion;
	const float mDecay;

	float* mDensitiesR;
	float* mDensitiesG;
	float* mDensitiesB;
	float* mDensitiesR0;
	float* mDensitiesG0;
	float* mDensitiesB0;

	float* mVelocitiesU;
	float* mVelocitiesV;
	float* mVelocitiesU0;
	float* mVelocitiesV0;

	float* mSourcesR;
	float* mSourcesG;
	float* mSourcesB;

	float mGravityU;
	float mGravityV;
};


#endif //FLUIDSIM_H
