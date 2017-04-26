#include "PixelToaster.h"
#include "types.h"
#include "FluidSim.h"
#include "Profiler.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <utility>

using namespace PixelToaster;

//------------------------------------------------------------------------------
// Constants:
namespace
{
	const char* const	APP_NAME		= "Fluid";

	const uint			SIMULATION_WIDTH			= 60;
	const uint			SIMULATION_HEIGHT			= 100;
	const uint			SCREEN_SCALE				= 5;
	const uint			SIMULATION_TIME_DELTA_MS	= 30;

	const float			SOURCE_DENSITY	= 15.0f;
	const float			PUSH_VELOCITY	= 40.0f;

	const float			VISCOSITY		= 0.0002f;
	const float			DIFFUSION		= 0.0001f;
	const float			DECAY			= 0.5f;
	const float			GRAVITY			= -10.0f;
}

//------------------------------------------------------------------------------
// Don't change these - automatically calculated
namespace
{
	const uint	SCREEN_WIDTH	= (SIMULATION_WIDTH * SCREEN_SCALE) - SCREEN_SCALE;
	const uint	SCREEN_HEIGHT	= (SIMULATION_HEIGHT * SCREEN_SCALE) - SCREEN_SCALE;
}

//------------------------------------------------------------------------------
#ifdef _MSC_VER
inline int Round(float a)
{
	int retval;

	__asm fld a
	__asm fistp retval

	return retval;
}
#else
inline int Round(float a)
{
	return (int)a;
}
#endif

//------------------------------------------------------------------------------
class Application : public Listener
{
public:
	Application()
		:	mDisplay( APP_NAME, SCREEN_WIDTH, SCREEN_HEIGHT )
		,	mSim( SIMULATION_WIDTH, SIMULATION_HEIGHT, VISCOSITY, DIFFUSION, DECAY )
		,	mMouseX( 0 )
		,	mMouseY( 0 )
		,	mColourR( 1.0f )
		,	mColourG( 1.0f )
		,	mColourB( 1.0f )
		,	mDrawing( false )
		,	mErasing( false )
		,	mForcingMouse( false )
		,	mForcingKeyboard( false )
		,	mUseGravity( false )
		,	mClampColours( false )
		,	mShowSources( false )
		,	mShowVelocity( false )
	{
		mDisplay.listener( this );
		mSimPixels.resize( SIMULATION_WIDTH * SIMULATION_HEIGHT );
		mDisplayPixels.resize( SCREEN_WIDTH * SCREEN_HEIGHT );

		if( mUseGravity )
		{
			mSim.SetGravity( 0.0f, GRAVITY );
		}
	}

	//Listener overrides
	virtual void onMouseMove( DisplayInterface & display, Mouse mouse )
	{
		const float xf = std::max( mouse.x, 0.0f );
		const float yf = std::max( mouse.y, 0.0f );

		mMouseX = (uint)xf / SCREEN_SCALE;
		mMouseY = (uint)yf / SCREEN_SCALE;

		mMouseX = std::min( mMouseX, SIMULATION_WIDTH - 1 );
		mMouseY = std::min( mMouseY, SIMULATION_HEIGHT - 1 );
	}

	virtual void onMouseButtonDown( DisplayInterface & display, Mouse mouse )
	{
		if( mouse.buttons.left )
		{
			ChangeColour();
			mSim.ApplyForce( mMouseX, mMouseY, PUSH_VELOCITY );
		}

		mDrawing		= mouse.buttons.left;
		mForcingMouse	= mouse.buttons.middle;
		mErasing		= mouse.buttons.right;
	}

	virtual void onMouseButtonUp( DisplayInterface & display, Mouse mouse )
	{
		mDrawing		= mouse.buttons.left;
		mForcingMouse	= mouse.buttons.middle;
		mErasing		= mouse.buttons.right;
	}

	virtual void onKeyDown( DisplayInterface & display, Key key )
	{
		switch( key )
		{
		case Key::G:
			mUseGravity = ! mUseGravity;
			mSim.SetGravity( 0.0f, mUseGravity ? GRAVITY : 0.0f );
			break;

		case Key::S:
			mShowSources = ! mShowSources;
			break;

		case Key::C:
			mSim.ClearSources();
			break;

		case Key::R:
			mSim.ClearDensity();
			break;

		case Key::X:
			ChangeColour();
			break;

		case Key::L:
			mClampColours = ! mClampColours;
			break;

		case Key::V:
			mShowVelocity = ! mShowVelocity;
			break;

		case Key::Space:
			mForcingKeyboard = true;
			break;
		};
	}

	virtual void onKeyUp( DisplayInterface & display, Key key )
	{
		switch( key )
		{
		case Key::Space:
			mForcingKeyboard = false;
			break;
		};
	}

	//App code
	void ProcessInput()
	{
		if( mErasing )
		{
			mSim.EraseSource( mMouseX, mMouseY );
		}
		else if( mDrawing )
		{
			mSim.PlaceSource( mMouseX, mMouseY, mColourR, mColourG, mColourB );
		}

		if( mForcingMouse || mForcingKeyboard )
		{
			mSim.ApplyForce( mMouseX, mMouseY, PUSH_VELOCITY );
		}
	}

	void ChangeColour()
	{
		mColourR = (std::rand() % 101) / 100.0f;	mColourR *= SOURCE_DENSITY;
		mColourG = (std::rand() % 101) / 100.0f;	mColourG *= SOURCE_DENSITY;
		mColourB = (std::rand() % 101) / 100.0f;	mColourB *= SOURCE_DENSITY;
	}

	void Upscale()
	{
		uint pixel_no = 0;

		for( uint y = 0; y < SCREEN_HEIGHT; ++y )
		{
			for( uint x = 0; x < SCREEN_WIDTH; ++x )
			{
				const float	fx = float(x) / float(SCREEN_SCALE);
				const float	fy = float(y) / float(SCREEN_SCALE);

				const float floorx = std::floor(fx);
				const float floory = std::floor(fy);

				const int	ix = Round(floorx);
				const int	iy = Round(floory);

				const float	x_ratio		= fx - floorx;
				const float y_ratio		= fy - floory;
				const float x_opposite	= 1.0f - x_ratio;
				const float y_opposite	= 1.0f - y_ratio;

				const uint index = (iy * SIMULATION_WIDTH) + ix;

				const Pixel& p1 = mSimPixels[ index ];
				const Pixel& p2 = mSimPixels[ index + 1 ];
				const Pixel& p3 = mSimPixels[ index + SIMULATION_WIDTH ];
				const Pixel& p4 = mSimPixels[ index + SIMULATION_WIDTH + 1 ];

				const float r = (p1.r * x_opposite + p2.r * x_ratio) * y_opposite +
								(p3.r * x_opposite + p4.r * x_ratio) * y_ratio;

				const float g = (p1.g * x_opposite + p2.g * x_ratio) * y_opposite +
								(p3.g * x_opposite + p4.g * x_ratio) * y_ratio;

				const float b = (p1.b * x_opposite + p2.b * x_ratio) * y_opposite +
								(p3.b * x_opposite + p4.b * x_ratio) * y_ratio;

				mDisplayPixels[ pixel_no ].r = r;
				mDisplayPixels[ pixel_no ].g = g;
				mDisplayPixels[ pixel_no ].b = b;

				++pixel_no;
			}
		}
	}

	void Run()
	{
		time_t last_update_time = std::clock();

		while( mDisplay.open() )
		{
			ProcessInput();

			const time_t current_time = std::clock();
			const time_t delta_time = current_time - last_update_time;
			if( delta_time >= SIMULATION_TIME_DELTA_MS )
			{
				//Time for a sim update
				last_update_time = current_time;

				mSim.Update( SIMULATION_TIME_DELTA_MS / 1000.0f );
			}

			mSim.Draw( mSimPixels, mClampColours, mShowSources, mShowVelocity );
			Upscale();
			mDisplay.update( mDisplayPixels );
		}
	}

private:
	Display			mDisplay;
	FluidSim		mSim;
	vector<Pixel>	mSimPixels;
	vector<Pixel>	mDisplayPixels;

	uint			mMouseX;
	uint			mMouseY;

	float			mColourR;
	float			mColourG;
	float			mColourB;

	bool			mDrawing;
	bool			mErasing;
	bool			mForcingMouse;
	bool			mForcingKeyboard;
	bool			mUseGravity;
	bool			mShowSources;
	bool			mClampColours;
	bool			mShowVelocity;
};

//------------------------------------------------------------------------------
int main()
{
	srand((uint)time(0));

	std::cout
		<< "Fluid\n"
		<< "=====\n"
		<< "\n"
		<< "Left Mouse\t"	<< "Draw sources (releasing mouse button changes colour)\n"
		<< "Right Mouse\t"	<< "Erase sources\n"
		<< "Middle Mouse\t" << "Create turbulence\n"
		<< "Space\t\t"		<< "Create turbulence at cursor position\n"
		<< "C\t\t"			<< "Clear all sources\n"
		<< "S\t\t"			<< "Toggle show sources\n"
		<< "X\t\t"			<< "Change colour\n"
		<< "R\t\t"			<< "Reset density\n"
		<< "G\t\t"			<< "Toggle gravity\n"
		<< "L\t\t"			<< "Clamp colours\n"
		<< "V\t\t"			<< "Toggle velocity field display\n"
		<< "Esc\t\t"		<< "Quit\n\n"
		<< "\n";

	Application the_app;
	the_app.Run();
}
