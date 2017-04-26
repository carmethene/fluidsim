#include "Profiler.h"
#include <windows.h>
#include <cstdio>
#include <cassert>

namespace Profiler
{
	static LARGE_INTEGER	gFrequency;
	static bool				gHaveFrequency	= false;

	static LARGE_INTEGER	gStartTime;
	static const char*		gTag			= NULL;

	void StartCap( const char* tag )
	{
		if( ! gHaveFrequency )
		{
			QueryPerformanceFrequency( &gFrequency );
			gHaveFrequency = true;
		}

		if( gTag != NULL )
		{
			//This is the worst profiler ever and doesn't support nested calls
			assert(false);
		}

		gTag = tag;
		QueryPerformanceCounter( &gStartTime );
	}

	void StopCap()
	{
		LARGE_INTEGER stop_time;
		QueryPerformanceCounter( &stop_time );

		const double deltad = (double(stop_time.QuadPart - gStartTime.QuadPart) * 1000.0) / gFrequency.QuadPart;
		const unsigned int delta = (unsigned int)deltad;

		const size_t buffer_size = 1024;
		char buffer[ buffer_size ];
		sprintf_s( buffer, buffer_size, "PROFILE: %dms - %s\n", delta, gTag );
		OutputDebugStringA( buffer );

		gTag = NULL;
	}
}
