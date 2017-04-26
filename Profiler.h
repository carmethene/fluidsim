#ifndef PROFILER_H
#define PROFILER_H

//Be sure to define USE_PROFILER in the file you're testing

namespace Profiler
{
	void StartCap( const char* tag );
	void StopCap();
}


#ifdef USE_PROFILER
	#define PROFILER_START(x) Profiler::StartCap(x);
	#define PROFILER_STOP() Profiler::StopCap();
#else
	#define PROFILER_START(x)
	#define PROFILER_STOP()
#endif


#endif //PROFILER_H
