#include <Psapi.h>
#pragma comment(lib,"Psapi.lib")
#include <stdio.h>

void printMemoryUsage(int id, char* algName = 0, const char * pathFileName = 0, int _problemId = 0, int gridWidth = 0, int gridHeight = 0, float density = 0)
{
	static unsigned int firstMemory = 0;
	static unsigned int secondMemory = 0;

	PROCESS_MEMORY_COUNTERS memCounter;
	bool result = GetProcessMemoryInfo(GetCurrentProcess(),
		&memCounter,
		sizeof(memCounter));

	//printf("MemUsage %dB Peak:%dB \n",(memCounter.WorkingSetSize-firstMemory),memCounter.WorkingSetSize);
	if (id == 2)
	{
		char tempFName[128];
		sprintf(tempFName, "../results/Memory/memoryOutput_%s.txt", algName);
		static FILE* memoryOutput = fopen(tempFName, "a");
		fprintf(memoryOutput, "%s %d %d %d %f %d %d %d\n", pathFileName, _problemId, gridWidth, gridHeight, density, firstMemory, secondMemory, memCounter.PeakWorkingSetSize);
		fflush(memoryOutput);

	}
	else if (id == 0)
	{
		result = GetProcessMemoryInfo(GetCurrentProcess(),
			&memCounter,
			sizeof(memCounter));
		firstMemory = memCounter.WorkingSetSize;
	}
	else if (id == 1)
	{
		secondMemory = memCounter.WorkingSetSize;
	}
}