#include "Astar.h"
#include "BL_JPS.h"
//Author: Jason Traish jtraish@csu.edu.au narsue2@gmail.com
//Created: 2015 December 21


//------------------------//
//------Description-------//
//------------------------//
//This program is used to measure the speed of different pathfinding algorithms, the memory they use and the time they take to preprocess.
//Additionally there are functions that are designed to verify the stability of an algorithm over multiple map types by comparing its path length against another algorithm
//See main() for the different fucntions

//------------------------//
//----Compiler flags------//
//------------------------//
//This visual studio profile has 4 compiler settings. Release and Debug modes in UNBLOCKED and BLOCKED modes.
//The example shown below. Key: (.) is a free space. (x) is a blocked space. (S) is current position. (D) destination position.
//When DIAG_UNBLOCKED is defined the following movement is illegal
// .xD
// .Sx
// ...
// with this flag enabled both cardinal directions (up and right) must be free to step to the north east.
//If DIAG_UNBLOCKED is undefined the above example is legal.



#include "JPS.h"
#include "JPS_Plus.h"
#include "BL_JPS_PLUS.h"
#include "BL_JPS_Experimental_2.h"
#include "BL_JPS_Experimental_3.h"
#include "BL_JPS_Experimental_4.h"
#include "BL_JPS_Experimental_5.h"
#include "SubgoalGraph.h"

#include "MapGridData.h"
#include "getFileNames.h"
#include "MapProblemContainer.h"
#include "Timer.h"
#include "RecordMemoryUsage.h"

#define WRITE_SPEED_UP_TO_FILE


double benchmarkQuery(PathFindingAlgorithm * algorithm, MapPathQuery &query, vector<Coordinate> &solution)
{
	Timer t;
	t.StartTimer();
	int iterations = 0;
	do
	{
		for (int repeatQueryId = 0; repeatQueryId < 10; ++repeatQueryId)
			algorithm->findSolution(query.sx, query.sy, query.dx, query.dy, solution);
		iterations += 10;
	} while (t.EndTimer() < 0.001);
	return (double)t.GetElapsedTime()*1000 / iterations;
}
FILE* openSpeedResultsFile(PathFindingAlgorithm * algorithm)
{
	char fileName[128];
	sprintf(fileName, "../results/SpeedResults_%s.txt", algorithm->getAlgorithmName());
	FILE* fp = fopen(fileName, "w");
	if (fp == 0)
	{
		printf("Failed to open the file %s. Create the folder or check folder permissions\n", fileName);
		system("pause");
	}
	return fp;
}
void writeBenchMarkToFile(PathFindingAlgorithm * algorithm, MapPathQuery &query, FILE* fileOutHandle,const char * mapName, int problemId)
{
	static vector<Coordinate> solution;
	double timeTaken = benchmarkQuery(algorithm, query, solution);
	if (fileOutHandle)
		fprintf(fileOutHandle, "%s %d %f %f\n", mapName, problemId, pathLength(solution), timeTaken);
}
PathFindingAlgorithm * getAlgorithmType(AlgorithmType algType, char * mapData,vector<bool> &bits, int mapWidth, int mapHeight)
{
	if (algType == AT_JPS)
		return (new JPS(mapData, mapWidth, mapHeight));
	if (algType == AT_BL_JPS)
		return (new BL_JPS(bits, mapWidth, mapHeight));
	if (algType == AT_JPS_PLUS)
		return (new JPS_PLUS(mapData, mapWidth, mapHeight));
	if (algType == AT_BL_JPS_PLUS)
		return (new BL_JPS_PLUS(mapData, mapWidth, mapHeight));
	if (algType == AT_ASTAR)
		return (new AStar(mapData, mapWidth, mapHeight));
	if (algType == AT_BL_JPS_EXP2)
		return (new BL_JPS_Experimental_2(bits, mapWidth, mapHeight));
	if (algType == AT_BL_JPS_EXP3)
		return (new BL_JPS_Experimental_3(bits, mapWidth, mapHeight));
	if (algType == AT_BL_JPS_EXP4)
		return (new BL_JPS_Experimental_4(bits, mapWidth, mapHeight));
	if (algType == AT_BL_JPS_EXP5)
		return (new BL_JPS_Experimental_5(bits, mapWidth, mapHeight));
	//if (algType == AT_JPS_BJ)
	//	return (new JPS_BlockJumps(mapData, mapWidth, mapHeight));
	if (algType == AT_SUBGOAL)
		return (new SubgoalGraph(bits, mapWidth, mapHeight, "pre.txt"));
	return 0;
}
void addAlgorithmToTest(AlgorithmType algType, vector<FILE *> &speedUpResults, vector<AlgorithmType> &algorithms)
{
	algorithms.push_back(algType);
	speedUpResults.push_back(0);
}

void testAlgorithmSolutionsAllMapPoints(AlgorithmType algTypeA, AlgorithmType algTypeB)
{
	MapProblemContainer mapProblems(true, "../../maps");
	int mapId = 118;
	for (int mapId = 0; mapId < mapProblems.getNumberOfMaps(); mapId++)//87 243
	{
		int mapWidth = 0, mapHeight = 0;
		vector<bool> mapData;
		char* rawData = 0;
		mapProblems.getMapData(mapId, mapData, rawData, mapWidth, mapHeight);
		if (mapWidth*mapHeight>100 * 99)// && mapId!=109)
			continue;
		//Create the BL-JPS algorithm with the map data, width and height
		PathFindingAlgorithm * algorithmA = getAlgorithmType(algTypeA, rawData, mapData, mapWidth, mapHeight);
		PathFindingAlgorithm * algorithmB = getAlgorithmType(algTypeB, rawData, mapData, mapWidth, mapHeight);
		printf("%s: Map %d: (%d,%d) %s\n", algorithmA->getAlgorithmName(), mapId, mapWidth, mapHeight, mapProblems.getCurrentMapName());

		//Preprocess the map
		algorithmA->preProcessGrid();
		if (1)
		{
			char tempFileName[256];
			sprintf(tempFileName, "../bin/PreprocessedData/%s.pre", algorithmA->getAlgorithmName());// mapId PreprocessedData/%d-%s
			algorithmA->dumpPreprocessedDataToFile(tempFileName);
			delete algorithmA;
			algorithmA = getAlgorithmType(algTypeA, rawData, mapData, mapWidth, mapHeight);
			algorithmA->readPreprocessedDataToFile(tempFileName);
		}


		algorithmB->preProcessGrid();
		int x1 = 127;
		for (int x1 = 0; x1 < mapWidth*mapHeight; x1++)
		{
			if (x1%1000==0)
				printf("%d/%d\n", x1, mapWidth*mapHeight);
			int x2 = 1717;
			for (int x2 = 0; x2 < mapWidth*mapHeight; x2++)
			{
				MapPathQuery query(x1%mapWidth, x1 / mapWidth, x2%mapWidth, x2 / mapWidth);
				//MapPathQuery query(44,62,24,74);//6244,7424

				vector<Coordinate> solution, solution2;
				algorithmA->findSolution(query.sx, query.sy, query.dx, query.dy, solution);
				algorithmB->findSolution(query.sx, query.sy, query.dx, query.dy, solution2);

				float dist = pathLength(solution);
				float dist2 = pathLength(solution2);
				bool validSolution = mapProblems.validateLegalPath(solution);
				bool solutionInCorrectOrder = solution.size() && !(solution.begin()->x == query.dx && solution.begin()->y == query.dy && solution.back().x == query.sx && solution.back().y == query.sy);


				if (nonSimilarFloat(dist, dist2) || !validSolution || solutionInCorrectOrder)
				{
					if (!validSolution)
						printf("!!  Invalid Path don't match(%d,%d) %d %d %d %d %d %4.2f (%d,%d)->(%d,%d) !!\n\n", mapWidth, mapHeight, mapId, x1, x2, solution.size(), solution2.size(), pathLength(solution) - pathLength(solution2), query.sx, query.sy, query.dx, query.dy);
					if (solutionInCorrectOrder)
						printf("!!  Path in wrong order(%d,%d) %d %d %d %d %d %4.2f !!\n\n", mapWidth, mapHeight, mapId, x1, x2, solution.size(), solution2.size(), pathLength(solution) - pathLength(solution2));
					if (nonSimilarFloat(pathLength(solution), pathLength(solution2)))
						printf("!!  Path lengths don't match map:%d %d %d (%d,%d)->(%d,%d) %d %d %4.2f[%f] !!\n\n", mapId, x1, x2, query.sx, query.sy, query.dx, query.dy, solution.size(), solution2.size(), pathLength(solution) - pathLength(solution2), pathLength(solution2));
					system("pause");
				}
			}
		}

		//Delete the algorithm after it has finished a map, recreate it for new maps
		delete algorithmA;
		delete algorithmB;
	}

}
void runAllStaticMapsAndQueries(AlgorithmType algType)
{
#ifdef _DEBUG
	printf("Benchmark algorithms in release mode!\n");
	return ;
#endif
	MapProblemContainer mapProblems(true, "../../maps");
	
	vector<FILE *> speedUpResults;
	vector<AlgorithmType> algorithms;
	addAlgorithmToTest(algType, speedUpResults, algorithms);

	for (int algorithmId = 0; algorithmId < algorithms.size(); algorithmId++)
	{

		for (int mapId = 0; mapId < mapProblems.getNumberOfMaps(); mapId++)//mapProblems.getNumberOfMaps()
		{
			int mapWidth = 0, mapHeight = 0;
			vector<bool> mapData;
			char* rawData = 0;
			mapProblems.getMapData(mapId, mapData, rawData, mapWidth, mapHeight);

			//Create the BL-JPS algorithm with the map data, width and height
			PathFindingAlgorithm * algorithm = getAlgorithmType(algType, rawData, mapData, mapWidth, mapHeight);
			printf("%s Map %d:(%d,%d) %s\n", algorithm->getAlgorithmName(),mapId, mapWidth, mapHeight, mapProblems.getCurrentMapName());

			#ifdef WRITE_SPEED_UP_TO_FILE
			if (speedUpResults[algorithmId] == 0)
				speedUpResults[algorithmId] = openSpeedResultsFile(algorithm);
			#endif

			if (0)
			{
				char tempFileName[256];
				sprintf(tempFileName, "../bin/PreprocessedData/%d-%s.pre", mapId, algorithm->getAlgorithmName());
				algorithm->preProcessGrid();

				algorithm->dumpPreprocessedDataToFile(tempFileName);
				delete algorithm;
				algorithm = getAlgorithmType(algType, rawData, mapData, mapWidth, mapHeight);
				algorithm->readPreprocessedDataToFile(tempFileName);
			}
			else
				//Preprocess the map
				algorithm->preProcessGrid();

			for (int queryId = 0; queryId < mapProblems.getNumQueries(); queryId++)
			{
				//printf("%d\n", queryId);
				writeBenchMarkToFile(algorithm, mapProblems.getQueryId(queryId), speedUpResults[algorithmId], mapProblems.getCurrentMapName(), queryId);
			}
			//Delete the algorithm after it has finished a map, recreate it for new maps
			delete algorithm;
		}
		if (speedUpResults[algorithmId])
		{
			fclose(speedUpResults[algorithmId]);
			speedUpResults[algorithmId] = 0;
		}
	}
}
bool solutionHasPoint(int x, int y, vector<Coordinate>& solution)
{
	for (int ii = 0; ii < solution.size(); ii++)
		if (solution[ii].x == x && solution[ii].y == y)
			return true;

	return false;
}
void printSolutionOnMap(int mapWidth,vector<bool>& mapData, vector<Coordinate>& solution)
{
	int xCell = 0;
	int yCell = 0;
	FILE*fp = fopen("../bin/mapSolution.txt","w");
	for (int i = 0; i < mapData.size(); i++)
	{

		if (mapData[i])
		{
			if (solutionHasPoint(xCell, yCell, solution))			
				fprintf(fp, "x");
			else
				fprintf(fp, ".");
		}
		else
			if (solutionHasPoint(xCell, yCell, solution))
				fprintf(fp, "X");
			else
				fprintf(fp, "@");

		xCell++;
		if (xCell == mapWidth)
		{
			xCell = 0;
			yCell++;
			fprintf(fp, "\n");
		}
	}
	fclose(fp);
}
float testSingleMapPath(AlgorithmType algType, int mapId, int queryId,float testTime)
{
	//Specifices the location where all the map files are stored
	//Caches all the available map file names
	MapProblemContainer mapProblems(true, "../../maps");

	//Next we request the data for the given mapId
	//The data we get back is the width and height of the map and a bit packed copy of the map contents
	int mapWidth = 0, mapHeight = 0;
	vector<bool> mapData;
	char* rawData = 0;
	mapProblems.getMapData(mapId, mapData, rawData, mapWidth, mapHeight);

	//Creates the algorithm requested - see the definition of AlgorithmType for all algorithms
	PathFindingAlgorithm * algorithm = getAlgorithmType(algType, rawData, mapData, mapWidth, mapHeight);

	//Preprocesses the grid if required
	algorithm->preProcessGrid();
	/*algorithm->dumpPreprocessedDataToFile("dempData.dat");
	delete algorithm;
	algorithm = getAlgorithmType(algType, rawData, mapData, mapWidth, mapHeight);
	algorithm->readPreprocessedDataToFile("dempData.dat");*/

	//Gets the query for id requested, its basically just a start and destination on the given map
	if (queryId == -1)
		queryId = mapProblems.getNumQueries() - 1;
	if (queryId<0 && queryId > mapProblems.getNumQueries())
	{
		printf("Error specified query id out of range\n");
		return 0;
	}
	MapPathQuery query = mapProblems.getQueryId(queryId);
	printf("%s:Run query on map %s (%d,%d)->(%d,%d)\n", algorithm->getAlgorithmName(),mapProblems.getCurrentMapName(), query.sx, query.sy, query.dx, query.dy);
	vector<Coordinate> solution;

	//Runs the algorithm for the given query, it returns a vector of Coordinates that make up the path
	Timer t;
	t.StartTimer();
	int iterations = 0;
	do
	{
		iterations+=100;
		for (int i = 0; i < 100; i++)
			algorithm->findSolution(query.sx, query.sy, query.dx, query.dy, solution);
	} while (t.EndTimer() < testTime);
	float iterationsPerms = iterations/(t.GetElapsedTime() * 1000.0)/5.0;
	printSolutionOnMap(mapWidth, mapData, solution);

	printf("Found a solution of length %f %d iterations (%f)\n", pathLength(solution), iterations, iterationsPerms);
	delete algorithm;
	return iterationsPerms;
}
void buildMemoryUsageBatFile()
{
	MapProblemContainer mapProblems(true, "../../maps");
	FILE*fp = fopen("../bin/runMemoryTests.bat", "w");
	for (int mapId = 0; mapId < mapProblems.getNumberOfMaps(); mapId++)
	//	for (int queryId = 0; queryId < mapProblems.getNumQueries(); queryId++)
			fprintf(fp, "test.exe -mem %d %d\n", mapId, 0);
	fclose(fp);
}

void testMemoryUsage(AlgorithmType algType, int mapId, int queryId)
{
	//Specifices the location where all the map files are stored
	//Caches all the available mpa file names
	MapProblemContainer mapProblems(true, "../../maps");

	//Next we request the data for the given mapId
	//The data we get back is the width and height of the map and a bit packed copy of the map contents
	int mapWidth = 0, mapHeight = 0;
	vector<bool> mapData;
	char* rawData = 0;
	mapProblems.getMapData(mapId, mapData, rawData, mapWidth, mapHeight);

	//Creates the algorithm requested - see the definition of AlgorithmType for all algorithms
	printMemoryUsage(0);
	PathFindingAlgorithm * algorithm = getAlgorithmType(algType, rawData, mapData, mapWidth, mapHeight);

	//Preprocesses the grid if required
	algorithm->preProcessGrid();
	printMemoryUsage(1);


	//printf("Run query on map %s (%d,%d)->(%d,%d)\n", mapProblems.getCurrentMapName(), query.sx, query.sy, query.dx, query.dy);
	vector<Coordinate> solution;

	//Runs the algorithm for the given query, it returns a vector of Coordinates that make up the path
	//Gets the query for id requested, its basically just a start and destination on the given map	
	for (int queryId = 0; queryId < mapProblems.getNumQueries(); queryId++)
	{
		MapPathQuery query = mapProblems.getQueryId(queryId);
		algorithm->findSolution(query.sx, query.sy, query.dx, query.dy, solution);
		printMemoryUsage(2, (char*)algorithm->getAlgorithmName(), mapProblems.getCurrentMapName(), queryId, mapWidth, mapHeight, 0);
	}
	delete algorithm;
	//printf("Found a solution of length %f\n", pathLength(solution));
}
void testPreprocessingTime(AlgorithmType algType)
{
	//Specifices the location where all the map files are stored
	//Caches all the available mpa file names
	MapProblemContainer mapProblems(true, "../../maps");
	FILE * fp = 0;

	//Next we request the data for the given mapId
	//The data we get back is the width and height of the map and a bit packed copy of the map contents
	int mapWidth = 0, mapHeight = 0;
	char* rawData = 0;

	for (int mapId = 0; mapId < mapProblems.getNumberOfMaps(); mapId++)//89 243
	{
		vector<bool> mapData;

		mapProblems.getMapData(mapId, mapData, rawData, mapWidth, mapHeight);

		//Creates the algorithm requested - see the definition of AlgorithmType for all algorithms
		PathFindingAlgorithm * algorithm = getAlgorithmType(algType, rawData, mapData,  mapWidth, mapHeight);

		Timer t;
		t.StartTimer();
		algorithm->preProcessGrid();
		t.EndTimer();
		char tempFileName[256];
		sprintf(tempFileName, "../bin/PreprocessedData/%d-%s.pre", mapId, algorithm->getAlgorithmName());
		algorithm->dumpPreprocessedDataToFile(tempFileName);

		if (fp == 0)
		{
			char fileName[255];
			sprintf(fileName, "../results/PreprocessTimes/PreprocessingTime_%s.txt", algorithm->getAlgorithmName());
			fp = fopen(fileName,"w");
			if (fp == 0)
			{
				printf("Could not open file to write results (%s)\n", fileName);
				system("pause");
				exit(0);
			}
		}
		printf("Preprocess time:%d/%d %s %s %f\n", mapId, mapProblems.getNumberOfMaps(),mapProblems.getCurrentMapName(), algorithm->getAlgorithmName(), t.GetElapsedTime());

		delete algorithm;

		fprintf(fp, "%s %f\n", mapProblems.getCurrentMapName(),t.GetElapsedTime());
	}
	fclose(fp);
}

void testAlgorithmSolutions(AlgorithmType algTypeA, AlgorithmType algTypeB)
{
	MapProblemContainer mapProblems(true, "../../maps");
	int mapId = 0;
	for (int mapId = 0; mapId < mapProblems.getNumberOfMaps(); mapId++)//89 243 161
	{
		int mapWidth = 0, mapHeight = 0;
		vector<bool> mapData;
		char* rawData = 0;
		mapProblems.getMapData(mapId, mapData, rawData, mapWidth, mapHeight);

		//Create the BL-JPS algorithm with the map data, width and height
		PathFindingAlgorithm * algorithmA = getAlgorithmType(algTypeA, rawData,mapData,  mapWidth, mapHeight);
		PathFindingAlgorithm * algorithmB = getAlgorithmType(algTypeB, rawData,mapData, mapWidth, mapHeight);
		printf("%s: Map %d: (%d,%d) %s\n", algorithmA->getAlgorithmName(), mapId, mapWidth, mapHeight, mapProblems.getCurrentMapName());

		//Preprocess the map
		algorithmA->preProcessGrid();
		
		if (0)
		{
			char tempFileName[256];
			sprintf(tempFileName, "../bin/PreprocessedData/%s.pre", algorithmA->getAlgorithmName());// mapId PreprocessedData/%d-%s
			algorithmA->dumpPreprocessedDataToFile(tempFileName);
			delete algorithmA;
			algorithmA = getAlgorithmType(algTypeA, rawData, mapData, mapWidth, mapHeight);
			algorithmA->readPreprocessedDataToFile(tempFileName);
		}
		algorithmB->preProcessGrid();
		bool error = false;
		int errorCount = 0;
		for (int queryId =0; queryId < mapProblems.getNumQueries(); queryId++)
		{
			//printf("%d\n", queryId);
			MapPathQuery query = mapProblems.getQueryId(queryId);
			vector<Coordinate> solution, solution2;
			algorithmA->findSolution(query.sx, query.sy, query.dx, query.dy, solution);
			algorithmB->findSolution(query.sx, query.sy, query.dx, query.dy, solution2);
			float dist = pathLength(solution);
			float dist2 = pathLength(solution2);
			bool validSolution = mapProblems.validateLegalPath(solution);
			bool solutionInCorrectOrder = solution.size() && !( solution.begin()->x == query.dx && solution.begin()->y == query.dy && solution.back().x == query.sx && solution.back().y == query.sy);
			if (nonSimilarFloat(pathLength(solution), pathLength(solution2)) || !validSolution   || solutionInCorrectOrder)
			{
				if (!error)
				{
					if (!validSolution)
						printf("!!  Invalid Path don't match(%d,%d) %d %d %d %d %4.2f (%d,%d)->(%d,%d) !!\n\n", mapWidth, mapHeight, mapId, queryId, solution.size(), solution2.size(), pathLength(solution) - pathLength(solution2), query.sx, query.sy, query.dx, query.dy);
					if (solutionInCorrectOrder)
						printf("!!  Path in wrong order(%d,%d) %d %d %d %d %4.2f !!\n\n", mapWidth, mapHeight, mapId, queryId, solution.size(), solution2.size(), pathLength(solution) - pathLength(solution2));
					if (nonSimilarFloat(pathLength(solution), pathLength(solution2)))
					{
						printf("!!  Path lengths don't match(%d,%d) %d %d %d %d %4.2f  (%d,%d)->(%d,%d)!!\n", mapWidth, mapHeight, mapId, queryId, solution.size(), solution2.size(), pathLength(solution) - pathLength(solution2), query.sx, query.sy, query.dx, query.dy);
						printf("!! Expected Length %f\n", pathLength(solution2));
					}
					system("pause");
					error++;
				}
				errorCount++;
			}
		}
		if (errorCount)
			printf("Error Count:%d\n", errorCount);
			//Delete the algorithm after it has finished a map, recreate it for new maps
		delete algorithmA;
		delete algorithmB;
	}

}
int main(int argc, char **argv)
{

	//If its your first look at this project start by examining testSingleMapPath
	//Test a single path, this is a simple example that will help you get up and running quickly
	int mapId		= 113; //The map id refers to which map to use
	int testQuery	= 163; //The query id refers to the query id to use as loaded from the .scen file associated with the given map
	float testTime  = 10;  //The amount of time(seconds) to spend repeating the qury. Useful if your there is a specific path you want to focus on optimising

	//testSingleMapPath(AT_JPS, mapId, testQuery, testTime);
	//testSingleMapPath(AT_BL_JPS, mapId, testQuery, testTime);
	//testSingleMapPath(AT_BL_JPS_EXP2, mapId, testQuery, testTime);
	//testSingleMapPath(AT_BL_JPS_EXP3, mapId, testQuery, testTime);
	//testSingleMapPath(AT_BL_JPS_EXP4, mapId, testQuery, testTime);
	testSingleMapPath(AT_BL_JPS_EXP5, mapId, testQuery, testTime);
	

	//This function will iterate over all the queries for all the maps and write the results to file if WRITE_SPEED_UP_TO_FILE is defined
	
	//runAllStaticMapsAndQueries(AT_BL_JPS_EXP5);
	//runAllStaticMapsAndQueries(AT_BL_JPS_EXP4);
	//runAllStaticMapsAndQueries(AT_BL_JPS_EXP3);
	//runAllStaticMapsAndQueries(AT_BL_JPS_EXP2);
	//runAllStaticMapsAndQueries(AT_BL_JPS);
	//runAllStaticMapsAndQueries(AT_JPS);
	//runAllStaticMapsAndQueries(AT_SUBGOAL);
	//runAllStaticMapsAndQueries(AT_ASTAR);


	//Used to build a batch file that will run a new instance of this program for each map
	//buildMemoryUsageBatFile();

	//Used for recording memory usage. It should run a single maps path quueries record the memory consumption in preprocessing and post search.
	//if (argc>1&&strcmp(argv[1], "-mem") == 0)
	//	testMemoryUsage(AT_BL_JPS, atoi(argv[2]), atoi(argv[3]));


	//The following functions run all the paths in the maps and compare the resulting path lengths and validy of the paths.
	//If there is a difference in path length then the an error will be reported

	//testAlgorithmSolutions(AT_BL_JPS_EXP5, AT_BL_JPS_EXP3);
	//testAlgorithmSolutions(AT_BL_JPS_EXP4, AT_BL_JPS_EXP3);
	//testAlgorithmSolutions(AT_BL_JPS, AT_BL_JPS_EXP3);
	//testAlgorithmSolutions(AT_BL_JPS_EXP2, AT_BL_JPS_EXP3);


	//Tests all possible paths on smaller maps. Good for giving an algorithm a thorough error testing.
	//If the path lengths do not patch or the path returned is invalid an error is given

	//testAlgorithmSolutionsAllMapPoints(AT_BL_JPS_EXP3, AT_JPS);// done
	//testAlgorithmSolutionsAllMapPoints(AT_BL_JPS_EXP2, AT_BL_JPS_EXP5);// done
	//testAlgorithmSolutionsAllMapPoints(AT_BL_JPS, AT_BL_JPS_EXP5);// done

	//Only performs preprocessing on the given algorithm for all maps and writes the time taken to file

	//testPreprocessingTime(AT_JPS);
	//testPreprocessingTime(AT_BL_JPS);
	//testPreprocessingTime(AT_BL_JPS_EXP2);
	//testPreprocessingTime(AT_BL_JPS_EXP3);
	//testPreprocessingTime(AT_BL_JPS_EXP4);
	//testPreprocessingTime(AT_BL_JPS_EXP5);


	system("pause");

	return 0;
}