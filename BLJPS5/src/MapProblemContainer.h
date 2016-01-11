#pragma once
#include "getFileNames.h"
#include "MapGridData.h"
#include <vector>
#include <string>
struct MapPathQuery
{
	MapPathQuery(int _sx, int _sy, int _dx, int _dy)
	{
		sx = _sx;
		sy = _sy;
		dx = _dx;
		dy = _dy;
	}
	int sx, sy, dx, dy;
};

class MapProblemContainer
{
	int currentProblemId;
	std::vector<char*> problemFileNames;
	vector<MapPathQuery> queryLocations;
	MapGridData mapData;
	char* mapsDirectory;
	string currentMapName;

	bool loadMapData(int mapProblemId)
	{
		queryLocations.clear();
		FILE* fpProblemSet = fopen(problemFileNames[mapProblemId], "r");
		if (fpProblemSet == 0)
		{
			printf("Failed to load problem file %s\n", problemFileNames[mapProblemId]);
			system("pause");
			return false;
		}
		fscanf(fpProblemSet, "%*s %*f"); //Version 1.0	
		char fileName[512];
		int sX, sY, eX, eY;
		while (!feof(fpProblemSet))
			if (fscanf(fpProblemSet, "%*d %s %*d %*d %d %d %d %d %*f", fileName, &sX, &sY, &eX, &eY) != 5)
				break;
			else
				queryLocations.push_back(MapPathQuery(sX, sY, eX, eY));
		fclose(fpProblemSet);

		currentMapName = string(mapsDirectory);
		currentMapName += "/";
		currentMapName += fileName;
		if (!mapData.fillGrid(currentMapName.c_str()))
		{
			printf("Failed to load map file %s associated with the problem file %s\n", fileName, problemFileNames[mapProblemId]);
			system("pause");
			return false;
		}
		return true;
	}
	bool isPassable(const Coordinate &c)
	{
		int index = gridIndex(c);
		if (index == -1)
			return false;
		return  !(mapData.gridDataCopy[index / 8] & (1 << (index % 8)));
	}
	int gridIndex(const Coordinate &c)
	{
		if (c.x < 0 || c.x >= mapData.gridWidth || c.y < 0 || c.y >= mapData.gridHeight)
			return -1;
		return (c.y*mapData.gridWidth) + c.x;
	}
public:
	MapProblemContainer(bool staticMaps, char * _mapsDirectory) :mapsDirectory(_mapsDirectory)
	{
		getMapProblemFiles(staticMaps, mapsDirectory, problemFileNames);
		if (problemFileNames.size() && loadMapData(0))
		{
			currentProblemId = 0;
		}
		else
		{
			currentProblemId = -1;
			printf("Failed to load any problems directory may be incorrect\n");
			printf("Directory: %s\n", _mapsDirectory);
			system("pause");
		}
	}
	~MapProblemContainer()
	{
		for (int i = 0; i < problemFileNames.size(); i++)
			delete[] problemFileNames[i];
	}
	void getMapData(int mapId, vector<bool> &bits, char * &rawMapData, int &width, int &height)
	{
		if (currentProblemId != mapId)
		{
			loadMapData(mapId);
			currentProblemId = mapId;
		}
		rawMapData = mapData.gridDataCopy;
		width = mapData.gridWidth;
		height = mapData.gridHeight;
		for (int i = 0; i < width*height; i++)
			bits.push_back(!(rawMapData[i / 8] & (1 << (i & 7))));
	}

	bool validateLegalPath( vector<Coordinate> & solution)
	{

		if (solution.size() == 0)
			return true;
		for (int i = 0; i < solution.size()-1; i++)
		{
			int dx = solution[i + 1].x - solution[i].x;
			int dy = solution[i + 1].y - solution[i].y;
			if (abs(dx)>1)
				dx = dx > 0 ? 1 : -1;
			if (abs(dy)>1)
				dy = dy > 0 ? 1 : -1;
			Coordinate offset(dx, dy);
			bool diagStep = dx != 0 && dy != 0;
			Coordinate currentStep(solution[i]);
			while (!(currentStep.x == solution[i + 1].x && currentStep.y == solution[i + 1].y))
			{
#ifdef DIAG_UNBLOCKED
				if (!isPassable(currentStep)|| (diagStep&& (!isPassable(Coordinate(currentStep.x + offset.x, currentStep.y)) || !isPassable(Coordinate(currentStep.x, currentStep.y + offset.y)))))
					return false;
#else
				if (!isPassable(currentStep))
					return false;
#endif

				currentStep.add(offset);
			}
			if (!isPassable(solution[i]))
				return false;
		}
		if (!isPassable(solution.back()))
			return false;

		return true;
	}
	MapPathQuery getQueryId(int id)
	{
		return queryLocations[id];
	}
	int getNumberOfMaps()
	{
		return problemFileNames.size();
	}
	int getNumberOfQueriesForMap(int mapId)
	{
		if (mapId<0||mapId > getNumberOfMaps())
			return 0;
		if (currentProblemId != mapId)
		{
			loadMapData(mapId);
			currentProblemId = mapId;
		}

		return queryLocations.size();
	}
	int getNumQueries()
	{
		return queryLocations.size();
	}
	const char * getCurrentMapName()
	{
		return currentMapName.c_str();
	}
};