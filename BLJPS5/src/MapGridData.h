#pragma once
#include <iostream>
#include "Node.h"

struct MapGridData
{
	MapGridData()
	{
		gridData = 0;
		gridDataCopy = 0;
		gridSizeByes = 0;
		gridWidth = 0;
		gridHeight = 0;
		strcpy(fileName, "NoName");
	}
	~MapGridData()
	{
		if (gridData)
		{
			delete[] gridData;
			delete[] gridDataCopy;
		}
	}
	int gridIndex(const Coordinate &c)
	{
		if (c.x<0 || c.x >= gridWidth || c.y<0 || c.y >= gridHeight)
			return -1;
		return (c.y*gridWidth) + c.x;
	}
	bool isCoordinateBlocked(const Coordinate &c)
	{
		int index = gridIndex(c);
		if (index == -1 || gridData == 0)
			return false;
		return  !(gridData[index / 8] & (1 << (index % 8)));
	}
	bool fillGrid(const char * _fileName)
	{

		//If we have last processed this map then update the grid contents from the backup copy and return the data
		if (sameFileName(_fileName))
		{
			memcpy(gridData, gridDataCopy, gridWidth*gridHeight / 8 + 1);
			return gridData;
		}
		strcpy(fileName, _fileName);

		FILE* fp = fopen(fileName, "r");
		if (fp == 0)
			return false;
		int width, height;
		fscanf(fp, "%*s %*s\n");
		fscanf(fp, "%*s %d\n", &height);
		fscanf(fp, "%*s %d\n", &width);
		fscanf(fp, "%*s\n");

		int stepSize = 1;

		//If we already have an array allocated large enough to store the data just fill it in
		//Otherwise we remake the arrays larger
		if (gridSizeByes<width*height/8+1)
		{
			if (gridData)
			{
				delete[] gridData;
				delete[] gridDataCopy;
			}

			gridSizeByes = width*height / 8 + 1;

			gridData = new char[gridSizeByes];
			gridDataCopy = new char[gridSizeByes];

			//printf("Resize map data to %d\n", gridSizeByes);
		}

		gridWidth = width;
		gridHeight = height;

		memset(gridData, 0, gridSizeByes);
		int errorCount = 0;

		for (int y = 0; y<height; y += stepSize)
		{
			for (int x = 0; x<width; x += stepSize)
			{
				char temp = 2;
				while (!(temp == '0' || temp == '1' || temp == '.' || temp == '@' || temp == 'T'))
					fread(&temp, 1, 1, fp);

				int index = y*width + x;
				if (temp == '0' || temp == '@' || temp == 'T')
				{
					gridData[index / 8] |= (1 << index % 8);
					if (index / 8>gridSizeByes - 1)
						printf("MapGridData: Error buffer overflow\n");
				}

			}
		}
		fclose(fp);
		memcpy(gridDataCopy, gridData, gridSizeByes);

		return true;
	}
	bool sameFileName(const char * fName)
	{
		return strcmp(fName, fileName) == 0;
	}
	void cleanUp()
	{
		if (gridData)
		{
			delete[] gridData;
			delete[] gridDataCopy;
			gridData = 0;
			gridSizeByes = 0;
			gridWidth = 0;
			gridHeight = 0;
		}
	}
	char *  gridData;
	char *  gridDataCopy;
	int		gridSizeByes;
	char	fileName[512];
	int		gridWidth, gridHeight;
};