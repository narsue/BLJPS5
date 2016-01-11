#pragma once
#include "Node.h"
#include "PathFindingAlgorithm.h"
using namespace std;


//Directions
// N, NE, E, SE, S, SW, W, NW , StartPosition
#define NO_DIRECTION 8
class JPS_PLUS : public PathFindingAlgorithm
{
private:
	//Special container classes that help with general allocation of memory and search
	NodeContainer nodesC;
	BinaryHeap openListBh;

	//Table of flags indicating is a map location has been searched (Closed List)
	char *testedGrid;

	//Map data
	char * gridData;
	int gridWidth, gridHeight;

	//Goal node position and index
	int eX, eY, endIndex;

	//Jump Point Lookup table and a backup
	unsigned int *preprocessedJumpPoints, *preprocessedJumpPointsBackup;

	bool inBounds(const int index)
	{
		return index<gridWidth*gridHeight&&index >= 0;
	}
	int gridIndex(const Coordinate &c)
	{
		if (c.x<0 || c.x >= gridWidth || c.y<0 || c.y >= gridHeight)
			return -1;
		return (c.y*gridWidth) + c.x;
	}
	Coordinate nextCoordinate(const Coordinate& c, const int dir)
	{
		static char dirMov[] = { 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0, -1, -1, 0, 0 };
		return Coordinate(c.x + dirMov[dir * 2], c.y + dirMov[dir * 2 + 1]);
	}
	int dirIsDiagonal(const int dir)
	{
		return (dir % 2) != 0;
	}
	inline int implies(const int a, const int b)
	{
		return a ? b : 1;
	}
	inline unsigned char addDirectionToSet(const unsigned char dirs, const int dir)
	{
		return dirs | 1 << dir;
	}
	unsigned char forcedNeighbours(const Coordinate &coord, const int dir)
	{
		if (dir > 7)
			return 0;

		unsigned char dirs = 0;
#define ENTERABLE(n) isPassable ( nextCoordinate (coord, (dir + (n)) % 8))
		if (dirIsDiagonal(dir)) {
			if (!implies(ENTERABLE(6), ENTERABLE(5)))
				dirs = addDirectionToSet(dirs, (dir + 6) % 8);
			if (!implies(ENTERABLE(2), ENTERABLE(3)))
				dirs = addDirectionToSet(dirs, (dir + 2) % 8);

		}
		else {
			if (!implies(ENTERABLE(7), ENTERABLE(6)))
				dirs = addDirectionToSet(dirs, (dir + 7) % 8);
			if (!implies(ENTERABLE(1), ENTERABLE(2)))
				dirs = addDirectionToSet(dirs, (dir + 1) % 8);
		}
#undef ENTERABLE	

		return dirs;
	}


public:

	const Coordinate indexToCoordinate(const int index)
	{
		return Coordinate(index%gridWidth, index / gridWidth);
	}

	bool isPassable(const Coordinate &c)
	{
		int index = gridIndex(c);
		if (index == -1)
			return false;
		return  !(gridData[index / 8] & (1 << (index % 8)));
	}
	int getGridWidth()
	{
		return gridWidth;
	}
	int getGridHeight()
	{
		return gridHeight;
	}
	bool isCoordinateBlocked(const Coordinate &c)
	{
		return isPassable(c);
	}
	inline void clearAllPreJumpData(const short x, const short y)
	{
		memset(&preprocessedJumpPoints[y*gridWidth * 8 + x * 8], -1, 8 * 4);
	}
	void flushReProcess()
	{
		return;

	}
	void verifyreProcessing()
	{
		return;
		for (int y = 0; y<gridHeight; y++)
			for (int x = 0; x<gridWidth; x++)
				if (isPassable(Coordinate(x, y)))
					for (int dir = 0; dir<8; dir++)
					{
						Coordinate endP(x, y);
						int index = jumpOld(Coordinate(x, y), dir, &endP);
						unsigned int val = gridIndex(endP) | (index == -1 ? 0 : (1 << 31));
						if (preprocessedJumpPoints[y*gridWidth * 8 + x * 8 + dir] != val &&preprocessedJumpPoints[y*gridWidth * 8 + x * 8 + dir] != -1)
						{
							printf("Incorrectly stored jump point\n");
							system("pause");
						}
					}
	}
	void reProcessGrid(int lx, int rx, int ty, int by)
	{
		memset(testedGrid, 0, gridWidth*gridHeight / 8 + 1);

		for (int y = ty - 1; y<by + 1; y++)
			for (int x = lx - 1; x<rx + 1; x++)
				for (int dir = 0; dir<8; dir += 2)
				{
					Coordinate startP1(x, y);
					int index1 = gridIndex(startP1);

					while (index1 != -1 && isPassable(startP1))//  preprocessedJumpPoints[startP1.y*gridWidth*8+startP1.x*8+ dir]!= -1
					{
						clearAllPreJumpData(startP1.x, startP1.y);
						//setChecked(index1);

						//preprocessedJumpPoints[startP1.y*gridWidth*8+startP1.x*8+ dir]=-1;
						int dirSet[] = { (dir + 7) % 8, (dir + 1) % 8 };
						for (int id = 0; id<2; id++)
						{
							int dir2 = dirSet[id];
							Coordinate startP(startP1.x, startP1.y);
							startP = nextCoordinate(startP, dir2);
							int index = gridIndex(startP);


							while (index != -1 && !isChecked(index) && isPassable(startP))// dir!= dir2&& (dir2+4)%8!= dir &&dir2%2==0 &&  preprocessedJumpPoints[startP.y*gridWidth*8+startP.x*8+dir2]!=-1 memcmp(compareVal,&preprocessedJumpPoints[startP.y*gridWidth*8+startP.x*8],4*8)
							{

								clearAllPreJumpData(startP.x, startP.y);
								setChecked(index);
								startP = nextCoordinate(startP, dir2);
								index = gridIndex(startP);


							}
						}
						startP1 = nextCoordinate(startP1, dir);
						index1 = gridIndex(startP1);
						/*if (isChecked(index1))
						break;
						else
						setChecked(index1);*/
					}
				}
		verifyreProcessing();


	}
	void backupPreProcess()
	{
		preprocessedJumpPointsBackup = new unsigned int[gridWidth*gridHeight * 8];
		memcpy(preprocessedJumpPointsBackup, preprocessedJumpPoints, gridWidth*gridHeight * 8 * 4);
	}
	void useBackupData()
	{
		memcpy(preprocessedJumpPoints, preprocessedJumpPointsBackup, gridWidth*gridHeight * 8 * 4);
	}
	void preProcessGrid()
	{
		memset(preprocessedJumpPoints, -1, gridWidth*gridHeight * 8 * 4);
		clearChecked();
		
		for (int y = 0; y<gridHeight; y++)
			for (int x = 0; x<gridWidth; x++)
				if (isPassable(Coordinate(x, y)))
					for (int dir = 0; dir<8; dir++)
						if (preprocessedJumpPoints[y*gridWidth * 8 + x * 8 + dir] == -1)
						{
							Coordinate endP(x, y);
							int index = jumpOld(Coordinate(x, y), dir, &endP);
							if (index != -1)
								endP = indexToCoordinate(index);

							unsigned int val = gridIndex(endP) | (index == -1 ? 0 : (1 << 31));
							preprocessedJumpPoints[y*gridWidth * 8 + x * 8 + dir] = val;
						}
	}
	JPS_PLUS(char * grid, int width, int height) : PathFindingAlgorithm("JPS+", AT_JPS_PLUS)
	{
		gridData						= grid;
		gridWidth						= width;
		gridHeight						= height;
		testedGrid						= new char[gridWidth*gridHeight / 8 + 1];
		preprocessedJumpPoints			= new unsigned int[gridWidth*gridHeight * 8];
		preprocessedJumpPointsBackup	= 0;

		memset(preprocessedJumpPoints, -1, gridWidth*gridHeight * 8 * 4);
	}
	void copyPreprocessedJumpPoints(unsigned int * data)
	{
		memcpy(preprocessedJumpPoints, data, gridWidth*gridHeight * 8 * 4);
	}
	unsigned int* getPreprocessedData()
	{
		unsigned int*data = new unsigned int[gridWidth*gridHeight * 8];
		memcpy(data, preprocessedJumpPoints, gridWidth*gridHeight * 8 * 4);
		return data;
	}
	~JPS_PLUS()
	{
		delete[] testedGrid;
		delete[] preprocessedJumpPoints;
		if (preprocessedJumpPointsBackup)
			delete preprocessedJumpPointsBackup;
	}
	unsigned char naturalNeighbours(const int dir)
	{
		if (dir == NO_DIRECTION)
			return 255;

		unsigned char dirs = 0;
		dirs = addDirectionToSet(dirs, dir);
		if (dirIsDiagonal(dir)) {
			dirs = addDirectionToSet(dirs, (dir + 1) % 8);
			dirs = addDirectionToSet(dirs, (dir + 7) % 8);
		}
		return dirs;
	}
	unsigned char nextDirectionInSet(unsigned char *dirs)
	{
		for (int i = 0; i < 8; i++) {
			char bit = 1 << i;
			if (*dirs & bit) {
				*dirs ^= bit;
				return i;
			}
		}
		return NO_DIRECTION;
	}
	inline int mag(int i)
	{
		if (i<1)
			return -1;
		return 1;
	}
	int jump2(const Coordinate &To, Coordinate &from, bool isJumpPoint, int dir)
	{
		//if (!isPassable(c))
		//	return -1;
		int minX = min(from.x, To.x);
		int maxX = max(from.x, To.x);
		int minY = min(from.y, To.y);
		int maxY = max(from.y, To.y);
		//Passes through the destination as a straight line
		Coordinate next = nextCoordinate(Coordinate(0, 0), dir);

		if (dir % 2 == 0)
		{
			if (minX <= eX && maxX >= eX && minY <= eY && maxY >= eY)
				return endIndex;
		}
		//Passes the destination as a diagonal line insert a new point
		else if ((minX <= eX && maxX >= eX) || (minY <= eY && maxY >= eY))
			if (abs(from.x - eX) == abs(from.y - eY) && (minX <= eX && maxX >= eX) && (minY <= eY && maxY >= eY))
				return endIndex;
			else
				if (abs(from.x - eX) && abs(from.y - eY)) // make sure that the point is offset and isnt on a straight line
					if (abs(from.x - eX)<abs(from.y - eY))
					{
			if (mag(eY - from.y) == next.y && minX <= eX && maxX >= eX)
				return gridIndex(Coordinate(eX, from.y - abs(from.x - eX) *mag(from.y - eY)));
					}
					else if (mag(eX - from.x) == next.x && minY <= eY && maxY >= eY)
						return gridIndex(Coordinate(from.x - abs(from.y - eY) *mag(from.x - eX), eY));

		if (isJumpPoint)
		{
			return gridIndex(To);
		}
		return -1;
	}

	int jumpOld(const Coordinate &c, const char dir, Coordinate *lastC)
	{
		Coordinate nc = nextCoordinate(c, dir);
		if (!isPassable(nc))
			return -1;
		if (lastC)
			*lastC = nc;

		int index = gridIndex(nc);
		unsigned char dirs;
		if (dirs = forcedNeighbours(nc, dir))
			return index;
		if (dirIsDiagonal(dir))
		{
			int next = jumpOld(nc, (dir + 7) % 8, 0);
			if (next >= 0)
				return index;
			next = jumpOld(nc, (dir + 1) % 8, 0);
			if (next >= 0)
				return index;
		}

		return jumpOld(nc, dir, lastC);
	}
	void setChecked(const int index)
	{
		testedGrid[index / 8] = testedGrid[index / 8] | (1 << (index % 8));
	}
	bool isChecked(const int index)
	{
		return (testedGrid[index / 8] & (1 << (index % 8)));
	}
	void clearChecked()
	{
		memset(testedGrid, 0, gridWidth*gridHeight / 8 + 1);
	}
	void findSolution(int sX, int sY, int _eX, int _eY, vector<Coordinate> & solution)
	{
		eX = _eX;
		eY = _eY;


		solution.clear();

		endIndex = gridIndex(Coordinate(eX, eY));

		if (!(sX >= 0 && sX<gridWidth &&
			sY >= 0 && sY<gridHeight &&
			eX >= 0 && eX<gridWidth &&
			eY >= 0 && eY<gridHeight &&
			isPassable(Coordinate(sX, sY)) && isPassable(Coordinate(eX, eY))))
		{
			//printf("Invalid coordinates\n");
			return;
		}
		if (sX == eX && sY == eY)
			return;
		//Ready the openlist, closed list and node container for search
		openListBh.clear();
		nodesC.reset();

		//Insert the start nodes into the openList
		Node* startNode = nodesC.getNewNode(Coordinate(sX, sY), eX, eY, 8, 0);
		openListBh.Insert(startNode);
		setChecked(gridIndex(startNode->pos));

		//Keep iterating over openlist until a solution is found or list is empty
		while (openListBh.Count())
		{
			Node* currentNode = openListBh.PopMax();

			unsigned char dirs = forcedNeighbours(currentNode->pos, currentNode->dir) | naturalNeighbours(currentNode->dir);


			for (int dir = 0; dir < 8; dir++)
			{
				if ((1 << dir)&dirs)
				{
					int val = preprocessedJumpPoints[currentNode->pos.y*gridWidth * 8 + currentNode->pos.x * 8 + dir];

					int index = -1;
					const unsigned int comparitor = -1;
					//If a grid cell doesn't have a lookup value create it
					unsigned int & jpsval = preprocessedJumpPoints[currentNode->pos.y*gridWidth * 8 + currentNode->pos.x * 8 + dir];
					if (jpsval == comparitor  &&isPassable(currentNode->pos))//
					{
						Coordinate endP(currentNode->pos.x, currentNode->pos.y);
						int index = jumpOld(currentNode->pos, dir, &endP);
						if (index != -1)
							endP = indexToCoordinate(index);

						unsigned int val = gridIndex(endP) | (index == -1 ? 0 : (1 << 31));
						jpsval = val;
					}


					if (jpsval != comparitor)
					{
						Coordinate jumpP = indexToCoordinate(jpsval&(0x7FFFFFFF));
						index = jump2(jumpP, currentNode->pos, jpsval & 0x80000000, dir);
					}
					if (inBounds(index))
					{
						Coordinate CoordinateNewC = indexToCoordinate(index);

						if (index == endIndex)
						{
							Coordinate end(eX, eY);
							solution.push_back(end);
							Node*solutionNode = currentNode;
							while (solutionNode)
							{
								solution.push_back(solutionNode->pos);
								solutionNode = solutionNode->parent;
							}
							clearChecked();
							return;
						}

						if (!isChecked(index))
						{
							openListBh.Insert(nodesC.getNewNode(CoordinateNewC, eX, eY, dir, currentNode));
							setChecked(index);
						}
						else
						{
							Node * t = nodesC.getNewNode(CoordinateNewC, eX, eY, dir, currentNode);
							openListBh.InsertSmaller(t);
						}
					}
				}
			}

		}
		clearChecked();
	}


};