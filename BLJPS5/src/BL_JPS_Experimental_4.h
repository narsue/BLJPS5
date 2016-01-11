#pragma once

//Author: Jason Traish jtraish@csu.edu.au narsue2@gmail.com
//Created: 2015 December 21


//------------------------//
//------Description-------//
//------------------------//
//This is is a variation of the Boundary Lookup optimisation of Jump Point Search (BLJPS)
//It speeds up the search by only storing an open list of forced neighbours.
//The start node is found using BLJPS3 method of quickly using stored lists of diagonal and cardinal forced neighbours
//The end node is checked for every new node with a very efficient check by prechecking what the open spaces of the destination grid index is on


//------------------------//
//----Compiler flags------//
//------------------------//
//The example shown below. Key: (.) is a free space. (x) is a blocked space. (S) is current position. (D) destination position.
//When DIAG_UNBLOCKED is defined the following movement is illegal
// .xD
// .Sx
// ...
// with this flag enabled both cardinal directions (up and right) must be free to step to the north east.
//If DIAG_UNBLOCKED is undefined the above example is legal.
//Compiler flag: USE_BLJPS_PREPROCESS_BLJPS4 specifies to use BLJPS to speed up the preprocessing of jump points. Turning this off will use a recursive slower method to obtain jump points in preprocessing.

#include <stdio.h>
#include <string.h>
#include "PathFindingAlgorithm.h"
#include "DiagJumpNode.h"
#include <set>
using namespace std;
#define BLJPSEXP4_REVISION "-r19"
#define USE_BLJPS_PREPROCESS_BLJPS4

#ifdef DIAG_UNBLOCKED
#define BL_JPS_OFFSET 0
#define BLJPSEXP4_ALG_NAME string("BLJPS-EXP4-UNBLOCKED")+string(BLJPSEXP4_REVISION)
#else
#define BL_JPS_OFFSET 1
#define BLJPSEXP4_ALG_NAME string("BLJPS-EXP4-BLOCKED")+string(BLJPSEXP4_REVISION)
#endif


//Directions
// N, NE, E, SE, S, SW, W, NW , StartPosition
#define NO_DIRECTION 8
struct GraphNode
{
	short x, y;
	short limits[8];
	char neighbouringCells;
	unsigned int id;
	GraphNode(short _x, short _y,unsigned int _id)
	{
		id = _id;
		x = _x;
		y = _y;
		neighbouringCells = 0;
		for (int i = 0; i < 8; i++)
			limits[i] = -1;
	}
	vector<GraphNode*> nextJumps[8];

};


typedef unsigned int costType;
struct BLJPS4_HeapElement
{
	costType fVal;
	char dir;
	GraphNode * gn;
	BLJPS4_HeapElement(GraphNode* _gn, costType _fVal, char _dir) : gn(_gn), fVal(_fVal), dir(_dir){}
};

#define CARD_COST 2378
#define DIAG_COST 3363
#define DIAG_DIFF 985

//The heap code used here is a modified version from the SUBGOAL optimal algorithm submitted to the pathfinding competition. Accessed from http://movingai.com/GPPC/
class BL_JPS_Experimental_4 : public  PathFindingAlgorithm
{
private:
	bool isRawPassable(const Coordinate &c)
	{
		return isPassable(c);
	}
	bool isPassable(const Coordinate &c)
	{
		int index = gridIndex(c);
		if (index == -1)
			return false;
		return gridData[index];// (gridData[index / 8] & (1 << (index & 7)));//bits[index];//
	}
	bool canReplaceTop;
	std::vector<BLJPS4_HeapElement> theHeap;
	GraphNode **parents;
	costType *costs;

	bool IsOpen(int sg){ return (testedGrid[(sg >> 3)] & (1 << (sg & 7))) != 0; }
	void SetOpen(int sg){ testedGrid[sg >> 3] |= (1 << (sg & 7)); }
	void SetClosed(int sg){ testedGrid[sg >> 3] &= ~(1 << (sg & 7)); }





	inline void AddToOpen(GraphNode * gn, costType fVal, char dir)
	{
		if (canReplaceTop)	// If the top element of the heap can be replaced,
		{
			theHeap[0] = (BLJPS4_HeapElement(gn, fVal, dir));	// .. replace it
			PercolateDown(0);		// and percolate down
			canReplaceTop = false;	// the top element is no longer replaceable
		}
		else
		{	// add element as usual
			theHeap.push_back(BLJPS4_HeapElement(gn, fVal, dir));
			PercolateUp(theHeap.size() - 1);
		}
	}

	inline void PopMin()
	{
		canReplaceTop = true;	// Don't pop it immediately, just mark it as replaceable
	}
	inline void PopReplacableTop()
	{	// Force the heap to remove the top element, without waiting for a replacement
		if (canReplaceTop)
		{
			theHeap[0] = theHeap.back();
			theHeap.pop_back();
			if (theHeap.size()>0)
				PercolateDown(0);
			canReplaceTop = false;
		}
	}
	inline BLJPS4_HeapElement GetMin()
	{
		return theHeap.front();
	}

	inline void InsertSmaller(GraphNode * gn, costType fVal, char dir)
	{
		for (unsigned int i = 0; i<theHeap.size(); i++)
			if (gn == theHeap[i].gn)
			{
				if (fVal<theHeap[i].fVal)
				{
					theHeap[i] = BLJPS4_HeapElement(gn, fVal, dir);
					PercolateUp(i);
					return ;
				}
				else
					return ;
			}
		return ;

	}
	inline void PercolateUp(int index)
	{
		BLJPS4_HeapElement elem = theHeap[index];
		int parent;
		parent = (index - 1) >> 1;

		while (index > 0 && theHeap[parent].fVal > elem.fVal)
		{
			theHeap[index] = theHeap[parent];
			index = parent;
			parent = (index - 1) >> 1;
		}

		theHeap[index] = elem;
	}
	inline void PercolateDown(int index)
	{
		BLJPS4_HeapElement elem = theHeap[index];
		int maxSize = theHeap.size();

		while (true)
		{
			int child1 = (index << 1) + 1;
			if (child1 >= maxSize)
				break;

			int child2 = child1 + 1;

			// If the first child has smaller key
			if (child2 == maxSize || theHeap[child1].fVal <= theHeap[child2].fVal)
			{
				if (theHeap[child1].fVal < elem.fVal)
				{
					theHeap[index] = theHeap[child1];
					index = child1;
				}
				else
					break;
			}

			else if (theHeap[child2].fVal < elem.fVal)
			{
				theHeap[index] = theHeap[child2];
				index = child2;
			}
			else
				break;
		}

		theHeap[index] = elem;
	}
	costType hCost(short xf, short yf, short xt, short yt)
	{
		int dx = (xf>xt) ? (xf - xt) : (xt - xf);	int dy = (yf>yt) ? (yf - yt) : (yt - yf);
		return (dx>dy) ? (dx*CARD_COST + dy*DIAG_DIFF) : (dy*CARD_COST + dx*DIAG_DIFF);
	}

	//Table of flags indicating is a map location has been searched (Closed List)
	char *testedGrid;

	//Boundary lookup tables for the x and y axis
	vector<vector<short> > xBoundaryPoints, yBoundaryPoints;

	vector<vector<pair<short, short> > > jumpLookup[4];
	vector<vector<DiagJumpNode> > diagJumpLookup[4];
	vector<GraphNode*> allGraphNodes;
	char *graphCells;
	GraphNode**graphLookup;

	//Map data
	vector<bool> &gridData;
	int gridWidth, gridHeight;

	//Goal node position and index
	int eX, eY, endIndex, eXSpace[2], eYSpace[2], eXSpaceId, eYSpaceId;


	bool inBounds(const int index)
	{
		return index < gridWidth*gridHeight&&index >= 0;
	}
	int gridIndex(const Coordinate &c)
	{
		if (c.x < 0 || c.x >= gridWidth || c.y < 0 || c.y >= gridHeight)
			return -1;
		return (c.y*gridWidth) + c.x;
	}

	Coordinate nextCoordinate(const Coordinate& c, const int dir)
	{
		static char dirMov[] = { 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0, -1, -1, 0, 0 };
		return Coordinate(c.x + dirMov[dir * 2], c.y + dirMov[dir * 2 + 1]);
	}
	Coordinate nextCoordinate(short x, short y, const int dir, int amount)
	{
		static char dirMov[] = { 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0, -1, -1, 0, 0 };
		return Coordinate(x + dirMov[dir * 2] * amount, y + dirMov[dir * 2 + 1] * amount);
	}
	inline bool dirIsDiagonal(const int dir)
	{
		return (dir & 1);
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
#define ENTERABLE(n) isPassable ( nextCoordinate (coord, (dir + (n)) &7))
		if (dirIsDiagonal(dir)) {
#ifdef DIAG_UNBLOCKED
			if (ENTERABLE(5) && ENTERABLE(3))
			{
				if (ENTERABLE(1)  && !ENTERABLE(2))
					dirs = addDirectionToSet(dirs, (dir + 1) & 7);
				if (ENTERABLE(7)  && !ENTERABLE(6))
					dirs = addDirectionToSet(dirs, (dir + 7) & 7);
			}
#else
			if (!implies(ENTERABLE(6), ENTERABLE(5)))
				dirs = addDirectionToSet(dirs, (dir + 6) & 7);
			if (!implies(ENTERABLE(2), ENTERABLE(3)))
				dirs = addDirectionToSet(dirs, (dir + 2) & 7);
#endif
		}
		else {
#ifdef DIAG_UNBLOCKED
			if (ENTERABLE(4))
			{
				if (!implies(ENTERABLE(2), ENTERABLE(3)))
				{
					dirs = addDirectionToSet(dirs, (dir + 2) & 7);
					if (ENTERABLE(1))
						dirs = addDirectionToSet(dirs, (dir + 1) & 7);
				}
				if (!implies(ENTERABLE(6), ENTERABLE(5)))
				{
					dirs = addDirectionToSet(dirs, (dir + 6) & 7);
					if (ENTERABLE(7))

						dirs = addDirectionToSet(dirs, (dir + 7) & 7);
				}
			}
#else
			if (!implies(ENTERABLE(7), ENTERABLE(6)))
				dirs = addDirectionToSet(dirs, (dir + 7) & 7);
			if (!implies(ENTERABLE(1), ENTERABLE(2)))
				dirs = addDirectionToSet(dirs, (dir + 1) & 7);
#endif
		}
#undef ENTERABLE

		return dirs;
	}
	bool t1(int n,int dir,char data)
	{
		return  (data & (1 << ((dir + n) & 7))) != 0;
	}
	unsigned char forcedNeighbours2(const int dir, char data)
	{
		if (dir > 7)
			return 0;

		unsigned char dirs = 0;
#define ENTERABLE(n) isPassable ( data & (1<<((dir + n) &7)))
		if (dirIsDiagonal(dir)) {
#ifdef DIAG_UNBLOCKED
				if ( t1(dir, 5, data) && t1(dir, 3, data))
				{
					if (t1(dir, 1, data) && !t1(dir, 2, data) )
						dirs = addDirectionToSet(dirs, (dir + 1) & 7);
					if (t1(dir, 7, data) && !t1(dir, 6, data))
						dirs = addDirectionToSet(dirs, (dir + 7) & 7);
				}
#else
			if (!implies(t1(6, dir, data), t1(5, dir, data)))
				dirs = addDirectionToSet(dirs, (dir + 6) & 7);
			if (!implies(t1(2, dir, data), t1(3, dir, data)))
				dirs = addDirectionToSet(dirs, (dir + 2) & 7);
#endif
		}
		else {
#ifdef DIAG_UNBLOCKED
			if (t1(dir, 4, data))
			{
				if (!implies(t1(dir, 2, data), t1(dir, 3, data)))
				{
					dirs = addDirectionToSet(dirs, (dir + 2) & 7);
					if (t1(dir, 1, data))
						dirs = addDirectionToSet(dirs, (dir + 1) & 7);
				}
				if (!implies(t1(dir, 6, data), t1(dir, 5, data)))
				{
					dirs = addDirectionToSet(dirs, (dir + 6) & 7);
					if (t1(dir, 7, data))

						dirs = addDirectionToSet(dirs, (dir + 7) & 7);
				}
			}
#else
			if (!implies(t1(7, dir, data), t1(6, dir, data)))
				dirs = addDirectionToSet(dirs, (dir + 7) & 7);
			if (!implies(t1(1, dir, data), t1(2, dir, data)))
				dirs = addDirectionToSet(dirs, (dir + 1) & 7);
#endif
		}
#undef ENTERABLE

		return dirs;
	}
public:

	bool isCoordinateBlocked(const Coordinate &c)
	{
		return isPassable(c);
	}
	void flushReProcess()
	{
		for (int y = 0; y < gridHeight; y++)
		{
			if (xBoundaryPoints[y].size() == 0)
			{
				bool currentPassable = false;
				xBoundaryPoints[y].clear();
				for (int x = 0; x < gridWidth; x++)
				{
					if (isRawPassable(Coordinate(x, y)) != currentPassable)
					{
						xBoundaryPoints[y].push_back(x);
						currentPassable = !currentPassable;
					}
				}
				//if (currentPassable)
				xBoundaryPoints[y].push_back(gridWidth);
			}
		}

		for (int x = 0; x < gridWidth; x++)
		{
			if (yBoundaryPoints[x].size() == 0)
			{
				bool currentPassable = false;
				yBoundaryPoints[x].clear();

				for (int y = 0; y < gridHeight; y++)
				{
					if (isRawPassable(Coordinate(x, y)) != currentPassable)
					{
						yBoundaryPoints[x].push_back(y);
						currentPassable = !currentPassable;
					}
				}
				//if (currentPassable)
				yBoundaryPoints[x].push_back(gridHeight);
			}
		}

	}
	void reProcessGrid(int lx, int rx, int ty, int by)
	{
		for (int y = ty; y < by; y++)
			xBoundaryPoints[y].clear();

		for (int x = lx; x < rx; x++)
			yBoundaryPoints[x].clear();
	}
	void backupPreProcess()
	{
		//xBoundaryPointsBackup=xBoundaryPoints;
		//yBoundaryPointsBackup=yBoundaryPoints;
	}
	void useBackupData()
	{
		//xBoundaryPoints=xBoundaryPointsBackup;
		//yBoundaryPoints=yBoundaryPointsBackup;
	}
	void insertionOrdered(vector<pair<short, short> > & vec, pair<short, short> v)
	{
		for (unsigned int i = 0; i < vec.size(); i++)
			if (vec[i].second > v.second)
			{
				vec.insert(vec.begin() + i, v);
				return;
			}
		vec.push_back(v);
	}
	void dumpJumpPointData(FILE* fp, vector<pair<short, short> > & vec)
	{
		short numBoundaries = vec.size();
		fwrite(&numBoundaries, 2, 1, fp);
		for (unsigned int i = 0; i<vec.size(); i++)
			fwrite(&vec[i], 4, 1, fp);
	}
	void readJumpPointData(FILE* fp, vector<pair<short, short> > & vec)
	{
		short numBoundaries = 0;
		fread(&numBoundaries, 2, 1, fp);

		for (int i = 0; i<numBoundaries; i++)
		{
			pair<short, short> temp;
			fread(&temp, 4, 1, fp);
			vec.push_back(temp);
		}
	}
	void dumpDiagonalJumpList(FILE* fp)
	{
		for (int dir = 0; dir < 4; dir++)
		{
			vector<vector<DiagJumpNode> > & currentDiagList = diagJumpLookup[dir];
			unsigned short currentDiagListSize = currentDiagList.size();
			fwrite(&currentDiagListSize, 2, 1, fp);
			for (int dimId = 0; dimId < currentDiagListSize; dimId++)
			{
				vector<DiagJumpNode> & currentDiagList2 = currentDiagList[dimId];
				unsigned short currentDiagListSize2 = currentDiagList2.size();
				fwrite(&currentDiagListSize2, 2, 1, fp);
				for (int diagId = 0; diagId < currentDiagListSize2; diagId++)
				{
					DiagJumpNode & diagNode = currentDiagList2[diagId];
					fwrite(&diagNode.sI, 2, 1, fp);
					fwrite(&diagNode.dI, 2, 1, fp);
					unsigned short diagNodeJumps = diagNode.jumps.size();
					fwrite(&diagNodeJumps, 2, 1, fp);
					for (int jumpId = 0; jumpId < diagNodeJumps; jumpId++)
						fwrite(&diagNode.jumps[jumpId], 2, 1, fp);
				}
			}
		}
	}
	void readDiagonalJumpList(FILE* fp)
	{
		for (int dir = 0; dir < 4; dir++)
		{
			vector<vector<DiagJumpNode> > & currentDiagList = diagJumpLookup[dir];
			unsigned short currentDiagListSize = 0;// currentDiagList.size();
			fread(&currentDiagListSize, 2, 1, fp);
			currentDiagList.resize(currentDiagListSize);
			for (int dimId = 0; dimId < currentDiagListSize; dimId++)
			{
				vector<DiagJumpNode> & currentDiagList2 = currentDiagList[dimId];
				unsigned short currentDiagListSize2 = 0;// currentDiagList2.size();
				fread(&currentDiagListSize2, 2, 1, fp);
				currentDiagList2.resize(currentDiagListSize2);
				for (int diagId = 0; diagId < currentDiagListSize2; diagId++)
				{
					DiagJumpNode & diagNode = currentDiagList2[diagId];
					fread(&diagNode.sI, 2, 1, fp);
					fread(&diagNode.dI, 2, 1, fp);
					unsigned short diagNodeJumps = 0;// diagNode.jumps.size();
					fread(&diagNodeJumps, 2, 1, fp);
					diagNode.jumps.resize(diagNodeJumps);
					for (int jumpId = 0; jumpId < diagNodeJumps; jumpId++)
						fread(&diagNode.jumps[jumpId], 2, 1, fp);
				}
			}
		}
	}
	void dumpGraphNodesToFile(FILE* fp)
	{
		unsigned short numberOfGraphNodes = allGraphNodes.size();
		fwrite(&numberOfGraphNodes, 2, 1, fp);
		for (int graphNodeId = 0; graphNodeId < numberOfGraphNodes; graphNodeId++)
		{
			GraphNode &gNode = *allGraphNodes[graphNodeId];
			fwrite(&gNode.id, sizeof(gNode.id), 1, fp);
			fwrite(&gNode.limits, sizeof(gNode.limits[0])*8, 1, fp);
			fwrite(&gNode.neighbouringCells, sizeof(gNode.neighbouringCells), 1, fp);
			fwrite(&gNode.x, sizeof(gNode.x), 1, fp);
			fwrite(&gNode.y, sizeof(gNode.y), 1, fp);
			for (int dir = 0; dir < 8; dir++)
			{
				unsigned short numberOfJumps = gNode.nextJumps[dir].size();
				fwrite(&numberOfJumps, 2, 1, fp);
				for (int jumpId = 0; jumpId < numberOfJumps; jumpId++)
					fwrite(&gNode.nextJumps[dir][jumpId]->id, sizeof(gNode.id), 1, fp);
			}
		}
	}
	void readGraphNodesToFile(FILE* fp)
	{
		unsigned short numberOfGraphNodes = 0;// allGraphNodes.size();
		fread(&numberOfGraphNodes, 2, 1, fp);
		allGraphNodes.resize(numberOfGraphNodes);

		if (testedGrid)
			delete[] testedGrid;
		testedGrid = new char[allGraphNodes.size() / 8 + 1];

		memset(graphCells, 0, gridWidth*gridHeight / 8 + 1);
		memset(graphLookup, 0, sizeof(GraphNode*)*gridWidth*gridHeight);


		if (parents)
			delete[] parents;
		if (costs)
			delete[] costs;
		parents = new GraphNode*[allGraphNodes.size()];
		costs = new costType[allGraphNodes.size()];


		for (int graphNodeId = 0; graphNodeId < numberOfGraphNodes; graphNodeId++)
			allGraphNodes[graphNodeId] = new GraphNode(0, 0, graphNodeId);
		for (int graphNodeId = 0; graphNodeId < numberOfGraphNodes; graphNodeId++)
		{
			GraphNode &gNode = *allGraphNodes[graphNodeId];
			fread(&gNode.id, sizeof(gNode.id), 1, fp);
			fread(&gNode.limits, sizeof(gNode.limits[0]) * 8, 1, fp);
			fread(&gNode.neighbouringCells, sizeof(gNode.neighbouringCells), 1, fp);
			fread(&gNode.x, sizeof(gNode.x), 1, fp);
			fread(&gNode.y, sizeof(gNode.y), 1, fp);

			int index = gridIndex(Coordinate(gNode.x, gNode.y));
			graphLookup[gNode.y*gridWidth + gNode.x] = allGraphNodes[graphNodeId];
			graphCells[index / 8] |= (1 << (index & 7));

			for (int dir = 0; dir < 8; dir++)
			{
				unsigned short numberOfJumps = 0;// gNode.nextJumps[dir].size();
				fread(&numberOfJumps, 2, 1, fp);

				gNode.nextJumps[dir].resize(numberOfJumps);
				for (int jumpId = 0; jumpId < numberOfJumps; jumpId++)
				{
					unsigned int id = 0;
					fread(&id, sizeof(gNode.id), 1, fp);
					gNode.nextJumps[dir][jumpId] = allGraphNodes[id];
				}
			}
		}
	}
	void dumpBoundaryTable(FILE* fp)
	{
		for (int y = 0; y<gridHeight; y++)
		{
			short numBoundaries = xBoundaryPoints[y].size() - 1;
			fwrite(&numBoundaries, 2, 1, fp);
			for (unsigned int i = 0; i<xBoundaryPoints[y].size() - 1; i++)
				fwrite(&xBoundaryPoints[y][i], 2, 1, fp);
		}
		for (int x = 0; x<gridWidth; x++)
		{
			short numBoundaries = yBoundaryPoints[x].size() - 1;
			fwrite(&numBoundaries, 2, 1, fp);
			for (unsigned int i = 0; i<yBoundaryPoints[x].size() - 1; i++)
				fwrite(&yBoundaryPoints[x][i], 2, 1, fp);
		}
	}
	void readBoundaryTable(FILE* fp)
	{
		for (int y = 0; y<gridHeight; y++)
		{
			xBoundaryPoints.push_back(vector<short>());
			short numBoundaries = 0;
			fread(&numBoundaries, 2, 1, fp);
			for (int i = 0; i<numBoundaries; i++)
			{
				short tempVal = 0;
				fread(&tempVal, 2, 1, fp);
				xBoundaryPoints[y].push_back(tempVal);
			}
			xBoundaryPoints[y].push_back(gridWidth);
		}
		for (int x = 0; x<gridWidth; x++)
		{
			yBoundaryPoints.push_back(vector<short>());
			short numBoundaries = 0;
			fread(&numBoundaries, 2, 1, fp);
			for (int i = 0; i<numBoundaries; i++)
			{
				short tempVal = 0;
				fread(&tempVal, 2, 1, fp);
				yBoundaryPoints[x].push_back(tempVal);
			}
			yBoundaryPoints[x].push_back(gridHeight);
		}
	}

	void dumpCardinalJumpPoints(FILE* fp)
	{
		for (int y = 0; y < gridHeight; y++)
		{
			dumpJumpPointData(fp, jumpLookup[1][y]);
			dumpJumpPointData(fp, jumpLookup[3][y]);
		}
		for (int x = 0; x < gridWidth; x++)
		{
			dumpJumpPointData(fp, jumpLookup[2][x]);
			dumpJumpPointData(fp, jumpLookup[0][x]);
		}
	}
	void readCardinalJumpPoints(FILE* fp)
	{
		for (int y = 0; y < gridHeight; y++)
		{
			jumpLookup[1].push_back(vector<pair<short, short> >());
			jumpLookup[3].push_back(vector<pair<short, short> >());
			readJumpPointData(fp, jumpLookup[1].back());
			readJumpPointData(fp, jumpLookup[3].back());

		}
		for (int x = 0; x < gridWidth; x++)
		{
			jumpLookup[2].push_back(vector<pair<short, short> >());
			jumpLookup[0].push_back(vector<pair<short, short> >());
			readJumpPointData(fp, jumpLookup[2].back());
			readJumpPointData(fp, jumpLookup[0].back());
		}
	}


	void dumpPreprocessedDataToFile(const char * fileName)
	{
		FILE * fp = fopen(fileName, "wb");
		if (fp == 0)
		{
			printf("Unable to open preprocessing file %s\n", fileName);
			return;
		}

		dumpBoundaryTable(fp);

		dumpCardinalJumpPoints(fp);

		dumpDiagonalJumpList(fp);

		dumpGraphNodesToFile(fp);

		fclose(fp);
	}
	void readPreprocessedDataToFile(const char * fileName)
	{
		FILE * fp = fopen(fileName, "rb");
		if (fp == 0)
		{
			printf("Unable to open preprocessing file %s\n", fileName);
			return;
		}

		readBoundaryTable(fp);

		readCardinalJumpPoints(fp);

		readDiagonalJumpList(fp);

		readGraphNodesToFile(fp);

		fclose(fp);
	}
	
	bool isGraphNode(short x,short y)
	{
		int index = gridIndex(Coordinate(x,y));
		if (index == -1)
			return false;
		return  (graphCells[index / 8] & (1 << (index & 7)))!=0;//bits[index];//
	}
	void setGraphNode(GraphNode* n)
	{
		int index = gridIndex(Coordinate(n->x,n->y));
		graphLookup[n->y*gridWidth + n->x] = n;
		graphCells[index / 8] |= (1 << (index & 7));
		allGraphNodes.push_back(n);

	}
	GraphNode * getGraphNode(short x,short  y)
	{
		return graphLookup[y*gridWidth + x];
	}
	//Builds the open diagonal lists quickly as SE and NW share the same gaps as do NE and SW
	void buildDiagOpenGaps(vector<DiagJumpNode> & diagList1, vector<DiagJumpNode> &diagList2, int diagIndex, int dir1, int dirId1)
	{
		//Gets the starting coordinates for a diagonal point
		int y = 0;
		int x = 0;

		if (dir1 == 1)
		{
			y = min(diagIndex, gridHeight - 1);
			x = diagIndex - y;
		}
		else//dir ==5
		{
			x = min(diagIndex, gridWidth - 1);
			y = x - diagIndex + (gridHeight - 1);
		}

		bool currentPassable = false;
		int sX = 0;
		while (x >= 0 && x < gridWidth && y >= 0 && y<gridHeight)
		{
			if (isPassable(Coordinate(x, y)) != currentPassable)
			{
				if (currentPassable)
				{
					diagList1.back().dI = x + (dir1 == 1 ? -1 : 1); //A diagonal region closes
					if (dir1 != 1)
						swap(diagList1.back().dI, diagList1.back().sI);
				}
				else
					diagList1.push_back(DiagJumpNode(x)); //A diagonal region opens
				currentPassable = !currentPassable;
			}
#ifdef DIAG_UNBLOCKED
			//If the corresponding cardinal directions must be unblocked before making a diagonal move
			//We must perform the following code. Where diagonal sections are now seperated not only by diagonal obstacles but 
			//by corresponding cardinal diagonal movement obstacles
			if (currentPassable && (!isPassable(Coordinate(x + (dir1 == 1 ? 1 : -1), y)) || !isPassable(Coordinate(x, y - 1))))
			{
				diagList1.back().dI = x;// +(dir1 == 1 ? -1 : 1); //A diagonal region closes
				if (dir1 != 1)
					swap(diagList1.back().dI, diagList1.back().sI);
				currentPassable = false;
			}
#endif
			x += dir1 == 1 ? 1 : -1;
			y--;
		}
		if (currentPassable)
		{
			diagList1.back().dI = x + (dir1 == 1 ? -1 : 1); //A diagonal region closes
			if (dir1 != 1)
				swap(diagList1.back().dI, diagList1.back().sI);
		}
		diagList2 = diagList1;
		int dir2 = (dir1 + 4) & 7;

		if (dir2 == 3 || dir2 == 5)
			std::reverse(diagList2.begin(), diagList2.end());
		for (unsigned int i = 0; i < diagList1.size(); i++)
		{
			int ret = 1;
			while (ret != -1)
			{
				Coordinate nc(diagList1[i].sI, diagIndex - diagList1[i].sI);
				if (dir1 != 1)
					nc = Coordinate(diagList1[i].dI, diagList1[i].dI - diagIndex + (gridHeight - 1));
				while (ret != -1)
				{
					ret = jumpDiag(nc, dir1);
					if (ret != -1)
						diagList1[i].jumps.push_back(ret);
				}
			}
		}

		for (unsigned int i = 0; i < diagList2.size(); i++)
		{
			int ret = 1;
			while (ret != -1)
			{
				Coordinate nc(diagList2[i].dI, diagIndex - diagList2[i].dI);
				if (dir1 != 1)
					nc = Coordinate(diagList2[i].sI, diagList2[i].sI - diagIndex + (gridHeight - 1));
				while (ret != -1)
				{
					ret = jumpDiag(nc, dir2);
					if (ret != -1)
						diagList2[i].jumps.push_back(ret);
				}
			}
		}
	}
	void preProcessBoundaryLists()
	{
		for (int y = 0; y<gridHeight; y++)
		{
			bool currentPassable = false;
			xBoundaryPoints.push_back(vector<short>());
			for (int x = 0; x<gridWidth; x++)
			{
				if (isRawPassable(Coordinate(x, y)) != currentPassable)
				{
					xBoundaryPoints[y].push_back(x);
					currentPassable = !currentPassable;
				}
			}
			//if (currentPassable)
			xBoundaryPoints[y].push_back(gridWidth);
		}

		for (int x = 0; x<gridWidth; x++)
		{
			bool currentPassable = false;
			yBoundaryPoints.push_back(vector<short>());
			for (int y = 0; y<gridHeight; y++)
			{
				if (isRawPassable(Coordinate(x, y)) != currentPassable)
				{
					yBoundaryPoints[x].push_back(y);
					currentPassable = !currentPassable;
				}
			}
			//if (currentPassable)
			yBoundaryPoints[x].push_back(gridHeight);
		}
	}
	void preProcessCardinalJumpPoints()
	{
		for (int y = 0; y < gridHeight; y++)
		{
			jumpLookup[1].push_back(vector<pair<short, short> >());
			jumpLookup[3].push_back(vector<pair<short, short> >());
			vector<pair<short, short> > & vec = jumpLookup[1].back();
			for (unsigned int xId = 0; xId < xBoundaryPoints[y].size(); xId += 2)
			{
				int x = xBoundaryPoints[y][xId];
				int index;
				do
				{
					index = jump(Coordinate(x, y), 2);

					if (index != -1)
					{
						int newX = indexToCoordinate(index).x;
						vec.push_back(pair<short, short>(x, newX));
						x = newX;
					}
				} while (index != -1);
			}
			vector<pair<short, short> > & vecB = jumpLookup[3].back();
			for (unsigned int xId = 1; xId < xBoundaryPoints[y].size(); xId += 2)
			{
				int x = xBoundaryPoints[y][xId] - 1;
				int index;
				do
				{
					index = jump(Coordinate(x, y), 6);

					if (index != -1)
					{
						int newX = indexToCoordinate(index).x;
						insertionOrdered(vecB, pair<short, short>(x, newX));
						x = newX;
					}
				} while (index != -1);
			}
		}
		for (int x = 0; x < gridWidth; x++)
		{
			jumpLookup[2].push_back(vector<pair<short, short> >());
			jumpLookup[0].push_back(vector<pair<short, short> >());
			vector<pair<short, short> > & vec = jumpLookup[2].back();
			for (unsigned int yId = 0; yId < yBoundaryPoints[x].size(); yId += 2)
			{
				int y = yBoundaryPoints[x][yId];
				int index;
				do
				{
					index = jump(Coordinate(x, y), 4);
					if (index != -1)
					{
						int newY = indexToCoordinate(index).y;
						vec.push_back(pair<short, short>(y, newY));
						y = newY;
					}
				} while (index != -1);
			}
			vector<pair<short, short> > & vecB = jumpLookup[0].back();
			for (unsigned int yId = 1; yId < yBoundaryPoints[x].size(); yId += 2)
			{
				int y = yBoundaryPoints[x][yId] - 1;
				int index;
				do
				{
					index = jump(Coordinate(x, y), 0);
					if (index != -1)
					{
						int newY = indexToCoordinate(index).y;
						insertionOrdered(vecB, pair<short, short>(y, newY));
						y = newY;
					}
				} while (index != -1);
			}

		}
	}
	void preProcessDiagonalJumpPoints()
	{
		int diagSize = gridWidth + (gridHeight - 1);

		for (int dir = 1; dir < 8; dir += 2)
			diagJumpLookup[(dir - 1) / 2].resize(diagSize);

		for (int diagId = 0; diagId < diagSize; diagId++)
			buildDiagOpenGaps(diagJumpLookup[(1 - 1) / 2][diagId], diagJumpLookup[(((1 + 4) & 7) - 1) / 2][diagId], diagId, 1, (1 - 1) / 2);
		for (int diagId = 0; diagId < diagSize; diagId++)
			buildDiagOpenGaps(diagJumpLookup[(7 - 1) / 2][diagId], diagJumpLookup[(((7 + 4) & 7) - 1) / 2][diagId], diagId, 7, (7 - 1) / 2);

		for (int x = 0; x < gridWidth; x++)
			for (int y = 0; y < gridHeight; y++)
			{
				if (isPassable(Coordinate(x, y)))
				{
					for (int dir = 0; dir < 8; dir += 1)
					{
						if (forcedNeighbours(Coordinate(x, y), dir))
						{
							if (!isGraphNode(x, y))
								setGraphNode(new GraphNode(x, y, allGraphNodes.size()));
						}
					}
				}
			}
	}
	void preProcessAllJumpPointNeighbouringCells()
	{
		for (unsigned int i = 0; i < allGraphNodes.size(); i++)
		{
			allGraphNodes[i]->neighbouringCells |= (char)isPassable(Coordinate(allGraphNodes[i]->x, allGraphNodes[i]->y - 1));
			allGraphNodes[i]->neighbouringCells |= isPassable(Coordinate(allGraphNodes[i]->x + 1, allGraphNodes[i]->y - 1)) << 1;
			allGraphNodes[i]->neighbouringCells |= isPassable(Coordinate(allGraphNodes[i]->x + 1, allGraphNodes[i]->y)) << 2;
			allGraphNodes[i]->neighbouringCells |= isPassable(Coordinate(allGraphNodes[i]->x + 1, allGraphNodes[i]->y + 1)) << 3;
			allGraphNodes[i]->neighbouringCells |= isPassable(Coordinate(allGraphNodes[i]->x, allGraphNodes[i]->y + 1)) << 4;
			allGraphNodes[i]->neighbouringCells |= isPassable(Coordinate(allGraphNodes[i]->x - 1, allGraphNodes[i]->y + 1)) << 5;
			allGraphNodes[i]->neighbouringCells |= isPassable(Coordinate(allGraphNodes[i]->x - 1, allGraphNodes[i]->y)) << 6;
			allGraphNodes[i]->neighbouringCells |= isPassable(Coordinate(allGraphNodes[i]->x - 1, allGraphNodes[i]->y - 1)) << 7;
		}
	}

	void preProcessJumpPointNodeLinks()
	{
		for (unsigned int i = 0; i < allGraphNodes.size(); i++)
		{
			char dirs = 0;
			for (int dirF = 0; dirF < 8; dirF++)
			{
				if (isPassable(nextCoordinate(Coordinate(allGraphNodes[i]->x, allGraphNodes[i]->y), (dirF + 4) & 7)))
				{
					dirs |= forcedNeighbours(Coordinate(allGraphNodes[i]->x, allGraphNodes[i]->y), dirF);
					dirs |= naturalNeighbours(dirF);
				}
			}
			for (int dirT = 0; dirT < 8; dirT++)
				if ((1 << dirT)&dirs)
				{
				/*	vector<GraphNode*> l;
					vector<GraphNode*> r;
					jumpAll(Coordinate(allGraphNodes[i]->x, allGraphNodes[i]->y), dirT, l);
					jumpAll2(Coordinate(allGraphNodes[i]->x, allGraphNodes[i]->y), dirT, r);*/
					//for (int c = 0; c < min(l.size(), r.size()); c++)
					//	if (l[c] != r[c])
					//		printf("1");

					jumpAll2(Coordinate(allGraphNodes[i]->x, allGraphNodes[i]->y), dirT, allGraphNodes[i]->nextJumps[dirT]);
				}
		}
	}
	void preProcessLimits()
	{
		//Set limits
		for (unsigned int i = 0; i < allGraphNodes.size(); i++)
		{
			for (int dir = 0; dir < 8; dir++)
			{
				Coordinate c(allGraphNodes[i]->x, allGraphNodes[i]->y);
				c = nextCoordinate(c, dir);
				short limitCount = 0;
#ifdef DIAG_UNBLOCKED
				Coordinate offset = nextCoordinate(Coordinate(0, 0), dir);
				while (isPassable(c) && (isPassable(Coordinate(c.x - offset.x, c.y)) && isPassable(Coordinate(c.x, c.y - offset.y))))
#else
				while (isPassable(c))
#endif
				{
					limitCount++;
					c = nextCoordinate(c, dir);
				}
				allGraphNodes[i]->limits[dir] = limitCount;
			}
		}
	}
	void preProcessGrid()
	{
		preprocessedData = true;

		preProcessBoundaryLists();

		preProcessCardinalJumpPoints();

		preProcessDiagonalJumpPoints();

		preProcessAllJumpPointNeighbouringCells();

		preProcessJumpPointNodeLinks();

		preProcessLimits();

		testedGrid = new char[allGraphNodes.size() / 8 + 1];
		parents = new GraphNode*[allGraphNodes.size()];
		costs = new costType[allGraphNodes.size()];
	}
#ifdef USE_BLJPS_PREPROCESS_BLJPS4
	short binarySearch(const vector<short> & v, short val)
	{
		short l = 0, r = v.size() - 1;
		short index = r / 2;
		while (1)
		{
			if (v[index] <= val && v[index + 1] > val)
				return  index;
			if (v[index] > val)
				r = index - 1;
			else
				l = index + 1;

			index = l + (r - l) / 2;

		}
		return -1;
	}
	pair<short, short> getEastEndPoshortReOpen(short x, short y)
	{
		if (y<0 || y >= gridHeight)
			return pair<short, short>(gridWidth, gridWidth);

		if (xBoundaryPoints[y][0]>x)
			return pair<short, short>(xBoundaryPoints[y][0], xBoundaryPoints[y][0]);

		short i = binarySearch(xBoundaryPoints[y], x);
		if (i & 1)
			return pair<short, short>(xBoundaryPoints[y][i + 1], xBoundaryPoints[y][i + 1]);
		else
			return pair<short, short>(xBoundaryPoints[y][i + 1] - 1, i + 2 < (int)xBoundaryPoints[y].size() ? xBoundaryPoints[y][i + 2] : gridWidth);
	}
	pair<short, short> getWestEndPoshortReOpen(short x, short y)
	{
		if (y<0 || y >= gridHeight)
			return pair<short, short>(-1, -1);

		if (xBoundaryPoints[y][0]>x)
			return pair<short, short>(-1, -1);

		short i = binarySearch(xBoundaryPoints[y], x);
		if (i & 1)
			return pair<short, short>(xBoundaryPoints[y][i] - 1, xBoundaryPoints[y][i] - 1);
		else
			return pair<short, short>(xBoundaryPoints[y][i], i - 1 < 0 ? -1 : xBoundaryPoints[y][i - 1] - 1);
	}

	pair<short, short> getSouthEndPoshortReOpen(short x, short y)
	{
		if (x<0 || x >= gridWidth)
			return pair<short, short>(gridHeight, gridHeight);

		if (yBoundaryPoints[x][0]>y)
			return pair<short, short>(yBoundaryPoints[x][0], yBoundaryPoints[x][0]);

		short i = binarySearch(yBoundaryPoints[x], y);
		if (i & 1)
			return pair<short, short>(yBoundaryPoints[x][i + 1], yBoundaryPoints[x][i + 1]);
		else
			return pair<short, short>(yBoundaryPoints[x][i + 1] - 1, i + 2<(int)yBoundaryPoints[x].size() ? yBoundaryPoints[x][i + 2] : gridHeight);

	}
	pair<short, short> getNorthEndPointReOpen(short x, short y)
	{
		if (x<0 || x >= gridWidth)
			return pair<short, short>(-1, -1);

		if (yBoundaryPoints[x][0]>y)
			return pair<short, short>(-1, -1);

		short i = binarySearch(yBoundaryPoints[x], y);
		if (i & 1)
			return pair<short, short>(yBoundaryPoints[x][i] - 1, yBoundaryPoints[x][i] - 1);
		else
			return pair<short, short>(yBoundaryPoints[x][i], i - 1 < 0 ? -1 : yBoundaryPoints[x][i - 1] - 1);
	}
	bool getJumpPointOld(Coordinate s, const char direction, Coordinate & jp)
	{
#ifndef DIAG_UNBLOCKED
		s = nextCoordinate(s, direction);
		if (!isPassable(s))
			return false;
#endif

		bool ret = false;
		pair<short, short> up, center, down;
		switch (direction)
		{

		case 0://North
			up = getNorthEndPointReOpen(s.x - 1, s.y);
			center = getNorthEndPointReOpen(s.x, s.y);
			down = getNorthEndPointReOpen(s.x + 1, s.y);

			if (down.first != -1 && ((down.second>-1 && down.first > center.first && down.second + (1 + BL_JPS_OFFSET)>center.first) || (down.first == down.second && down.first + (1 + BL_JPS_OFFSET)>center.first)))
			{
				jp = Coordinate(s.x, down.second + BL_JPS_OFFSET);
				ret = true;
			}
			if (up.first != -1 && ((up.second>-1 && up.first>center.first&&up.second + (1 + BL_JPS_OFFSET)>center.first) || (up.first == up.second && up.first + (1 + BL_JPS_OFFSET)>center.first)))
			{
				jp = Coordinate(s.x, ret ? max(jp.y, (short)(up.second + BL_JPS_OFFSET)) : (short)(up.second + BL_JPS_OFFSET));
				return true;
			}
			return ret;
		case 2://EAST
			up = getEastEndPoshortReOpen(s.x, s.y - 1);
			center = getEastEndPoshortReOpen(s.x, s.y);
			down = getEastEndPoshortReOpen(s.x, s.y + 1);

			if (down.first != gridWidth && ((down.second<gridWidth&&down.first < center.first && down.second - (1 + BL_JPS_OFFSET)<center.first) || (down.first == down.second && down.first - (1 + BL_JPS_OFFSET)<center.first)))
			{
				jp = Coordinate(down.second - BL_JPS_OFFSET, s.y);
				ret = true;
			}
			if (up.first != gridWidth && ((up.second<gridWidth&&up.first<center.first&&up.second - (1 + BL_JPS_OFFSET)<center.first) || (up.first == up.second && up.first - (1 + BL_JPS_OFFSET)<center.first)))
			{
				jp = Coordinate(ret ? min(jp.x, (short)(up.second - BL_JPS_OFFSET)) : (short)(up.second - BL_JPS_OFFSET), s.y);
				return true;
			}
			return ret;
		case 4://SOUTH
			up = getSouthEndPoshortReOpen(s.x - 1, s.y);
			center = getSouthEndPoshortReOpen(s.x, s.y);
			down = getSouthEndPoshortReOpen(s.x + 1, s.y);


			if (down.first != gridHeight && ((down.second<gridHeight&& down.first < center.first && down.second - (1 + BL_JPS_OFFSET)<center.first) || (down.first == down.second && down.first - (1 + BL_JPS_OFFSET)<center.first)))
			{
				jp = Coordinate(s.x, down.second - BL_JPS_OFFSET);
				ret = true;
			}
			if (up.first != gridHeight && ((up.second<gridHeight&&up.first<center.first&&up.second - (1 + BL_JPS_OFFSET)<center.first) || (up.first == up.second && up.first - (1 + BL_JPS_OFFSET)<center.first)))
			{
				jp = Coordinate(s.x, ret ? min(jp.y, (short)(up.second - BL_JPS_OFFSET)) : (short)(up.second - BL_JPS_OFFSET));
				return true;
			}
			return ret;
		case 6://WEST
			up = getWestEndPoshortReOpen(s.x, s.y - 1);
			center = getWestEndPoshortReOpen(s.x, s.y);
			down = getWestEndPoshortReOpen(s.x, s.y + 1);

			if (down.first != -1 && ((down.second>-1 && down.first > center.first && down.second + (1 + BL_JPS_OFFSET)>center.first) || (down.first == down.second && down.first + (1 + BL_JPS_OFFSET)>center.first)))
			{
				jp = Coordinate(down.second + BL_JPS_OFFSET, s.y);
				ret = true;
			}
			if (up.first != -1 && ((up.second>-1 && up.first>center.first&&up.second + (1 + BL_JPS_OFFSET)>center.first) || (up.first == up.second && up.first + (1 + BL_JPS_OFFSET)>center.first)))
			{
				jp = Coordinate(ret ? max(jp.x, (short)(up.second + BL_JPS_OFFSET)) : (short)(up.second + BL_JPS_OFFSET), s.y);
				return true;
			}
			return ret;
		}
		return ret;
	}

	int jump(const Coordinate &c, const char dir)
	{
		Coordinate nc = nextCoordinate(c, dir);
		bool isDiag = dirIsDiagonal(dir);
		Coordinate offset(0, 0);
		offset = nextCoordinate(offset, dir);

		while (1)
		{
			bool b = true;
#ifdef DIAG_UNBLOCKED
			if (dir & 1)
			{
				Coordinate offset = nextCoordinate(Coordinate(0, 0), dir);
				b = isPassable(Coordinate(nc.x - offset.x, nc.y)) && isPassable(Coordinate(nc.x, nc.y - offset.y));
		}
#endif
			if (!isPassable(nc) || !b)
				return -1;
			int index = gridIndex(nc);
			if (forcedNeighbours(nc, dir))
				return index;
			if (isDiag)
			{
				Coordinate newP(-1, -1);
				if (getJumpPointOld(nc, (dir + 7) & 7, newP))
					return index;
				if (getJumpPointOld(nc, (dir + 1) & 7, newP))
					return index;
			}
			else
			{
				Coordinate newP(-1, -1);
#ifdef DIAG_UNBLOCKED
				getJumpPointOld(nc, dir, newP);
#else
				getJumpPointOld(c, dir, newP);
#endif
				return gridIndex(newP);
			}
			nc.add(offset);

		}
		return -1;
	}
#else




	int jump(const Coordinate &c, const char dir)
	{
		Coordinate nc = nextCoordinate(c, dir);
		bool b = true;
#ifdef DIAG_UNBLOCKED
		if (dir & 1)
		{
			Coordinate offset = nextCoordinate(Coordinate(0, 0), dir);
			b=isPassable(Coordinate(nc.x - offset.x, nc.y)) && isPassable(Coordinate(nc.x, nc.y - offset.y));
		}
#endif
		if (!isPassable(nc) || !b)
			return -1;

		int index = gridIndex(nc);
		unsigned char dirs;
		if (index == endIndex || (dirs = forcedNeighbours(nc, dir)))
			return index;

		if (dirIsDiagonal(dir))
		{
			int next = jump(nc, (dir + 7) & 7);
			if (next >= 0)
				return index;
			next = jump(nc, (dir + 1) & 7);
			if (next >= 0)
				return index;
		}

		return jump(nc, dir);
	}

#endif
	
	int jumpAll2(const Coordinate &c, const char dir, vector<GraphNode*> & nodes)
	{
		Coordinate nc = nextCoordinate(c, dir);
#ifdef DIAG_UNBLOCKED
		Coordinate offset = nextCoordinate(Coordinate(0, 0), dir);
		if (!isPassable(nc) || (!isPassable(Coordinate(nc.x - offset.x, nc.y)) || !isPassable(Coordinate(nc.x, nc.y - offset.y))))
#else
		if (!isPassable(nc))
#endif
			return -1;



		int index = gridIndex(nc);
		unsigned char dirs;
		if (dirs = forcedNeighbours(nc, dir))
		{
			GraphNode * gn = getGraphNode(nc.x, nc.y);
			if (gn)
			{
				nodes.push_back(gn);
				return index;
			}
		}

		if (dirIsDiagonal(dir))
		{
			Coordinate newP(-1, -1);
			if (getJumpPointNew(nc, (dir + 7) & 7, newP))
			{
				GraphNode* gn = getGraphNode(newP.x, newP.y);
				if (gn)
					nodes.push_back(gn);
			}
			if (getJumpPointNew(nc, (dir + 1) & 7, newP))
			{
				GraphNode* gn = getGraphNode(newP.x, newP.y);
				if (gn)
					nodes.push_back(gn);
			}

		}
		else
		{
			Coordinate newP(-1, -1);
			if (getJumpPointNew(nc, dir, newP))
			{
				GraphNode* gn = getGraphNode(newP.x, newP.y);
				if (gn)
					nodes.push_back(gn);
			}
			return -1;
		}
		return jumpAll2(nc, dir, nodes);
	}
	/*int jumpAll(const Coordinate &c, const char dir, vector<GraphNode*> & nodes)
	{
		Coordinate nc = nextCoordinate(c, dir);
#ifdef DIAG_UNBLOCKED
		Coordinate offset = nextCoordinate(Coordinate(0, 0), dir);
		if (!isPassable(nc) || (!isPassable(Coordinate(nc.x - offset.x, nc.y)) || !isPassable(Coordinate(nc.x, nc.y - offset.y))))
#else
		if (!isPassable(nc))
#endif
			//if (!isPassable(nc))
			return -1;

		int index = gridIndex(nc);
		unsigned char dirs;
		if (dirs = forcedNeighbours(nc, dir))
		{
			GraphNode * gn = getGraphNode(nc.x, nc.y);
			nodes.push_back(gn);
			//if ((dir & 1)==0)
			return index;
		}

		if (dirIsDiagonal(dir))
		{
			int next = jumpAll(nc, (dir + 7) & 7, nodes);
			next = jumpAll(nc, (dir + 1) & 7, nodes);
		}

		return jumpAll(nc, dir, nodes);
	}*/

	//Only precocess directions 1,3,5,7
	int jumpDiag(Coordinate &c, const char dir)
	{
		c = nextCoordinate(c, dir);
		bool b = true;
#ifdef DIAG_UNBLOCKED
		Coordinate offset(0, 0);
		offset = nextCoordinate(offset, dir);
		b = ((dir & 1) == 0) || ((dir & 1) && (isPassable(Coordinate(c.x - offset.x, c.y)) && isPassable(Coordinate(c.x, c.y - offset.y))));
#endif
		if (!isPassable(c) || !b)
			return -1;

		//int index = gridIndex(nc);
		unsigned char dirs;
		if ((dirs = forcedNeighbours(c, dir)))
			return c.x;

		if (dirIsDiagonal(dir))
		{

			Coordinate newP(-1, -1);
			if (getJumpPointNew(c, (dir + 7) & 7, newP))
				return c.x;
			if (getJumpPointNew(c, (dir + 1) & 7, newP))
				return c.x;
		}

		return jumpDiag(c, dir);
	}
	const Coordinate indexToCoordinate(const int index)
	{
		return Coordinate(index%gridWidth, index / gridWidth);
	}

	int getGridWidth()
	{
		return gridWidth;
	}
	int getGridHeight()
	{
		return gridHeight;
	}
	BL_JPS_Experimental_4(vector<bool> & grid, int width, int height) : gridData(grid), PathFindingAlgorithm(BLJPSEXP4_ALG_NAME, AT_BL_JPS_EXP4)
	{
		gridWidth = width;
		gridHeight = height;
		eX = eY = endIndex = -1;
		graphCells = new char[gridWidth*gridHeight / 8 + 1];
		graphLookup = new GraphNode*[gridWidth*gridHeight];
		memset(graphCells, 0, gridWidth*gridHeight / 8 + 1);
		memset(graphLookup, 0, sizeof(GraphNode*)*gridWidth*gridHeight );

		theHeap.reserve(500);
		testedGrid = 0;

		parents = 0;
		costs = 0;

	}
	~BL_JPS_Experimental_4()
	{
		delete[] testedGrid;
		for (unsigned int i = 0; i < allGraphNodes.size(); i++)
			delete allGraphNodes[i];
		delete 	[] graphCells;
		delete []  graphLookup;

		delete[] parents;
		delete[] costs;
	}
	unsigned char naturalNeighbours(const int dir)
	{
		if (dir == NO_DIRECTION)
			return 255;

		unsigned char dirs = 0;
		dirs = addDirectionToSet(dirs, dir);
		if (dirIsDiagonal(dir)) {
			dirs = addDirectionToSet(dirs, (dir + 1) & 7);
			dirs = addDirectionToSet(dirs, (dir + 7) & 7);
		}
		return dirs;
	}


	void setChecked(const int index)
	{
		testedGrid[index / 8] = testedGrid[index / 8] | (1 << (index & 7));
	}
	bool isChecked(const int index)
	{
		return (testedGrid[index / 8] & (1 << (index & 7)))>0;
	}


	void getEndSpaceIds(short spaceX, short spaceY)
	{

		for (unsigned int i = 0; i<xBoundaryPoints[spaceY].size() - 1; i++)
			if (xBoundaryPoints[spaceY][i] <= spaceX && xBoundaryPoints[spaceY][i + 1] > spaceX)
			{
				eXSpace[0] = xBoundaryPoints[spaceY][i];
				eXSpace[1] = xBoundaryPoints[spaceY][i + 1];
				eXSpaceId = i;
				break;
			}
		for (unsigned int i = 0; i<yBoundaryPoints[spaceX].size() - 1; i++)
			if (yBoundaryPoints[spaceX][i] <= spaceY && yBoundaryPoints[spaceX][i + 1] > spaceY)
			{
				eYSpace[0] = yBoundaryPoints[spaceX][i];
				eYSpace[1] = yBoundaryPoints[spaceX][i + 1];
				eYSpaceId = i;

				break;
			}
	}
	int getSpaceIdY(short spaceX, short spaceY)
	{
		for (unsigned int i = 0; i<yBoundaryPoints[spaceX].size() - 1; i++)
			if (yBoundaryPoints[spaceX][i] <= spaceY && yBoundaryPoints[spaceX][i + 1]>spaceY)
				return i;
		return -1;
	}
	bool isSpaceIdY(int spaceId, short spaceX, short spaceY)
	{
		if (spaceId == -1)
		{
			if (yBoundaryPoints[spaceX].size()>1)
			{
				//int i=yBoundaryPoints[spaceX].size()-2;
				if (yBoundaryPoints[spaceX][0]>spaceY)
					return true;
				else
					return false;
			}
			else
				return true;
		}
		else
			if (yBoundaryPoints[spaceX][spaceId] <= spaceY && yBoundaryPoints[spaceX][spaceId + 1]>spaceY)
				return true;
		return false;
	}
	int getSpaceIdX(short spaceX, short spaceY)
	{
		for (unsigned int i = 0; i<xBoundaryPoints[spaceY].size() - 1; i++)
			if (xBoundaryPoints[spaceY][i] <= spaceX && xBoundaryPoints[spaceY][i + 1]>spaceX)
				return i;
		return -1;
	}
	bool isSpaceIdX(int spaceId, short spaceX, short spaceY)
	{
		if (spaceId == -1)
		{
			if (xBoundaryPoints[spaceY].size()>1)
			{
				//int i=xBoundaryPoints[spaceY].size()-3;
				if (xBoundaryPoints[spaceY][0]>spaceX)
					return true;
				else
					return false;
			}
			else
				return true;
		}
		else
			if (xBoundaryPoints[spaceY][spaceId] <= spaceX && xBoundaryPoints[spaceY][spaceId + 1]>spaceX)
				return true;
		return false;
	}

	bool directSolution(short sX, short sY, short eX, short eY, vector<Coordinate> & sol)
	{
		if (sY == eY)
		{
			if (eXSpace[0] <= sX && sX <= eXSpace[1])
			{
				sol.push_back(Coordinate(eX, eY));
				sol.push_back(Coordinate(sX, sY));

				return true;
			}
		}
		else if (sX == eX)
		{
			if (eYSpace[0] <= sY && sY <= eYSpace[1])
			{
				sol.push_back(Coordinate(eX, eY));
				sol.push_back(Coordinate(sX, sY));

				return true;
			}
		}
		else
		{
			int dir = getNewDir2(sX, sY, eX, eY);
			int dirId = (dir - 1) / 2;
			int diagCellIndex = sX + sY;// dirs 1 and 5
			int goalCellIndex = eX + eY;
			if (dir == 7 || dir == 3)
			{
				diagCellIndex = sX - sY + (gridHeight - 1);
				goalCellIndex = eX - eY + (gridHeight - 1);
			}
			DiagJumpNode * jumpNode = 0;

			for (unsigned int i = 0; i < diagJumpLookup[dirId][diagCellIndex].size() && jumpNode == 0; i++)
				if (diagJumpLookup[dirId][diagCellIndex][i].sI <= sX && sX <= diagJumpLookup[dirId][diagCellIndex][i].dI)
					jumpNode = &diagJumpLookup[dirId][diagCellIndex][i]; 

			//if (jumpNode == 0)// The requested space was in a blocked area
			//	return -1;
			//Check if we step over the goal diagnally
			if (jumpNode  &&diagCellIndex == goalCellIndex)
			{
				if (jumpNode->sI <= eX && eX <= jumpNode->dI) //Going east check
				{
					sol.push_back(Coordinate(eX, eY));
					sol.push_back(Coordinate(sX, sY));
					return true;
				}
			}
			else
			{
				int mx = eX - sX;
				int my = eY - sY;
				if (mx != 0)
					mx = mx<0 ? -1 : 1;
				if (my != 0)
					my = my<0 ? -1 : 1;

				//if ((((dir == 1 || dir == 3) && mx >= 0) || ((dir == 5 || dir == 7) && mx <= 0)) &&
				//	(((dir == 3 || dir == 5) && my >= 0) || ((dir == 1 || dir == 7) && my <= 0))
				//	)
				{
					int diagMovements = min(abs(sX - eX), abs(sY - eY));
					bool verticalExtension = abs(sY - eY) > abs(sX - eX);

					int dx = sX + diagMovements*mx;
					int dy = sY + diagMovements*my;
					if (jumpNode)
						if (((dir == 1 || dir == 3) && (sX <= dx && dx <= jumpNode->dI)) || ((dir == 5 || dir == 7) && (jumpNode->sI <= dx && dx <= sX)))
						{
							if (verticalExtension)
							{
								if (isSpaceIdY(eYSpaceId, dx, dy))
								{
									sol.push_back(Coordinate(eX, eY));

									sol.push_back(Coordinate(dx, dy));
									sol.push_back(Coordinate(sX, sY));
									return true;
								}
								//return gridIndex(Coordinate(dx, dy));
							}
							else
							{
								if (isSpaceIdX(eXSpaceId, dx, dy))
								{
									sol.push_back(Coordinate(eX, eY));

									sol.push_back(Coordinate(dx, dy));
									sol.push_back(Coordinate(sX, sY));
									return true;

								}
								//return gridIndex(Coordinate(dx, dy));
							}
						}
					if (verticalExtension)
					{
						dy = sY + (abs(sY - eY) - diagMovements)*my;
						if (isSpaceIdY(getSpaceIdY(sX, sY), sX, dy))
						{
							jumpNode = 0;
							int diagCellIndex2 = sX + dy;// dirs 1 and 5
							if (dir == 7 || dir == 3)
								diagCellIndex2 = sX - dy + (gridHeight - 1);

							for (unsigned int i = 0; i < diagJumpLookup[dirId][diagCellIndex2].size() && jumpNode == 0; i++)
								if (diagJumpLookup[dirId][diagCellIndex2][i].sI <= sX && sX <= diagJumpLookup[dirId][diagCellIndex2][i].dI) //Going east check
									jumpNode = &diagJumpLookup[dirId][diagCellIndex2][i]; //doubling up

							//Check if we step over the goal diagnally
							if (jumpNode  &&diagCellIndex2 == goalCellIndex)
							{
								if (jumpNode->sI <= eX && eX <= jumpNode->dI) //Going east check
								{
									sol.push_back(Coordinate(eX, eY));
									sol.push_back(Coordinate(sX, dy));
									sol.push_back(Coordinate(sX, sY));
									return true;
								}
							}

						}
					}
					else
					{
						dx = sX + (abs(sX - eX) - diagMovements)*mx;

						if (isSpaceIdX(getSpaceIdX(sX, sY), dx, sY))
						{
							jumpNode = 0;
							int diagCellIndex2 = dx + sY;// dirs 1 and 5
							if (dir == 7 || dir == 3)
								diagCellIndex2 = dx - sY + (gridHeight - 1);

							for (unsigned int i = 0; i < diagJumpLookup[dirId][diagCellIndex2].size() && jumpNode == 0; i++)
								if (diagJumpLookup[dirId][diagCellIndex2][i].sI <= dx && dx <= diagJumpLookup[dirId][diagCellIndex2][i].dI) //Going east check
									jumpNode = &diagJumpLookup[dirId][diagCellIndex2][i]; //doubling up

							if (jumpNode == 0)// The requested space was in a blocked area
								return false;
							//Check if we step over the goal diagnally
							if (jumpNode  &&diagCellIndex2 == goalCellIndex)
							{
								if (jumpNode->sI <= eX && eX <= jumpNode->dI) //Going east check
								{
									sol.push_back(Coordinate(eX, eY));
									sol.push_back(Coordinate(dx, sY));
									sol.push_back(Coordinate(sX, sY));
									return true;
								}
							}
						}
					}
				}

			}



		}
		return false;
	}

	

	void addStartNode(void* startNode, int px, int py, int dir,int sX,int sY)
	{
		GraphNode* gn = getGraphNode(px, py);
		costs[gn->id] = hCost(px, py, sX, sY);
		AddToOpen(gn, costs[gn->id]+hCost(px, py, eX, eY), dir);
		SetClosed(gn->id);
		parents[gn->id] = 0;
	}

	void findStartNodes(void* startNode,int sX,int sY)
	{
		if (isGraphNode(sX, sY))
		{
			addStartNode(startNode, sX, sY, 8, sX, sY);
		}
		//	startNodes.push_back(getGraphNode(sY, sY));
		//else
		{
			for (int dir = 0; dir < 8; dir++)
			{
				bool repeat = true;
				Coordinate currentCoordinate(sX, sY);

				while (repeat)
				{
					int index = jumpNew(currentCoordinate, dir);

					if (inBounds(index))
					{
						Coordinate CoordinateNewC = indexToCoordinate(index);
						//openListBh.Insert(nodesC.getNewNode(CoordinateNewC, eX, eY, dir, startNode, getGraphNode(CoordinateNewC.x, CoordinateNewC.y)));
						if (dir & 1)
						{

							if (isGraphNode(CoordinateNewC.x, CoordinateNewC.y))
								addStartNode(startNode, CoordinateNewC.x, CoordinateNewC.y, dir, sX, sY);

								//openListBh.Insert(nodesC.getNewNode(CoordinateNewC, eX, eY, dir, startNode, getGraphNode(CoordinateNewC.x, CoordinateNewC.y)));
							else
							{
								int index2 = jumpNew(CoordinateNewC, (dir + 1) & 7);
								if (inBounds(index2))
								{
									Coordinate CoordinateNewC2 = indexToCoordinate(index2);
									//if (getGraphNode(CoordinateNewC2.x, CoordinateNewC2.y) == 0)
									//	int a = 1;
									addStartNode(startNode, CoordinateNewC2.x, CoordinateNewC2.y, (dir + 1) & 7, sX, sY);

									//openListBh.Insert(nodesC.getNewNode(CoordinateNewC2, eX, eY, (dir + 1) & 7, startNode, getGraphNode(CoordinateNewC2.x, CoordinateNewC2.y)));
								}
								index2 = jumpNew(CoordinateNewC, (dir + 7) & 7);
								if (inBounds(index2))
								{
									Coordinate CoordinateNewC2 = indexToCoordinate(index2);
									addStartNode(startNode, CoordinateNewC2.x, CoordinateNewC2.y, (dir + 7) & 7, sX, sY);

									//openListBh.Insert(nodesC.getNewNode(CoordinateNewC2, eX, eY, (dir + 7) & 7, startNode, getGraphNode(CoordinateNewC2.x, CoordinateNewC2.y)));
								}
							}
							currentCoordinate = CoordinateNewC;
						}
						else
						{
							addStartNode(startNode, CoordinateNewC.x, CoordinateNewC.y, dir, sX, sY);

							//openListBh.Insert(nodesC.getNewNode(CoordinateNewC, eX, eY, dir, startNode, getGraphNode(CoordinateNewC.x, CoordinateNewC.y)));
							repeat = false;
						}

					}
					else
						repeat = false;
				}

			}
		}
	}
	/*void clearEndNodes(vector<GraphNode*> &endNodes)
	{
		for (int i = 0; i < endNodes.size(); i++)
			endNodes[i]->removeGoalNode();
	}*/
	int getNewDir(int fX,int fY, int dX,int dY)
	{
		int dx = abs(dX - fX);
		int dy = abs(dY - fY);
		if (dx == dy && dx>0 && dy > 0)
		{
			if (dX  > fX)
			{
				if (dY   > fY)
					return 3;
				return 1;
			}
			if (dY  > fY)
				return 5;
			return 7;
		}
		if (dx > dy)
			if (dX > fX)
				return 2;
			else
				return 6;
		else
			if (dY > fY)
				return 4;
		return 0;
	}
	int getNewDir2(int fX, int fY, int dX, int dY)
	{
		int dx = dX - fX;
		int dy = dY - fY;
		if (dx > 0)
		{
			if (dy == 0)
				return 2;
			if (dy > 0)
				return 3;
			if (dy < 0)
				return 1;
		}
		if (dx < 0)
		{
			if (dy == 0)
				return 6;
			if (dy > 0)
				return 5;
			if (dy < 0)
				return 7;
		}
		if (dy > 0)
			return 4;
		return 0;
	}
	bool isConnectedToSolution(GraphNode* gn, int dir)
	{
		int x = gn->x;
		int y = gn->y;
		int limitCount = gn->limits[dir];
		//straight solution first
		if ((dir & 1) == 0)
		{
			if ((dir == 2 || dir == 6) && eY == y)
			{
				if (dir == 2 && x <= eX && eX<= x + limitCount )
					return true;
				else if (dir == 6 && x - limitCount<= eX &&  eX <= x )
					return true;
				return false;
			}

			if ((dir == 0 || dir == 4) && eX == x)
			{
				if (dir == 4 && y <= eY &&  eY<= y + limitCount )
					return true;
				else if (dir == 0 && y - limitCount <= eY &&  eY <=y)
					return true;
				return false;
			}
			return false;
		}
		int dirId = (dir - 1) / 2;
		int diagCellIndex = x + y;// dirs 1 and 5
		int goalCellIndex = eX + eY;
		if (dir == 7 || dir == 3)
		{
			diagCellIndex = x - y + (gridHeight - 1);
			goalCellIndex = eX - eY + (gridHeight - 1);
		}
		
		//Check if we step over the goal diagnally
		if (diagCellIndex == goalCellIndex)
		{
			if ((dir == 7 || dir == 5) && (x - limitCount <= eX && eX <= x ))
				return true;
			if ((dir == 1 || dir == 3) && (x <= eX && eX<=x + limitCount ))
				return true;
			return false;
		}
		else
		{
			int mx = eX - x;
			int my = eY - y;
			if (mx != 0)
				mx = mx<0 ? -1 : 1;
			if (my != 0)
				my = my<0 ? -1 : 1;

			if ((((dir == 1 || dir == 3) && mx >= 0) || ((dir == 5 || dir == 7) && mx <= 0)) &&
				(((dir == 3 || dir == 5) && my >= 0) || ((dir == 1 || dir == 7) && my <= 0))
				)
			{
				int diagMovements = min(abs(x - eX), abs(y - eY));
				bool verticalExtension = abs(y - eY) > abs(x - eX);

				int dx = x + diagMovements*mx;
				int dy = y + diagMovements*my;

				if (((dir == 1 || dir == 3) && (x <= dx && dx <= x + limitCount)) || ((dir == 5 || dir == 7) && (x - limitCount <= dx && dx <= x )))
				{
					if (verticalExtension)
					{
						if (eYSpace[0] <= dy && dy<= eYSpace[1])
							return true;
					}
					else
					{
						if (eXSpace[0] <= dx && dx <= eXSpace[1])
							return true;
					}
				}
			}


		}
		return false;
	}
	void findSolution(int sX, int sY, int _eX, int _eY, vector<Coordinate> & sol)
	{
		eX = _eX;
		eY = _eY;

		sol.clear();

		endIndex = gridIndex(Coordinate(eX, eY));

		if (!(sX >= 0 && sX<gridWidth &&
			sY >= 0 && sY<gridHeight &&
			eX >= 0 && eX<gridWidth &&
			eY >= 0 && eY<gridHeight &&
			isPassable(Coordinate(sX, sY)) && isPassable(Coordinate(eX, eY))
			))
		{
			return;
		}
		if (sX == eX && sY == eY)
			return;

		getEndSpaceIds(eX, eY);

		if (directSolution(sX, sY, eX, eY, sol))
		{
			return;
		}

		vector<GraphNode*> endNodes;

		memset(testedGrid, 0xFF,(allGraphNodes.size()>>3)+1);
		canReplaceTop = false;
		theHeap.clear();
		findStartNodes(0,sX,sY);
		//Keep iterating over openlist until a solution is found or list is empty
		while (theHeap.size())
		{
			GraphNode * gn = theHeap[0].gn;
			PopMin();
			
			unsigned char dirs = forcedNeighbours2(theHeap[0].dir, gn->neighbouringCells) | naturalNeighbours(theHeap[0].dir);
			{
				for (int dir = 0; dir < 8; dir++)
				{
					if ((1 << dir)&dirs)
					{
						if ( isConnectedToSolution(gn, dir))

						{
							Coordinate end(eX, eY);
							sol.push_back(end);
							{
								int dx = abs(gn->x - eX);
								int dy = abs(gn->y - eY);
								if (dx != dy && dx != 0 && dy != 0)
								{
									int diagMovements = min(dx, dy);//we -1 as we don't need to check the original or destination points as they are checked earlier
									int mx = gn->x - eX < 0 ? -1 : 1;
									int my = gn->y - eY < 0 ? -1 : 1;
									sol.push_back(Coordinate(gn->x - mx*diagMovements, gn->y - my*diagMovements));
								}
							}
							GraphNode * currentGNNode = gn;
							while (currentGNNode)
							{
								int dx = abs(currentGNNode->x - sol.back().x);
								int dy = abs(currentGNNode->y - sol.back().y);
								if (dx != dy && dx != 0 && dy != 0 && gn != currentGNNode)
								{
									int diagMovements = min(dx, dy);//we -1 as we don't need to check the original or destination points as they are checked earlier
									int mx = currentGNNode->x - sol.back().x < 0 ? -1 : 1;
									int my = currentGNNode->y - sol.back().y < 0 ? -1 : 1;
									sol.push_back(Coordinate(currentGNNode->x - mx*diagMovements, currentGNNode->y - my*diagMovements));
								}
								sol.push_back(Coordinate(currentGNNode->x, currentGNNode->y));
								currentGNNode = parents[currentGNNode->id];
							}
							if (sol.back().x!=sX||sol.back().y!=sY)
							{
								int dx = abs(sX - sol.back().x);
								int dy = abs(sY - sol.back().y);
								if (dx != dy && dx != 0 && dy != 0)
								{
									int diagMovements = min(dx, dy);//we -1 as we don't need to check the original or destination points as they are checked earlier
									int mx = sX - sol.back().x < 0 ? -1 : 1;
									int my = sY - sol.back().y < 0 ? -1 : 1;
									sol.push_back(Coordinate(sX - mx*diagMovements, sY - my*diagMovements));
								}
								sol.push_back(Coordinate(sX, sY));
							}
							return;
						}


						for (unsigned int i = 0; i < gn->nextJumps[dir].size(); i++)
						{
							GraphNode * newNode = gn->nextJumps[dir][i];
							int index = newNode->id;// gridIndex(Coordinate(newNode->x, newNode->y));
							int nDir = getNewDir(gn->x, gn->y, newNode->x, newNode->y);
							if (IsOpen(index))
							{
								costs[index] = costs[gn->id] + hCost(gn->x, gn->y, newNode->x, newNode->y);
								parents[index] = gn;
								SetClosed(index);
								AddToOpen(newNode, costs[index] + hCost(newNode->x, newNode->y, eX, eY), nDir);
							}
							else 
							{
								costType newGCost = costs[gn->id] + hCost(gn->x, gn->y, newNode->x, newNode->y);
								if (newGCost < costs[index])
								{
									costs[index] = newGCost;
									parents[index] = gn;
									InsertSmaller(newNode, costs[index] + hCost(newNode->x, newNode->y, eX, eY), nDir);
								}
							}
						}
					}
					
				}
			}
			PopReplacableTop();
		}
	}


	short binarySearchR(const vector<pair<short, short> > & v, short val)
	{
		short l = 0, r = v.size() - 1;
		if (r < 0)
			return -1;
		short index = r / 2;
		while (1)
		{
			if (v[index].first >= val && v[index].second <= val)
				return  v[index].second;
			if (v[index].second > val)
			{
				r = index - 1;
				if (r <0 || r<l)
					return -1;
			}
			else
			{
				l = index + 1;
				if (l >(int)v.size() - 1 || r<l)
					return -1;
			}

			index = l + (r - l) / 2;

		}
		return -1;
	}
	short binarySearchL(const vector<pair<short, short> > & v, short val)
	{
		short l = 0, r = v.size() - 1;
		if (r < 0)
			return -1;
		short index = r / 2;
		while (1)
		{
			if (v[index].first <= val && v[index].second >= val)
				return  v[index].second;
			if (v[index].second > val)
			{
				r = index - 1;
				if (r <0 || r<l)
					return -1;
			}
			else
			{
				l = index + 1;
				if (l > (int)v.size() - 1 || r<l)
					return -1;
			}

			index = l + (r - l) / 2;

		}
		return -1;
	}
	bool getJumpPointNew(Coordinate s, const char direction, Coordinate & jp)
	{
		/*if (forwardStep)
		{
			s = nextCoordinate(s, direction);

			if (!isPassable(s))
				return false;
		}*/
		//		bool ret = false;

		int index;
		switch (direction)
		{

		case 0://North
			index = binarySearchR(jumpLookup[0][s.x], s.y);
			if (s.x == eX&& s.y >= eY &&s.y >= eYSpace[0] && s.y <= eYSpace[1])
			{
				jp = Coordinate(eX, eY);
				return true;
			}
			if (index != -1)
				jp = Coordinate(s.x, index);
			return index>-1;
		case 2://EAST
			index = binarySearchL(jumpLookup[1][s.y], s.x);
			if (s.y == eY&& s.x <= eX &&s.x <= eXSpace[1] && s.x >= eXSpace[0])
			{
				jp = Coordinate(eX, eY);
				return true;
			}
			if (index != -1)
				jp = Coordinate(index, s.y);
			return index>-1;
		case 4://SOUTH
			index = binarySearchL(jumpLookup[2][s.x], s.y);

			if (s.x == eX&& s.y <= eY &&s.y <= eYSpace[1] && s.y >= eYSpace[0])
			{
				jp = Coordinate(eX, eY);
				return true;
			}
			if (index != -1)
				jp = Coordinate(s.x, index);
			return index>-1;
		case 6://WEST
			index = binarySearchR(jumpLookup[3][s.y], s.x);
			if (s.y == eY&& s.x >= eX &&s.x >= eXSpace[0] && s.x <= eXSpace[1])
			{
				jp = Coordinate(eX, eY);
				return true;
			}
			if (index != -1)
				jp = Coordinate(index, s.y);
			return index>-1;
		}
		return false;
	}
	int getDiagJumpNew(const short x, const short y, const char dir)
	{
		int dirId = (dir - 1) / 2;
		int diagCellIndex = x + y;// dirs 1 and 5
		//int goalCellIndex = eX + eY;
		if (dir == 7 || dir == 3)
		{
			diagCellIndex = x - y + (gridHeight - 1);
			//goalCellIndex = eX - eY + (gridHeight - 1);
		}
		DiagJumpNode * jumpNode = 0;

		for (unsigned int i = 0; i < diagJumpLookup[dirId][diagCellIndex].size() && jumpNode == 0; i++)
			if (diagJumpLookup[dirId][diagCellIndex][i].sI <= x && diagJumpLookup[dirId][diagCellIndex][i].dI >= x) //Going east check
				jumpNode = &diagJumpLookup[dirId][diagCellIndex][i]; //doubling up

		if (jumpNode == 0)// The requested space was in a blocked area
			return -1;
		//Check if we step over the goal diagnally
		/*if (diagCellIndex == goalCellIndex)
		{
		if (jumpNode->sI <= eX && jumpNode->dI >= eX) //Going east check
		return endIndex;
		}
		else
		{
		int mx = eX - x;
		int my = eY - y;
		if (mx != 0)
		mx = mx<0 ? -1 : 1;
		if (my != 0)jumpNew
		my = my<0 ? -1 : 1;

		if ((((dir == 1 || dir == 3) && mx >= 0) || ((dir == 5 || dir == 7) && mx <= 0)) &&
		(((dir == 3 || dir == 5) && my >= 0) || ((dir == 1 || dir == 7) && my <= 0))
		)
		{
		int diagMovements = min(abs(x - eX), abs(y - eY));
		bool verticalExtension = abs(y - eY) > abs(x - eX);

		int dx = x + diagMovements*mx;
		int dy = y + diagMovements*my;

		if (((dir == 1 || dir == 3) && (x <= dx && jumpNode->dI >= dx)) || ((dir == 5 || dir == 7) && (x >= dx && jumpNode->sI <= dx)))
		{
		if (verticalExtension)
		{
		if (isSpaceIdY(eYSpaceId, dx, dy))
		return gridIndex(Coordinate(dx, dy));
		}
		else
		{
		if (isSpaceIdX(eXSpaceId, dx, dy))
		return gridIndex(Coordinate(dx, dy));
		}
		}
		}
		}*/

		int nextJumpId = -1;
		if (dir == 1 || dir == 3)
		{
			for (unsigned int i = 0; i < jumpNode->jumps.size() && nextJumpId == -1; i++)
				if (jumpNode->jumps[i] >= x)
					return gridIndex(Coordinate(jumpNode->jumps[i], dir == 1 ? diagCellIndex - jumpNode->jumps[i] : jumpNode->jumps[i] - diagCellIndex + (gridHeight - 1)));
		}
		else
		{
			for (unsigned int i = 0; i < jumpNode->jumps.size() && nextJumpId == -1; i++)
				if (jumpNode->jumps[i] <= x)
					return gridIndex(Coordinate(jumpNode->jumps[i], dir == 5 ? diagCellIndex - jumpNode->jumps[i] : jumpNode->jumps[i] - diagCellIndex + (gridHeight - 1)));
		}
		return -1;
	}
	int jumpNew(const Coordinate &c, const char dir)
	{

		Coordinate nc = nextCoordinate(c, dir);
#ifdef DIAG_UNBLOCKED
		Coordinate offset(0, 0);
		offset = nextCoordinate(offset, dir);
#endif
		bool isDiag = dirIsDiagonal(dir);
		if (isDiag)
		{
#ifdef DIAG_UNBLOCKED

			if (!isPassable(Coordinate(nc.x - offset.x, nc.y)) || !isPassable(Coordinate(nc.x, nc.y - offset.y)))
				return -1;
#endif
			return getDiagJumpNew(nc.x, nc.y, dir);
		}
#ifndef DIAG_UNBLOCKED
		Coordinate offset(0, 0);
		offset = nextCoordinate(offset, dir);
#endif
		while (1)
		{
			bool b = true;
#ifdef DIAG_UNBLOCKED
			b = ((dir & 1) == 0) || ((dir & 1) && (isPassable(Coordinate(nc.x - offset.x, nc.y)) && isPassable(Coordinate(nc.x, nc.y - offset.y))));
#endif
			if (!isPassable(nc) || !b)
				return -1;
			int index = gridIndex(nc);
			if (forcedNeighbours(nc, dir) || endIndex == index)
				return index;
			if (isDiag)
			{
				Coordinate newP(-1, -1);
				if (getJumpPointNew(nc, (dir + 7) & 7, newP))
					return index;
				if (getJumpPointNew(nc, (dir + 1) & 7, newP))
					return index;
			}
			else
			{
				Coordinate newP(-1, -1);
				getJumpPointNew(nc, dir, newP);
				return gridIndex(newP);
			}
			nc.add(offset);
		}
		return -1;
	}



};
