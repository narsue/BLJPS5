#pragma once
#include "Node.h"
#include "PathFindingAlgorithm.h"
#include "JumpPointNode.h"
#include <limits>

#include <map>
#include <unordered_map>
using namespace std;


#define PREPROCESS_STARTING_NODES
//#define PREPROCESS_END_NODES
#define USE_PAIR_WISE_DISTANCES
#define STATIC_START_END

#ifdef DIAG_UNBLOCKED
#define BLJPS_SUBGOAL_ALG_BLOCKED_NAME "UNBLOCKED"
#else
#define BLJPS_SUBGOAL_ALG_BLOCKED_NAME "BLOCKED"
#endif
#ifdef PREPROCESS_END_NODES
#define BLJPS_SUBGOAL_ALG_ENDS_NAME "-ENDS"
#else
#define BLJPS_SUBGOAL_ALG_ENDS_NAME ""
#endif

#ifdef PREPROCESS_STARTING_NODES
#define BLJPS_SUBGOAL_ALG_START_NAME "-STARTS"
#else
#define BLJPS_SUBGOAL_ALG_START_NAME ""
#endif
#ifdef USE_PAIR_WISE_DISTANCES
#define BLJPS_SUBGOAL_ALG_DIST_NAME "-DIST"
#else
#define BLJPS_SUBGOAL_ALG_DIST_NAME ""
#endif

#define BLJPS_SUBGOAL_ALG_NAME (string("BLJPS_SUBGOAL_NEW-")+string(BLJPS_SUBGOAL_ALG_BLOCKED_NAME)+string(BLJPS_SUBGOAL_ALG_ENDS_NAME)+string(BLJPS_SUBGOAL_ALG_START_NAME)+string(BLJPS_SUBGOAL_ALG_DIST_NAME))

////
#define MEMORY_LIMIT 50000000
#define REPLACE_POPPED

//Directions
// N, NE, E, SE, S, SW, W, NW , StartPosition
#define NO_DIRECTION 8
typedef JumpPointNode2* JumpNode;

struct CoordinateHash
{
	static int MAPWIDTH;
	size_t CoordinateHash::operator()(const pair<short, short>& val) const
	{
		return hash<unsigned int>()(val.second)*MAPWIDTH + val.first;
	}
};
int CoordinateHash::MAPWIDTH = 5000;
struct JumpPointTableEntry
{
	JumpPointTableEntry(short _first, short _second, short _third, JumpNode _jp) :first(_first), second(_second), jp(_jp), third(_third)
	{
	}
	short first, second, third;
	JumpNode jp;
};
struct DiagonalJumpEntry
{
	DiagonalJumpEntry(Coordinate _first, Coordinate _second, JumpNode _jp) :from(_first), to(_second), jp(_jp)
	{}
	Coordinate from, to;
	JumpNode jp;
};
struct heapElement2
{
	JumpNode sg;
	float fVal;
	char dir;
	heapElement2(JumpNode _sg, float _fVal, char _dir) : sg(_sg), fVal(_fVal), dir(_dir){}
};
typedef pair<JumpNode, char> JumpNodeDir;

class BL_JPS_PLUS_SUBGOAL : public PathFindingAlgorithm
{
private:
	//Special container classes that help with general allocation of memory and search
	std::vector<heapElement2> theHeap;

	//NodeContainer2 nodesC;
	//BinaryHeap2 openListBh;
	//PairingHeap openList2;

	float * gCost;
	JumpNode* parent;

	//Table of flags indicating is a map location has been searched (Closed List)
	bool canReplaceTop;
	char *testedGrid;
	char* open;
	unsigned short* generated;	// Keep track of the last time a subgoal has been generated
	unsigned short search;	/* The current search number. When a search terminates, only increment this
							* to reset the previous search. search == generated[subgoalX] iff
							* subgoalX is generated for the current search
							*/
	pair<JumpNode,char>* incomingConnections;
	unsigned short incomingConnectionsSize;
	vector<vector<JumpNodeDir > > incomingConnectionsVector;
	void addIncomingConnection(int id, int dir, JumpNodeDir data)
	{
		incomingConnectionsVector[id * 8 + dir].push_back(data);
	}
	JumpNodeDir getIncomingConnection(int id, int dir,int i)
	{
		return incomingConnectionsVector[id * 8 + dir][i];
	}
	void eraseIncomingConnection(int id, int dir, int i)
	{
		incomingConnectionsVector[id * 8 + dir].erase(incomingConnectionsVector[id * 8 + dir].begin()+i);
	}
	unsigned int getNoIncConnections(int id, int dir)
	{
		return incomingConnectionsVector[id * 8 + dir].size();
	}


	vector<vector<JumpNode>> connectionsVector;
	vector<vector<JumpNodeDir>> highConnectionsVector;
	void addHighConnection(int id, int dir, JumpNodeDir data)
	{
		highConnectionsVector[id * 8 + dir].push_back(data);
	}
	void addConnection(int id, int dir, JumpNode data)
	{
		for (unsigned int i = 0; i < connectionsVector[id * 8 + dir].size(); i++)
			if (connectionsVector[id * 8 + dir][i] == data)
				return;
		connectionsVector[id * 8 + dir].push_back(data);
	}
	int getConnectionSize(int id, int dir)
	{
		return connectionsVector[id * 8 + dir].size();
	}
	int getHighConnectionSize(int id, int dir)
	{
		return highConnectionsVector[id * 8 + dir].size();
	}
	JumpNode getConnectionItem(int id, int dir,int i )
	{
		return connectionsVector[id * 8 + dir][i];
	}
	JumpNodeDir getHighConnectionItem(int id, int dir, int i)
	{
		return highConnectionsVector[id * 8 + dir][i];
	}
	unsigned short* sizeOfConnections;
	unsigned short* sizeOfHighConnections;
	JumpNode* rawConnections;
	JumpNodeDir*  rawHighConnections;
	JumpNode** rawConnectionStart;
	JumpNodeDir** rawHighConnectionStart;

	void	finalizeConnectionData()
	{
		int totalConnections =0, highConnections =0;
		for (int i = 0; i < jumpPoints.size(); i++)
			for (int dir = 0; dir < 8; dir++)
			{
				totalConnections += getConnectionSize(i, dir);
				highConnections += getHighConnectionSize(i, dir);
			}
		sizeOfConnections = new  unsigned short[jumpPoints.size()*8];
		sizeOfHighConnections = new  unsigned short[jumpPoints.size() * 8];
		rawConnections = new  JumpPointNode2*[totalConnections];
		rawHighConnections = new  JumpNodeDir[highConnections];
		rawConnectionStart = new  JumpNode*[jumpPoints.size() * 8];
		rawHighConnectionStart = new  JumpNodeDir*[jumpPoints.size() * 8];
		int connectionCounter = 0;
		int highConnectionCounter = 0;
		for (int i = 0; i < jumpPoints.size(); i++)
			for (int dir = 0; dir < 8; dir++)
			{
				sizeOfConnections[i * 8 + dir] = getConnectionSize(i, dir);
				sizeOfHighConnections[i * 8 + dir] = getHighConnectionSize(i, dir);
				rawConnectionStart[i * 8 + dir] = &rawConnections[connectionCounter];
				rawHighConnectionStart[i * 8 + dir] = &rawHighConnections[highConnectionCounter];


				for (int id = 0; id < getConnectionSize(i, dir); id++)
					rawConnections[connectionCounter+id] = getConnectionItem(i, dir, id);
				for (int id = 0; id < getHighConnectionSize(i, dir); id++)
					rawHighConnections[highConnectionCounter + id] = getHighConnectionItem(i, dir, id);

				connectionCounter += getConnectionSize(i, dir);
				highConnectionCounter += getHighConnectionSize(i, dir);
			}
		connectionsVector.clear();
		highConnectionsVector.clear();
	}
	inline int getConnectionSize2(int id, int dir)
	{
		return sizeOfConnections[id * 8 + dir];
	}
	inline int getHighConnectionSize2(int id, int dir)
	{
		return sizeOfHighConnections[id * 8 + dir];
	}
	inline JumpNode getConnectionItem2(int id, int dir, int i)
	{
		return rawConnectionStart[id * 8 + dir][i];
	}
	inline JumpNodeDir getHighConnectionItem2(int id, int dir, int i)
	{
		return rawHighConnectionStart[id * 8 + dir][i];
	}
	//Boundary lookup tables for the x and y axis
	vector<vector<short>> xBoundaryPoints, yBoundaryPoints;
	//vector<vector<short>> xBoundaryPointsBackup, yBoundaryPointsBackup;
	//unordered_map< int, PairNode*> keyVals;
	//Map data
	char * gridData;
	int gridWidth, gridHeight;

	JumpNode startNodeArray; //Set when reading in preprocessed data, so we can deallocate an an array as opposed to individual node allocations
	//Goal node position and index
	int eX, eY, endIndex, eXSpace[2], eYSpace[2];
	short partitionEX, partitionEY;
	vector<vector<JumpPointTableEntry>> jumpLookup[4];

	//Jump Point Lookup table and a backup
	//unsigned int *preprocessedJumpPoints, *preprocessedJumpPointsBackup;
	unordered_map<pair<short, short>, JumpNode, CoordinateHash> jumpPoints;
	int numGlobalGoals;

#ifdef PREPROCESS_END_NODES
	vector<vector<JumpNode>> backwardJumpRefs;
#else
	vector<vector<DiagonalJumpEntry>> diagonalJumps[4];
#endif

#ifdef PREPROCESS_STARTING_NODES
	vector<vector<pair<JumpNode, char>>> forwardJumpRefs;
#endif

#ifdef USE_PAIR_WISE_DISTANCES
	float **pairDist;
#endif

	void ResetSearch()
	{
		// Set last search and generated values to 0, so that when the search is incremented, all the states will be not-generated
		search = 0;
		memset(generated, 0, jumpPoints.size()*sizeof(unsigned short));
	}
	bool IsOpen(unsigned int sg){ return open[(sg >> 3)] & (1 << (sg & 7)); }
	void SetOpen(unsigned int sg){ open[sg >> 3] |= (1 << (sg & 7)); }
	void SetClosed(unsigned int sg){ open[sg >> 3] &= ~(1 << (sg & 7)); }
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
	inline bool dirIsDiagonal(const int dir)
	{
		return (dir & 1) != 0;
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
#define ENTERABLE(n) isPassable ( nextCoordinate (coord, (dir + (n)) & 7))
		if (dirIsDiagonal(dir)) {
#ifndef DIAG_UNBLOCKED
			if (!implies(ENTERABLE(6), ENTERABLE(5)))
				dirs = addDirectionToSet(dirs, (dir + 6) & 7);
			if (!implies(ENTERABLE(2), ENTERABLE(3)))
				dirs = addDirectionToSet(dirs, (dir + 2) & 7);
#endif
		}
		else {
#ifdef DIAG_UNBLOCKED
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
		return  !(gridData[index / 8] & (1 << (index & 7)));
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
	}
	void flushReProcess()
	{
		return;
		for (int y = 0; y < gridHeight; y++)
		{
			if (xBoundaryPoints[y].size() == 0)
			{
				bool currentPassable = false;
				xBoundaryPoints[y].clear();
				for (int x = 0; x < gridWidth; x++)
				{
					if (isPassable(Coordinate(x, y)) != currentPassable)
					{
						xBoundaryPoints[y].push_back(x);
						currentPassable = !currentPassable;
					}
				}
				if (currentPassable)
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
					if (isPassable(Coordinate(x, y)) != currentPassable)
					{
						yBoundaryPoints[x].push_back(y);
						currentPassable = !currentPassable;
					}
				}
				if (currentPassable)
					yBoundaryPoints[x].push_back(gridHeight);
			}
		}


	}
	int jump(const Coordinate &c, const char dir)
	{
		Coordinate nc = nextCoordinate(c, dir);
		if (!isPassable(nc))
			return -1;

		int index = gridIndex(nc);
		unsigned char dirs;
		if (dirs = forcedNeighbours(nc, dir))
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
	int jumpBuildBackwardRef(const Coordinate &c, const char dir,const char origDir, JumpNode n)
	{
		Coordinate nc = nextCoordinate(c, dir);
		bool b = true;
#ifdef DIAG_UNBLOCKED
		b = ((dir & 1) == 0) || ((dir & 1) && (isPassable(Coordinate(c.x, nc.y)) && isPassable(Coordinate(nc.x, c.y))));
#endif
		if (!b || !isPassable(nc))
			return -1;
		int index = gridIndex(nc);
		unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.find(pair<short, short>(nc.x, nc.y));
		
		if (it != jumpPoints.end())
		{
			int nDir = getDir(it->second->pos, n->pos);
			it->second->incDirs[nDir].push_back(pair<JumpNode, char>(n, origDir));
		}
		//backwardJumpRefs[index].push_back(n);

		unsigned char dirs;
		if (dirs = forcedNeighbours(nc, dir) && !dirIsDiagonal(dir))//
			return index;

		if (dirIsDiagonal(dir))
		{
			int next = jumpBuildBackwardRef(nc, (dir + 7) & 7, dir, n);
			//if (next >= 0)
			//	return index;
			next = jumpBuildBackwardRef(nc, (dir + 1) & 7, dir, n);
			//if (next >= 0)
			//	return index;
		}

		return jumpBuildBackwardRef(nc, dir, dir, n);
	}


#ifdef PREPROCESS_END_NODES
	int jumpBuildBackwardRef(const Coordinate &c, const char dir, JumpNode n)
	{
		Coordinate nc = nextCoordinate(c, dir);
		bool b = true;
#ifdef DIAG_UNBLOCKED
		b = ((dir & 1) == 0) || ((dir & 1) && (isPassable(Coordinate(c.x, nc.y)) && isPassable(Coordinate(nc.x, c.y))));
#endif
		if (!b || !isPassable(nc))
			return -1;
		int index = gridIndex(nc);
		backwardJumpRefs[index].push_back(n);

		unsigned char dirs;
		if (dirs = forcedNeighbours(nc, dir) && !dirIsDiagonal(dir))//
			return index;

		if (dirIsDiagonal(dir))
		{
			int next = jumpBuildBackwardRef(nc, (dir + 7) & 7, n);
			//if (next >= 0)
			//	return index;
			next = jumpBuildBackwardRef(nc, (dir + 1) & 7, n);
			//if (next >= 0)
			//	return index;
		}

		return jumpBuildBackwardRef(nc, dir, n);
	}
#endif
	void verifyreProcessing()
	{

	}
	void reProcessGrid(int lx, int rx, int ty, int by)
	{

	}
	void backupPreProcess()
	{

	}
	void useBackupData()
	{

	}
	void insertionOrdered(vector<JumpPointTableEntry> & vec, JumpPointTableEntry v)
	{
		for (int i = 0; i < vec.size(); i++)
			if (vec[i].second > v.second)
			{
				vec.insert(vec.begin() + i, v);
				return;
			}
		vec.push_back(v);
	}
	char getForcedGridDir(short x, short y)
	{
		char dirs = 0;

#ifdef DIAG_UNBLOCKED
		if (!isPassable(Coordinate(x - 1, y - 1)))
			dirs |= (1 << 2) | (1 << 4) | (1 << 1) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 6) | (1 << 0);
		if (!isPassable(Coordinate(x + 1, y + 1)))
			dirs |= (1 << 6) | (1 << 0) | (1 << 1) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 2) | (1 << 4);
		if (!isPassable(Coordinate(x + 1, y - 1)))
			dirs |= (1 << 6) | (1 << 4) | (1 << 1) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 2) | (1 << 0);
		if (!isPassable(Coordinate(x - 1, y + 1)))
			dirs |= (1 << 2) | (1 << 0) | (1 << 1) | (1 << 3) | (1 << 5) | (1 << 7) | (1 << 6) | (1 << 4);
		return dirs;
#endif

		if (!isPassable(Coordinate(x, y - 1)) && isPassable(Coordinate(x + 1, y - 1)) && isPassable(Coordinate(x - 1, y - 1)))
			return 0xFF - (1 << 4);
		if (!isPassable(Coordinate(x, y + 1)) && isPassable(Coordinate(x + 1, y + 1)) && isPassable(Coordinate(x - 1, y + 1)))
			return 0xFF - (1 << 0);
		if (!isPassable(Coordinate(x - 1, y)) && isPassable(Coordinate(x - 1, y - 1)) && isPassable(Coordinate(x - 1, y + 1)))
			return 0xFF - (1 << 2);
		if (!isPassable(Coordinate(x + 1, y)) && isPassable(Coordinate(x + 1, y - 1)) && isPassable(Coordinate(x + 1, y + 1)))
			return 0xFF - (1 << 6);


		if ((!isPassable(Coordinate(x - 1, y - 1)) && !isPassable(Coordinate(x, y - 1))) || (!isPassable(Coordinate(x - 1, y + 1)) && !isPassable(Coordinate(x, y + 1))))
			dirs |= (1 << 1) | (1 << 2) | (1 << 3);
		if ((!isPassable(Coordinate(x + 1, y - 1)) && !isPassable(Coordinate(x, y - 1))) || (!isPassable(Coordinate(x + 1, y + 1)) && !isPassable(Coordinate(x, y + 1))))
			dirs |= (1 << 5) | (1 << 6) | (1 << 7);

		if ((!isPassable(Coordinate(x - 1, y - 1)) && !isPassable(Coordinate(x - 1, y))) || (!isPassable(Coordinate(x + 1, y - 1)) && !isPassable(Coordinate(x + 1, y))))
			dirs |= (1 << 3) | (1 << 4) | (1 << 5);

		if ((!isPassable(Coordinate(x - 1, y + 1)) && !isPassable(Coordinate(x - 1, y))) || (!isPassable(Coordinate(x + 1, y + 1)) && !isPassable(Coordinate(x + 1, y))))
			dirs |= (1 << 7) | (1 << 0) | (1 << 1);

		return dirs;
	}
	void preprocessBoundaryLookupTables()
	{
		for (int y = 0; y < gridHeight; y++)
		{
			bool currentPassable = false;
			if (xBoundaryPoints.size() <= y)
				xBoundaryPoints.push_back(vector<short>());
			else
				xBoundaryPoints[y].clear();
			for (int x = 0; x < gridWidth; x++)
			{
				if (isPassable(Coordinate(x, y)) != currentPassable)
				{
					xBoundaryPoints[y].push_back(x);
					currentPassable = !currentPassable;
				}
			}
			//if (currentPassable)
			xBoundaryPoints[y].push_back(gridWidth);
		}

		for (int x = 0; x < gridWidth; x++)
		{
			bool currentPassable = false;
			if (yBoundaryPoints.size() <= x)
				yBoundaryPoints.push_back(vector<short>());
			else
				yBoundaryPoints[x].clear();
			for (int y = 0; y < gridHeight; y++)
			{
				if (isPassable(Coordinate(x, y)) != currentPassable)
				{
					yBoundaryPoints[x].push_back(y);
					currentPassable = !currentPassable;
				}
			}
			//if (currentPassable)
			yBoundaryPoints[x].push_back(gridHeight);
		}
	}
	void preprocessJumpPointLookupTables(vector<JumpNode> &allNodes)
	{
		jumpLookup[1].resize(gridHeight);
		jumpLookup[3].resize(gridHeight);
		jumpLookup[0].resize(gridWidth);
		jumpLookup[2].resize(gridWidth);

		map<pair<short, short>, vector<pair<JumpNode, int> > > linkCardinalJPS;
		//East
		for (int y = 0; y < gridHeight; y++)
		{
			//jumpLookup[1].push_back(vector<JumpPointTableEntry>());
			vector<JumpPointTableEntry> & vec = jumpLookup[1][y];
			for (int xId = 0; xId < xBoundaryPoints[y].size(); xId += 2)
			{
				int x = xBoundaryPoints[y][xId];
				JumpNode lastPoint = 0;
				int index;
				bool attemptedSet = true;
				bool linkCardinalFlag = false;
				bool addedToList = false;
				do
				{
					index = jump(Coordinate(x, y), 2);
#ifdef DIAG_UNBLOCKED
					if (index != -1 && attemptedSet)
					{
						unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.find(pair<short, short>(x, y));
						attemptedSet = false;
						if (it != jumpPoints.end())
							lastPoint = it->second;
						else
							linkCardinalFlag = true;
					}
#endif
					if (index != -1)
					{
						int newX = indexToCoordinate(index).x;
						JumpNode newPoint = 0;
						//newPoint = new JumpPointNode(newX, y, jumpPoints.size());
						unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.find(pair<short, short>(newX, y));

						if (it == jumpPoints.end())
						{
							newPoint = new JumpPointNode2(newX, y, jumpPoints.size());
							jumpPoints.insert(pair<pair<short, short>, JumpNode>(pair<short, short>(newX, y), newPoint));
							connectionsVector.resize(connectionsVector.size()+8);
						}
						else
						{
							newPoint = it->second;
						}
						newPoint->addNatForcedNeighbours(naturalNeighbours(2) | forcedNeighbours(newPoint->pos, 2), 2);
						if (addedToList)
							vec.back().third = newX - 1;
						vec.push_back(JumpPointTableEntry(x, newX, xBoundaryPoints[y][xId+1], newPoint));
						addedToList = true;
						if (lastPoint)
							addConnection(lastPoint->id, 2, newPoint);
							//lastPoint->addNeighbour(newPoint, 2);
						else if (linkCardinalFlag)
						{
							map<pair<short, short>, vector<pair<JumpNode, int> > >::iterator it = linkCardinalJPS.find(pair<short, short>(x, y));
							if (it == linkCardinalJPS.end())
								linkCardinalJPS[pair<short, short>(x, y)] = vector<pair<JumpNode, int> >();
							linkCardinalJPS[pair<short, short>(x, y)].push_back(pair<JumpNode, short>(newPoint, 2));

							linkCardinalFlag = false;
						}
						lastPoint = newPoint;

						x = newX;
					}
				} while (index != -1);
			}
		}
		//West
		for (int y = 0; y < gridHeight; y++)
		{
			//jumpLookup[3].push_back(vector<JumpPointTableEntry>());
			vector<JumpPointTableEntry> & vecB = jumpLookup[3][y];
			for (int xId = 1; xId < xBoundaryPoints[y].size(); xId += 2)
			{
				int x = xBoundaryPoints[y][xId] - 1;
				int index;
				JumpNode lastPoint = 0;
				bool attemptedSet = true;
				bool linkCardinalFlag = false;
				bool addedToList=false;
				do
				{
					index = jump(Coordinate(x, y), 6);
#ifdef DIAG_UNBLOCKED
					if (index != -1 && attemptedSet)
					{
						unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.find(pair<short, short>(x, y));
						attemptedSet = false;
						if (it != jumpPoints.end())
							lastPoint = it->second;
						else
							linkCardinalFlag = true;
					}
#endif
					if (index != -1)
					{
						int newX = indexToCoordinate(index).x;
						JumpNode newPoint = 0;
						unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.find(pair<short, short>(newX, y));

						if (it == jumpPoints.end())
						{
							newPoint = new JumpPointNode2(newX, y, jumpPoints.size());
							jumpPoints.insert(pair<pair<short, short>, JumpNode>(pair<short, short>(newX, y), newPoint));
							connectionsVector.resize(connectionsVector.size() + 8);

						}
						else
						{
							newPoint = it->second;
						}
						if (addedToList)
							vecB.back().third = newX;
						insertionOrdered(vecB, JumpPointTableEntry(x, newX, xBoundaryPoints[y][xId-1], newPoint));

						newPoint->addNatForcedNeighbours(naturalNeighbours(6) | forcedNeighbours(newPoint->pos, 6), 6);
						addedToList = true;
						if (lastPoint)
							addConnection(lastPoint->id, 6, newPoint);

							//lastPoint->addNeighbour(newPoint, 6);
						else if (linkCardinalFlag)
						{
							map<pair<short, short>, vector<pair<JumpNode, int> > >::iterator it = linkCardinalJPS.find(pair<short, short>(x, y));
							if (it == linkCardinalJPS.end())
								linkCardinalJPS[pair<short, short>(x, y)] = vector<pair<JumpNode, int> >();
							linkCardinalJPS[pair<short, short>(x, y)].push_back(pair<JumpNode, short>(newPoint, 6));
							linkCardinalFlag = false;
						}
						lastPoint = newPoint;

						x = newX;
					}
				} while (index != -1);
			}
		}
		//South
		for (int x = 0; x < gridWidth; x++)
		{
			//jumpLookup[2].push_back(vector<JumpPointTableEntry>());
			vector<JumpPointTableEntry> & vec = jumpLookup[2][x];// .back();
			for (int yId = 0; yId < yBoundaryPoints[x].size(); yId += 2)
			{
				int y = yBoundaryPoints[x][yId];
				int index;
				JumpNode lastPoint = 0;
				bool attemptedSet = true;
				bool linkCardinalFlag = false;
				bool addedToList = false;
				
				do
				{
					index = jump(Coordinate(x, y), 4);
#ifdef DIAG_UNBLOCKED
					if (index != -1 && attemptedSet)
					{
						unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.find(pair<short, short>(x, y));
						attemptedSet = false;
						if (it != jumpPoints.end())
							lastPoint = it->second;
						else
							linkCardinalFlag = true;
					}
#endif
					if (index != -1)
					{
						int newY = indexToCoordinate(index).y;
						JumpNode newPoint = 0;
						unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.find(pair<short, short>(x, newY));
						if (it == jumpPoints.end())
						{
							newPoint = new JumpPointNode2(x, newY, jumpPoints.size());
							jumpPoints.insert(pair<pair<short, short>, JumpNode>(pair<short, short>(x, newY), newPoint));
							connectionsVector.resize(connectionsVector.size() + 8);

						}
						else
						{
							newPoint = it->second;
						}
						newPoint->addNatForcedNeighbours(naturalNeighbours(4) | forcedNeighbours(newPoint->pos, 4), 4);
						if (addedToList )
							vec.back().third = newY - 1;

						vec.push_back(JumpPointTableEntry(y, newY, yBoundaryPoints[x][yId+1], newPoint));
						addedToList = true;
						if (lastPoint)
							addConnection(lastPoint->id, 4, newPoint);

							//lastPoint->addNeighbour(newPoint, 4);
						else if (linkCardinalFlag)
						{
							map<pair<short, short>, vector<pair<JumpNode, int> > >::iterator it = linkCardinalJPS.find(pair<short, short>(x, y));
							if (it == linkCardinalJPS.end())
								linkCardinalJPS[pair<short, short>(x, y)] = vector<pair<JumpNode, int> >();
							linkCardinalJPS[pair<short, short>(x, y)].push_back(pair<JumpNode, short>(newPoint, 4));
							linkCardinalFlag = false;
						}
						lastPoint = newPoint;
						y = newY;
					}
				} while (index != -1);
			}
		}
		//North
		for (int x = 0; x < gridWidth; x++)
		{
			//jumpLookup[0].push_back(vector<JumpPointTableEntry>());
			vector<JumpPointTableEntry> & vecB = jumpLookup[0][x];
			for (int yId = 1; yId < yBoundaryPoints[x].size(); yId += 2)
			{
				int y = yBoundaryPoints[x][yId] - 1;
				int index;
				JumpNode lastPoint = 0;
				bool attemptedSet = true;
				bool linkCardinalFlag = false;
				bool addedToList = false;

				do
				{
					index = jump(Coordinate(x, y), 0);
#ifdef DIAG_UNBLOCKED
					if (index != -1 && attemptedSet)
					{
						unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.find(pair<short, short>(x, y));
						attemptedSet = false;
						if (it != jumpPoints.end())
							lastPoint = it->second;
						else
							linkCardinalFlag = true;
					}
#endif
					if (index != -1)
					{
						int newY = indexToCoordinate(index).y;
						JumpNode newPoint = 0;
						unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.find(pair<short, short>(x, newY));
						if (it == jumpPoints.end())
						{
							newPoint = new JumpPointNode2(x, newY, jumpPoints.size());
							jumpPoints.insert(pair<pair<short, short>, JumpNode>(pair<short, short>(x, newY), newPoint));
							connectionsVector.resize(connectionsVector.size() + 8);

						}
						else
						{
							newPoint = it->second;
						}
						newPoint->addNatForcedNeighbours(naturalNeighbours(0) | forcedNeighbours(newPoint->pos, 0), 0);
						if (addedToList)
							vecB.back().third = newY;

						insertionOrdered(vecB, JumpPointTableEntry(y, newY, yBoundaryPoints[x][yId - 1], newPoint));
						addedToList = true;

						if (lastPoint)						
						{
							addConnection(lastPoint->id, 0, newPoint);
						}

							//lastPoint->addNeighbour(newPoint, 0);
						else if (linkCardinalFlag)
						{
							map<pair<short, short>, vector<pair<JumpNode, int> > >::iterator it = linkCardinalJPS.find(pair<short, short>(x, y));
							if (it == linkCardinalJPS.end())
								linkCardinalJPS[pair<short, short>(x, y)] = vector<pair<JumpNode, int> >();
							linkCardinalJPS[pair<short, short>(x, y)].push_back(pair<JumpNode, short>(newPoint, 0));
							linkCardinalFlag = false;
						}
						lastPoint = newPoint;
						y = newY;
					}
				} while (index != -1);
			}
		}
#ifdef DIAG_UNBLOCKED
		{
			map<pair<short, short>, vector<pair<JumpNode, int> > >::iterator it = linkCardinalJPS.begin();
			while (it != linkCardinalJPS.end())
			{
				unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator jumpPointNew = jumpPoints.find(pair<short, short>(it->first.first, it->first.second));
				if (jumpPointNew != jumpPoints.end())
				{
					for (int i = 0; i < it->second.size(); i++)
					{
						addConnection((*jumpPointNew).second->id, it->second[i].second, it->second[i].first);

						//(*jumpPointNew).second->addNeighbour(it->second[i].first, it->second[i].second);
					}
				}
				it++;
			}
		}
#endif

#ifdef PREPROCESS_END_NODES
		backwardJumpRefs.resize(gridHeight*gridWidth);
#endif
		{
			allNodes.resize(jumpPoints.size());
			int id = 0;
			unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.begin();
			while (it != jumpPoints.end())
			{
				allNodes[id] = (it->second);
				it++;
				id++;
			}
		}
		for (int i = 0; i < allNodes.size(); i++)
		{
			int x = allNodes[i]->pos.x;
			int y = allNodes[i]->pos.y;
			for (int dir = 0; dir < 8; dir++)
			{
				allNodes[i]->addNatForcedNeighbours(naturalNeighbours(dir) | forcedNeighbours(allNodes[i]->pos, dir), dir);
				jumpBuildBackwardRef(allNodes[i]->pos, dir, dir, allNodes[i]);
			}

			for (int dir = 1; dir < 8; dir += 2)
			{
				if (isPassable(nextCoordinate(allNodes[i]->pos, (dir + 4) & 7)) || (allNodes[i]->neighbours[(dir + 1) & 7] & (1 << dir)) || (allNodes[i]->neighbours[(dir + 7) & 7] & (1 << dir)))
				{
					vector<JumpNode> diagPoints;
					jumpNewDiags(allNodes[i]->pos, dir, diagPoints);
					//if (diagPoints.size())
					//	allNodes[i]->addNatForcedNeighbours(naturalNeighbours(dir) | forcedNeighbours(allNodes[i]->pos, dir), dir);
					for (int ii = 0; ii < diagPoints.size(); ii++)
					{
						JumpNode newPoint = diagPoints[ii];
						/*if (jumpPoints.find(pair<short, short>(diagPoints[ii].x, diagPoints[ii].y)) == jumpPoints.end())
						{
						newPoint = new JumpPointNode(diagPoints[ii].x, diagPoints[ii].y, jumpPoints.size());
						jumpPoints.insert(pair<pair<short, short>, JumpNode>(pair<short, short>(diagPoints[ii].x, diagPoints[ii].y), newPoint));
						allNodes.push_back(newPoint);
						}
						else
						{
						newPoint = jumpPoints.find(pair<short, short>(diagPoints[ii].x, diagPoints[ii].y))->second;
						}*/
						JumpNode newJP;
						if (getJumpPointNew(diagPoints[ii]->pos, (dir + 1) & 7, newJP))
							addConnection(newPoint->id, (dir + 1) & 7,newJP );
						//	newPoint->addToDirs(newJP, (dir + 1) & 7);
						if (getJumpPointNew(diagPoints[ii]->pos, (dir + 7) & 7, newJP))
							addConnection(newPoint->id, (dir + 7) & 7, newJP);

						//	newPoint->addToDirs(newJP, (dir + 7) & 7);
						addConnection(allNodes[i]->id, dir, newPoint);

						//allNodes[i]->addNeighbour(newPoint, dir);
					}
					JumpNode newJP;
					if (getJumpPointNew(allNodes[i]->pos, (dir + 1) & 7, newJP))
						addConnection(allNodes[i]->id, (dir + 1) & 7, newJP);

						//allNodes[i]->addToDirs(newJP, (dir + 1) & 7);
					if (getJumpPointNew(allNodes[i]->pos, (dir + 7) & 7, newJP))
						addConnection(allNodes[i]->id, (dir + 7) & 7, newJP);
						//allNodes[i]->addToDirs(newJP, (dir + 7) & 7);
					//allNodes[i]->addNeighbour(newPoint, dir);
				}
			}
#ifdef PREPROCESS_END_NODES

			Coordinate nc = allNodes[i]->pos;

			char dirs = getForcedGridDir(allNodes[i]->pos.x, allNodes[i]->pos.y);
			backwardJumpRefs[gridIndex(allNodes[i]->pos)].push_back(allNodes[i]);
			for (int dir = 0; dir < 8; dir++)
				if ((1 << dir)&dirs)
				{
					jumpBuildBackwardRef(allNodes[i]->pos, dir, allNodes[i]);
				}
#endif
			highConnectionsVector.resize(connectionsVector.size());
		}


#ifdef PREPROCESS_STARTING_NODES
		forwardJumpRefs.resize(gridHeight*gridWidth);
		for (int y = 0; y < gridHeight; y++)
			for (int x = 0; x < gridWidth; x++)
			{
				if (isPassable(Coordinate(x, y)))
				{
					vector<pair<JumpNode, char>> startNodes;
					for (int dir = 0; dir < 8; dir++)
					{
						if (dirIsDiagonal(dir))
						{
							vector<JumpNode> diagJumps;
							jumpNewDiags(Coordinate(x, y), dir, diagJumps);
							for (int i = 0; i < diagJumps.size(); i++)
								startNodes.push_back(pair<JumpNode, char>(diagJumps[i], dir));
						}
						else
						{
							JumpNode jp = jumpNew(Coordinate(x, y), dir);
							if (jp)
							{
								//Coordinate nc = indexToCoordinate(index);
								startNodes.push_back(pair<JumpNode, char>(jp, dir));
							}
						}
					}
					forwardJumpRefs[x + y*gridWidth] = (startNodes);
				}
			}
#endif
	}
	Coordinate diagonalDirection(Coordinate c, int dir)
	{
#ifdef DIAG_UNBLOCKED
		while (isPassable(c) && isPassable(Coordinate(nextCoordinate(c, dir).x, c.y)) && isPassable(Coordinate(c.x, nextCoordinate(c, dir).y)))
#else
		while (isPassable(c))
#endif
			c = nextCoordinate(c, dir);
		if (!isPassable(c))
			c = nextCoordinate(c, (dir + 4) & 7);
		return c;

	}
	void preProcessSubGoalGraph(vector<JumpNode> &allNodes)
	{
		incomingConnectionsVector.resize(allNodes.size() * 8);

		vector<JumpPointNode2*> allNodes2;
		for (int i = 0; i < allNodes.size(); i++)
		{
			bool flag = false;
			for (int ii = 0; ii < allNodes2.size() && !flag; ii++)
				//if (allNodes[i]->pos.y < allNodes2[ii]->pos.y || (allNodes[i]->pos.y == allNodes2[ii]->pos.y && allNodes[i]->pos.x < allNodes2[ii]->pos.x))
				if (allNodes[i]->id <allNodes2[ii]->id)
				{
					allNodes2.insert(allNodes2.begin() + ii, allNodes[i]);
					flag = true;
				}
			if (!flag)
				allNodes2.push_back(allNodes[i]);

		}


		vector<Coordinate> solution;
		for (int i = 0; i < allNodes.size(); i++)
			for (int dirI = 0; dirI < 8; dirI++)
				for (int ii = 0; ii < getConnectionSize(allNodes[i]->id, dirI) ; ii++)
				{

					//int newDir = getDiagDir(allNodes[i]->pos, allNodes[i]->getDirItem(dirI, ii)->pos, dirI);
					int newDir = getDiagDir(allNodes[i]->pos, getConnectionItem(allNodes[i]->id,dirI, ii)->pos, dirI);

					
					//addIncomingConnection(allNodes[i]->id, newDir, pair<JumpNode, char>(allNodes[i], dirI));
					addIncomingConnection(getConnectionItem(allNodes[i]->id, dirI, ii)->id, newDir, pair<JumpNode, char>(allNodes[i], dirI));
					//allNodes[i]->getDirItem(dirI, ii)->addToIncDirs(pair<JumpNode, char>(allNodes[i], dirI), newDir);
				}
		for (int i = 0; i < allNodes.size(); i++)
		{
			for (int dir = 0; dir < 8; dir++)
			{
				bool found = 0;
				Coordinate nc = allNodes[i]->pos;
				{
					char dirs = naturalNeighbours(dir) | forcedNeighbours(allNodes[i]->pos, dir);

					for (int dirI = 0; dirI < 8 && !found; dirI++)
					{
						if (dirs&(1 << dirI))
							for (int ii = 0; ii < getConnectionSize(allNodes[i]->id, dirI) && !found; ii++)
							{
								//int newDir = getDiagDir(allNodes[i]->pos, allNodes[i]->incDirs[dirI][ii]->pos, dir);
								char dirs2 = 0xff - ((1 << (dirI + 4) & 7) | (1 << ((dirI + 1 + 4) & 7)) | (1 << ((dirI - 1 + 4) & 7)));// naturalNeighbours(newDir) | forcedNeighbours(allNodes[i]->dirs[dirI][ii]->pos, newDir);
								for (int dirII = 0; dirII < 8 && !found; dirII++)
								{
									//char dirs2B = naturalNeighbours((dirII + 4) & 7) | forcedNeighbours(allNodes[i]->pos, (dirII + 4) & 7);

									if (dirs2&(1 << dirII))// && dirs2B&(1 << dirII))
									{
										int incSize = getNoIncConnections(allNodes[i]->id, dirII);
										for (int iii = 0; iii < incSize && !found; iii++)
										{
											solution.clear();
											JumpNode b = allNodes[i];
											JumpNode c = getConnectionItem(allNodes[i]->id, dirI, ii);// dirs[dirI][ii];
											JumpNode a = getIncomingConnection(allNodes[i]->id, dirII, iii).first;// allNodes[i]->incDirs[dirII][iii].first;
											bool sameDist = abs(Node::estimateDistance(a->pos, c->pos) - Node::estimateDistance(a->pos, b->pos) - Node::estimateDistance(b->pos, c->pos)) < 0.1;
											if (!sameDist)
												found = true;
											if (!directSolution(c->pos.x, c->pos.y, a->pos.x, a->pos.y, solution))
												found = true;
										}
									}
								}
							}
					}
					if (!found)
					{
						JumpNode a = allNodes[i];

						if (getConnectionSize(allNodes[i]->id, dir)==0)
							allNodes[i]->addEdgeNode(dir);
						allNodes[i]->pruneEdge(dir);
						//char dirs = naturalNeighbours(dir) | forcedNeighbours(allNodes[i]->pos, dir);
						char dirs = ((1 << ((dir)& 7)) | (1 << ((dir + 1) & 7)) | (1 << ((dir + 7) & 7)));
						for (int dirII = 0; dirII < 8; dirII++)
							if (dirs&(1 << dirII))
							{
								for (int iii = 0; iii <getNoIncConnections(allNodes[i]->id, dirII); iii++)
									for (int iv = 0; iv < getConnectionSize(allNodes[i]->id, dir); iv++)
									{
										char oldDir = getIncomingConnection(allNodes[i]->id, dirII, iii).second;
										//if (oldDir == dirII)
										{
											JumpNode b = getConnectionItem(allNodes[i]->id, dir, iv);// dirs[dir][iv];
											pair<JumpNode, char> c = getIncomingConnection(allNodes[i]->id, dirII, iii);
											addConnection(c.first->id,oldDir,b);
											//c.first->addToDirs(b, oldDir);
											addIncomingConnection(b->id, dirII,c);
											//b->addToIncDirs(c, dirII);

											//if (dir == 3 && c->pos.y > b->pos.y)
											//	int aa = 1;

										}
									}

								if (dirIsDiagonal(dirII))
								{
									char dirIIOff = (dirII + 1) & 7;

									for (int iii = 0; iii < getNoIncConnections(allNodes[i]->id, dirIIOff); iii++)
										for (int iv = 0; iv < getConnectionSize(allNodes[i]->id, dir); iv++)
										{
											char oldDir = getIncomingConnection(allNodes[i]->id, dirIIOff, iii).second;

											if (oldDir == dirII)
											{
												JumpNode b = getConnectionItem(allNodes[i]->id, dir, iv);
												pair<JumpNode, char> c = getIncomingConnection(allNodes[i]->id, dirIIOff, iii);
												//c.first->addToDirs(b, oldDir);
												addConnection(c.first->id, oldDir, b);

												addIncomingConnection(b->id, dirII, c);

												//b->addToIncDirs(c, dirII);
											}
										}
									dirIIOff = (dirII + 7) & 7;

									for (int iii = 0; iii < getNoIncConnections(allNodes[i]->id, dirIIOff); iii++)
										for (int iv = 0; iv < getConnectionSize(allNodes[i]->id, dir); iv++)
										{
											char oldDir = getIncomingConnection(allNodes[i]->id, dirIIOff, iii).second;

											if (oldDir == dirII)
											{
												JumpNode b = getConnectionItem(allNodes[i]->id, dir, iv);
												pair<JumpNode, char> c = getIncomingConnection(allNodes[i]->id, dirIIOff, iii);
												//c.first->addToDirs(b, oldDir);
												addConnection(c.first->id, oldDir, b);

												//b->addToIncDirs(c, dirII);
												addIncomingConnection(b->id, dirII, c);

											}
										}

								}

							}
					}

				}


			}
		}
		//	printf("edge nodes:%d %d : %4.2f %%\n", numUselessNodes, allNodes.size()*8,numUselessNodes / ((float)allNodes.size()*8) * 100);
		for (int i = 0; i < allNodes.size(); i++)
		{
			Coordinate nc = allNodes[i]->pos;

			for (int dirI = 0; dirI < 8; dirI++)
				for (int ii = 0; ii < getConnectionSize(allNodes[i]->id, dirI); ii++)
				{
					solution.clear();
					JumpNode a = getConnectionItem(allNodes[i]->id, dirI, ii);
					//bool hasDirectSolution = directSolution(allNodes[i]->pos.x, allNodes[i]->pos.y, a->pos.x, a->pos.y, solution);
					bool firstConnection = false;
					directSolution(allNodes[i]->pos, a->pos, firstConnection);
					char d2 = getDir2(allNodes[i]->pos, a->pos);
					bool b1 = (firstConnection && (!a->isPrunedEdge(dirI) || a->isEdgeNode(dirI)));//|| allNodes[i]->isEdgeNode(dirI)
					bool b2 = (!firstConnection && (!a->isPrunedEdge(d2) || a->isEdgeNode(d2)));//|| allNodes[i]->dirs[dirI][ii]->isEdgeNode(d2)
					int id = allNodes[i]->id;
					if (b1 || b2)// || (!firstConnection&&hasDirectSolution))
					{
						if (firstConnection)
							addHighConnection(allNodes[i]->id, dirI, pair<JumpNode, char>(a, getDiagDir(allNodes[i]->pos, a->pos, getDir(allNodes[i]->pos, a->pos))));
							//allNodes[i]->addHighGoal(pair<JumpNode, char>(a, getDiagDir(allNodes[i]->pos, a->pos, getDir(allNodes[i]->pos, a->pos))), dirI);
						//allNodes[i]->highGoals[dirI].push_back(pair<JumpNode, char>(a, getDiagDir(allNodes[i]->pos, a->pos, getDir(allNodes[i]->pos, a->pos))));
						else
							addHighConnection(allNodes[i]->id, dirI, pair<JumpNode, char>(a, d2));

							//allNodes[i]->addHighGoal(pair<JumpNode, char>(a, d2), dirI);
						//allNodes[i]->highGoals[dirI].push_back(pair<JumpNode, char>(a, d2));

					}
				}
		}
		for (int i = 0; i < allNodes.size(); i++)
			for (int dirI = 0; dirI < 8; dirI++)
				for (int ii = 0; ii < getNoIncConnections(allNodes[i]->id, dirI) ; ii++)
				{
					if (getIncomingConnection(allNodes[i]->id, dirI, ii).second != dirI)
					{
						addIncomingConnection(allNodes[i]->id, getIncomingConnection(allNodes[i]->id, dirI, ii).second, getIncomingConnection(allNodes[i]->id, dirI, ii));
						eraseIncomingConnection(allNodes[i]->id, dirI, ii);
						//allNodes[i]->incDirs[dirI].erase(allNodes[i]->incDirs[dirI].begin() + ii);
						ii--;
					}
				}
#ifndef PREPROCESS_END_NODES
		for (int i = 0; i < 4; i++)
			diagonalJumps[i].resize(gridWidth + gridHeight);

		for (int i = 0; i < allNodes.size(); i++)
		{
			Coordinate c = allNodes[i]->pos;
			for (int dir = 1; dir < 8; dir += 2)
				if (getConnectionSize(allNodes[i]->id, dir) || (allNodes[i]->neighbours[(dir + 1) & 7] & (1 << dir)) || (allNodes[i]->neighbours[(dir + 7) & 7] & (1 << dir)))
				{
					Coordinate from(allNodes[i]->pos);
					Coordinate to(diagonalDirection(allNodes[i]->pos, dir));

					if (!(from.x == to.x &&  from.y == to.y))
					{
						if (dir == 1 || dir == 5)
							addByX(diagonalJumps[(dir - 1) / 2][from.x + from.y], DiagonalJumpEntry(from, to, allNodes[i]));
						//diagonalJumps[(dir - 1) / 2][from.x + from.y].push_back(DiagonalJumpEntry(from, to, allNodes[i]));
						else
							addByX(diagonalJumps[(dir - 1) / 2][from.x + (gridHeight - from.y - 1)], DiagonalJumpEntry(from, to, allNodes[i]));

						//diagonalJumps[(dir - 1) / 2][from.x + (gridHeight-from.y-1)].push_back(DiagonalJumpEntry(from, to, allNodes[i]));
					}

					int a = 1;
				}
		}

#endif
		for (int i = 0; i < allNodes.size(); i++)
			for (int dir = 0; dir < 8; dir++)
				for (int j = 0; j < getConnectionSize(i,dir); j++)
					for (int k = 0; k < getConnectionSize(i, dir); k++)
					{
						if (getConnectionItem(i, dir, k)->pos.y > getConnectionItem(i, dir, j)->pos.y || (getConnectionItem(i, dir, k)->pos.y == getConnectionItem(i, dir, j)->pos.y && getConnectionItem(i, dir, k)->pos.x > getConnectionItem(i, dir, j)->pos.x))
							swap(connectionsVector[i * 8 + dir][k], connectionsVector[i * 8 + dir][j]);
					}
		for (int i = 0; i < allNodes.size(); i++)
			for (int dir = 0; dir < 8; dir++)
				for (int j = 0; j < getHighConnectionSize(i, dir); j++)
					for (int k = 0; k < getHighConnectionSize(i, dir); k++)
					{
						if (getHighConnectionItem(i, dir, k).first->pos.y > getHighConnectionItem(i, dir, j).first->pos.y || (getHighConnectionItem(i, dir, k).first->pos.y == getHighConnectionItem(i, dir, j).first->pos.y && getHighConnectionItem(i, dir, k).first->pos.x > getHighConnectionItem(i, dir, j).first->pos.x))
							swap(highConnectionsVector[i * 8 + dir][k], highConnectionsVector[i * 8 + dir][j]);
					}
		FILE * fp = fopen("test.txt", "w");
		for (int i = 0; i < allNodes2.size(); i++)
		{
			fprintf(fp, "(%d,%d):", allNodes2[i]->pos.x, allNodes2[i]->pos.y);
		//	for (int ii = 0; ii < 8; ii++)

		//		for (int j = 0; j < getConnectionSize(allNodes2[i]->id, ii); j++)
		//			fprintf(fp, "(%d,%d),", getConnectionItem(allNodes2[i]->id, ii, j)->pos.x, getConnectionItem(allNodes2[i]->id, ii, j)->pos.y, -1);//ii

			for (int ii = 0; ii < 8; ii++)

				for (int j = 0; j < getHighConnectionSize(allNodes2[i]->id, ii); j++)
				{
					int sec = getHighConnectionItem(allNodes2[i]->id, ii, j).second;
					fprintf(fp, "(%d,%d[%d]),", getHighConnectionItem(allNodes2[i]->id, ii, j).first->pos.x, getHighConnectionItem(allNodes2[i]->id, ii, j).first->pos.y, getHighConnectionItem(allNodes2[i]->id, ii, j).second);
				}
			/*fprintf(fp, "  ||  ", allNodes2[i]->pos.x, allNodes2[i]->pos.y);

			for (int ii = 0; ii < 8; ii++)
			{
			for (int j = 0; j < allNodes2[i]->incDirs[ii].size(); j++)
			fprintf(fp, "(%d,%d[%d]),", allNodes2[i]->incDirs[ii][j].first->pos.x, allNodes2[i]->incDirs[ii][j].first->pos.y, allNodes2[i]->incDirs[ii][j].second);


			}*/
			fprintf(fp, "\n");
		}
		fclose(fp);

	}
	void addByX(vector<DiagonalJumpEntry> & vec, DiagonalJumpEntry val)
	{
		for (int i = 0; i < vec.size(); i++)
			if (vec[i].from.x > val.from.x)
			{
				vec.insert(vec.begin() + i, val);
				return;
			}
		vec.push_back(val);
	}
	void addgobalGoalNode(JumpNode jp, vector<JumpNode> & globalGoals, int &count)
	{
		for (int d = 0; d < 8; d++)
		{
			getHighConnectionSize(jp->id, d);
			int numHighGoals = getHighConnectionSize(jp->id, d); //jp->getNumHighGoals(d);
			for (int ii = 0; ii < numHighGoals; ii++)
			{
				JumpNode newJp = getHighConnectionItem(jp->id,d,ii).first;// jp->getHighGoal(d, ii).first;
				if (newJp->globalGoalId == -1)
				{
					newJp->setGlobalGoalId(count);
					globalGoals.push_back(newJp);
					count++;
				}
			}
		}
	}
	void dumpJumpPointData(FILE* fp, vector<JumpPointTableEntry > & vec)
	{
		short numBoundaries = vec.size();
		fwrite(&numBoundaries, 2, 1, fp);
		for (int i = 0; i < vec.size(); i++)
		{
			fwrite(&vec[i], 4, 1, fp);
			fwrite(&vec[i].jp->id, 4, 1, fp);
		}
	}
	void readJumpPointData(FILE* fp, vector<JumpPointTableEntry > & vec, JumpNode &jps)
	{
		short numBoundaries = 0;
		fread(&numBoundaries, 2, 1, fp);

		for (int i = 0; i < numBoundaries; i++)
		{
			JumpPointTableEntry temp(0, 0, 0,0);
			fread(&temp, 4, 1, fp);
			int JPId = 0;
			fread(&JPId, 4, 1, fp);
			temp.jp = &jps[JPId];
			vec.push_back(temp);
		}
	}
	void readAllJumpPointData(FILE* fp, JumpNode &jps)
	{
		int numberOfJumpPoints = 0;
		fread(&numberOfJumpPoints, sizeof(int), 1, fp);
		jps = new JumpPointNode2[numberOfJumpPoints];
		startNodeArray = jps;
		for (int index = 0; index < numberOfJumpPoints; index++)
		{
			Coordinate p(0, 0);
			int id = 0;
			fread(&p, sizeof(Coordinate), 1, fp);
			fread(&id, sizeof(int), 1, fp);
			JumpNode jp = &jps[id];
			*jp = JumpPointNode2(p.x, p.y, id);
			jumpPoints[pair<short, short>(p.x, p.y)] = jp;

			fread(&jp->edgeNode, sizeof(char), 1, fp);
			fread(&jp->pruned, sizeof(char), 1, fp);
			fread(&jp->globalGoalId, sizeof(short), 1, fp);
			fread(jp->neighbours, sizeof(char) * 8, 1, fp);
			/*fread(jp->i1, sizeof(short) * 8, 1, fp);
			short dirsSize = 0;
			fread(&dirsSize, sizeof(short), 1, fp);
			for (int i = 0; i < dirsSize; i++)
			{
				int dirPointId = 0;
				fread(&dirPointId, sizeof(int), 1, fp);
				char tempDir = 0;
				fread(&tempDir, sizeof(char), 1, fp);
				jp->dirs.push_back(pair<JumpNode, char>(&jps[dirPointId], tempDir));
			}*/
			/*for (int j = 0; j < 8; j++)
			{
				short IncdirsSize = 0;
				fread(&IncdirsSize, sizeof(short), 1, fp);
				for (int i = 0; i < IncdirsSize; i++)
				{
					int incdirPointId = 0;
					fread(&incdirPointId, sizeof(int), 1, fp);
					char tempDir = 0;
					fread(&tempDir, sizeof(char), 1, fp);
					jp->incDirs[j].push_back(pair<JumpNode, char>(&jps[incdirPointId], tempDir));
				}
			}
			dirsSize = 0;
			fread(&dirsSize, sizeof(short), 1, fp);
			if (dirsSize != 0)
			{
				jp->optPointer = new vector<pair<JumpNode, char>>();
				for (int i = 0; i < dirsSize; i++)
				{
					int dirPointId = 0;
					fread(&dirPointId, sizeof(int), 1, fp);
					char tempDir = 0;
					fread(&tempDir, sizeof(char), 1, fp);
					jp->optPointer->push_back(pair<JumpNode, char>(&jps[dirPointId], tempDir));
				}
			}*/
		}
	}
	void dumpAllJumpPointData(FILE* fp)
	{
		unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.begin();
		{
			int numJumpPoints = jumpPoints.size();
			fwrite(&numJumpPoints, sizeof(int), 1, fp);
		}
		while (it != jumpPoints.end())
		{
			fwrite(&it->second->pos, sizeof(Coordinate), 1, fp);
			fwrite(&it->second->id, sizeof(int), 1, fp);

			fwrite(&it->second->edgeNode, sizeof(char), 1, fp);
			fwrite(&it->second->pruned, sizeof(char), 1, fp);
			fwrite(&it->second->globalGoalId, sizeof(short), 1, fp);
			fwrite(it->second->neighbours, sizeof(char) * 8, 1, fp);
			/*fwrite(it->second->i1, sizeof(short) * 8, 1, fp);
			short dirsSize = it->second->dirs.size();
			fwrite(&dirsSize, sizeof(short), 1, fp);
			for (int i = 0; i < dirsSize; i++)
			{
				int dirPointId = it->second->dirs[i].first->id;
				fwrite(&dirPointId, sizeof(int), 1, fp);
				fwrite(&it->second->dirs[i].second, sizeof(char), 1, fp);
			}
			for (int j = 0; j < 8; j++)
			{
				short IncdirsSize = it->second->incDirs[j].size();
				fwrite(&IncdirsSize, sizeof(short), 1, fp);
				for (int i = 0; i < IncdirsSize; i++)
				{
					int incdirPointId = it->second->incDirs[j][i].first->id;
					fwrite(&incdirPointId, sizeof(int), 1, fp);
					fwrite(&it->second->incDirs[j][i].second, sizeof(char), 1, fp);
				}
			}
			if (it->second->optPointer == 0)
			{
				dirsSize = 0;
				fwrite(&dirsSize, sizeof(short), 1, fp);
			}
			else
			{
				dirsSize = it->second->optPointer->size();
				fwrite(&dirsSize, sizeof(short), 1, fp);
				for (int i = 0; i < dirsSize; i++)
				{
					int dirPointId = it->second->optPointer->at(i).first->id;
					fwrite(&dirPointId, sizeof(int), 1, fp);
					fwrite(&it->second->optPointer->at(i).second, sizeof(char), 1, fp);
				}
			}*/
			it++;
		}
	}
#ifdef PREPROCESS_END_NODES
	void dumpPreprocessedEndNodes(FILE* fp)
	{
		int endNodesSize = backwardJumpRefs.size();
		fwrite(&endNodesSize, sizeof(int), 1, fp);
		for (int i = 0; i < endNodesSize; i++)
		{
			short endNodeReferences = backwardJumpRefs[i].size();
			fwrite(&endNodeReferences, sizeof(short), 1, fp);
			for (int j = 0; j < endNodeReferences; j++)
				fwrite(&backwardJumpRefs[i][j]->id, sizeof(int), 1, fp);
		}
	}
	void readPreprocessedEndNodes(FILE* fp, JumpNode &jps)
	{
		int endNodesSize = 0;
		fread(&endNodesSize, sizeof(int), 1, fp);
		for (int i = 0; i < endNodesSize; i++)
		{
			backwardJumpRefs.push_back(vector<JumpNode>());
			short endNodeReferences = 0;
			fread(&endNodeReferences, sizeof(short), 1, fp);
			for (int j = 0; j < endNodeReferences; j++)
			{
				int id = 0;
				fread(&id, sizeof(int), 1, fp);
				backwardJumpRefs[i].push_back(&jps[id]);
			}
		}
	}
#else
	void dumpPreprocessedEndDiagonals(FILE* fp, vector<DiagonalJumpEntry > & vec)
	{
		short numBoundaries = vec.size();
		fwrite(&numBoundaries, 2, 1, fp);
		for (int i = 0; i < vec.size(); i++)
		{
			fwrite(&vec[i], sizeof(Coordinate) * 2, 1, fp);
			fwrite(&vec[i].jp->id, 4, 1, fp);
		}
	}
	void readPreprocessedEndDiagonals(FILE* fp, vector<DiagonalJumpEntry > & vec, JumpNode &jps)
	{
		short numBoundaries = 0;
		fread(&numBoundaries, 2, 1, fp);
		for (int i = 0; i < numBoundaries; i++)
		{
			DiagonalJumpEntry temp(Coordinate(0, 0), Coordinate(0, 0), 0);
			fread(&temp, sizeof(Coordinate) * 2, 1, fp);

			int JPId = 0;
			fread(&JPId, 4, 1, fp);
			temp.jp = &jps[JPId];
			vec.push_back(temp);
		}
	}

#endif //PREPROCESS_END_NODES

#ifdef PREPROCESS_STARTING_NODES
	void dumpPreprocessedStartNodes(FILE* fp)
	{
		int startNodesSize = forwardJumpRefs.size();
		fwrite(&startNodesSize, sizeof(int), 1, fp);
		for (int i = 0; i < startNodesSize; i++)
		{
			short startNodeReferences = forwardJumpRefs[i].size();
			fwrite(&startNodeReferences, sizeof(short), 1, fp);
			for (int j = 0; j < startNodeReferences; j++)
			{
				fwrite(&forwardJumpRefs[i][j].first->id, sizeof(int), 1, fp);
				fwrite(&forwardJumpRefs[i][j].second, sizeof(char), 1, fp);
			}
		}
	}
	void readPreprocessedStartNodes(FILE* fp, JumpNode &jps)
	{
		int startNodesSize = 0;
		fread(&startNodesSize, sizeof(int), 1, fp);
		for (int i = 0; i < startNodesSize; i++)
		{
			forwardJumpRefs.push_back(vector<pair<JumpNode, char> >());
			short startNodeReferences = 0;
			fread(&startNodeReferences, sizeof(short), 1, fp);
			for (int j = 0; j < startNodeReferences; j++)
			{
				int id = 0;
				fread(&id, sizeof(int), 1, fp);
				char dir = 0;
				fread(&dir, sizeof(char), 1, fp);
				forwardJumpRefs[i].push_back(pair<JumpNode, char>(&jps[id], dir));
			}
		}
	}
#endif //PREPROCESS_STARTING_NODES

#ifdef USE_PAIR_WISE_DISTANCES
	void dumpPairWiseDistances(FILE* fp)
	{
		if (pairDist)
		{
			fwrite(&numGlobalGoals, sizeof(int), 1, fp);
			for (int i = 0; i < numGlobalGoals; i++)
				fwrite(pairDist[i], sizeof(float)*numGlobalGoals, 1, fp);
		}
		else
		{
			fwrite(&pairDist, sizeof(int), 1, fp);
		}

	}
	void readPairWiseDistances(FILE* fp)
	{
		fread(&numGlobalGoals, sizeof(int), 1, fp);
		if (numGlobalGoals)
		{
			pairDist = new float*[numGlobalGoals];
			for (int i = 0; i < numGlobalGoals; i++)
			{
				pairDist[i] = new float[numGlobalGoals];
				fread(pairDist[i], sizeof(float)*numGlobalGoals, 1, fp);
			}
		}

	}
#endif //USE_PAIR_WISE_DISTANCES


	void dumpPreprocessedDataToFile(const char * fileName)
	{
		FILE * fp = fopen(fileName, "wb");
		if (fp == 0)
		{
			printf("Unable to open preprocessing file %s\n", fileName);
			return;
		}
		for (int y = 0; y<gridHeight; y++)
		{
			short numBoundaries = xBoundaryPoints[y].size() - 1;
			fwrite(&numBoundaries, 2, 1, fp);
			for (int i = 0; i<xBoundaryPoints[y].size() - 1; i++)
				fwrite(&xBoundaryPoints[y][i], 2, 1, fp);
		}
		for (int x = 0; x<gridWidth; x++)
		{
			short numBoundaries = yBoundaryPoints[x].size() - 1;
			fwrite(&numBoundaries, 2, 1, fp);
			for (int i = 0; i<yBoundaryPoints[x].size() - 1; i++)
				fwrite(&yBoundaryPoints[x][i], 2, 1, fp);
		}


		dumpAllJumpPointData(fp);

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

#ifdef PREPROCESS_END_NODES
		dumpPreprocessedEndNodes(fp);
#else
		for (int i = 0; i < 4; i++)
		{
			short size = diagonalJumps[i].size();
			fwrite(&size, sizeof(short), 1, fp);
			for (int j = 0; j < size; j++)
				dumpPreprocessedEndDiagonals(fp, diagonalJumps[i][j]);
		}
#endif //PREPROCESS_END_NODES
#ifdef PREPROCESS_STARTING_NODES
		dumpPreprocessedStartNodes(fp);
#endif //PREPROCESS_STARTING_NODES
#ifdef USE_PAIR_WISE_DISTANCES
		dumpPairWiseDistances(fp);
#endif //USE_PAIR_WISE_DISTANCES
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
		JumpNode jps = 0;
		readAllJumpPointData(fp, jps);
		//testedGrid = new char[jumpPoints.size() / 4 + 1];
		//		clearChecked();
		for (int y = 0; y < gridHeight; y++)
		{
			jumpLookup[1].push_back(vector<JumpPointTableEntry >());
			jumpLookup[3].push_back(vector<JumpPointTableEntry >());
			readJumpPointData(fp, jumpLookup[1].back(), jps);
			readJumpPointData(fp, jumpLookup[3].back(), jps);

		}
		for (int x = 0; x < gridWidth; x++)
		{
			jumpLookup[2].push_back(vector<JumpPointTableEntry >());
			jumpLookup[0].push_back(vector<JumpPointTableEntry >());
			readJumpPointData(fp, jumpLookup[2].back(), jps);
			readJumpPointData(fp, jumpLookup[0].back(), jps);
		}

#ifdef PREPROCESS_END_NODES
		readPreprocessedEndNodes(fp, jps);
#else
		for (int i = 0; i < 4; i++)
		{
			short size = 0;
			fread(&size, sizeof(short), 1, fp);
			for (int j = 0; j < size; j++)
			{
				diagonalJumps[i].push_back(vector<DiagonalJumpEntry>());

				readPreprocessedEndDiagonals(fp, diagonalJumps[i][j], jps);
			}
		}

#endif //PREPROCESS_END_NODES
#ifdef PREPROCESS_STARTING_NODES
		readPreprocessedStartNodes(fp, jps);
#endif //PREPROCESS_STARTING_NODES
#ifdef USE_PAIR_WISE_DISTANCES
		readPairWiseDistances(fp);
#endif //USE_PAIR_WISE_DISTANCES

		fclose(fp);
	}
	void preProcessGrid()
	{

		preprocessBoundaryLookupTables();
		vector<JumpNode> allNodes;

		preprocessJumpPointLookupTables(allNodes);

		//	testedGrid = new char[jumpPoints.size() / 4 + 1];
		//		clearChecked();
		preProcessSubGoalGraph(allNodes);

		numGlobalGoals = 0;
		vector<JumpNode> globalGoals;
		for (int i = 0; i < allNodes.size(); i++)
			addgobalGoalNode(allNodes[i], globalGoals, numGlobalGoals);

		finalizeConnectionData();
#ifdef USE_PAIR_WISE_DISTANCES
		// Floyd-Warshall algorithm to compute pairwise distances between all global subgoals
		// Set self-costs to 0; rest to infinity
		if (numGlobalGoals*numGlobalGoals * 4 < MEMORY_LIMIT)
		{
			pairDist = new float*[numGlobalGoals];
			for (int i = 0; i < numGlobalGoals; i++)
			{

				pairDist[i] = new float[numGlobalGoals];
				for (int ii = 0; ii < numGlobalGoals; ii++)
					pairDist[i][ii] = std::numeric_limits<float>::infinity();

				pairDist[i][i] = 0;
			}
			for (int i = 0; i < numGlobalGoals; i++)
				for (int dir = 0; dir < 8; dir++)
				{

					int numGoals = getHighConnectionSize2( globalGoals[i]->id,dir);
					for (int ii = 0; ii < numGoals; ii++)
					{
						JumpNode neighbour = getHighConnectionItem2(globalGoals[i]->id,dir, ii).first;
						if (neighbour->globalGoalId != -1)
						{
							const float d = Node::estimateDistance(globalGoals[i]->pos, neighbour->pos);
							pairDist[i][neighbour->globalGoalId] = d;
						}
					}
				}
			for (int k = 0; k < numGlobalGoals; k++)
				for (int i = 0; i < numGlobalGoals; i++)
					for (int j = 0; j < numGlobalGoals; j++)
						if (pairDist[i][j]>pairDist[i][k] + pairDist[k][j])
							pairDist[i][j] = pairDist[i][k] + pairDist[k][j];
		}
#endif
		/*FILE*fp = fopen("../bin/text.txt", "w");
		for (int y = 0; y < gridHeight; y++)
		{
		for (int x = 0; x < gridWidth; x++)
		if (isPassable(Coordinate(x, y)))
		{
		if (jumpPoints[pair<short,short>(x, y)] == 0)
		fprintf(fp, ".");
		else
		fprintf(fp, "x");

		}
		else
		fprintf(fp, "T");

		fprintf(fp, "\n");
		}
		fclose(fp);*/
		int amountMemoryUsedJPN = 0;
		/*int dirs = 0;
		int inc = 0;
		for (int i = 0; i < allNodes.size(); i++)
		{
			for (int d = 0; d < 8; d++)
				inc += allNodes[i]->incDirs[d].capacity() * 5;
			dirs += allNodes[i]->dirs.capacity() * 5;
			amountMemoryUsedJPN += 12 + sizeof(JumpPointNode);
		}
		amountMemoryUsedJPN += dirs + inc;*/
		int t1 = 0;
		for (int i = 0; i < 4; i++)
			for (int ii = 0; ii < jumpLookup[i].size(); ii++)
				t1 += jumpLookup[i][ii].capacity()*sizeof(JumpPointTableEntry) + sizeof(vector<JumpPointTableEntry>);
		int openClosedListFlags = (jumpPoints.size() / 4 + 1);
		generated = new unsigned short[jumpPoints.size()];	// +2 for possible start and goal states
		open = new char[jumpPoints.size() / 8 + 1];
		parent = new JumpNode[jumpPoints.size()];
		gCost = new float[jumpPoints.size()];

		ResetSearch();
		memset(open, jumpPoints.size() / 8 + 1, 0);
		
	}
	BL_JPS_PLUS_SUBGOAL(char * grid, int width, int height) : PathFindingAlgorithm(BLJPS_SUBGOAL_ALG_NAME.c_str(), AT_BL_JPS_SUBGOAL)
	{
		CoordinateHash::MAPWIDTH = gridWidth;
		gridData = grid;
		gridWidth = width;
		gridHeight = height;
		testedGrid = 0;
		startNodeArray = 0;
		open = 0;
		generated = 0;
		gCost = 0;
		parent = 0;

		sizeOfConnections = 0;
		sizeOfHighConnections= 0;
		rawConnections= 0;
		rawHighConnections = 0;
		 rawConnectionStart = 0;
		 rawHighConnectionStart = 0;
#ifdef USE_PAIR_WISE_DISTANCES
		pairDist = 0;
		numGlobalGoals = 0;
#endif

	}
	void copyPreprocessedJumpPoints(unsigned int * data)
	{
	}
	unsigned int* getPreprocessedData()
	{
		return 0;
	}
	~BL_JPS_PLUS_SUBGOAL()
	{
		if (sizeOfConnections)
		delete[] sizeOfConnections;

		if (sizeOfHighConnections)
		delete[] sizeOfHighConnections;

		if (rawConnections)
		delete[] rawConnections;

		if (rawHighConnections)
		delete[] rawHighConnections;

		if (rawConnectionStart)
		delete[] rawConnectionStart;

		if (rawHighConnectionStart)
		delete[] rawHighConnectionStart;

		if (testedGrid)
		{
			delete[] testedGrid;
			testedGrid = 0;
		}
		if (generated)
		{
			delete[]generated;
			generated = 0;
		}
		if (gCost)
		{
			delete[]gCost;
			gCost = 0;
		}
		if (parent)
		{
			delete[]parent;
			parent = 0;
		}
		if (open)
		{
			delete[]open;
			open = 0;
		}
		if (startNodeArray)
		{
			delete[]startNodeArray;
		}
		else
		{
			unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.begin();
			while (it != jumpPoints.end())
			{
				delete (JumpNode)it->second;
				it++;
			}
		}
#ifdef USE_PAIR_WISE_DISTANCES
		if (pairDist)
		{
			for (int i = 0; i < numGlobalGoals; i++)
				delete[] pairDist[i];
			delete pairDist;
			pairDist = 0;
			numGlobalGoals = 0;
		}

#endif
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
	inline int mag(const int i)
	{
		return i < 1 ? -1 : 1;
	}


	/*	void setClosedChecked(const int index)
	{
	testedGrid[index / 4] |= (1 << ((index & 3) << 1));
	}
	void setOpenChecked(const int index)
	{
	testedGrid[index / 4] |= (1 << ((index & 3) << 1) + 1);
	}
	char isClosedAndOpenCheck(const int index)
	{
	return (testedGrid[index / 4] & (0x3 << ((index & 3) << 1))) >> ((index & 3) << 1);
	}

	void clearChecked()
	{
	memset(testedGrid, 0, jumpPoints.size() / 4 + 1);
	}*/
	int getSpaceIdY(short spaceX, short spaceY)
	{
		for (int i = 0; i<yBoundaryPoints[spaceX].size() - 1; i++)
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
		for (int i = 0; i<xBoundaryPoints[spaceY].size() - 1; i++)
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
	void getAllStartingNodes(short sx, short sy, vector<pair<JumpNode, char>> &startNodes)
	{
#ifdef PREPROCESS_STARTING_NODES
		startNodes = forwardJumpRefs[sx + sy*gridWidth];
#else
		for (int dir = 0; dir < 8; dir++)
		{
			if (dirIsDiagonal(dir))
			{
				static vector<JumpNode> diagJumps;
				diagJumps.clear();
				jumpNewDiags(Coordinate(sx, sy), dir, diagJumps);
				//startNodes.insert(startNodes.begin(), diagJumps.begin(), diagJumps.end());
				for (int i = 0; i < diagJumps.size(); i++)
					startNodes.push_back(pair<JumpNode, char>(diagJumps[i], dir));
			}
			else
			{
				JumpNode jp = jumpNew(Coordinate(sx, sy), dir);
				if (jp)
					startNodes.push_back(pair<JumpNode, char>(jp, dir));

			}
		}
#endif
	}

	bool addEndNode(vector<pair<JumpNode,char>> &endNodes, JumpNode jp)
	{
		for (int i = 0; i < endNodes.size(); i++)
			if (endNodes[i].first == jp)
				return false;
		endNodes.push_back(pair<JumpNode, char>(jp,8));
		return true;
	}
	int binarySearchL(const vector<DiagonalJumpEntry> & v, short val)
	{
		short l = 0, r = v.size() - 1;
		if (r < 0)
			return 0;
		short index = r / 2;
		while (1)
		{
			if ((index == 0 || (index>0 && v[index - 1].from.x < val)) && v[index].from.x >= val)
				return  index;

			if (v[index].from.x >= val)
			{
				r = index - 1;
				if (r <0 || r<l)
					return v.size();
			}
			else
			{
				l = index + 1;
				if (l > v.size() - 1 || r<l)
					return v.size();
			}

			index = l + (r - l) / 2;

		}
		return v.size();
	}

	void getAllEndNodes(short ex, short ey, vector<pair<JumpNode, char> >&endNodes)
	{
		/*unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.find(pair<short, short>(ex, ey));

		if (it != jumpPoints.end())
		{
			endNodes.push_back(pair<JumpNode, char>(it->second,8));
			return;
		}*/
#ifdef PREPROCESS_STARTING_NODES
		endNodes = forwardJumpRefs[ex + ey*gridWidth];
#else
		for (int dir = 0; dir < 8; dir++)
		{
			if (dirIsDiagonal(dir))
			{
				static vector<JumpNode> diagJumps;
				diagJumps.clear();
				jumpNewDiags(Coordinate(ex, ey), dir, diagJumps);

				//startNodes.insert(startNodes.begin(), diagJumps.begin(), diagJumps.end());
				for (int i = 0; i < diagJumps.size(); i++)
					endNodes.push_back(pair<JumpNode, char>(diagJumps[i], dir));
			}
			else
			{
				//unordered_map<pair<short, short>, JumpNode, CoordinateHash>::iterator it = jumpPoints.find(pair<short, short>(ex, ey));

				//if (it != jumpPoints.end())
				JumpNode jp = jumpNew(Coordinate(ex, ey), dir);
				if (jp)
					endNodes.push_back(pair<JumpNode, char>(jp, dir));

			}
		}
#endif
	}
	char getDiagDir(const Coordinate& c, const Coordinate& nxt, char dir)
	{
		if (!dirIsDiagonal(dir))
			return dir;
		int xDif = nxt.x - c.x;
		int yDif = nxt.y - c.y;
		if (abs(xDif) == abs(yDif))
			return dir;
		if (abs(xDif) > abs(yDif))
			return xDif > 0 ? 2 : 6;
		return yDif > 0 ? 4 : 0;
	}
	char getDir(const Coordinate& from, const Coordinate& to)
	{
		int xDif = to.x-from.x ;
		int yDif = to.y - from.y;
		if (xDif == 0)
			return yDif < 0 ? 0 : 4;
		if (yDif == 0)
			return xDif < 0 ? 6 : 2;
		return xDif<0 ? (yDif<0 ? 7 : 5) : yDif<0 ? 1 : 3;
	}
	char getDir2(const Coordinate& c, const Coordinate& nxt)
	{
		int xDif = nxt.x - c.x;
		int yDif = nxt.y - c.y;

		return xDif<0 ? (yDif<0 ? 7 : 5) : yDif<0 ? 1 : 3;
	}
	void getDiagCoordinate(const Coordinate& nxt, vector<Coordinate> & solution)
	{
		int xDif = nxt.x - solution.back().x;
		int yDif = nxt.y - solution.back().y;
		if (!(abs(xDif) == abs(yDif) || xDif == 0 || yDif == 0))
			if (abs(xDif)>abs(yDif))
				solution.push_back(Coordinate(solution.back().x + mag(xDif)*abs(yDif), solution.back().y + yDif));
			else
				solution.push_back(Coordinate(solution.back().x + xDif, solution.back().y + mag(yDif)*abs(xDif)));
		solution.push_back(nxt);
	}
	bool pathSegmentValid(Coordinate &prev, const Coordinate & nxt)
	{
		int dx = nxt.x - prev.x;
		int dy = nxt.y - prev.y;
		if (abs(dx)>1)
			dx = dx > 0 ? 1 : -1;
		else if (dx == 0)
			return isSpaceIdY(getSpaceIdY(prev.x, prev.y), nxt.x, nxt.y);

		if (abs(dy)>1)
			dy = dy > 0 ? 1 : -1;
		else if (dy == 0)
			return isSpaceIdX(getSpaceIdX(prev.x, prev.y), nxt.x, nxt.y);
		Coordinate offset(dx, dy);
		bool diagStep = dx != 0 && dy != 0;
		Coordinate currentStep(prev);
		while (!(currentStep.x == nxt.x && currentStep.y == nxt.y))
		{
#ifdef DIAG_UNBLOCKED
			if (!isPassable(currentStep) || (diagStep && (!isPassable(Coordinate(currentStep.x + offset.x, currentStep.y)) || !isPassable(Coordinate(currentStep.x, currentStep.y + offset.y)))))
				return false;
#else
			if (!isPassable(currentStep))
				return false;
#endif
			currentStep.add(offset);
		}
		if (!isPassable(nxt))
			return false;
		return true;
	}
	void getDiagCoordinate2(const Coordinate& nxt, vector<Coordinate> & solution)
	{
		int xDif = solution.back().x - nxt.x;
		int yDif = solution.back().y - nxt.y;
		if (!(abs(xDif) == abs(yDif) || xDif == 0 || yDif == 0))
		{
			if (abs(xDif)>abs(yDif))
			{
				//int diagDif = abs(xDif) - abs(yDif);

				solution.push_back(Coordinate(nxt.x + mag(xDif)*abs(yDif), nxt.y + yDif));

				if (!pathSegmentValid(*(solution.end() - 2), solution.back()) || !pathSegmentValid(solution.back(), nxt))
					solution.back() = Coordinate(nxt.x - mag(xDif)*(abs(yDif) - abs(xDif)), nxt.y);
			}
			else
			{

				solution.push_back(Coordinate(nxt.x + xDif, nxt.y + mag(yDif)*abs(xDif)));
				if (!pathSegmentValid(*(solution.end() - 2), solution.back()) || !pathSegmentValid(solution.back(), nxt))
					solution.back() = Coordinate(nxt.x, nxt.y + mag(yDif)*(abs(yDif) - abs(xDif)));

			}

		}
		solution.push_back(nxt);
	}
	bool directSolution(short sX, short sY, short eX, short eY, vector<Coordinate> & sol)
	{
		if (sY == eY)
		{
			if (isSpaceIdX(getSpaceIdX(sX, sY), eX, eY))
			{
				sol.push_back(Coordinate(eX, eY));
				sol.push_back(Coordinate(sX, sY));

				return true;
			}
		}
		else if (sX == eX)
		{
			if (isSpaceIdY(getSpaceIdY(sX, sY), eX, eY))
			{
				sol.push_back(Coordinate(eX, eY));
				sol.push_back(Coordinate(sX, sY));

				return true;
			}
		}
		else
		{
			bool isDiagOnly = (abs(sX - eX) - abs(sY - eY)) == 0;

			int diagMovements = min(abs(sX - eX), abs(sY - eY)) - 1;//we -1 as we don't need to check the original or destination points as they are checked earlier
			int mx = sX - eX<0 ? -1 : 1;
			int my = sY - eY<0 ? -1 : 1;
			Coordinate offset(sX - eX<0 ? 1 : -1, sY - eY<0 ? 1 : -1);
			Coordinate check(sX, sY);
			bool bPass = true;
			if (!isDiagOnly)
			{
				Coordinate check2(sX + offset.x*(diagMovements + 1), sY + offset.y*(diagMovements + 1));
				if (abs(sX - eX) < abs(sY - eY))
				{
					if (!isSpaceIdY(getSpaceIdY(check2.x, check2.y), eX, eY))
						bPass = false;
				}
				else
					if (!isSpaceIdX(getSpaceIdX(check2.x, check2.y), eX, eY))
						bPass = false;
			}
			if (bPass)
			{
				while (diagMovements)
				{
					diagMovements--;
					bool b = true;
#ifdef DIAG_UNBLOCKED
					b = (isPassable(Coordinate(check.x + offset.x, check.y)) && isPassable(Coordinate(check.x, check.y + offset.y)));
#endif

					check.add(offset);
					if (!isPassable(check) || !b)
					{
						bPass = false;
						diagMovements = 0;
						break;
					}
				}
#ifdef DIAG_UNBLOCKED
				if (bPass && min(abs(sX - eX), abs(sY - eY)) != 0)
					bPass = (isPassable(Coordinate(check.x + offset.x, check.y)) && isPassable(Coordinate(check.x, check.y + offset.y)));
#endif

				if (bPass)
					if (isDiagOnly)//only diagonal movement
					{
						sol.push_back(Coordinate(eX, eY));
						sol.push_back(Coordinate(sX, sY));
						return true;
					}
					else //diagonal movement and then horiz/vertic
					{
						check.add(offset);
						sol.push_back(Coordinate(eX, eY));

						sol.push_back(Coordinate(check.x, check.y));
						sol.push_back(Coordinate(sX, sY));

						return true;
					}
			}
			if (isDiagOnly)
				return false;
			bPass = true;
			diagMovements = min(abs(sX - eX), abs(sY - eY)) - 1;
			offset.x *= -1;
			offset.y *= -1;
			check = Coordinate(eX, eY);
			Coordinate check2(eX + offset.x*(diagMovements + 1), eY + offset.y*(diagMovements + 1));
			if (abs(sX - eX) < abs(sY - eY))
			{
				if (!isSpaceIdY(getSpaceIdY(check2.x, check2.y), sX, sY))
					bPass = false;
			}
			else
				if (!isSpaceIdX(getSpaceIdX(check2.x, check2.y), sX, sY))
					bPass = false;

			if (bPass)
			{
				while (diagMovements)
				{
					diagMovements--;
					bool b = true;
#ifdef DIAG_UNBLOCKED
					b = (isPassable(Coordinate(check.x + offset.x, check.y)) && isPassable(Coordinate(check.x, check.y + offset.y)));
#endif
					check.add(offset);
					if (!isPassable(check) || !b)
					{
						bPass = false;
						diagMovements = 0;
						break;
					}
				}
#ifdef DIAG_UNBLOCKED
				if (bPass && min(abs(sX - eX), abs(sY - eY)) != 0)
					bPass = (isPassable(Coordinate(check.x + offset.x, check.y)) && isPassable(Coordinate(check.x, check.y + offset.y)));
#endif
				if (bPass)
				{
					check.add(offset);
					sol.push_back(Coordinate(eX, eY));

					sol.push_back(Coordinate(check.x, check.y));
					sol.push_back(Coordinate(sX, sY));

					return true;
				}
			}



		}
		return false;
	}
	bool directSolution(Coordinate s, Coordinate e, bool &firstConnection)
	{
		short sX = s.x;
		short sY = s.y;
		short eX = e.x;
		short eY = e.y;
		firstConnection = false;

		if (sY == eY)
		{
			if (isSpaceIdX(getSpaceIdX(sX, sY), eX, eY))
			{
				firstConnection = true;
				return true;
			}
		}
		else if (sX == eX)
		{
			if (isSpaceIdY(getSpaceIdY(sX, sY), eX, eY))
			{
				firstConnection = true;
				return true;
			}
		}
		else
		{
			bool isDiagOnly = (abs(sX - eX) - abs(sY - eY)) == 0;

			int diagMovements = min(abs(sX - eX), abs(sY - eY)) - 1;//we -1 as we don't need to check the original or destination points as they are checked earlier
			int mx = sX - eX<0 ? -1 : 1;
			int my = sY - eY<0 ? -1 : 1;
			Coordinate offset(sX - eX<0 ? 1 : -1, sY - eY<0 ? 1 : -1);
			Coordinate check(sX, sY);
			bool bPass = true;
			if (!isDiagOnly)
			{
				Coordinate check2(sX + offset.x*(diagMovements + 1), sY + offset.y*(diagMovements + 1));
				if (abs(sX - eX) < abs(sY - eY))
				{
					if (!isSpaceIdY(getSpaceIdY(check2.x, check2.y), eX, eY))
						bPass = false;
				}
				else
					if (!isSpaceIdX(getSpaceIdX(check2.x, check2.y), eX, eY))
						bPass = false;
			}
			if (bPass)
			{
				while (diagMovements)
				{
					diagMovements--;
					//	check.add(offset);
					bool b = true;
#ifdef DIAG_UNBLOCKED
					b = (isPassable(Coordinate(check.x + offset.x, check.y)) && isPassable(Coordinate(check.x, check.y + offset.y)));
#endif

					check.add(offset);
					if (!isPassable(check) || !b)
					{
						bPass = false;
						diagMovements = 0;
						break;
					}

#ifdef DIAG_UNBLOCKED
					if (bPass && min(abs(sX - eX), abs(sY - eY)) != 0)
						bPass = (isPassable(Coordinate(check.x + offset.x, check.y)) && isPassable(Coordinate(check.x, check.y + offset.y)));
#endif
				}
				if (bPass)
					if (isDiagOnly)//only diagonal movement
					{
						firstConnection = false;
						return true;
					}
					else //diagonal movement and then horiz/vertic
					{
						firstConnection = true;
						return true;
					}
			}


		}
		return false;
	}
	bool checkIfSolution(JumpNode jpn, vector<Coordinate> & solution, Coordinate startCoord)
	{
		//unsigned char dirs = jpn->neighbours[currentNode->dir];//forcedNeighbours(currentNode->pos, currentNode->dir) | naturalNeighbours(currentNode->dir);//

		if (jpn->getGoalNode()&&jpn->parent==0)
		{
			//static vector<Coordinate> tempSol;

			//if (minV != -1&& jpn->v + Node::estimateDistance(jpn->pos, Coordinate(eX, eY)) > minV + 0.1f)
			//	return false;
			solution.push_back(Coordinate(eX, eY));
			JumpNode cNode = jpn;
			//if (jpn->parent)
			//{
			//	getDiagCoordinate2(jpn->parent->pos, solution);
			//}

			/*if (parent[jpn->id])
			{

				int nDir = 1 << getDir(Coordinate(eX, eY), jpn->pos);
				int nDir2 = getDir(parent[jpn->id]->pos, jpn->pos);
				int nDir4 = (1 << ((nDir2 + 3) & 7)) | (1 << ((nDir2 + 5) & 7)) | (1 << ((nDir2 + 4) & 7));
				nDir2 = (1 << nDir2) | (1 << ((nDir2 + 1) & 7)) | (1 << ((nDir2 + 7) & 7));
				//if (jpn->getGoalNode() == 0xFF)
					//nDir4 = nDir2;
				int nDir3 = getDir(Coordinate(eX, eY), jpn->pos);
				nDir3 = (1 << nDir3) | (1 << ((nDir3 + 1) & 7)) | (1 << ((nDir3 + 7) & 7));
				if ((nDir3 &nDir) && (nDir & nDir2) && (hasDirectConnection(partitionEX, partitionEY, jpn->pos, Coordinate(eX, eY))))
				{

					cNode = parent[jpn->id];
					tempSol.clear();

					if (!directSolution(cNode->pos.x, cNode->pos.y, eX, eY, tempSol))
					{
						getDiagCoordinate2(jpn->pos, solution);


					}

				}
				else
				{
					tempSol.clear();

					if (directSolution(parent[jpn->id]->pos.x, parent[jpn->id]->pos.y, eX, eY, tempSol))
						cNode = parent[jpn->id];

				}

			}*/

			//Node::diagAndStraightCost(nStartNode->pos, endCoord, p);

			while (cNode)
			{
				getDiagCoordinate2(cNode->pos, solution);
				cNode = parent[cNode->id];
			}
			getDiagCoordinate2(startCoord, solution);

			return true;
		}
		return false;
	}
#ifdef USE_PAIR_WISE_DISTANCES
	void getPairWiseSolution(short sX, short sY, short eX, short eY, vector<Coordinate> & solution, vector<pair<JumpNode, char> > &startingNodes, vector<pair<JumpNode, char> > &endNodes)
	{


//		vector<JumpNode>  endNodes;

		int startIdNum = startingNodes.size();
		vector<pair<float, int>> startParents;
		for (int id = 0; id < startIdNum; id++)
			startParents.push_back(pair<float, int>(Node::estimateDistance(Coordinate(sX, sY), startingNodes[id].first->pos), -1));

	/*	for (int id = 0; id < startIdNum; id++)
			if (startingNodes[id].first->globalGoalId == -1)
			{
#ifdef DIAG_UNBLOCKED
				char dir = startingNodes[id].second;
				dir = 0xff - (1 << (dir + 4) & 7) - (1 << ((dir + 3) & 7)) - (1 << ((dir + 5) & 7));
#else
				char dir = startingNodes[id].first->neighbours[startingNodes[id].second];
#endif
				for (int d = 0; d < 8; d++)
					if (dir & (1 << d))
					{
						int numH = getHighConnectionSize2(startingNodes[id].first->id, d);
						for (int ii = 0; ii < numH; ii++)
						{
							startingNodes.push_back(getHighConnectionItem2(startingNodes[id].first->id,d, ii));
							startParents.push_back(pair<float, int>(Node::estimateDistance(Coordinate(sX, sY), startingNodes[id].first->pos) + Node::estimateDistance(getHighConnectionItem2(startingNodes[id].first->id,d, ii).first->pos, startingNodes[id].first->pos), id));
						}
					}
			}*/

		vector<pair<float, int>> endParents;
		//for (int id = 0; id < nendNodes.size(); id++)
		//	endParents.push_back(pair<float, int>(Node::estimateDistance(Coordinate(eX, eY), nendNodes[id].first->pos), -1));
		static vector<vector<pair<JumpNode, char>>*> endNodeJoints;
		int requestedNum = 0;
		for (int i = 0; i < endNodes.size(); i++)
		{
			endParents.push_back(pair<float, int>(Node::estimateDistance(Coordinate(eX, eY), endNodes[i].first->pos), i));// +Node::estimateDistance(getIncomingConnection(endParents[i].first->id, ii, i).first->pos, nendNodes[id].first->pos), id));
			if (endNodes[i].first->parent)
				endParents.back().first += Node::estimateDistance(endNodes[i].first->parent->pos, endNodes[i].first->pos);
		}
		/*for (int id = 0; id < nendNodes.size(); id++)
		{
			char newDir = (getDir(Coordinate(eX, eY), nendNodes[id].first->pos) + 4) & 7;
			char chosenDir2 = 1 << newDir;
			for (int ii = 0; ii < 8; ii++)
				if (nendNodes[id].first->isPrunedEdge(ii))
#ifndef DIAG_UNBLOCKED
					if (nendNodes[id].first->neighbours[ii] & chosenDir2)
#endif
					{
						for (int i = 0; i <getNoIncConnections(nendNodes[id].first->id, ii); i++)
						{
							//bool pruned = nendNodes[id].first->incDirs[ii][i].first->isPrunedEdge(ii);
							//char dd2 = nendNodes[id].first->incDirs[ii][i].first->getGoalNode();
							//if (!nendNodes[id].first->incDirs[ii][i].first->isPrunedEdge(ii) && !nendNodes[id].first->incDirs[ii][i].first->getGoalNode())
							{
								endNodes.push_back(getIncomingConnection(nendNodes[id].first->id,ii,i).first);
								endParents.push_back(pair<float, int>(Node::estimateDistance(Coordinate(eX, eY), nendNodes[id].first->pos) + Node::estimateDistance(getIncomingConnection(nendNodes[id].first->id, ii, i).first->pos, nendNodes[id].first->pos), id));
								if (getIncomingConnection(nendNodes[id].first->id, ii, i).first->optPointer)
									getIncomingConnection(nendNodes[id].first->id, ii, i).first->optPointer->push_back(pair<JumpNode, char>(nendNodes[id].first, ii));
								else
								{
									if (endNodeJoints.size() == requestedNum)
									{
										endNodeJoints.push_back(new vector<pair<JumpNode, char>>(1, pair<JumpNode, char>(nendNodes[id].first, ii)));
									}
									else
									{
										endNodeJoints[requestedNum]->clear();
										endNodeJoints[requestedNum]->push_back(pair<JumpNode, char>(nendNodes[id].first, ii));
									}
									getIncomingConnection(nendNodes[id].first->id, ii, i).first->setHighLevelGoalNode(endNodeJoints[requestedNum]);
									requestedNum++;
								}

							}
						}
					}
			if (!nendNodes[id].first->getGoalNode())
			{
				endNodes.push_back(nendNodes[id].first);
			//	nendNodes[id].first->index = endParents.size();
				endParents.push_back(pair<float, int>(Node::estimateDistance(Coordinate(eX, eY), nendNodes[id].first->pos), -1));
			}
			nendNodes[id].first->setGoalNode();

		}*/

		vector<pair<JumpNode, float>> possibleEnds;
		vector<pair<float, int>> endPossibleParents;
		for (int id = 0; id < endNodes.size(); id++)
		{
			if (endNodes[id].first->globalGoalId != -1)
			{
				possibleEnds.push_back(pair<JumpNode, float>(endNodes[id].first, Node::estimateDistance(Coordinate(eX, eY), endNodes[id].first->pos)));
			//	endNodes[id].first->parent;
				endPossibleParents.push_back(endParents[id]);
			}
			//endNodes[id]->setGoalNode();
		}
		int _startId = -1;
		float minV = std::numeric_limits<float>::infinity();
		JumpNode minNode = 0;
		JumpNode minNodeB = 0;
		for (int i = 0; i < startingNodes.size(); i++)
			for (int ii = 0; ii < endNodes.size(); ii++)
				if (startingNodes[i].first == endNodes[ii].first)
				{
					float t = startParents[i].first + endParents[ii].first;
					if (t<minV)
					{
						minV = t;
						minNode = startingNodes[i].first;
						minNodeB = startingNodes[i].first;
						_startId = i;
					}
				}


		vector<pair<JumpNode, float>> possibleStarts;
		vector<pair<float, int>> startPossibleParents;

		//Insert the start nodes into the openList
		for (int i = 0; i < startingNodes.size(); i++)
			if (startingNodes[i].first->globalGoalId != -1)
			{
				possibleStarts.push_back(pair<JumpNode, float>(startingNodes[i].first, Node::estimateDistance(Coordinate(sX, sY), startingNodes[i].first->pos)));
				startPossibleParents.push_back(startParents[i]);
			}

		float _minV = std::numeric_limits<float>::infinity();
		JumpNode _minNode = 0;
		JumpNode _minNodeB = 0;
		int startId = -1;
		//float destDist = -1;
		for (int i = 0; i < possibleStarts.size(); i++)
			for (int ii = 0; ii < possibleEnds.size(); ii++)
			{
				int id1 = possibleStarts[i].first->globalGoalId;
				int id2 = possibleEnds[ii].first->globalGoalId;
				float t = startPossibleParents[i].first + endPossibleParents[ii].first + pairDist[id1][id2];
				//JumpNodel = possibleEnds[ii].second;
				//JumpNoder = possibleStarts[i].second;
				if (possibleEnds[ii].first == possibleStarts[i].first && t - .1 <= _minV)
				{
					_minV = t;
					_minNode = possibleStarts[i].first;
					_minNodeB = possibleEnds[ii].first;
					startId = i;
					//	destDist = endPossibleParents[ii].first;
				}
				else
					if (t<_minV)
					{
						_minV = t;
						_minNode = possibleStarts[i].first;
						_minNodeB = possibleEnds[ii].first;
						//destDist = endPossibleParents[ii].first;

						startId = i;
					}
			}
		if (minV == _minV && _minV == std::numeric_limits<float>::infinity())
		{
			//printf("ahhhhhh\n\n");
#ifndef  STATIC_START_END
			for (int id = 0; id < endNodes.size(); id++)
				endNodes[id].first->clearGoalNode();
#endif
			return;
		}
		if (minV < _minV || (abs(minV - _minV)<0.1))
		{
			solution.push_back(Coordinate(sX, sY));
			if (startParents[_startId].second != -1)

				getDiagCoordinate2(startingNodes[startParents[_startId].second].first->pos, solution);

			getDiagCoordinate2(minNode->pos, solution);
			/*if (!minNode->getGoalNode() && minNode->optPointer)
			{
				float mm = std::numeric_limits<float>::infinity();
				JumpNode n = 0;
				for (int opId = 0; opId < minNode->optPointer->size(); opId++)
				{
					float opCost = Node::estimateDistance(Coordinate(eX, eY), minNode->optPointer->at(opId).first->pos) + Node::estimateDistance(minNode->optPointer->at(opId).first->pos, minNode->pos);
					if (opCost < mm)
					{
						mm = opCost;
						n = minNode->optPointer->at(opId).first;
					}
				}
				getDiagCoordinate2(n->pos, solution);
			}*/
			getDiagCoordinate2(Coordinate(eX, eY), solution);
			std::reverse(solution.begin(), solution.end());
#ifndef  STATIC_START_END
			for (int id = 0; id < endNodes.size(); id++)
				endNodes[id].first->clearGoalNode();
#endif
			return;
		}
		if (_minNode == _minNodeB)
		{
			solution.push_back(Coordinate(sX, sY));
			getDiagCoordinate2(_minNode->pos, solution);
			/*if (!minNode->getGoalNode() && minNode->optPointer)
			{
				float mm = std::numeric_limits<float>::infinity();
				JumpNode n = 0;
				for (int opId = 0; opId < minNode->optPointer->size(); opId++)
				{
					float opCost = Node::estimateDistance(Coordinate(eX, eY), minNode->optPointer->at(opId).first->pos) + Node::estimateDistance(minNode->optPointer->at(opId).first->pos, minNode->pos);
					if (opCost < mm)
					{
						mm = opCost;
						n = minNode->optPointer->at(opId).first;
					}
				}
				getDiagCoordinate2(n->pos, solution);
			}*/
			getDiagCoordinate2(Coordinate(eX, eY), solution);
		}
		else
		{
			solution.push_back(Coordinate(sX, sY));
			if (startPossibleParents[startId].second != -1)
				getDiagCoordinate2(startingNodes[startPossibleParents[startId].second].first->pos, solution);
			getDiagCoordinate2(_minNode->pos, solution);
			int endNodeId = _minNodeB->globalGoalId;
			float lastMinV = minV;
			//int trials = 0;
			while (1)
			{
				//	trials++;
				float minVt = std::numeric_limits<float>::infinity();
				JumpNode nxtStep = 0;
				for (int d = 0; d < 8; d++)
				{
					int numHigh = getHighConnectionSize2(_minNode->id, d);// getNumHighGoals(d);
					for (int hId = 0; hId < numHigh; hId++)
					{
						JumpNode nxtNode = getHighConnectionItem2(_minNode->id,d, hId).first;
						float t1 = Node::estimateDistance(_minNode->pos, nxtNode->pos);
						float t2 = pairDist[nxtNode->globalGoalId][endNodeId];;
						float t = t1 + t2;
						if (t < minVt)
						{
							nxtStep = nxtNode;
							minVt = t;
						}
					}
				}
				/*for (int d = 0; d < 8; d++)
				{
				int numHigh = _minNode->getDirSize(d);
				for (int hId = 0; hId < numHigh; hId++)
				{
				JumpNodenxtNode = _minNode->getDirItem(d, hId);
				if (nxtNode == _minNodeB)
				{
				nxtStep = nxtNode;
				minVt = -1;
				}
				}
				}*/


				/*if (_minNode->optPointer)
				for (int hId = 0; hId < _minNode->optPointer->size(); hId++)
				{
				JumpNodenxtNode = _minNode->optPointer->at(hId).first;
				float t = Node::estimateDistance(_minNode->pos, nxtNode->pos) + Node::estimateDistance(nxtNode->pos, Coordinate(eX, eY));
				if (t < minVt)
				{
				nxtStep = nxtNode;
				minVt = t;
				}
				}*/
				if (minVt>lastMinV)
					nxtStep = _minNodeB;

				if (nxtStep == _minNodeB)
				{
					getDiagCoordinate2(nxtStep->pos, solution);
					/*if (!nxtStep->getGoalNode() && nxtStep->optPointer)
					{
						float mm = std::numeric_limits<float>::infinity();
						JumpNode n = 0;
						for (int opId = 0; opId < nxtStep->optPointer->size(); opId++)
						{
							float opCost = Node::estimateDistance(Coordinate(eX, eY), nxtStep->optPointer->at(opId).first->pos) + Node::estimateDistance(nxtStep->optPointer->at(opId).first->pos, nxtStep->pos);
							if (opCost < mm)
							{
								mm = opCost;
								n = nxtStep->optPointer->at(opId).first;
							}
						}
						getDiagCoordinate2(n->pos, solution);
					}*/
					getDiagCoordinate2(Coordinate(eX, eY), solution);
					//printf("PTrails:%d\n", trials);

#ifndef  STATIC_START_END
					for (int id = 0; id < endNodes.size(); id++)
						endNodes[id].first->clearGoalNode();
#endif
					std::reverse(solution.begin(), solution.end());
					return;
				}
				else
				{
					_minNode = nxtStep;
					if (nxtStep)
						getDiagCoordinate2(_minNode->pos, solution);
					if (solution.size()>1900 || nxtStep == 0)
					{
						//printf("PTrails:%d\n", trials);

#ifndef  STATIC_START_END
						for (int id = 0; id < endNodes.size(); id++)
							endNodes[id].first->clearGoalNode();
#endif
						std::reverse(solution.begin(), solution.end());
						return;
					}
				}
				lastMinV = minVt;


			}
		}
#ifndef  STATIC_START_END
		for (int id = 0; id < endNodes.size(); id++)
			endNodes[id].first->clearGoalNode();
#endif


	}
#endif USE_PAIR_WISE_DISTANCES

	void PercolateUp(int index)
	{
		heapElement2 elem = theHeap[index];
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
	void PercolateDown(int index)
	{
		heapElement2 elem = theHeap[index];
		int maxSize = theHeap.size();

		while (true)
		{
			int child1 = (index << 1) + 1;
			if (child1 >= maxSize)
				break;

			int child2 = child1 + 1;

			// If the first child has smaller key
#ifdef USE_FLOAT_DIST
			if (child2 == maxSize || theHeap[child1].fVal < theHeap[child2].fVal || abs(theHeap[child1].fVal - theHeap[child2].fVal)<0.1)

#else
			if (child2 == maxSize || theHeap[child1].fVal <= theHeap[child2].fVal)

#endif
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


	/// Functions for managing the heap
	inline void AddToOpen(JumpNode sg, float fVal, char dir)
	{
#ifdef REPLACE_POPPED
		if (canReplaceTop)	// If the top element of the heap can be replaced,
		{
			theHeap[0] = (heapElement2(sg, fVal, dir));	// .. replace it
			PercolateDown(0);		// and percolate down
			canReplaceTop = false;	// the top element is no longer replaceable
		}
		else
#endif
		{	// add element as usual
			theHeap.push_back(heapElement2(sg, fVal, dir));
			PercolateUp(theHeap.size() - 1);
		}
	}
	void PopMin()
	{
#ifdef REPLACE_POPPED
		canReplaceTop = true;	// Don't pop it immediately, just mark it as replaceable
#else
		theHeap[0] = theHeap.back();
		theHeap.pop_back();
		if (theHeap.size())

			PercolateDown(0);
#endif
	}
#ifdef REPLACE_POPPED
	void PopReplacableTop()
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
#endif
	heapElement2 GetMin()
	{
		return theHeap.front();
	}
	bool hasDirectConnection(short partitionEx,short partitionEy,Coordinate &c,Coordinate  endCoord)
	{
		endCoord.x -= c.x;
		endCoord.y -= c.y;
		bool sameDir = (endCoord.x <= 0) == (endCoord.y <= 0);
		if (abs(endCoord.x) >= abs(endCoord.y))
			return	isSpaceIdX(partitionEx, c.x + (sameDir ? endCoord.y : -endCoord.y), c.y + endCoord.y);
		return	isSpaceIdY(partitionEy, c.x + endCoord.x, c.y + (sameDir ? endCoord.x : -endCoord.x));
	}
	char getMoveableDirections(int sX, int sY)
	{
		char ret = 0;
		bool e = isPassable(Coordinate(sX + 1, sY));
		bool w = isPassable(Coordinate(sX - 1, sY));
		bool s = isPassable(Coordinate(sX , sY+1));
		bool n = isPassable(Coordinate(sX , sY-1));
		if (e)
		{
			ret |= 1 << 2;
			if (s && isPassable(Coordinate(sX + 1, sY+1)))
				ret |= 1 << 3;
			if (n && isPassable(Coordinate(sX + 1, sY - 1)))
				ret |= 1 << 1;
		}
		if (w)
		{
			ret |= 1 << 6;
			if (s && isPassable(Coordinate(sX - 1, sY + 1)))
				ret |= 1 << 5;
			if (n && isPassable(Coordinate(sX - 1, sY - 1)))
				ret |= 1 << 7;
		}
		if (n)
			ret |= 1 << 0;
		if (s)
			ret |= 1 << 4;
		return ret;
	}
	void findSolution(int sX, int sY, int _eX, int _eY, vector<Coordinate> & solution)
	{


		//return;
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
		if (directSolution(sX, sY, eX, eY, solution))
			return;
		partitionEX=getSpaceIdX(eX, eY);
		partitionEY=getSpaceIdY(eX, eY);
		if (search >= 50000)
			ResetSearch();

		search++;


		static vector<pair<JumpNode, char> > startingNodes;// [8];
		static vector<pair<JumpNode, char> > endNodes;
		Coordinate endCoord(eX, eY);
		Coordinate startCoord(sX, sY);
#ifdef STATIC_START_END
		static int originalEndSize;
		if (startingNodes.size() == 0)
		{
			startingNodes.clear();
			getAllStartingNodes(sX, sY, startingNodes);
		}
		if (endNodes.size() == 0)
		{
#else
		startingNodes.clear();
		getAllStartingNodes(sX, sY, startingNodes);

#endif
	/*	int d0 = getDir(Coordinate(0, 0), Coordinate(0, -1));
		int d1 = getDir(Coordinate(0, 0), Coordinate(1, -1));
		int d2= getDir(Coordinate(0, 0), Coordinate(1, 0));
		int d3 = getDir(Coordinate(0, 0), Coordinate(1, 1));
		int d4 = getDir(Coordinate(0, 0), Coordinate(0, 1));
		int d5 = getDir(Coordinate(0, 0), Coordinate(-1, 1));
		int d6 = getDir(Coordinate(0, 0), Coordinate(-1, 0));
		int d7 = getDir(Coordinate(0, 0), Coordinate(-1, -1));*/

		endNodes.clear();
		getAllEndNodes(eX, eY, endNodes);

			//getAllEndNodes(eX, eY, endNodes);
#ifdef STATIC_START_END
			originalEndSize = endNodes.size();
#else
			int originalEndSize = endNodes.size();

#endif


			char forcedDirsEnd = getMoveableDirections(eX,eY);
			for (int i = 0; i < originalEndSize; i++)
				endNodes[i].first->setGoalNode(0xFF);

			for (int i = 0; i < originalEndSize; i++)
			{
				char dir2 = getDiagDir(endCoord, endNodes[i].first->pos, endNodes[i].second);
				char forcedDirs = (dir2 & 1 ? forcedNeighbours(endNodes[i].first->pos, (endNodes[i].second + 1) & 7) | forcedNeighbours(endNodes[i].first->pos, (endNodes[i].second + 7) & 7) : forcedNeighbours(endNodes[i].first->pos, dir2));
				char dirs = naturalNeighbours(dir2) | forcedDirs;
				endNodes[i].first->setGoalNode(0xFF);
				for (int nDir = 0; nDir < 8;nDir++)

				{
					if (((1 << nDir)&dirs) || (nDir&1)==0)
					{
						vector<pair<JumpPointNode2*, char> > & incNodes = endNodes[i].first->incDirs[nDir];
						for (int j = 0; j < incNodes.size(); j++)
						{
							JumpNode n = incNodes[j].first;
						//	int nDir = getDir(endNodes[i].first->pos, n->pos);
							int nDir3 = getDir(endCoord, n->pos);
							int diagDir = getDiagDir(endCoord, n->pos, nDir3);

							if (diagDir == dir2&&(dir2 == 2 || dir2 == 6) && (n->pos.y == endNodes[i].first->pos.y))
							{
								if (n->getGoalNode() == 0)
								{
									endNodes.push_back(pair<JumpPointNode2*, char> (n,8));

									n->setGoalNode(1);
									if (n->isPrunedEdge((incNodes[j].second)))
									{
										int newDir = 0;// (incNodes[j].second + 1) & 7;
										for (int l = 0; l < 3; l++)
										{
											newDir = (incNodes[j].second + 7+l) & 7;

											int incConnections = getNoIncConnections(n->id, newDir);
											for (int k = 0; k < incConnections; k++)
											{
												JumpNode n2 = getIncomingConnection(n->id, newDir, k).first;

												if (n2->getGoalNode()==0)
												{
													endNodes.push_back(pair<JumpPointNode2*, char>(n2, 8));
													n2->parent = n;
													n2->setGoalNode(1);
												}
											}

											//
										}
									}
								}
							}
							else if (diagDir == dir2 &&  (dir2 == 0 || dir2 == 4) && (n->pos.x == endNodes[i].first->pos.x))
							{
								if (n->getGoalNode() == 0)
								{
									endNodes.push_back(pair<JumpPointNode2*, char>(n, 8));
									n->setGoalNode(1);
									if (n->isPrunedEdge((incNodes[j].second)))
									{
										int newDir = 0;
										for (int l = 0; l < 3; l++)
										{
											newDir = (incNodes[j].second + 7 + l) & 7;
											int incConnections = getNoIncConnections(n->id, newDir);
											for (int k = 0; k < incConnections; k++)
											{
												JumpNode n2 = getIncomingConnection(n->id, newDir, k).first;
												if (n2->getGoalNode() == 0)
												{
													endNodes.push_back(pair<JumpPointNode2*, char>(n2, 8));

													n2->parent = n;
													n2->setGoalNode(1);
												}
											}
											//newDir = (incNodes[j].second + 7) & 7;

											//
										}
									}
								}
							}
							else if ((1 << nDir)&dirs)// && (endNodes[i].first->isPrunedEdge((nDir)&7)))
							{
								if (n->getGoalNode() == 0)
								{
									endNodes.push_back(pair<JumpPointNode2*, char>(n, 8));
									n->setGoalNode(1);

									n->parent = endNodes[i].first;
										
								}
								else if (n->parent)
								{
									float cost1 = Node::estimateDistance(endCoord, endNodes[i].first->pos) + Node::estimateDistance(n->pos, endNodes[i].first->pos);
									float cost2 = Node::estimateDistance(endCoord, n->parent->pos) + Node::estimateDistance(n->pos, n->parent->pos);
									if (cost1<cost2)
										n->parent = endNodes[i].first;
									//int a = 1;
								}
							}
						}
					}
				}
			}


#ifdef STATIC_START_END

		}
#endif
#ifdef USE_PAIR_WISE_DISTANCES
		if (pairDist)
		{
			getPairWiseSolution(sX, sY, eX, eY, solution, startingNodes, endNodes);
			return;
		}
#endif
		theHeap.clear();
		canReplaceTop = false;
		
		float minV = -1;
		JumpNode minNode = 0;
		JumpNode minNodeB = 0;
		bool skipStep1 = false;
		//if (openListBh.Count() == 0)
		{

			vector<Coordinate> tMpath;

			for (int i = 0; i < startingNodes.size(); i++)
				for (int dir = 0; dir < 8; dir++)
				{
					const int dirSize = getConnectionSize2(startingNodes[i].first->id, dir);
					for (int ii = 0; ii < dirSize; ii++)
					{
						JumpNode nStartNode = getConnectionItem2(startingNodes[i].first->id, dir, ii);// startingNodes[i].first->getDirItem(dir, ii);
						if (nStartNode->getGoalNode()==0xFF)
						{
							char nDir = getDir(endCoord, nStartNode->pos);
							char nDir2 = getDir(startingNodes[i].first->pos, endCoord);
							bool tempSkip = true;
							//Node::diagAndStraightCost(nStartNode->pos, endCoord, p);
							pair<int, int> p(0,0);
							if ((nDir == nDir2 || nDir == (nDir2 + 1) & 7 || nDir == (nDir2 + 7) & 7))
							{
								Node::diagAndStraightCost(endCoord, startingNodes[i].first->pos, p);
							}
							else
							{
								Node::diagAndStraightCost(nStartNode->pos, startingNodes[i].first->pos, p);
								Node::diagAndStraightCost(endCoord, nStartNode->pos, p);
								tempSkip = false;
							}
							Node::diagAndStraightCost(startingNodes[i].first->pos, Coordinate(sX, sY), p);

							//Node::diagAndStraightCost(startingNodes[i].first->pos, nStartNode->pos, p);
							float tempMin = Node::calcDistCost(p);
							if (tempMin < minV || minV == -1)
							{
								tMpath.clear();
								if ((tempSkip&& directSolution(startingNodes[i].first->pos.x, startingNodes[i].first->pos.y, endCoord.x, endCoord.y, tMpath)) || !tempSkip)
								{
									minV = tempMin;
									minNode = startingNodes[i].first;
									minNodeB = nStartNode;
									skipStep1 = tempSkip;
								}
							}
						}
					}
				}
		}


		//Insert the start nodes into the openList
		for (int i = 0; i < startingNodes.size(); i++)
		{
			if (!startingNodes[i].first->globalGoalId != -1)
			{
				SetOpen(startingNodes[i].first->getId());
				generated[startingNodes[i].first->getId()] = search;
				parent[startingNodes[i].first->getId()] = 0;
				startingNodes[i].first->v = Node::estimateDistance(startingNodes[i].first->pos, Coordinate(sX, sY));
				float heuristic = Node::estimateDistance(startingNodes[i].first->pos, endCoord);
				if (minV==-1 || (startingNodes[i].first->v + heuristic <= minV))
					AddToOpen(startingNodes[i].first, startingNodes[i].first->v + heuristic, getDiagDir(Coordinate(sX, sY), startingNodes[i].first->pos, startingNodes[i].second));
			}
		}
		//printf("%d %d %d", startingNodes.size(), endNodes.size(), originalEndSize);
		//Keep iterating over openlist until a solution is found or list is empty
	////	int trials = 0;
		int added = 0;
		//vector<Coordinate> checkedSol;
		while (!theHeap.empty())
		{
			//	trials++;
			//Node2* currentNode = openListBh.PopMax();
			//checkedSol.push_back(currentNode->pos);
			JumpNode jpn = GetMin().sg;// (JumpNode)currentNode->data;
			char currentDir = GetMin().dir;
			PopMin();
			//printf("\n(%d,%d)->", jpn->pos.x, jpn->pos.y);
			if (IsOpen(jpn->getId()))
			{
				SetClosed(jpn->getId());
				unsigned char dirs = jpn->neighbours[currentDir];//forcedNeighbours(currentNode->pos, currentNode->dir) | naturalNeighbours(currentNode->dir);//

				if (checkIfSolution(jpn, solution, startCoord))
				{
#ifndef STATIC_START_END

					for (int id = 0; id < endNodes.size(); id++)
						endNodes[id].first->clearGoalNode();
#endif
					return;
				}
				if (jpn->parent)
				{
					if (generated[jpn->parent->getId()] != search)
					{
						float heuristic = 0;
						float newCost = jpn->v;

						heuristic = Node::estimateDistance(endCoord, jpn->parent->pos);
						newCost += Node::estimateDistance(jpn->pos, jpn->parent->pos);
						jpn->parent->v = newCost;
						AddToOpen(jpn->parent, jpn->parent->v + heuristic-0.1f, 0xff);
						SetOpen(jpn->parent->getId());
						generated[jpn->parent->getId()] = search;
						parent[jpn->parent->getId()] = jpn;
					}
					else if (IsOpen(jpn->parent->getId()))
					{
						float newCost = jpn->v;
						newCost += Node::estimateDistance(jpn->pos, jpn->parent->pos);
						if (newCost <  jpn->parent->v)
						{
							jpn->parent->v = newCost;
							float heuristic = 0;

							heuristic = Node::estimateDistance(endCoord, jpn->parent->pos);
							AddToOpen(jpn->parent, jpn->parent->v + heuristic, 0xff);
							parent[jpn->parent->getId()] = jpn;
							//added++;

						}
					}
				}

				for (int dir = 0; dir < 8; dir++)
				{
					if ((1 << dir)&dirs)
					{
						int numHighGoals = sizeOfHighConnections[jpn->id * 8 + dir];
						//printf("\n(HGn:%d:%d)",dir, numHighGoals);
						JumpNodeDir* highGoalData = rawHighConnectionStart[jpn->id * 8 + dir];
						for (int i = 0; i < numHighGoals; i++)
						{
							pair<JumpNode, char>  pt = highGoalData[i];// highGoals[i];// getHighConnectionItem(jpn->id, dir, i);g
							//printf("(HG_%d:%d:[%d,%d])", i, dir, pt.first->pos.x, pt.first->pos.y);
							if (generated[pt.first->getId()] != search)
							{
								float newCost = jpn->v;
								float heuristic = 0;
								heuristic = Node::estimateDistance(endCoord, pt.first->pos);
								newCost += Node::estimateDistance(jpn->pos, pt.first->pos);
								if (minV == -1 || (newCost + heuristic) <= minV)
								{
									pt.first->v = newCost;
									AddToOpen(pt.first, pt.first->v + heuristic, pt.second);
									SetOpen(pt.first->getId());
									generated[pt.first->getId()] = search;
									parent[pt.first->getId()] = jpn;
								//	added++;
								}

							}
							else if (IsOpen(pt.first->getId()))
							{
								float newCost =jpn->v;
								newCost += Node::estimateDistance(jpn->pos, pt.first->pos);

								
								if (newCost < pt.first->v)
								{
									pt.first->v = newCost;
									float heuristic = Node::estimateDistance(endCoord, pt.first->pos);

									AddToOpen(pt.first, pt.first->v + heuristic, pt.second);
									parent[pt.first->getId()] = jpn;
									//added++;

								}
							}

						}
					}
				}
				
				if (theHeap.size() > 0 && GetMin().fVal > minV + 0.1f && minV != -1)
					break;
			}
			PopReplacableTop();
		}


		if (minNode)
		{
			solution.push_back(Coordinate(eX, eY));
			if (!skipStep1)
				getDiagCoordinate2(minNodeB->pos, solution);
			getDiagCoordinate2(minNode->pos, solution);
			getDiagCoordinate2(Coordinate(sX, sY), solution);
		}
#ifndef STATIC_START_END

		for (int id = 0; id < endNodes.size(); id++)
			endNodes[id].first->clearGoalNode();
#endif
	}
	bool hasPoint(Coordinate c, vector<Coordinate> &cs)
	{
		for (int i = 0; i < cs.size(); i++)
			if (cs[i].x == c.x && cs[i].y == c.y)
				return true;
		return false;
	}
	void getEndSpaceIds(short spaceX, short spaceY)
	{
		for (int i = 0; i<xBoundaryPoints[spaceY].size() - 1; i++)
			if (xBoundaryPoints[spaceY][i] <= spaceX && xBoundaryPoints[spaceY][i + 1] > spaceX)
			{
				eXSpace[0] = xBoundaryPoints[spaceY][i];
				eXSpace[1] = xBoundaryPoints[spaceY][i + 1];
				break;
			}
		for (int i = 0; i<yBoundaryPoints[spaceX].size() - 1; i++)
			if (yBoundaryPoints[spaceX][i] <= spaceY && yBoundaryPoints[spaceX][i + 1] > spaceY)
			{
				eYSpace[0] = yBoundaryPoints[spaceX][i];
				eYSpace[1] = yBoundaryPoints[spaceX][i + 1];
				break;
			}
	}

	JumpNode binarySearchR(const vector<JumpPointTableEntry> & v, short val)
	{
		short l = 0, r = v.size() - 1;
		if (r < 0)
			return 0;
		short index = r / 2;
		while (1)
		{
			if (v[index].first >= val && v[index].second <= val)
			{
				if (v[index].first == val && index + 1 < v.size() && v[index + 1].second == val)
					return v[index + 1].jp;
				return  v[index].jp;
			}
			if (v[index].second > val)
			{
				r = index - 1;
				if (r <0 || r<l)
					return 0;
			}
			else
			{
				l = index + 1;
				if (l > v.size() - 1 || r<l)
					return 0;
			}

			index = l + (r - l) / 2;

		}
		return 0;
	}
	JumpNode binarySearchL(const vector<JumpPointTableEntry> & v, short val)
	{
		short l = 0, r = v.size() - 1;
		if (r < 0)
			return 0;
		short index = r / 2;
		while (1)
		{
			if (v[index].first <= val && v[index].second >= val)
			{
				if (v[index].first == val && index - 1 >-1 && v[index - 1].second == val)
					return v[index - 1].jp;
				return  v[index].jp;
			}
			if (v[index].second > val)
			{
				r = index - 1;
				if (r <0 || r<l)
					return 0;
			}
			else
			{
				l = index + 1;
				if (l > v.size() - 1 || r<l)
					return 0;
			}

			index = l + (r - l) / 2;

		}
		return 0;
	}
	JumpNode binarySearchLBackwards(const vector<JumpPointTableEntry> & v, short val)
	{
		short l = 0, r = v.size() - 1;
		if (r < 0)
			return 0;
		short index = r / 2;
		while (1)
		{
			if (v[index].third <= val && v[index].second >= val)
			{
				if (v[index].third == val && index - 1 >-1 && v[index - 1].second == val)
					return v[index - 1].jp;
				return  v[index].jp;
			}
			if (v[index].third > val)
			{
				r = index - 1;
				if (r <0 || r<l)
					return 0;
			}
			else
			{
				l = index + 1;
				if (l > v.size() - 1 || r<l)
					return 0;
			}

			index = l + (r - l) / 2;

		}
		return 0;
	}
	JumpNode binarySearchRBackwards(const vector<JumpPointTableEntry> & v, short val)
	{
		short l = 0, r = v.size() - 1;
		if (r < 0)
			return 0;
		short index = r / 2;
		while (1)
		{
			if (v[index].third >= val && v[index].second <= val)
			{
				if (v[index].third == val && index + 1 < v.size() && v[index + 1].second == val)
					return v[index + 1].jp;
				return  v[index].jp;
			}
			if (v[index].second > val)
			{
				r = index - 1;
				if (r <0 || r<l)
					return 0;
			}
			else
			{
				l = index + 1;
				if (l > v.size() - 1 || r<l)
					return 0;
			}

			index = l + (r - l) / 2;

		}
		return 0;
	}
	bool getJumpPointNew(Coordinate s, const char direction, JumpNode& jp)
	{
		s = nextCoordinate(s, direction);
		bool ret = false;

		int index;
		switch (direction)
		{
		case 0://North
			jp = binarySearchR(jumpLookup[0][s.x], s.y);
			return jp != 0;
		case 2://EAST
			jp = binarySearchL(jumpLookup[1][s.y], s.x);
			return jp != 0;
		case 4://SOUTH
			jp = binarySearchL(jumpLookup[2][s.x], s.y);
			return jp != 0;
		case 6://WEST
			jp = binarySearchR(jumpLookup[3][s.y], s.x);
			return jp != 0;
		}
		return -1;
	}
	bool getJumpPointNewEnds(Coordinate s, const char direction, JumpNode& jp)
	{
		s = nextCoordinate(s, direction);
		bool ret = false;

		int index;
		switch (direction)
		{
		case 0://North
			jp = binarySearchRBackwards(jumpLookup[2][s.x], s.y);
			return jp != 0;
		case 2://EAST
			jp = binarySearchLBackwards(jumpLookup[3][s.y], s.x);
			return jp != 0;
		case 4://SOUTH
			jp = binarySearchLBackwards(jumpLookup[0][s.x], s.y);
			return jp != 0;
		case 6://WEST
			jp = binarySearchRBackwards(jumpLookup[1][s.y], s.x);
			return jp != 0;
		}
		return -1;
	}
	
	JumpNode jumpNew(const Coordinate &c, const char dir)
	{
		Coordinate nc = nextCoordinate(c, dir);
		bool isDiag = dirIsDiagonal(dir);
		Coordinate offset(0, 0);
		offset = nextCoordinate(offset, dir);

		while (1)
		{
			bool b = true;
#ifdef DIAG_UNBLOCKED
			b = ((dir & 1) == 0) || ((dir & 1) && (isPassable(Coordinate(nc.x - offset.x, nc.y)) && isPassable(Coordinate(nc.x, nc.y - offset.y))));
#endif
			if (!isPassable(nc) || !b)
				return 0;
			int index = gridIndex(nc);
			if (forcedNeighbours(nc, dir))
			{

				JumpNode n = jumpPoints[pair<short, short>(nc.x, nc.y)];
				if (n == 0)
				{
					n = new JumpPointNode2(nc.x, nc.y, jumpPoints.size());
					jumpPoints[pair<short, short>(nc.x, nc.y)] = n;
				}
				return n;
				// 0;
			}
			if (isDiag)
			{
				JumpNode newP = 0;
				if (getJumpPointNew(nc, (dir + 7) & 7, newP))
					return newP;
				if (getJumpPointNew(nc, (dir + 1) & 7, newP))
					return newP;
			}
			else
			{
				JumpNode newP;
				getJumpPointNew(c, dir, newP);
				return newP;
			}
			nc.add(offset);
		}
		return 0;
	}

	void jumpNewDiags(const Coordinate &c, const char dir, vector<JumpNode> &nodes)
	{
		Coordinate nc = nextCoordinate(c, dir);
		bool isDiag = dirIsDiagonal(dir);
		Coordinate offset(0, 0);
		offset = nextCoordinate(offset, dir);

		while (1)
		{
			bool b = true;
#ifdef DIAG_UNBLOCKED
			b = ((dir & 1) == 0) || ((dir & 1) && (isPassable(Coordinate(nc.x - offset.x, nc.y)) && isPassable(Coordinate(nc.x, nc.y - offset.y))));
#endif
			if (!isPassable(nc) || !b)
				return;
			int index = gridIndex(nc);


			if (isDiag)
			{
				JumpNode newP;
				if (getJumpPointNew(nc, (dir + 7) & 7, newP))
					nodes.push_back(newP);

				if (getJumpPointNew(nc, (dir + 1) & 7, newP))
					nodes.push_back(newP);
			}
			else
			{
				JumpNode newP;
				if (getJumpPointNew(c, dir, newP))
					nodes.push_back(newP);
				return;
			}
			if (forcedNeighbours(nc, dir) || forcedNeighbours(nc, (dir + 1) & 7) || forcedNeighbours(nc, (dir + 7) & 7))
			{
				JumpNode n = jumpPoints[pair<short, short>(nc.x, nc.y)];
				if (n == 0)
				{
					n = new JumpPointNode2(nc.x, nc.y, jumpPoints.size());
					jumpPoints[pair<short, short>(nc.x, nc.y)] = n;
				}
				nodes.push_back(n);
				return;
			}
			nc.add(offset);
		}

	}
	void jumpNewDiagsEnds(const Coordinate &c, const char dir, vector<pair<JumpNode, char> > &nodes, bool reverseSearch)
	{
		Coordinate nc = nextCoordinate(c, dir);
		bool isDiag = dirIsDiagonal(dir);
		Coordinate offset(0, 0);
		offset = nextCoordinate(offset, dir);

		while (1)
		{
			bool b = true;
#ifdef DIAG_UNBLOCKED
			b = ((dir & 1) == 0) || ((dir & 1) && (isPassable(Coordinate(nc.x - offset.x, nc.y)) && isPassable(Coordinate(nc.x, nc.y - offset.y))));
#endif
			if (!isPassable(nc) || !b)
				return;
			int index = gridIndex(nc);
			
			if ((isPassable(nextCoordinate(nc, (dir + 1) & 7)) && forcedNeighbours(nc, (dir + 5) & 7)) || (isPassable(nextCoordinate(nc, (dir + 7) & 7)) && forcedNeighbours(nc, (dir + 3) & 7)))
			{
				JumpNode n = jumpPoints[pair<short, short>(nc.x, nc.y)];
				if (n == 0)
				{
					n = new JumpPointNode2(nc.x, nc.y, jumpPoints.size());
					jumpPoints[pair<short, short>(nc.x, nc.y)] = n;
				}
				nodes.push_back(pair<JumpNode, char>(n,dir));
				
				int dir2 = (dir + 1) & 7;
				if (isPassable(nextCoordinate(nc, (dir + 1) & 7)) && forcedNeighbours(nc, (dir + 5) & 7))
				{
					int numHighGoals = sizeOfHighConnections[n->id * 8 + dir2];
					JumpNodeDir* highGoalData = rawHighConnectionStart[n->id * 8 + dir2];
					for (int i = 0; i < numHighGoals; i++)
						nodes.push_back(pair<JumpNode, char>(highGoalData[i].first,dir));
				}
				dir2 = (dir + 7) & 7;
				if (isPassable(nextCoordinate(nc, (dir + 7) & 7)) && forcedNeighbours(nc, (dir +3) & 7))
				{
					int numHighGoals = sizeOfHighConnections[n->id * 8 + dir2];
					JumpNodeDir* highGoalData = rawHighConnectionStart[n->id * 8 + dir2];
					for (int i = 0; i < numHighGoals; i++)
						nodes.push_back(pair<JumpNode, char>(highGoalData[i].first,dir));
				}
				//return;
			}
			if (isDiag)
			{
				JumpNode newP;
				bool b1 = isPassable(nextCoordinate(nc, (dir + 1) & 7));
				bool b2 = isPassable(nextCoordinate(nc, (dir + 7) & 7));
				if (b2)
					if (reverseSearch)
					{
						if (getJumpPointNewEnds(nc, (dir + 7) & 7, newP))
							nodes.push_back(pair<JumpNode, char>(newP,dir));
					}
					else
					{
						if (getJumpPointNew(nc, (dir + 7) & 7, newP))
							nodes.push_back(pair<JumpNode, char>(newP, dir));
						if (getJumpPointNewEnds(nc, (dir + 7) & 7, newP))
							nodes.push_back(pair<JumpNode, char>(newP, 8));
					}
				if (b1)
					if (reverseSearch)
					{
						if (getJumpPointNewEnds(nc, (dir + 1) & 7, newP))
							nodes.push_back(pair<JumpNode, char>(newP, dir));
					}
					else
					{
						if (getJumpPointNewEnds(nc, (dir + 1) & 7, newP))
							nodes.push_back(pair<JumpNode, char>(newP, 8));
						if (getJumpPointNew(nc, (dir + 1) & 7, newP))
							nodes.push_back(pair<JumpNode, char>(newP, dir));
					}
			}
			else
			{
				JumpNode newP;
				if (getJumpPointNewEnds(c, dir, newP))
					nodes.push_back(pair<JumpNode, char>(newP, dir));
				return;
			}
			nc.add(offset);
		}

	}

	

};
