#pragma once
struct JumpPointNode
{
	///
	JumpPointNode(short _x, short _y, int _id) :pos(_x, _y), id(_id)
	{
		memset(neighbours, 0, 8);
		edgeNode = 0;
		pruned = 0;
		optPointer = 0;
		memset(i1, 0, 8 * 2);
		globalGoalId = -1;
		//memset(i2, 0, 8 * 2);
	}
	JumpPointNode()
	{

	}
	bool addNeighbour(JumpPointNode * n, int dir)
	{
		addToDirs(n, dir);
		return neighbours[dir] == 0;
	}
	void addNatForcedNeighbours(unsigned char dirs, char dir)
	{
		neighbours[dir] = dirs;
	}
	int getId()
	{
		return id & 0x7FFFFFFF;
	}
	int getGoalNode()
	{
		return id & 0x80000000;
	}
	void setGoalNode()
	{
		id |= 0x80000000;
	}

	void clearGoalNode()
	{
		id &= 0x7FFFFFFF;
		optPointer = 0;
	}
	void addEdgeNode(int dir)
	{
		edgeNode |= 1 << dir;
	}
	bool isEdgeNode(int dir)
	{
		return edgeNode & (1 << dir);
	}
	void setHighLevelGoalNode(vector<pair<JumpPointNode*, char>> * _optPointer)
	{
		optPointer = _optPointer;
	}
	void pruneEdge(int dir)
	{
		pruned |= 1 << dir;
	}
	/*void unPruneEdge(int dir)
	{
		pruned &= ~(1 << dir);
	}*/
	bool isPrunedEdge(int dir)
	{
		return pruned & (1 << dir);
	}
	void addToDirs(JumpPointNode* n, int dir)
	{
		short pos = i1[dir];
		int size = getDirSize(dir);
		for (int i = i1[dir] - size; i < i1[dir]; i++)
			if (dirs[i].first == n)
				return;
		dirs.insert(dirs.begin() + pos, (pair<JumpPointNode*, char>(n,-1)));
			
		for (int i = dir; i < 8; i++)
			i1[i]++;
	}
	void addHighGoal(pair<JumpPointNode*, char> n, int dir)
	{
		int size = getDirSize(dir);
		int insertPos = -1;
		for (int i = i1[dir] - size; i < i1[dir]; i++)
		{
			if (insertPos == -1 && dirs[i].second == -1)
				insertPos = i;
			if (dirs[i].first == n.first)
			{
				dirs[i] = dirs[insertPos];
				dirs[insertPos] = n;
				return;
			}
		}
	}
	void addToIncDirs(pair<JumpPointNode*, char> s, int dir)
	{
		for (int i = 0; i < incDirs[dir].size(); i++)
			if (incDirs[dir][i].first == s.first)
				return;
		incDirs[dir].push_back(s);
	}
	int getDirSize(int dir)
	{
		return i1[dir]-(dir==0?0:i1[dir-1]);
	}
	pair<JumpPointNode*, char> getHighGoal(int dir, int itemId)
	{
		return dirs[i1[dir] - getDirSize(dir) + itemId];
	}
	JumpPointNode* getDirItem(int dir, int itemId)
	{
		return dirs[i1[dir] - getDirSize(dir) + itemId].first;
	}
	int getNumHighGoals(int dir)
	{
		int count = 0;
		for (int i = i1[dir] - getDirSize(dir); i < i1[dir]; i++)
			if (dirs[i].second != -1)
				++count;
		return count;
	}
	inline void setGlobalGoalId(int _GlobalGoalId)
	{
		globalGoalId = _GlobalGoalId;
	}
	Coordinate pos;
	char edgeNode;
	char pruned;
	short index;

	vector<pair<JumpPointNode*, char>> * optPointer;
	int id;

	//ifdef USE_PAIR_WISE_DISTANCES
	short globalGoalId;
	float v;
	unsigned char neighbours[8];
	short i1[8];
	//short i2[8];
	vector<pair<JumpPointNode*, char>> dirs;
//	vector<JumpPointNode*> dirs[8];
	vector<pair<JumpPointNode*,char>> incDirs[8];
//	vector<pair<JumpPointNode*, char>> highGoals[8];
};

//This is structure is a relica of the original but I'm slowly culling ellements to reduce memory size
struct JumpPointNode2
{
	///
	JumpPointNode2(short _x, short _y, int _id) :pos(_x, _y), id(_id)
	{
		memset(neighbours, 0, 8);
		edgeNode = 0;
		pruned = 0;
		goalDir = 0;
//		optPointer = 0;
//		memset(i1, 0, 8 * 2);
		globalGoalId = -1;
		parent = 0;
		//memset(i2, 0, 8 * 2);
	}
	JumpPointNode2()
	{

	}
	/*void addNeighbour(JumpPointNode2 * n, int dir)
	{
		//addToDirs(n, dir);
		//return neighbours[dir] == 0;
	}*/
	void addNatForcedNeighbours(unsigned char dirs, char dir)
	{
		neighbours[dir] = dirs;
	}
	int getId()
	{
		return id;
	}
	unsigned char getGoalNode()
	{
		return goalDir;
	}
	void setGoalNode(unsigned char dir)
	{
		goalDir |= dir;
	}

	void clearGoalNode()
	{
		goalDir=0;
		parent = 0;
		//optPointer = 0;
	}
	void addEdgeNode(int dir)
	{
		edgeNode |= 1 << dir;
	}
	bool isEdgeNode(int dir)
	{
		return edgeNode & (1 << dir);
	}
	/*void setHighLevelGoalNode(vector<pair<JumpPointNode2*, char>> * _optPointer)
	{
		optPointer = _optPointer;
	}*/
	void pruneEdge(int dir)
	{
		pruned |= 1 << dir;
	}
	/*void unPruneEdge(int dir)
	{
	pruned &= ~(1 << dir);
	}*/
	bool isPrunedEdge(int dir)
	{
		return pruned & (1 << dir);
	}
	/*void addToDirs(JumpPointNode2* n, int dir)
	{
		short pos = i1[dir];
		int size = getDirSize(dir);
		for (int i = i1[dir] - size; i < i1[dir]; i++)
			if (dirs[i].first == n)
				return;
		dirs.insert(dirs.begin() + pos, (pair<JumpPointNode2*, char>(n, -1)));

		for (int i = dir; i < 8; i++)
			i1[i]++;
	}
	void addHighGoal(pair<JumpPointNode2*, char> n, int dir)
	{
		int size = getDirSize(dir);
		int insertPos = -1;
		for (int i = i1[dir] - size; i < i1[dir]; i++)
		{
			if (insertPos == -1 && dirs[i].second == -1)
				insertPos = i;
			if (dirs[i].first == n.first)
			{
				dirs[i] = dirs[insertPos];
				dirs[insertPos] = n;
				return;
			}
		}
	}
	
	int getDirSize(int dir)
	{
		return i1[dir] - (dir == 0 ? 0 : i1[dir - 1]);
	}
	pair<JumpPointNode2*, char> getHighGoal(int dir, int itemId)
	{
		return dirs[i1[dir] - getDirSize(dir) + itemId];
	}
	JumpPointNode2* getDirItem(int dir, int itemId)
	{
		return dirs[i1[dir] - getDirSize(dir) + itemId].first;
	}
	int getNumHighGoals(int dir)
	{
		int count = 0;
		for (int i = i1[dir] - getDirSize(dir); i < i1[dir]; i++)
			if (dirs[i].second != -1)
				++count;
		return count;
	}*/
	inline void setGlobalGoalId(int _GlobalGoalId)
	{
		globalGoalId = _GlobalGoalId;
	}
	Coordinate pos;
	char edgeNode;
	char pruned;
	//short index;
	short globalGoalId;

//	vector<pair<JumpPointNode2*, char>> * optPointer;
	int id;
	JumpPointNode2*parent;
	//ifdef USE_PAIR_WISE_DISTANCES
	float v;
	unsigned char neighbours[8];
	unsigned char goalDir;
	vector<pair<JumpPointNode2*,char> > incDirs[8];
//	short i1[8];
	//short i2[8];
	//vector<pair<JumpPointNode2*, char>> dirs;
	//	vector<JumpPointNode*> dirs[8];

	//	vector<pair<JumpPointNode*, char>> highGoals[8];
};


#pragma once
