#pragma once
#include "Node.h"
#include "PathFindingAlgorithm.h"
using namespace std;


//Directions
// N, NE, E, SE, S, SW, W, NW , StartPosition
#define NO_DIRECTION 8
class BL_JPS_PLUS : public PathFindingAlgorithm
{
	private:
		//Special container classes that help with general allocation of memory and search
		NodeContainer nodesC;
		BinaryHeap openListBh;

		//Table of flags indicating is a map location has been searched (Closed List)
		char *testedGrid;

		//Boundary lookup tables for the x and y axis
		vector<vector<short>> xBoundaryPoints, yBoundaryPoints;
		vector<vector<short>> xBoundaryPointsBackup,yBoundaryPointsBackup; 

		//Map data
		char * gridData;
		int gridWidth, gridHeight;

		//Goal node position and index
		int eX, eY, endIndex;

		//Jump Point Lookup table and a backup
		unsigned int *preprocessedJumpPoints, *preprocessedJumpPointsBackup;

		bool inBounds(const int index)
		{
			return index<gridWidth*gridHeight&&index>=0;
		}
		int gridIndex(const Coordinate &c)
		{
			if (c.x<0 || c.x>=gridWidth || c.y<0 || c.y>=gridHeight)
				return -1;
			return (c.y*gridWidth)+c.x;
		}
		Coordinate nextCoordinate(const Coordinate& c,const int dir)
		{
			static char dirMov[]={0,-1,1,-1,1,0,1,1,0,1,-1,1,-1,0,-1,-1,0,0};
			return Coordinate(c.x+dirMov[dir*2],c.y+dirMov[dir*2+1]);
		}
		int dirIsDiagonal (const int dir)
		{
			return (dir % 2) != 0;
		}
		inline int implies (const int a,const int b)
		{
			return a ? b : 1;	
		}
		inline unsigned char addDirectionToSet (const unsigned char dirs,const int dir)
		{
			return dirs | 1 << dir;
		}
		unsigned char forcedNeighbours (const Coordinate &coord, const int dir)
		{
			if (dir > 7)
				return 0;

			unsigned char dirs = 0;
		#define ENTERABLE(n) isPassable ( nextCoordinate (coord, (dir + (n)) % 8))
			if (dirIsDiagonal (dir)) {
				if (!implies (ENTERABLE (6), ENTERABLE (5)))
					dirs = addDirectionToSet (dirs, (dir + 6) % 8);
				if (!implies (ENTERABLE (2), ENTERABLE (3)))
					dirs = addDirectionToSet (dirs, (dir + 2) % 8);

			}
			else {
				if (!implies (ENTERABLE (7), ENTERABLE (6)))
					dirs = addDirectionToSet (dirs, (dir + 7) % 8);
				if (!implies (ENTERABLE (1), ENTERABLE (2)))
					dirs = addDirectionToSet (dirs, (dir + 1) % 8);
			}		
			#undef ENTERABLE	

			return dirs;
		}


	public:

		const Coordinate indexToCoordinate(const int index)
		{
			return Coordinate(index%gridWidth,index/gridWidth );
		}
		
		bool isPassable(const Coordinate &c)
		{
			int index = gridIndex(c);
			if (index == -1)
				return false;
			return  !(gridData[index/8]&(1<<(index%8)));
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
		inline void clearAllPreJumpData(const short x,const short y)
		{
			memset(&preprocessedJumpPoints[y*gridWidth*8+x*8],-1,8*4);
		}
		void flushReProcess()
		{
			return;
			for (int y =0;y<gridHeight;y++)
			{
				if (xBoundaryPoints[y].size()==0)
				{
					bool currentPassable=false;
					xBoundaryPoints[y].clear();
					for (int x =0;x<gridWidth;x++)
					{
						if (isPassable(Coordinate(x,y))!= currentPassable)
						{
							xBoundaryPoints[y].push_back(x);
							currentPassable=!currentPassable;
						}
					}
					if (currentPassable)
						xBoundaryPoints[y].push_back(gridWidth);
				}
			}

			for (int x =0;x<gridWidth;x++)
			{
				if (yBoundaryPoints[x].size()==0)
				{
					bool currentPassable=false;
					yBoundaryPoints[x].clear();

					for (int y =0;y<gridHeight ;y++)
					{
						if (isPassable(Coordinate(x,y))!= currentPassable)
						{
							yBoundaryPoints[x].push_back(y);
							currentPassable=!currentPassable;
						}
					}
					if (currentPassable)
						yBoundaryPoints[x].push_back(gridHeight);
				}
			}

						
		}
		void verifyreProcessing()
		{
			return;
			for (int y =0;y<gridHeight;y++)
				for (int x =0;x<gridWidth;x++)
					if (isPassable(Coordinate(x,y)))
						for (int dir =0;dir<8;dir++)
						{
							Coordinate endP(x,y);
							int index = jumpNew(Coordinate(x, y), dir, endP);
							unsigned int val = gridIndex(endP) |(index==-1?0:(1<<31));
							if (preprocessedJumpPoints[y*gridWidth*8+x*8+dir]!=val &&preprocessedJumpPoints[y*gridWidth*8+x*8+dir]!=-1  )
							{
								printf("Incorrectly stored jump point\n");
								system("pause");
							}
						}
		}
		void reProcessGrid(int lx,int rx,int ty,int by)
		{
			memset(testedGrid,0,gridWidth*gridHeight/8+1);

			for (int y =ty;y<by;y++)
			{
				{
					bool currentPassable=false;
					xBoundaryPoints[y].clear();
					for (int x =0;x<gridWidth;x++)
					{
						if (isPassable(Coordinate(x,y))!= currentPassable)
						{
							xBoundaryPoints[y].push_back(x);
							currentPassable=!currentPassable;
						}
					}
					if (currentPassable)
						xBoundaryPoints[y].push_back(gridWidth);
				}
			}

			for (int x =lx;x<rx;x++)
			{
				{
					bool currentPassable=false;
					yBoundaryPoints[x].clear();

					for (int y =0;y<gridHeight ;y++)
					{
						if (isPassable(Coordinate(x,y))!= currentPassable)
						{
							yBoundaryPoints[x].push_back(y);
							currentPassable=!currentPassable;
						}
					}
					if (currentPassable)
						yBoundaryPoints[x].push_back(gridHeight);
				}
			}

			for (int y =ty-1;y<by+1;y++)
				for (int x =lx-1;x<rx+1;x++)
					for (int dir =0;dir<8;dir+=2)
					{
						Coordinate startP1(x,y);
						int index1 =gridIndex(startP1);

						while (index1!= -1  && isPassable(startP1))//  preprocessedJumpPoints[startP1.y*gridWidth*8+startP1.x*8+ dir]!= -1
						{
							clearAllPreJumpData(startP1.x,startP1.y);
							//setChecked(index1);

							//preprocessedJumpPoints[startP1.y*gridWidth*8+startP1.x*8+ dir]=-1;
							int dirSet[] = {(dir+7)%8,(dir+1)%8};
							for (int id=0;id<2;id++)
							{
								int dir2= dirSet[id];
								Coordinate startP(startP1.x,startP1.y);
								startP= nextCoordinate(startP,dir2);
								int index =gridIndex(startP);

							
								while (index!=-1 && !isChecked(index)  && isPassable(startP))// dir!= dir2&& (dir2+4)%8!= dir &&dir2%2==0 &&  preprocessedJumpPoints[startP.y*gridWidth*8+startP.x*8+dir2]!=-1 memcmp(compareVal,&preprocessedJumpPoints[startP.y*gridWidth*8+startP.x*8],4*8)
								{

									clearAllPreJumpData(startP.x,startP.y);
									setChecked(index);
									startP=nextCoordinate(startP,dir2);
									index =gridIndex(startP);
									

								}
							}
							startP1=nextCoordinate(startP1,dir);
							index1 =gridIndex(startP1);
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
			preprocessedJumpPointsBackup= new unsigned int[gridWidth*gridHeight*8];
			memcpy(preprocessedJumpPointsBackup,preprocessedJumpPoints,gridWidth*gridHeight*8*4);
			xBoundaryPointsBackup=xBoundaryPoints;
			yBoundaryPointsBackup=yBoundaryPoints;
		}
		void useBackupData()
		{
			memcpy(preprocessedJumpPoints,preprocessedJumpPointsBackup,gridWidth*gridHeight*8*4);
			xBoundaryPoints=xBoundaryPointsBackup;
			yBoundaryPoints=yBoundaryPointsBackup;
		}
		void preProcessGrid()
		{
			for (int y =0;y<gridHeight;y++)
			{
				bool currentPassable=false;
				if (xBoundaryPoints.size()<=y)
					xBoundaryPoints.push_back(vector<short>());
				else
					xBoundaryPoints[y].clear();
				for (int x =0;x<gridWidth;x++)
				{
					if (isPassable(Coordinate(x,y))!= currentPassable)
					{
						xBoundaryPoints[y].push_back(x);
						currentPassable=!currentPassable;
					}
				}
				//if (currentPassable)
					xBoundaryPoints[y].push_back(gridWidth);
			}

			for (int x =0;x<gridWidth;x++)
			{
				bool currentPassable=false;
				if (yBoundaryPoints.size()<=x)
					yBoundaryPoints.push_back(vector<short>());
				else
					yBoundaryPoints[x].clear();
				for (int y =0;y<gridHeight ;y++)
				{
					if (isPassable(Coordinate(x,y))!= currentPassable)
					{
						yBoundaryPoints[x].push_back(y);
						currentPassable=!currentPassable;
					}
				}
				//if (currentPassable)
					yBoundaryPoints[x].push_back(gridHeight);
			}

			memset(preprocessedJumpPoints, -1, gridWidth*gridHeight * 8 * 4);
			clearChecked();
	


			for (int y =0;y<gridHeight;y++)
				for (int x =0;x<gridWidth;x++)
					if (isPassable(Coordinate(x,y)))
						for (int dir =0;dir<8;dir++)
							if (preprocessedJumpPoints[y*gridWidth*8+x*8+dir]==-1 )
							{
								Coordinate endP(x,y);
								int index = jumpNew(Coordinate(x,y),dir,endP);
								if (index!=-1)
									endP = indexToCoordinate(index);

								unsigned int val = gridIndex(endP) |(index==-1?0:(1<<31));
								preprocessedJumpPoints[y*gridWidth*8+x*8+dir]= val ;
							}
		}
		BL_JPS_PLUS(char * grid, int width, int height) : PathFindingAlgorithm("BL-JPS+", AT_BL_JPS_PLUS)
		{
			gridData					= grid;
			gridWidth					= width;
			gridHeight					= height;
			testedGrid					= new char[gridWidth*gridHeight/8+1];
			preprocessedJumpPoints		= new unsigned int[gridWidth*gridHeight*8];
			memset(preprocessedJumpPoints,-1,gridWidth*gridHeight*8*4);
			preprocessedJumpPointsBackup=0;
		}
		void copyPreprocessedJumpPoints(unsigned int * data)
		{
			memcpy(preprocessedJumpPoints,data,gridWidth*gridHeight*8*4);
		}
		unsigned int* getPreprocessedData()
		{
			unsigned int*data= new unsigned int[gridWidth*gridHeight*8];
			memcpy(data,preprocessedJumpPoints,gridWidth*gridHeight*8*4);
			return data;
		}
		~BL_JPS_PLUS()
		{
			delete [] testedGrid;
			delete [] preprocessedJumpPoints;
			if (preprocessedJumpPointsBackup)
				delete preprocessedJumpPointsBackup;
		}
		unsigned char naturalNeighbours (const int dir)
		{
			if (dir == NO_DIRECTION)
				return 255;

			unsigned char dirs = 0;
			dirs = addDirectionToSet (dirs, dir);
			if (dirIsDiagonal (dir)) {
				dirs = addDirectionToSet (dirs, (dir + 1) % 8);
				dirs = addDirectionToSet (dirs, (dir + 7) % 8);
			}
			return dirs;
		}
		unsigned char nextDirectionInSet (unsigned char *dirs)
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
		int jump2(const Coordinate &To,Coordinate &from,bool isJumpPoint,int dir)
		{
			//if (!isPassable(c))
			//	return -1;
			int minX = min(from.x,To.x);
			int maxX = max(from.x,To.x);
			int minY = min(from.y,To.y);
			int maxY = max(from.y,To.y);
			//Passes through the destination as a straight line
			Coordinate next = nextCoordinate(Coordinate(0,0),dir);

			if (dir%2==0)
			{
				if (minX<= eX && maxX >= eX && minY<= eY && maxY >= eY  )
					return endIndex;
			}
			//Passes the destination as a diagonal line insert a new point
			else if ((minX<= eX && maxX >= eX) ||( minY<= eY && maxY >= eY))
				if (abs(from.x-eX) == abs(from.y-eY) &&(minX<= eX && maxX >= eX) &&( minY<= eY && maxY >= eY))
					return endIndex;
				else
					if (abs(from.x-eX) && abs(from.y-eY)) // make sure that the point is offset and isnt on a straight line
						if (abs(from.x-eX)<abs(from.y-eY))
						{
							if (mag(eY-from.y)== next.y && minX<= eX && maxX >= eX)
								return gridIndex(Coordinate(eX,from.y-abs(from.x-eX) *mag(from.y-eY) ));
						}
						else if (mag(eX-from.x)== next.x && minY<= eY && maxY >= eY )
								return gridIndex(Coordinate(from.x-abs(from.y-eY) *mag(from.x-eX),eY ));

			if (isJumpPoint)
			{
				return gridIndex(To);
			}
			return -1;
		}

		void setChecked(const int index)
		{
			testedGrid[index/8]= testedGrid[index/8]|(1<<(index%8));
		}
		bool isChecked(const int index)
		{
			return (testedGrid[index/8]&(1<<(index%8)));
		}
		void clearChecked()
		{
			memset(testedGrid, 0, gridWidth*gridHeight / 8 + 1);
		}
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
		bool directSolution(short sX, short sY, short eX, short eY, vector<Coordinate> & sol)
		{
			if (sY == eY)
			{
				if (isSpaceIdX(getSpaceIdX(sX, sY), eX, eY))
				{
					sol.push_back(Coordinate(sX, sY));
					sol.push_back(Coordinate(eX, eY));
					return true;
				}
			}
			else if (sX == eX)
			{
				if (isSpaceIdY(getSpaceIdY(sX, sY), eX, eY))
				{
					sol.push_back(Coordinate(sX, sY));
					sol.push_back(Coordinate(eX, eY));
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
						check.add(offset);
						if (!isPassable(check))
						{
							bPass = false;
							diagMovements = 0;
							break;
						}
					}
					if (bPass)
						if (isDiagOnly)//only diagonal movement
						{
						sol.push_back(Coordinate(sX, sY));
						sol.push_back(Coordinate(eX, eY));
						return true;
						}
						else //diagonal movement and then horiz/vertic
						{
							check.add(offset);
							sol.push_back(Coordinate(sX, sY));
							sol.push_back(Coordinate(check.x, check.y));
							sol.push_back(Coordinate(eX, eY));
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
						check.add(offset);
						if (!isPassable(check))
						{
							bPass = false;
							diagMovements = 0;
							break;
						}
					}
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
		void findSolution(int sX,int sY,int _eX,int _eY, vector<Coordinate> & solution)
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
				return ;
			}
			if (sX == eX && sY == eY)
				return ;
			
			if (directSolution(sX, sY,eX,eY, solution))
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
							//printf("Debug Message: shouldn't be called on static maps , preprocess first\n");
							Coordinate endP(currentNode->pos.x, currentNode->pos.y);
							int index = jumpNew(currentNode->pos, dir, endP);
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
								return ;
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

		pair<int, int> getEastEndPointReOpen(short x, short y)
		{
			if (y<0 || y >= gridHeight)
				return pair<int, int>(gridWidth, gridWidth);

			if (xBoundaryPoints[y][0]>x)
				return pair<int, int>(xBoundaryPoints[y][0], xBoundaryPoints[y][0]);

			for (int i = 1; i<xBoundaryPoints[y].size(); ++i)
				if (xBoundaryPoints[y][i] >= x)
					if (i % 2)
						return pair<int, int>(xBoundaryPoints[y][i] - 1, i + 1<xBoundaryPoints[y].size() ? xBoundaryPoints[y][i + 1] : gridWidth);
					else if (xBoundaryPoints[y][i] == x)
						return pair<int, int>(xBoundaryPoints[y][i + 1] - 1, i + 2<xBoundaryPoints[y].size() ? xBoundaryPoints[y][i + 2] : gridWidth);
					else
						return pair<int, int>(xBoundaryPoints[y][i], xBoundaryPoints[y][i]);

					return pair<int, int>(gridWidth, gridWidth);
		}
		pair<int, int> getWestEndPointReOpen(short x, short y)
		{
			if (y<0 || y >= gridHeight)
				return pair<int, int>(-1, -1);

			if (xBoundaryPoints[y][0]>x)
				return pair<int, int>(-1, -1);

			for (int i = 1; i<xBoundaryPoints[y].size(); ++i)
				if (xBoundaryPoints[y][i] >= x)
					if (i % 2 && xBoundaryPoints[y][i] == x)
						return pair<int, int>(xBoundaryPoints[y][i] - 1, (xBoundaryPoints[y][i] - 1));
					else if (xBoundaryPoints[y][i] == x)
						return pair<int, int>(xBoundaryPoints[y][i], i - 1<0 ? -1 : xBoundaryPoints[y][i - 1] - 1);
					else if (i % 2)
						return pair<int, int>(xBoundaryPoints[y][i - 1], i - 2<0 ? -1 : xBoundaryPoints[y][i - 2] - 1);
					else
						return pair<int, int>(xBoundaryPoints[y][i - 1] - 1, xBoundaryPoints[y][i - 1] - 1);

					return pair<int, int>(-1, -1);
		}

		pair<int, int> getSouthEndPointReOpen(short x, short y)
		{
			if (x<0 || x >= gridWidth)
				return pair<int, int>(gridHeight, gridHeight);

			if (yBoundaryPoints[x][0]>y)
				return pair<int, int>(yBoundaryPoints[x][0], yBoundaryPoints[x][0]);

			for (int i = 1; i<yBoundaryPoints[x].size(); ++i)
				if (yBoundaryPoints[x][i] >= y)
					if (i % 2)
						return pair<int, int>(yBoundaryPoints[x][i] - 1, i + 1<yBoundaryPoints[x].size() ? yBoundaryPoints[x][i + 1] : gridHeight);
					else if (yBoundaryPoints[x][i] == y)
						return pair<int, int>(yBoundaryPoints[x][i + 1] - 1, i + 2<yBoundaryPoints[x].size() ? yBoundaryPoints[x][i + 2] : gridHeight);
					else
						return pair<int, int>(yBoundaryPoints[x][i], yBoundaryPoints[x][i]);

					return pair<int, int>(gridHeight, gridHeight);
		}
		pair<int, int> getNorthEndPointReOpen(short x, short y)
		{
			if (x<0 || x >= gridWidth)
				return pair<int, int>(-1, -1);

			if (yBoundaryPoints[x][0]>y)
				return pair<int, int>(-1, -1);

			for (int i = 1; i<yBoundaryPoints[x].size(); ++i)
				if (yBoundaryPoints[x][i] >= y)
					if (i % 2 && yBoundaryPoints[x][i] == y)
						return pair<int, int>(yBoundaryPoints[x][i] - 1, (yBoundaryPoints[x][i] - 1));
					else if (yBoundaryPoints[x][i] == y)
						return pair<int, int>(yBoundaryPoints[x][i], i - 1<0 ? -1 : yBoundaryPoints[x][i - 1] - 1);
					else if (i % 2)
						return pair<int, int>(yBoundaryPoints[x][i - 1], i - 2<0 ? -1 : yBoundaryPoints[x][i - 2] - 1);
					else
						return pair<int, int>(yBoundaryPoints[x][i - 1] - 1, yBoundaryPoints[x][i - 1] - 1);
					return pair<int, int>(-1, -1);
		}

		bool getJumpPointNew(Coordinate s, int direction, Coordinate & jp, Coordinate *endCoordinate)
		{
			//if (!isPassable(s))
			//	return false;
			s = nextCoordinate(s, direction);

			if (!isPassable(s))
				return false;
			bool ret = false;
			if (direction == 2) //EAST
			{
				pair<int, int> up = getEastEndPointReOpen(s.x, s.y - 1);
				pair<int, int> center = getEastEndPointReOpen(s.x, s.y);
				pair<int, int> down = getEastEndPointReOpen(s.x, s.y + 1);

				if (endCoordinate)
					*endCoordinate = Coordinate(center.first , s.y);
				if (down.first != gridWidth && ((down.second<gridWidth&&down.first < center.first && down.second - 2<center.first) || (down.first == down.second && down.first - 2<center.first)))
				{
					jp = Coordinate(down.second - 1, s.y);
					ret = true;
				}
				if (up.first != gridWidth && ((up.second<gridWidth&&up.first<center.first&&up.second - 2<center.first) || (up.first == up.second && up.first - 2<center.first)))
				{
					jp = Coordinate(ret ? min(jp.x, up.second - 1) : up.second - 1, s.y);
					return true;
				}
			}
			else if (direction == 4)//SOUTH
			{
				pair<int, int> up = getSouthEndPointReOpen(s.x - 1, s.y);
				pair<int, int> center = getSouthEndPointReOpen(s.x, s.y);
				pair<int, int> down = getSouthEndPointReOpen(s.x + 1, s.y);

				if (endCoordinate)
					*endCoordinate = Coordinate(s.x, center.first);
				if (down.first != gridHeight && ((down.second<gridHeight&& down.first < center.first && down.second - 2<center.first) || (down.first == down.second && down.first - 2<center.first)))
				{
					jp = Coordinate(s.x, down.second - 1);
					ret = true;
				}
				if (up.first != gridHeight && ((up.second<gridHeight&&up.first<center.first&&up.second - 2<center.first) || (up.first == up.second && up.first - 2<center.first)))
				{
					jp = Coordinate(s.x, ret ? min(jp.y, up.second - 1) : up.second - 1);
					return true;
				}
			}
			else if (direction == 6) //WEST
			{
				pair<int, int> up = getWestEndPointReOpen(s.x, s.y - 1);
				pair<int, int> center = getWestEndPointReOpen(s.x, s.y);
				pair<int, int> down = getWestEndPointReOpen(s.x, s.y + 1);
				
				if (endCoordinate)
					*endCoordinate = Coordinate(center.first ,s.y);
				if (down.first != -1 && ((down.second>-1 && down.first > center.first && down.second + 2>center.first) || (down.first == down.second && down.first + 2>center.first)))
				{
					jp = Coordinate(down.second + 1, s.y);
					ret = true;
				}
				if (up.first != -1 && ((up.second>-1 && up.first>center.first&&up.second + 2>center.first) || (up.first == up.second && up.first + 2>center.first)))
				{
					jp = Coordinate(ret ? max(jp.x, up.second + 1) : up.second + 1, s.y);
					return true;
				}
			}
			else if (direction == 0) //North
			{
				pair<int, int> up = getNorthEndPointReOpen(s.x - 1, s.y);
				pair<int, int> center = getNorthEndPointReOpen(s.x, s.y);
				pair<int, int> down = getNorthEndPointReOpen(s.x + 1, s.y);

				if (endCoordinate)
					*endCoordinate = Coordinate(s.x,center.first );
				if (down.first != -1 && ((down.second>-1 && down.first > center.first && down.second + 2>center.first) || (down.first == down.second && down.first + 2>center.first)))
				{
					jp = Coordinate(s.x, down.second + 1);
					ret = true;
				}
				if (up.first != -1 && ((up.second>-1 && up.first>center.first&&up.second + 2>center.first) || (up.first == up.second && up.first + 2>center.first)))
				{
					jp = Coordinate(s.x, ret ? max(jp.y, up.second + 1) : up.second + 1);
					return true;
				}
			}
			return ret;

		}

		int jumpNew(const Coordinate &c, const char dir, Coordinate &endCoordinate)
		{
			endCoordinate = c;
			Coordinate nc = nextCoordinate(c, dir);
			bool isDiag = dirIsDiagonal(dir);
			Coordinate offset(0, 0);
			offset = nextCoordinate(offset, dir);

			while (1)
			{
				if (!isPassable(nc))
					return -1;
				endCoordinate = nc;

				int index = gridIndex(nc);

				if (forcedNeighbours(nc, dir) )
					return index;
				if (isDiag)
				{
					Coordinate newP(-1, -1);
					if (getJumpPointNew(nc, (dir + 7) % 8, newP,0))
						return index;
					if (getJumpPointNew(nc, (dir + 1) % 8, newP,0))
						return index;
				}
				else
				{
					Coordinate newP(-1, -1);
					getJumpPointNew(nc, dir, newP, &endCoordinate);
					return gridIndex(newP);
				}
				nc.add(offset);
			}
			return -1;
		}

};