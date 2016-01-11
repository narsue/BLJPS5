#pragma once
#include "Node.h"
#include "BinaryHeap.h"
#include "PathFindingAlgorithm.h"

#ifdef DIAG_UNBLOCKED
#define ASTAR_ALG_NAME "ASTAR_UNBLOCKED"
#else
#define ASTAR_ALG_NAME "ASTAR_BLOCKED"
#endif
using namespace std;
// N, NE, E, SE, S, SW, W, NW , StartPosition

class AStar : public PathFindingAlgorithm
{
	private:
		//Special container classes that help with general allocation of memory and search
		NodeContainer nodesC;
		BinaryHeap openListBh;

		//Table of flags indicating is a map location has been searched (Closed List)
		char *testedGrid;

		//Boundary lookup tables for the x and y axis
		vector<vector<short>> xBoundaryPoints, yBoundaryPoints;
		vector<vector<short>> xBoundaryPointsBackup, yBoundaryPointsBackup;

		//Map data
		char * gridData;
		int gridWidth, gridHeight;

		//Goal node position and index
		int eX, eY, endIndex;


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
		bool isCoordinateBlocked(const Coordinate &c)
		{
			return isPassable(c);
		}
		AStar(char * grid,int width,int height) : PathFindingAlgorithm(ASTAR_ALG_NAME,AT_ASTAR)
		{
			gridData=grid;
			gridWidth=width;
			gridHeight=height;
			testedGrid=new char[gridWidth*gridHeight/8+1];

		}
		int getGridWidth()
		{
			return gridWidth;
		}
		int getGridHeight()
		{
			return gridHeight;
		}
		~AStar()
		{
			delete [] testedGrid;
		}
		void setChecked(const int index)
		{
			testedGrid[index/8]= testedGrid[index/8]|(1<<(index%8));
		}
		bool isChecked(const int index)
		{
			return (testedGrid[index/8]&(1<<(index%8)))>0;
		}
		void backupPreProcess()
		{
		}
		void useBackupData()
		{
		}
		void findSolution(int sX,int sY,int _eX,int _eY, vector<Coordinate> & solution)
		{
			eX = _eX;
			eY = _eY;
			solution.clear();
			endIndex=gridIndex(Coordinate(eX,eY));

			if (!(sX>=0 && sX<gridWidth && 
				sY>=0 && sY<gridHeight && 
				eX>=0 && eX<gridWidth && 
				eY>=0 && eY<gridHeight &&
				isPassable(Coordinate(sX,sY)) && isPassable(Coordinate(eX,eY))) )
			{
				return ;
			}
			if (sX==eX && sY==eY)
				return ;

			openListBh.clear();
			memset(testedGrid,0,gridWidth*gridHeight/8+1);
			nodesC.reset();

			//Insert the start nodes into the openList
			Node* startNode = nodesC.getNewNode(Coordinate(sX,sY),eX,eY,8,0);
			openListBh.Insert(startNode);

			setChecked(gridIndex(startNode->pos));

			//Keep iterating over openlist until a solution is found or list is empty
			while (openListBh.Count())
			{
				Node* currentNode=openListBh.PopMax();	
				for (int dir = 0; dir < 8; dir++)
				{
					Coordinate nextC = nextCoordinate(currentNode->pos, dir);

					int index =gridIndex(nextC);

					bool b = true;
#ifdef DIAG_UNBLOCKED
					if (dir & 1)
					{
						Coordinate nextC2 = nextCoordinate(currentNode->pos, (dir + 1) % 8);
						Coordinate nextC3 = nextCoordinate(currentNode->pos, (dir + 7) % 8);
						int index2 = gridIndex(nextC2);
						int index3 = gridIndex(nextC3);
						b = inBounds(index2) && inBounds(index3) && isPassable(nextC2) && isPassable(nextC3);
					}
#endif 
					if (inBounds(index) && isPassable(nextC) && b)
					{
						if (index==endIndex)
						{
							Coordinate end(eX,eY);
							solution.push_back(end);
							Node*solutionNode=currentNode;
							while (solutionNode)
							{
								solution.push_back(solutionNode->pos);
								solutionNode= solutionNode->parent;
							}
							return ;
						}
		
						if (!isChecked(index) )
						{
							openListBh.Insert(nodesC.getNewNode(nextC,eX,eY,dir,currentNode));
							setChecked(index);
						}
						else
						{						
							Node * t =nodesC.getNewNode(nextC,eX,eY,dir,currentNode);
							openListBh.InsertSmaller(t);										
						}

					}
				}
			}
		}
};