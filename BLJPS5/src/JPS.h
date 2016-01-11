#pragma once
#include "Node.h"
using namespace std;
#include "PathFindingAlgorithm.h"
#define JPS_REVISION "-r3"


#ifdef DIAG_UNBLOCKED
#define JPS_ALG_NAME string("JPS_UNBLOCKED")+string(JPS_REVISION)
#else
#define JPS_ALG_NAME string("JPS_BLOCKED")+string(JPS_REVISION)
#endif
//Directions
// N, NE, E, SE, S, SW, W, NW , StartPosition
#define NO_DIRECTION 8
class JPS : public PathFindingAlgorithm
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
		inline bool dirIsDiagonal(const int dir)
		{
			return (dir & 1);
		}
		inline int implies (const int a,const int b)
		{
			return a ? b : 1;	
		}
		inline unsigned char addDirectionToSet (const unsigned char dirs,const int dir)
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
				if (ENTERABLE(1) && !ENTERABLE(2))
					dirs = addDirectionToSet(dirs, (dir + 1) & 7);
				if (ENTERABLE(7) && !ENTERABLE(6))
					dirs = addDirectionToSet(dirs, (dir + 7) & 7);
#else
				if (!implies(ENTERABLE(6), ENTERABLE(5)))
					dirs = addDirectionToSet(dirs, (dir + 6) & 7);
				if (!implies(ENTERABLE(2), ENTERABLE(3)))
					dirs = addDirectionToSet(dirs, (dir + 2) & 7);
#endif
			}
			else
			{
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
		void backupPreProcess()
		{
			
		}
		void useBackupData()
		{
			
		}
		const Coordinate indexToCoordinate(const int index)
		{
			return Coordinate(index%gridWidth,index/gridWidth );
		}
		bool isPassable(const Coordinate &c)
		{
			int index = gridIndex(c);
			if (index == -1)
				return false;
			return  !(gridData[index/8]&(1<<(index& 7)));
		}

		JPS(char * grid, int width, int height) : PathFindingAlgorithm(JPS_ALG_NAME, AT_JPS)
		{
			gridData=grid;
			gridWidth=width;
			gridHeight=height;
			testedGrid=new char[gridWidth*gridHeight/8+1];
		}
		~JPS()
		{
			delete [] testedGrid;
		}
		unsigned char naturalNeighbours (const int dir)
		{
			if (dir == NO_DIRECTION)
				return 255;

			unsigned char dirs = 0;
			dirs = addDirectionToSet (dirs, dir);
			if (dirIsDiagonal (dir)) {
				dirs = addDirectionToSet (dirs, (dir + 1) & 7);
				dirs = addDirectionToSet (dirs, (dir + 7) & 7);
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


		int jump(const Coordinate &c,const char dir)
		{
			Coordinate nc = nextCoordinate(c,dir);
			bool b = true;
#ifdef DIAG_UNBLOCKED
			if (dir & 1)
			{
				Coordinate nextC2 = nextCoordinate(c, (dir + 1) & 7);
				Coordinate nextC3 = nextCoordinate(c, (dir + 7) & 7);
				int index2 = gridIndex(nextC2);
				int index3 = gridIndex(nextC3);
				b = inBounds(index2) && inBounds(index3) && isPassable(nextC2) && isPassable(nextC3);
			}
#endif
			if (!isPassable(nc) ||!b)
				return -1;

			int index  = gridIndex(nc);
			unsigned char dirs;
			if (index == endIndex || (dirs=forcedNeighbours(nc,dir)))
				return index;

			if (dirIsDiagonal(dir))
			{
				int next = jump(nc,(dir+7)&7);
				if (next  >=0 )
					return index;
				next = jump(nc,(dir+1)& 7);
				if (next >=0)
					return index;
			}

			return jump(nc,dir);
		}
		void setChecked(const int index)
		{
			testedGrid[index/8]= testedGrid[index/8]|(1<<(index& 7));
		}
		bool isChecked(const int index)
		{
			return (testedGrid[index/8]&(1<<(index& 7)))>0;
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

			//Ready the openlist, closed list and node container for search
			openListBh.clear();
			memset(testedGrid, 0, gridWidth*gridHeight / 8 + 1);
			nodesC.reset();

			//Insert the start nodes into the openList
			Node* startNode = nodesC.getNewNode(Coordinate(sX, sY), eX, eY, 8, 0);
			openListBh.Insert(startNode);
			setChecked(gridIndex(startNode->pos));


			//Keep iterating over openlist until a solution is found or list is empty
			while (openListBh.Count())
			{
				Node* currentNode=openListBh.PopMax();
				unsigned char dirs = forcedNeighbours (currentNode->pos, currentNode->dir)  | naturalNeighbours (currentNode->dir);

				for (int dir = 0; dir < 8; dir ++)
				{
					if ((1<<dir)&dirs)
					{
						int index =jump(currentNode->pos,dir);
						if (inBounds(index))
						{
							Coordinate CoordinateNewC= indexToCoordinate(index);

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
								openListBh.Insert(nodesC.getNewNode(CoordinateNewC,eX,eY,dir,currentNode));
								setChecked(index);
							}
							else
							{
								Node * t =nodesC.getNewNode(CoordinateNewC,eX,eY,dir,currentNode);
								openListBh.InsertSmaller(t);
							}
						}
					}
				}
			}
			
		}
};