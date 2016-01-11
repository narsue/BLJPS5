#pragma once
#include <Windows.h>
bool checkDirectory(char* directoryPath, char* extension,vector<char*> &replayPaths)
{
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	TCHAR *szDir=directoryPath;
	hFind = FindFirstFile(szDir, &ffd);
	DWORD creationTime=0;
	char ext[60];

   do
   {
      if ((ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) &&(!(ffd.dwFileAttributes&FILE_ATTRIBUTE_HIDDEN)) )
      {
		  if (strcmp(ffd.cFileName,".")&&strcmp(ffd.cFileName,".."))
		  {
				char tempDirectory[600];
				strcpy(tempDirectory,szDir);
				tempDirectory[strlen(tempDirectory)-2]=0;
				strcat(tempDirectory,"/");

				strcat(tempDirectory,ffd.cFileName);
				strcat(tempDirectory,"/*");
				if (checkDirectory(tempDirectory, extension, replayPaths))
					return true;
		  }
      }
		else
	  {
		int size = strlen( ffd.cFileName);

		if (strcmp(strlwr(ffd.cFileName+size-strlen(extension)),extension)==0)
		{
			char* tempS = new char [500];
			replayPaths.push_back(tempS);
			strcpy(tempS,directoryPath);
			tempS[strlen(tempS)-1]=0;
			strcat(tempS, ffd.cFileName);
		}
	  }	  
   }
   while (FindNextFile(hFind, &ffd) != 0);
   return false;
}
void getMapProblemFiles(bool getStaticMapProblems, char * mapsDirectory, vector<char*> &replayPaths)
{
	char * newMapDir = new char[strlen(mapsDirectory)+4];
	strcpy(newMapDir, mapsDirectory);
	strcat(newMapDir,"/*");
	if (getStaticMapProblems)
	{

		checkDirectory(newMapDir, ".scen", replayPaths);
		//checkDirectory("../../maps/dao/problems/*", ".scen",replayPaths);
		//checkDirectory("../../maps/bgmaps/problems/*", ".scen", replayPaths);
		//checkDirectory("../../maps/rooms/problems/*", ".scen", replayPaths);
		//checkDirectory("../../maps/adaptiveDepth/problems/*", ".scen", replayPaths);
	}
	else
	{
		checkDirectory(newMapDir, ".dscen", replayPaths);
		//checkDirectory("../maps/dao/DynamicUnits/*",replayNo,replayId,fileName,directoryPath,ext);
		//checkDirectory("../maps/bgmaps/DynamicUnits/*",replayNo,replayId,fileName,directoryPath,ext);
		///checkDirectory("../maps/adaptiveDepth/DynamicUnits/*", replayNo, replayId, fileName, directoryPath, ext);
		//checkDirectory("../maps/rooms/DynamicUnits/*", replayNo, replayId, fileName, directoryPath, ext);
	}
	/*FILE * fp = fopen("maps.txt","w");
	for (int i =0;i<replayPaths.size();i++)
	{
		char tempPath[1024];
		char tempPath2[1024];
		char filename[1024];

		strcpy(tempPath,replayPaths[i]);
		int index = strlen(tempPath)-1;
		while (tempPath[index]!='/')
			index--;
		memset(filename,0,1024);
		memcpy(filename,&tempPath[index+1],strlen(tempPath)-1-index);
		tempPath[index]=0;
		
		sprintf(tempPath2,"%s %s/Problems/%s.scen\n",replayPaths[i],tempPath,filename);
		fprintf(fp,"./bljps2 -full %s",tempPath2);
	}
	fclose(fp);*/
	delete[] newMapDir;
}
void writeOutPathStats(char* scenarioName,int problemId,float pathDist,double time)
{
	static FILE* fp = fopen("pathStats.txt","w");
	fprintf(fp,"%s %d %f %f\n",scenarioName,problemId,pathDist,time);
	fflush(fp);
}
float getPathLen(std::vector<pair<short,short> >& thePath)
{
	float dist=0;
	if (thePath.size()==0)
		return 0;
	for (int i =0;i<thePath.size()-1;i++)
	{
		dist += sqrt(((float)(thePath[i].first - thePath[i + 1].first)*(thePath[i].first - thePath[i + 1].first) + (float)(thePath[i].second - thePath[i + 1].second)*(thePath[i].second - thePath[i + 1].second)));
	}
	return dist;
}