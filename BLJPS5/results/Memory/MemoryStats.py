from operator import itemgetter, attrgetter
global MAPNAME
MAPNAME= "dao"#"adaptiveDepth":#"rooms":#dao bgmaps adaptiveDepth

def readMapCounts(fileName):# returns a list of filenames and number paths each has
    fp = open(fileName,"r" )
    s = fp.read()
    fp.close()
    s= s.split('\n')
    mapStats = []
    print fileName
    for i in s:
        i=i.split(' ')
        while i[0][:3]== "../":
            i[0]= i[0][3:]          
        names= i[0].split('/')
        if len (names)>1:
           
            if names[1] == MAPNAME:
                if (len(i) >1):                
                    mapStats.append([names[2]+i[1],float(i[2])*float(i[3]),float(i[4])])
         
                    
    return mapStats

def readMapMemoryStats(fileName):# returns a list of filenames and number paths each has
    fp = open(fileName,"r" )
    s = fp.read()
    fp.close()
    s= s.split('\n')
    mapMemoryStats = []
    for i in s:
        i=i.split(' ')
        while i[0][:3]== "../":
            i[0]= i[0][3:]  
        names= i[0].split('/')
        if len (names)>1:
            if names[1] == MAPNAME:
                if (len(i) >1):                
                    mapMemoryStats.append([names[2]+i[1],0,float(i[6])-float(i[5]),float(i[7])-float(i[5])])
                    #name, 0, memoryUsageAfterPreprocessing, memoryUsageMaxAfterSearchingAllPaths
 
    return mapMemoryStats
 
allMaps = ["adaptiveDepth","bgmaps","dao","rooms"]
for mapId in allMaps:
    MAPNAME=mapId
    mapStats=readMapCounts("memoryOutput_BLJPS_UNBLOCKED-r6.txt")
    print "Read Map Counts",MAPNAME
    mapMemoryStats=[]

    mapMemoryStats.append([readMapMemoryStats("memoryOutput_JPS_UNBLOCKED.txt"),"JPS_UNBLOCKED"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_JPS_BLOCKED.txt"),"JPS_BLOCKED"])
    mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS_UNBLOCKED-r6.txt"),"blJPS_UNBLOCKED"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS_BLOCKED.txt"),"blJPS_BLOCKED"])
    
    mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS-EXP2-UNBLOCKED-r5.txt"),"blJPS-exp2-unblocked"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS-EXP2-BLOCKED.txt"),"blJPS-exp2-blocked"])

    mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS-EXP3-UNBLOCKED-r7.txt"),"blJPS-exp3-unblocked"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS-EXP3-BLOCKED.txt"),"blJPS-exp3-blocked"])
    
    mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS-EXP4-UNBLOCKED-r19.txt"),"blJPS-exp4-unblocked"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS-EXP4-BLOCKED.txt"),"blJPS-exp4-blocked"])
    
    mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS-EXP5-UNBLOCKED-r23.txt"),"blJPS-exp5-unblocked"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS-EXP5-BLOCKED.txt"),"blJPS-exp5-blocked"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS-EXP5-UNBLOCKED-r18.txt"),"blJPS-exp5-unblocked-r18"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS_SUBGOAL-UNBLOCKED.txt"),"BLJPS_SUBGOAL-UNBLOCKED"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS_SUBGOAL-UNBLOCKED-DIST.txt"),"BLJPS_SUBGOAL-UNBLOCKED-DIST"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS_SUBGOAL-UNBLOCKED-ENDS-STARTS.txt"),"BLJPS_SUBGOAL-UNBLOCKED-ENDS-STARTS"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_BLJPS_SUBGOAL-UNBLOCKED-ENDS-STARTS-DIST.txt"),"BLJPS_SUBGOAL-UNBLOCKED-ENDS-STARTS-DIST"])

    mapMemoryStats.append([readMapMemoryStats("memoryOutput_SubGoal.txt"),"SubGoal"])
    #mapMemoryStats.append([readMapMemoryStats("memoryOutput_SubGoalFast.txt"),"SubGoalFast"])

    for i in mapMemoryStats:
        avgNum=[0,0]
        avgMemVsGridSize=[0,0]
        averageGridSize=0
        avgCount=0
        idA =0
        #print len(i[0]),len(mapStats),i[0][3162][0]
        for ii in range(0,len(i[0])):
           # print i[0][ii][0],i[0][ii][2]/1024,i[0][ii][3]/1024
            avgNum[0]+=i[0][ii][2]/1024
            avgNum[1]+=i[0][ii][3]/1024
            averageGridSize+=mapStats[ii][1]
            avgMemVsGridSize[0]+=i[0][ii][2]/mapStats[ii][1]
            avgMemVsGridSize[1]+=i[0][ii][3]/mapStats[ii][1]
            #if mapStats[ii][0] != i[0][ii][0] :
           #     print MAPNAME,mapStats[ii],i[0][ii],idA
            #if i[0][ii][2]/mapStats[ii][1]>4000:
             #   print MAPNAME,mapStats[ii],i[0][ii],idA
            avgCount+=1
            idA+=1
        print MAPNAME,i[1],round(avgNum[0]/avgCount,1),"Kb average starting ram",round(avgNum[1]/avgCount,1),"Kb average max ram"#, 
       # print MAPNAME,i[1],round(avgMemVsGridSize[0]/avgCount,1),round(avgMemVsGridSize[1]/avgCount,1)
