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
                    mapStats.append(names[2])
        if len (names)>3:
            if names[3] == MAPNAME:
                if (len(i) >1):                
                    mapStats.append(names[4])          
                    
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
                    mapMemoryStats.append([names[2],float(i[1])])
        if len (names)>3:
            if names[3] == MAPNAME:
                if (len(i) >1):                
                    mapMemoryStats.append([names[4],float(i[1])])  
    return mapMemoryStats
 
allMaps = ["adaptiveDepth","bgmaps","dao","rooms"]
for mapId in allMaps:
    MAPNAME=mapId
    mapStats=readMapCounts("PreprocessingTime_BLJPS_UNBLOCKED-r6.txt")
    print len(mapStats)
    print "Read Map Counts",MAPNAME
    mapMemoryStats=[]
   # mapMemoryStats.append([readMapMemoryStats("memoryOutput_NovelAStar.txt"),"NovelAStar"])
  #  mapMemoryStats.append([readMapMemoryStats("memoryOutput_PPQ.txt"),"ppq"])
    

    mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS_UNBLOCKED-r6.txt"),"blJPS-unblocked-r5"])
    
    mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP2-UNBLOCKED-r5.txt"),"blJPS-exp2-unblocked-r4"])
    
    mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP3-UNBLOCKED-r7.txt"),"blJPS-exp3-unblocked-r6"])
    
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP4-UNBLOCKED-r14.txt"),"blJPS-exp4-unblocked-r14"])
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP4-UNBLOCKED-r15.txt"),"blJPS-exp4-unblocked-r15"])
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP4-UNBLOCKED-r16.txt"),"blJPS-exp4-unblocked-r16"])
   # mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP4-UNBLOCKED-r17.txt"),"blJPS-exp4-unblocked-r17"])
   # mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP4-UNBLOCKED-r18.txt"),"blJPS-exp4-unblocked-r18"])
    mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP4-UNBLOCKED-r19.txt"),"blJPS-exp4-unblocked-r19"])

    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP5-UNBLOCKED-r18.txt"),"blJPS-exp5-unblocked-r18"])
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP5-UNBLOCKED-r20.txt"),"blJPS-exp5-unblocked-r20"])
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP5-UNBLOCKED-r21.txt"),"blJPS-exp5-unblocked-r21"])
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP5-UNBLOCKED-r22.txt"),"blJPS-exp5-unblocked-r22"])
    mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP5-UNBLOCKED-r23.txt"),"blJPS-exp5-unblocked-r23"])
    
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS-EXP5-UNBLOCKED-r8.txt"),"blJPS-exp5-unblocked-r8"])
    
    mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_SubGoal.txt"),"SubGoal"]) 
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS_SUBGOAL-UNBLOCKED.txt"),"BLJPS_SUBGOAL-UNBLOCKED"])
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS_SUBGOAL-UNBLOCKED-DIST.txt"),"BLJPS_SUBGOAL-UNBLOCKED-DIST"])
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS_SUBGOAL-UNBLOCKED-ENDS-STARTS.txt"),"BLJPS_SUBGOAL-UNBLOCKED-ENDS-STARTS"])
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_BLJPS_SUBGOAL-UNBLOCKED-ENDS-STARTS-DIST.txt"),"BLJPS_SUBGOAL-UNBLOCKED-ENDS-STARTS-DIST"])
    #mapMemoryStats.append([readMapMemoryStats("PreprocessingTime_SubGoalFast.txt"),"SubGoalFast"])
    for i in mapMemoryStats:
        avgNum=[0]
        avgCount=0.0
        #print len(i[0]),len(mapStats),i[0][3162][0]
        for ii in range(0,len(i[0])):
            avgNum[0]+=float(i[0][ii][1])
            avgCount+=1.0

        print MAPNAME,i[1],(avgNum[0]/avgCount)*1000
       # print MAPNAME,i[1],round(avgMemVsGridSize[0]/avgCount,1),round(avgMemVsGridSize[1]/avgCount,1)
