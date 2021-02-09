import heapq

class Driver:

    def __init__(self):

        self.grid = []
        self.targets = []
        self.initializeVariables()

        if(self.algo[:-1] == 'BFS'):
            paths = self.runBFS(self.grid, self.start, self.targets)
        elif(self.algo[:-1] == 'UCS'):
            paths = self.runUCS(self.grid, self.start, self.targets)
        elif(self.algo[:-1] == 'A*'):
            paths = self.runAStar(self.grid, self.start, self.targets)

        else:
            self.writeFailFile()
            return

        self.writePaths(paths)
        return

    def getAStarPathCost(self,path_org):
        path = path_org[:]
        sum = 0
        current = path.pop(0)
        while(len(path)!=0):
            next = path.pop(0)
            stepCost = self.getStepCostAStar(current,next)
            sum += stepCost + abs(self.grid[current[1]][current[0]] - self.grid[next[1]][next[0]])
            current = next
        return sum

    def initializeVariables(self):
        file = open("input.txt", 'r')
        self.algo = file.readline().upper()
        wh_str = file.readline()
        self.w = int(wh_str.split()[0])
        self.h = int(wh_str.split()[1])
        xy_str = file.readline()
        self.landingW = int(xy_str.split()[0])
        self.landingH = int(xy_str.split()[1])
        self.maxElev = int(file.readline())
        noOfTargets = int(file.readline())
        for i in range(noOfTargets):
            str_target_pos = file.readline()
            pos = []
            pos.append(int(str_target_pos.split()[0]))
            pos.append(int(str_target_pos.split()[1]))
            self.targets.append(pos)
        for i in range(self.h):
            str_elevs = file.readline()
            elevs = []
            list_elev = list(str_elevs.split())

            for j in range(self.w):
                elevs.append(int(list_elev[j]))

            self.grid.append(elevs)

        self.start = [self.landingW, self.landingH]

    def writePaths(self,paths):
        if(len(paths) == 0):
            self.writeFailFile()
            return
        outFile = open("output.txt", "w")
        str_line = ''
        for path in paths:
            if(len(path)==0):
                str_line += 'FAIL\n'
            else:
                for pos in path:
                    str_line += str(pos[0])+','+str(pos[1])+' '
                str_line = str_line[:-1] + '\n'
        outFile.write(str_line[:-1])
        outFile.close()
        return

    @staticmethod
    def writeFailFile():
        outFile = open("output.txt", "w")
        outFile.write('FAIL')
        outFile.close()
        return

    def testGoal(self,current,goal):
        return (current[0] == goal[0] and goal[1] == current[1])

    def testGoals(self,current):
        goalsIndices = []
        for i in range(len(self.targets)):
            if(self.testGoal(current,self.targets[i])):
                goalsIndices.append(i)
        return goalsIndices

    def getNeighbours(self,current):
        w = current[0]
        h = current[1]
        hrange = [h-1,h,h+1]
        wrange = [w-1,w,w+1]
        nodeNeighbors = []
        for hh in hrange:
            for ww in wrange:
                if( not(h==hh and w==ww) and (hh >= 0 and hh < len(self.grid)) and (ww >= 0 and ww < len(self.grid[0])) ):
                    if abs(self.grid[h][w] - self.grid[hh][ww]) <= self.maxElev:
                        nodeNeighbors.append([ww,hh])
        return nodeNeighbors

    def runBFS(self, grid, start, targets):

        paths = []
        goalFound = []

        for i in range(len(targets)):
            paths.append([])
            goalFound.append(False)

        allGoalsFound = False

        visited = []
        for i in range(len(grid)):
            fal_arr = []
            for j in range(len(self.grid[0])):
                fal_arr.append(False)
            visited.append(fal_arr)

        frontier = []
        frontier.append(start)
        visited[start[1]][start[0]] = True
        came_from = []

        for i in range(len(grid)):
            none_arr = []
            for j in range(len(self.grid[0])):
                none_arr.append(None)
            came_from.append(none_arr)

        testIndices = self.testGoals(start)
        if(len(testIndices) != 0):
            for testIndex in testIndices:
                goalFound[testIndex] = True

            test = True
            for i in range(len(targets)):
                test = test and goalFound[i]
            allGoalsFound = test

        while(len(frontier)!=0 and not allGoalsFound):

            current = frontier.pop(0)

            for neighbour in self.getNeighbours(current):

                if(not visited[neighbour[1]][neighbour[0]]):

                    testIndices = self.testGoals(neighbour)
                    if(len(testIndices) != 0):

                        for testIndex in testIndices:
                            goalFound[testIndex] = True

                        test = True
                        for i in range(len(targets)):
                            test = test and goalFound[i]
                        allGoalsFound = test

                    else:
                        frontier.append(neighbour)
                    visited[neighbour[1]][neighbour[0]] = True
                    came_from[neighbour[1]][neighbour[0]] = current

        for i in range(len(targets)):
            if(goalFound[i]):
                path = []
                node = targets[i]

                while(node != None):
                   path.append(node)
                   node = came_from[node[1]][node[0]]

                path.reverse()
                paths[i] = path

        return paths

    def runUCS(self, grid, start, targets):

        paths = []

        goalFound = []

        for i in range(len(targets)):
            paths.append([])
            goalFound.append(False)

        allGoalsFound = False

        frontier = []
        heapq.heappush(frontier,(0,start))
        came_from = []
        cost_so_far = []

        for i in range(len(grid)):
            none_arr = []
            inf_arr = []
            for j in range(len(self.grid[0])):
                none_arr.append(None)
                inf_arr.append(2**30)
            came_from.append(none_arr)
            cost_so_far.append(inf_arr)
        cost_so_far[start[1]][start[0]] = 0

        testIndices = self.testGoals(start)
        if(len(testIndices) != 0):
            for testIndex in testIndices:
                goalFound[testIndex] = True

            test = True
            for i in range(len(targets)):
                test = test and goalFound[i]
            allGoalsFound = test

        while(len(frontier)!=0 and not allGoalsFound):
            currentPopped = heapq.heappop(frontier)
            current = currentPopped[1]

            testIndices = self.testGoals(current)
            if(len(testIndices) != 0):
                for testIndex in testIndices:
                    goalFound[testIndex] = True

                test = True
                for i in range(len(targets)):
                    test = test and goalFound[i]
                allGoalsFound = test
                if(allGoalsFound):
                    break

            for neighbour in self.getNeighbours(current):

                stepCost = self.getStepCostUCS(current, neighbour)
                newCost = stepCost + cost_so_far[current[1]][current[0]]
                if(newCost < cost_so_far[neighbour[1]][neighbour[0]]):
                    cost_so_far[neighbour[1]][neighbour[0]] = newCost
                    heapq.heappush(frontier,(newCost,neighbour))
                    came_from[neighbour[1]][neighbour[0]] = current

        for i in range(len(targets)):
            if(goalFound[i]):
                path = []
                node = targets[i]

                while(node != None):
                   path.append(node)
                   node = came_from[node[1]][node[0]]

                path.reverse()
                paths[i] = path

        return paths

    def runAStar(self, grid, start, targets):
        paths = []
        for target in targets:

            if(target[0] >= self.w or target[1] >= self.h or target[1]<0 or target[0]<0):
                paths.append([])
                continue

            goalFound = False

            frontier = []
            heapq.heappush(frontier, (0,start))
            came_from = []
            cost_so_far = []

            for i in range(len(grid)):
                none_arr = []
                inf_arr = []
                for j in range(len(self.grid[0])):
                    none_arr.append(None)
                    inf_arr.append(2**30)
                came_from.append(none_arr)
                cost_so_far.append(inf_arr)
            cost_so_far[start[1]][start[0]] = 0

            while(len(frontier)!=0):
                currentPopped = heapq.heappop(frontier)
                current = currentPopped[1]

                if(self.testGoal(current,target)):
                    goalFound = True
                    break

                for neighbour in self.getNeighbours(current):

                    stepCost = self.getStepCostAStar(current, neighbour)
                    newCost = stepCost + cost_so_far[current[1]][current[0]]
                    if(newCost < cost_so_far[neighbour[1]][neighbour[0]]):
                        cost_so_far[neighbour[1]][neighbour[0]] = newCost
                        heapq.heappush(frontier,(newCost + self.heuristic(neighbour,target),neighbour))
                        came_from[neighbour[1]][neighbour[0]] = current

            if(goalFound):
                path = []
                node = target

                while(node != None):
                    path.append(node)
                    node = came_from[node[1]][node[0]]

                path.reverse()
                paths.append(path)
            else:
                paths.append([])

        return paths

    def getStepCostUCS(self, neighbour, current):
        if(abs((neighbour[0]+neighbour[1])-(current[0]+current[1]))==1):
            return 10
        return 14

    def getStepCostAStar(self, neighbour, current):

        elevDiff = abs(self.grid[neighbour[1]][neighbour[0]] - self.grid[current[1]][current[0]])

        if(abs((neighbour[0]+neighbour[1])-(current[0]+current[1]))==1):
            return 10+elevDiff
        return 14+elevDiff

    def heuristic(self,target, neighbour):
        #2D Eucl + elevDiff
        return (( (target[0] - neighbour[0]) ** 2 + (target[1] - neighbour[1]) ** 2) ** 0.5 )*10 + abs(self.grid[target[1]][target[0]] - self.grid[neighbour[1]][neighbour[0]])

if __name__ == '__main__':
    try:
        driver = Driver()
    except:
        Driver.writeFailFile()