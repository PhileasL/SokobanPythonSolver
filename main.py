import time
import sys
import search
import copy
from search import Problem
from datetime import datetime

###########
# classes #
###########

class State:

    def __init__(self, actualPosition, boxes, costFocused = None):
        self.actualPosition=actualPosition
        self.boxes=boxes
        self.costFocused=costFocused

    def __eq__(self, other):
        return (self.actualPosition == other.actualPosition and self.boxes == other.boxes and self.costFocused == other.costFocused)

    def __hash__(self):
        tmpArray = []
        tmpArray.append(tuple(["pos", tuple(self.actualPosition), self.costFocused]))
        for i in self.boxes:
            tmpArray.append(tuple(i))
        return hash(tuple(tmpArray))

    def __str__(self, walls):
        copywalls = copy.deepcopy(walls)
        for box in self.boxes:
            copywalls[box[0]][box[1]] = '$'
        copywalls[self.actualPosition[0]][self.actualPosition[1]] = '@'
        printBeautifulPath(copywalls)

    def __lt__(self, other):
        return None

    def copy(self):
        newBoxes = []
        for i in self.boxes:
            newBoxes.append(copy.deepcopy(i))
        newActualPosition = copy.deepcopy(self.actualPosition)
        return State(newActualPosition, newBoxes, self.costFocused)

class FindEntry(Problem):

    def __init__(self, initial):
        self.initial = tuple(initial.get("goal"))
        self.walls = initial.get("walls")
        self.deadLocks = initial.get("deadLocks")
        self.goals = initial.get("goals")

    def goal_test(self, state):
        nbOfBlankSpaces = 0
        for i in getAroundPositions(state):
            if self.walls[i[0]][i[1]] != '#' and i not in self.deadLocks and i not in self.goals:
                nbOfBlankSpaces += 1
        if nbOfBlankSpaces >= 3:
            return True
        else:
            return False
    
    def actions(self, state):
        actions = []
        for i in getAroundPositions(state):
            if self.walls[i[0]][i[1]] != '#' and i not in self.deadLocks:
                actions.append(i)
        return actions
    
    def result(self, state, action):
        return tuple(action)

class ResolverCube(Problem):

    def __init__(self, initial):
        self.goalSolveOrder = initial.get("goalSolveOrder")
        self.goalsCosts = initial.get("goalsCosts")
        self.walls = initial.get("walls")
        self.deadLocks = initial.get("deadlocks")
        self.initial = State(initial.get("playerPos"), initial.get("boxes"), self.goalsCosts[0])
        self.heuristicCoeff = self.findHeuristicCoeff()

    def findHeuristicCoeff(self):
        coeff = [1, 2]
        cost = 0
        while abs(40-cost)>0.01:
            cost = 0
            for i in self.goalsCosts:
                cost += i * coeff[1]
            if cost == 40:
                return coeff[1]
            elif cost<40:
                coeff = [coeff[1], coeff[1]*2-coeff[0]]
            else:
                coeff = [coeff[0], coeff[1]-(coeff[1]-coeff[0])/2]
        return coeff[1]

    def goal_test(self, state):
        goalsAchieved = copy.deepcopy(self.goalSolveOrder)
        for i in state.boxes:
            if i in goalsAchieved:
                goalsAchieved.remove(i)
        if len(goalsAchieved) == 0:
            return True
        else:
            return False

    def actions(self, state):
        actions = []
        boxesOnWrongPlaces = copy.deepcopy(state.boxes)
        achievedGoals = []
        nextGoals = []
        for i in range(self.goalsCosts.index(state.costFocused)):
            achievedGoals.append(self.goalSolveOrder[i])
        if self.goalsCosts.count(state.costFocused) != 1:
            tmpGoals = []
            for i in range(len(self.goalsCosts)):
                if self.goalsCosts[i] == state.costFocused:
                    tmpGoals.append(self.goalSolveOrder[i])
                    nextGoals.append(self.goalSolveOrder[i])
            for i in tmpGoals:
                for box in state.boxes:
                    if i == box:
                        achievedGoals.append(box)
        else:
            nextGoals.append(self.goalSolveOrder[self.goalsCosts.index(state.costFocused)])
        for box in state.boxes:
            if box in achievedGoals:
                boxesOnWrongPlaces.remove(box)
        for nextGoal in nextGoals:
            for box in boxesOnWrongPlaces:
                results = pathCubeExists(self.walls, box, nextGoal, state.boxes, self.deadLocks, state.actualPosition)
                if results[0]:
                    actions.append([nextGoal, results[1], box])
        for box in boxesOnWrongPlaces:
            for around in getAroundPositions(box):
                i = box[0] + box[0] - around[0]
                j = box[1] + box[1] - around[1]
                if self.walls[around[0]][around[1]] == ' ' and around not in state.boxes and self.walls[i][j] == ' ' and [i,j] and [i,j] not in state.boxes \
                and around not in self.deadLocks and pathExists(self.walls, state.actualPosition, [i, j], state.boxes):
                    actions.append([around, box, box])
        return actions

    def result(self, state, action):
        newState = state.copy()
        newState.boxes[state.boxes.index(action[2])] = action[0]
        newState.actualPosition = action[1]
        goalsFocused = []
        if self.goalsCosts.count(state.costFocused) != 1:
            for i in range(len(self.goalsCosts)):
                if self.goalsCosts[i] == state.costFocused:
                    goalsFocused.append(self.goalSolveOrder[i])
        else:
            goalsFocused.append(self.goalSolveOrder[self.goalsCosts.index(state.costFocused)])
        for goal in goalsFocused:
            if goal == action[0]:
                try:
                    newState.costFocused = self.goalsCosts[self.goalSolveOrder.index(goal)+1]
                except IndexError:
                    None
                    #end of resolution
        return newState

    def h(self, node):
        cost = 0
        for i in self.goalsCosts:
            cost += i * self.heuristicCoeff
        for box in node.state.boxes:
            for i in range(len(self.goalsCosts)):
                if box == self.goalSolveOrder[i]:
                    cost -= self.goalsCosts[i] * self.heuristicCoeff
        return cost

    def path_cost(self, c, state1, action, state2):
        if manhattanHeuristicFunction(action[0], action[2])>1:
            return 0
        else:
            return c + 20

class PathFinderCube(Problem):

    def __init__(self, initial, goal):
        self.walls = initial.get("walls")
        self.deadLocks = initial.get("deadlocks")
        self.initial = State(initial.get("playerPos"), initial.get("boxes"), goal.costFocused)
        self.goal = goal
        self.indexBoxFocused = self.findIndexBoxFocused()

    def findIndexBoxFocused(self):
        for i in self.initial.boxes:
            if i not in self.goal.boxes:
                return self.initial.boxes.index(i)

    def goal_test(self, state):
        if state.boxes == self.goal.boxes and state.actualPosition == self.goal.actualPosition:
            return True
        else:
            return False
    
    def actions(self, state):
        actions = []
        if self.goal.boxes == state.boxes:
            if pathExists(self.walls, state.actualPosition, self.goal.actualPosition, state.boxes):
                return [[state.boxes[self.indexBoxFocused], self.goal.actualPosition, state.boxes[self.indexBoxFocused]]]

        for around in getAroundPositions(state.boxes[self.indexBoxFocused]):
            i = state.boxes[self.indexBoxFocused][0] + state.boxes[self.indexBoxFocused][0] - around[0]
            j = state.boxes[self.indexBoxFocused][1] + state.boxes[self.indexBoxFocused][1] - around[1]
            if self.walls[around[0]][around[1]] == ' ' and around not in state.boxes and self.walls[i][j] == ' '  and [i,j] not in state.boxes \
            and around not in self.deadLocks and pathExists(self.walls, state.actualPosition, [i, j], state.boxes):
                actions.append([around, state.boxes[self.indexBoxFocused], state.boxes[self.indexBoxFocused]])
        return actions

    def result(self, state, action):
        newState = state.copy()
        newState.boxes[state.boxes.index(action[2])] = action[0]
        newState.actualPosition = action[1]
        return newState

    def h(self, node):
        return manhattanHeuristicFunction(node.state.boxes[self.indexBoxFocused], self.goal.boxes[self.indexBoxFocused])
    
class PathFinderPlayer(Problem):

    def __init__(self, initial, goal):
        self.walls = initial.get("walls")
        self.initial = State(initial.get("playerPos"), initial.get("boxes"), goal.costFocused)
        self.goal = self.computeGoal(goal)

    def computeGoal(self, goal):
        index = -1
        for i in self.initial.boxes:
            if i not in goal.boxes:
                index = self.initial.boxes.index(i)
        if index == -1:
            return goal.actualPosition
        return [self.initial.boxes[index][0] - goal.boxes[index][0] + self.initial.boxes[index][0], self.initial.boxes[index][1] - goal.boxes[index][1] + self.initial.boxes[index][1]]

    def goal_test(self, state):
        if state.actualPosition == self.goal:
            return True
        else:
            return False

    def actions(self, state):
        actions = []
        for around in getAroundPositions(state.actualPosition):
            if self.walls[around[0]][around[1]] == ' ' and around not in state.boxes:
                actions.append(around)
        return actions

    def result(self, state, action):
        newState = state.copy()
        newState.actualPosition = action
        return newState

    def h(self, node):
        return manhattanHeuristicFunction(node.state.actualPosition, self.goal)

######################
# Auxiliary function #
######################

def getEssentialsPositions(array):
    player, boxes, walls = [0,0], [], [[' ' for _ in range(len(array[0]))] for _ in range(len(array))]
    for i in range(len(array)):
        for j in range(len(array[0])):
            if array[i][j] == '@':
                player = [i, j]
            if array[i][j] == '$':
                boxes.append([i,j])
            if array[i][j] == "#":
                walls[i][j] = '#'
    return player, boxes, walls

def getGoals(grid):
    goals = []
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == '.':
                goals.append([i,j])
    return goals

def openFile(path):
	monfichier= open(path)
	text=monfichier.read()
	monfichier.close()
	text=text.split('\n')
	del text[-1]
	array=list()
	for n in text :
		array.append(list(n))
	output=[]
	for d in array:
		for n in d:
			if n not in output:
				output.append(n)

	return array

def printBeautifulPath(array):
	for i in array:
		for j in i:
			print(j, end=' ')
		print()
	print()

directions = [ [0, -1], [0, 1], [-1, 0], [1, 0] ]

def pathCubeExists(actualGrid, start, end, boxes, deadLocks, playerPos):
    grid = copy.deepcopy(actualGrid)
    for i in boxes:
        if i != start:
            grid[i[0]][i[1]] = '#'
    visited = [ [0 for j in range(0, len(grid[0]))] for i in range(0, len(grid)) ]
    visited[start[0]][start[1]] = 1
    ok = pathCubeExistsDFS(grid, start, end, visited, playerPos, deadLocks, 0)
    return ok

def pathCubeExistsDFS(grid, start, end, visited, pos, deadLocks, nbOfMove):
    moves = nbOfMove + 1
    for d in directions:
        i = start[0] + d[0]
        j = start[1] + d[1]
        k = start[0] - d[0]
        l = start[1] - d[1]
        next = [i, j]
        if i == end[0] and j == end[1] and grid[k][l] == ' ' and pathExists(grid, pos, [k, l], [start]):
            return [True, start, moves]
        if grid[i][j] == ' ' and visited[i][j] != 1 and [i, j] not in deadLocks and grid[k][l]  == ' ' and pathExists(grid, pos, [k, l], [start]):
            visited[i][j] = 1
            exists = pathCubeExistsDFS(grid, next, end, visited, start, deadLocks, moves)
            if exists[0]:
                return exists
    return [False, [0,0], moves]

def pathExists(actualGrid, start, end, boxes):
    grid = copy.deepcopy(actualGrid)
    if boxes != None:
        for i in boxes:
            grid[i[0]][i[1]] = '#'
    visited = [ [0 for j in range(0, len(grid[0]))] for i in range(0, len(grid)) ]
    ok = pathExistsDFS(grid, start, end, visited)
    return ok

def pathExistsDFS(grid, start, end, visited):
    for d in directions:
        i = start[0] + d[0]
        j = start[1] + d[1]
        next = [i, j]
        if i == end[0] and j == end[1]:
            return True
        if grid[i][j] == ' ' and visited[i][j] != 1:
            visited[i][j] = 1
            exists = pathExistsDFS(grid, next, end, visited)
            if exists:
                return True
    return False

def coarseSolveOrder(walls, goals):
    costs = []
    for goal in goals:
        around = getAroundPositions(goal)
        cost = 0
        nbOfWalls = 0
        for i in around:
            if walls[i[0]][i[1]] == '#':
                cost += 1
                nbOfWalls += 1
            if i in goals:
                cost += 1
        if nbOfWalls != 0:
            for i in range(2):
                if walls[around[i][0]][around[i][1]] == '#':
                    cost += 1
                    break
            for i in range(2,4):
                if walls[around[i][0]][around[i][1]] == '#':
                    cost += 1
                    break
        costs.append([goal, cost])
    return costs

def getAroundPositions(pos):
    return [[pos[0]+1,pos[1]], [pos[0]-1,pos[1]], [pos[0],pos[1]+1], [pos[0],pos[1]-1]]

def getDeadlocks(walls, goals):
    deadLocks = []
    for i in range(len(walls)):
        for j in range(len(walls[0])):
            if walls[i][j] != '#':
                around = getAroundPositions([i,j])
                nbOfWalls = 0
                if walls[around[0][0]][around[0][1]] == '#':
                    nbOfWalls += 1
                elif walls[around[1][0]][around[1][1]] == '#':
                    nbOfWalls += 1
                if walls[around[2][0]][around[2][1]] == '#':
                    nbOfWalls += 1
                elif walls[around[3][0]][around[3][1]] == '#':
                    nbOfWalls += 1
                if nbOfWalls == 2 and [i,j] not in goals:
                    deadLocks.append([i,j])
    # if between 2 deadlocks corners there are either no goals and a continious wall, all the spaces are a deadlock position
    trueDeadLocks = copy.deepcopy(deadLocks)
    for i in deadLocks:
        directions = []
        tmpWalls = []
        aroundDeadPos = getAroundPositions(i)
        for j in range(len(aroundDeadPos)):
            if walls[aroundDeadPos[j][0]][aroundDeadPos[j][1]] != '#':
                directions.append(j)
            else:
                tmpWalls.append(j)
        for j in directions:
            tmpPath = []
            displacement = []
            if j == 0: displacement = [1, 0]
            elif j == 1: displacement = [-1, 0]
            elif j == 2: displacement = [0, +1]
            elif j == 3: displacement = [0, -1]
            pos = [i[0]+displacement[0], i[1]+displacement[1]]
            tmpPath.append(pos)
            followWall = True
            tmpWallsBis = copy.deepcopy(tmpWalls)
            while pos not in deadLocks and walls[pos[0]][pos[1]] != '#' and followWall and pos not in goals:
                aroundPos = getAroundPositions(pos)
                wallAroundPos = []
                for k in range(len(aroundPos)):
                    if walls[aroundPos[k][0]][aroundPos[k][1]] == '#':
                        wallAroundPos.append(k)
                anotherTmpWallsForThisNextLoop = copy.deepcopy(tmpWallsBis)
                for k in anotherTmpWallsForThisNextLoop:
                    if k not in wallAroundPos:
                        tmpWallsBis.remove(k)
                if len(tmpWallsBis) == 0:
                    followWall = False
                pos = [pos[0]+displacement[0], pos[1]+displacement[1]]
                tmpPath.append(pos)
            if pos in deadLocks and followWall:
                for path in tmpPath:
                    if path not in trueDeadLocks:
                        trueDeadLocks.append(path)
    return trueDeadLocks

def showDeadLocks(walls, deadLocks):
    copyWalls = copy.deepcopy(walls) 
    for i in range(len(copyWalls)):
        for j in range(len(copyWalls[0])):
            if [i,j] in deadLocks:
                copyWalls[i][j] = 'x'
    printBeautifulPath(copyWalls)

def fineSolveOrder(coarseOrder, walls, deadLocks, goals):
    fineOrder = []
    costsNonSorted = []
    for i in coarseOrder:
        costsNonSorted.append(i[1])
    maxCost = max(costsNonSorted)
    costs = []
    for i in range(maxCost +1):
        for j in costsNonSorted:
            if i == j:
                costs.append(j)
    costs = list(reversed(costs))
    sameCosts = []
    for i in costs:
        if costs.count(i) != 1 and i not in sameCosts:
            sameCosts.append(i)
    finalCost = []
    for costRising in costs:
        if costRising in sameCosts:
            for i in sameCosts:
                entries = []
                sameCostsGoals = []
                for j in coarseOrder:
                    if j[1] == i:
                        sameCostsGoals.append(j[0])
                for j in sameCostsGoals:
                    entries.append(getEntry(j, goals, walls, deadLocks))
                sameEntry = []
                for j in entries:
                    if entries.count(j) != 1 and j not in sameEntry:
                        sameEntry.append(j)
                for j in range(len(sameCostsGoals)):
                    if entries[j] not in sameEntry and sameCostsGoals[j] not in fineOrder:
                        fineOrder.append(sameCostsGoals[j])
                        finalCost.append(i)
                    elif sameCostsGoals[j] in fineOrder: None
                    else:
                        sameCostsSameEntry = []
                        for k in range(len(sameCostsGoals)):
                            if entries[j] == entries[k]:
                                sameCostsSameEntry.append(sameCostsGoals[k])
                        distanceToGoal = []
                        for k in sameCostsSameEntry:
                            distanceToGoal.append([manhattanHeuristicFunction(entries[j], k), k])
                        maxDist = 0
                        for k in distanceToGoal:
                            if maxDist < k[0]:
                                maxDist = k[0]
                        for k in range(maxDist+1):
                            for x in distanceToGoal:
                                if x[0] == maxDist-k:
                                    fineOrder.append(x[1])
                                    finalCost.append(i+(maxDist-k)/(maxDist+1))
        else: 
            for i in coarseOrder:
                if i[1] == costRising:
                    fineOrder.append(i[0])
                    finalCost.append(i[1])
    return fineOrder, finalCost

def getEntry(goal, goals, walls, deadLocks):
    problemParams = {
        "goal": goal,
        "walls": walls,
        "deadLocks": deadLocks,
        "goals": goals
    }
    findEntry = FindEntry(problemParams)
    resolution = search.depth_first_graph_search(findEntry)
    try:
        return resolution.solution()[-1]
    except IndexError:
        #entry is around itself...
        for i in getAroundPositions(goal):
                if walls[i[0]][i[1]] != '#' and i not in deadLocks and i not in goals:
                    return i

    return None #should never gone here

def manhattanHeuristicFunction(firstPoint, secondPoint):
	return abs(secondPoint[1] - firstPoint[1]) + abs(secondPoint[0] - firstPoint[0])

def cubePathResolution(state1, state2, params):
    params["playerPos"] = state1.actualPosition
    params["boxes"] = state1.boxes

    cubePathProblem = PathFinderCube(params, state2)
    cubePath = search.astar_search(cubePathProblem)

    return cubePath.path()

def playerPathFinder(state1, state2, params):
    params["playerPos"] = state1.actualPosition
    params["boxes"] = state1.boxes

    playerPathProblem = PathFinderPlayer(params, state2)
    playerPath = search.astar_search(playerPathProblem)

    return playerPath.path()

#####################
# Launch the search #
#####################

grid = openFile(sys.argv[1])
goalsGrid = openFile(sys.argv[2])

player, boxes, walls = getEssentialsPositions(grid)
goals = getGoals(goalsGrid)
coarseOrder = coarseSolveOrder(walls, goals)
deadLocks = getDeadlocks(walls, goals)
fineSolver, finalCosts = fineSolveOrder(coarseOrder, walls, deadLocks, goals)

#showDeadLocks(walls, deadLocks)

#print("fine solve order", fineSolver, finalCosts)

State(player, boxes).__str__(walls)

initialParams = {
    "playerPos": player,
    "boxes": boxes,
    "goalSolveOrder": fineSolver,
    "goalsCosts": finalCosts,
    "walls": walls,
    "deadlocks": deadLocks
}

now = datetime.now()

theProblem = ResolverCube(initialParams)
eventualResolution = search.astar_search(theProblem)

later = datetime.now()

########################
# Display the solution #
########################

if eventualResolution != None: 
    firstPathCube = eventualResolution.path()
    finalState = firstPathCube[-1].state

    for i in range(len(firstPathCube)-1):
        secondPathCube = cubePathResolution(firstPathCube[i].state, firstPathCube[i+1].state, initialParams)

        for j in range(len(secondPathCube)-1):
            thirdPathPlayer = playerPathFinder(secondPathCube[j].state, secondPathCube[j+1].state, initialParams)

            for k in range(len(thirdPathPlayer)):
                time.sleep(0.25)
                thirdPathPlayer[k].state.__str__(walls)

                if j == len(secondPathCube)-2 and k == len(thirdPathPlayer)-1 and thirdPathPlayer[-1].state != finalState and secondPathCube[j+1].state == finalState:
                    time.sleep(0.25)
                    secondPathCube[j+1].state.__str__(walls)
     
else:
    print("resolution impossible")
later2 = datetime.now()

print("Resolution took", (later - now).total_seconds(), "secondes")
print("Displaying took", (later2 - later).total_seconds(), "secondes")