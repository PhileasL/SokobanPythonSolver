import time
import sys
import search
from search import Problem
from datetime import datetime

#################
# Problem class #
#################

class State:
    def __init__(self, actualPosition, boxes):
        self.actualPosition=actualPosition
        self.boxes=boxes

    def __eq__(self, other):
        return (self.actualPosition == other.actualPosition and self.boxes == other.boxes)

    def __hash__(self):
        tmpArray = []
        tmpArray.append(tuple(["pos", tuple(self.actualPosition)]))
        for i in self.boxes:
            tmpArray.append(tuple(i))
        return hash(tuple(tmpArray))

    def __str__(self):
        return "nobody uses you"

    def __lt__(self, other):
        return None

    def copy(self):
        newBoxes = []
        for i in self.boxes:
            newBoxes.append(i.copy())
        newActualPosition = self.actualPosition.copy()
        return State(newActualPosition, newBoxes)

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

def removeSides(grid): #maybe not necessary
    newGrid = grid.copy()
    newGrid.remove(grid[0])
    newGrid.remove(grid[len(grid)-1])
    for i in range(len(grid)-2):
        newGrid[i].pop(0)
        newGrid[i].pop(len(grid)+1)
    return newGrid

directions = [ [0, -1], [0, 1], [-1, 0], [1, 0] ]

def pathExists(actualGrid, start, end, boxes):
    grid = actualGrid.copy()
    for i in boxes:
        grid[i[0]][i[1]] = '#'
    printBeautifulPath(grid)
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
        if grid[i][j] == ' ' and not visited[i][j]:
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
    trueDeadLocks = deadLocks.copy()
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
            tmpWallsBis = tmpWalls.copy()
            while pos not in deadLocks and walls[pos[0]][pos[1]] != '#' and followWall and pos not in goals:
                aroundPos = getAroundPositions(pos)
                wallAroundPos = []
                for k in range(len(aroundPos)):
                    if walls[aroundPos[k][0]][aroundPos[k][1]] == '#':
                        wallAroundPos.append(k)
                anotherTmpWallsForThisNextLoop = tmpWallsBis.copy()
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

    copyWalls = walls.copy()
    for i in range(len(copyWalls)):
        for j in range(len(copyWalls[0])):
            if [i,j] in trueDeadLocks:
                copyWalls[i][j] = 'x'
    printBeautifulPath(copyWalls)
    return trueDeadLocks

def fineSolveOrder(coarseOrder, walls, deadLocks):
    costs = []
    for i in coarseOrder:
        costs.append(i[1])
    print(costs)

#####################
# Launch the search #
#####################

grid = openFile(sys.argv[1])
goalsGrid = openFile(sys.argv[2])

printBeautifulPath(grid)
printBeautifulPath(goalsGrid)

player, boxes, walls = getEssentialsPositions(grid)
state = State(player, boxes)
goals = getGoals(goalsGrid)
coarseOrder = coarseSolveOrder(walls, goals)
deadLocks = getDeadlocks(walls, goals)

print("coarse solve order", coarseOrder)

print("deadlocks position:", deadLocks)

fineSolveOrder(coarseOrder, walls, deadLocks)