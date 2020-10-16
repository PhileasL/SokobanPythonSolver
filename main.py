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
            printBeautifulPath(visited)
            return True
        if grid[i][j] == ' ' and not visited[i][j]:
            visited[i][j] = 1
            exists = pathExistsDFS(grid, next, end, visited)
            if exists:
                printBeautifulPath(visited)
                return True
    printBeautifulPath(visited)
    return False

#####################
# Launch the search #
#####################

grid = openFile(sys.argv[1])

printBeautifulPath(grid)

player, boxes, walls = getEssentialsPositions(grid)
printBeautifulPath(walls)
state = State(player, boxes)
#sidesRemoved = removeSides(grid)
print(boxes)
print(pathExists(walls, [4,2], [4,9], boxes))
