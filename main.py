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
    player, boxes, walls = [0,0], [], []
    for i in range(len(array)):
        for j in range(len(array[0])):
            if array[i][j] == '@':
                player = [i, j]
            if array[i][j] == '$':
                boxes.append([i,j])
            if array[i][j] == "#":
                walls.append([i, j])
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

#####################
# Launch the search #
#####################

grid = openFile(sys.argv[1])

printBeautifulPath(grid)

player, boxes, walls = getEssentialsPositions(grid)

state = State(player, boxes)
print(state.__hash__())
