import csv
import math
import numpy as np
# import matplotlib.pyplot as plt
import Test_MAK1
import robot

# récup le csv et l'afficher 
#acquisition()

def makeArray(x):
    # reader = csv.reader(open("MAK.csv", "r"),delimiter=";")
    # x = list(reader)
    array = np.array(x).astype("int")
    # #split la matrice en deux liste X et Y 
    return(array)

# convertir les données et récup X_final et Y_final 
def pol2cart(dist,Angle):
    x_final =dist * np.cos(math.radians(Angle))
    y_final =dist * np.sin(math.radians(Angle))
    return (x_final,y_final)

# def polarToCart(coord):
#     xArr = []
#     yArr = []
#     for row in coord:
#         x = int(row[1]) * np.cos(math.radians(int(row[0])))
#         y = int(row[1]) * np.sin(math.radians(int(row[0])))+1000

#         if 0 < y < 2000 and 0 < x < 2000:
#             # print(x,y)
#             xArr.append(-x)
#             yArr.append(y)
#     return xArr, yArr

def filtre(array):
    X = []
    Y = []
    for tab_i in array :
        (x,y) = pol2cart(tab_i[1],tab_i[0])
        if(0<-x<2000 and -1000<y<1000):
            X.append(-x)
            Y.append(y+1000)
    return(X,Y)
## transformer les données en une matrice de maze 

new_array = np.matrix([[0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]])

# def remplirMatrix(matrix, xArr, yArr):
#     for index, x in enumerate(xArr):
#         y = yArr[index]
#         if(y+1<matrix.shape[0] and x+1<matrix.shape[1]):
#             matrix[y][x]=1.
#             matrix[y][x+1]=1. 
#             matrix[y][x-1]=1.  
#             matrix[y+1][x]=1.
#             matrix[y-1][x]=1.
#     # for i in range(matrix.shape[0]):
#     #     matrix[i][0] = 0. 
#     #     matrix[i][9] = 0.

#     return matrix
            
def remplire(X,Y,new_array):
    for i,x in enumerate(X):
        y=Y[i]
        y_coord = int(y/200)
        x_coord= int(x/200)
        if(y_coord+1<new_array.shape[0]-1 and x_coord+1<new_array.shape[1]-1):
            new_array[y_coord][x_coord]=1.
            #new_array[y_coord][x_coord+1]=1. 
            #new_array[y_coord][x_coord-1]=1.  
            #new_array[y_coord+1][x_coord]=1.
            #new_array[y_coord-1][x_coord]=1.

    for i in range(new_array.shape[0]):
        new_array[i][0] = 0. 
        new_array[i][9] = 0.
    return(new_array)    

class Node:
    def __init__(self, coord, dist, origin):
        self.coord = coord
        self.dist = dist
        self.origin = origin

def astar(maze, start, end):
    st_node = Node(start, 0, None)
    end_node = Node(end, math.inf, None)
    closed_list = []
    opened_list = [st_node]

    while len(opened_list) > 0:
        current, opened_list = getNearest(opened_list, end)
        closed_list.append(current)
        if current.coord == end_node.coord:
            return reconstituerChemin(current)
        
        children = []
        for pos in [(0,1),(1,0),(-1,0),(0,-1)]:
            next_pos = (current.coord[0]+pos[0], current.coord[1]+pos[1])
            if not validNeighbor(maze, next_pos):
                continue 

            child = Node(next_pos, current.dist+1, current)
            children.append(child)
        
        for child in children:
            if check(child, closed_list, opened_list):
                continue
            opened_list.append(child)

def check(child, close, openl):
    return any(node.coord == child.coord for node in close) or any(node.coord == child.coord and node.dist > child.dist for node in openl)


def getNearest(open_list, end):
    oldDist = math.inf
    nearNode = None
    indexNode = 0
    for index, node in enumerate(open_list):
        dist = math.pow(node.coord[0] - end[0],2) + math.pow(node.coord[1] - end[1],2)
        if dist < oldDist:
            oldDist = dist
            nearNode = node
            indexNode = index
            
    open_list.pop(indexNode)
    return nearNode, open_list

def reconstituerChemin(last_pos):
    listCord = [last_pos.coord]
    prevNode = last_pos.origin

    while prevNode is not None:
        listCord.append(prevNode.coord)
        prevNode = prevNode.origin

    return listCord[::-1]

def validNeighbor(mat, next_pos):
    if next_pos[0] < 0 or next_pos[1] < 0 or next_pos[0] >= mat.shape[0] or next_pos[1] >= mat.shape[1]:
        return False 
    if mat[next_pos[0]][next_pos[1]] == 0.:
        return True
    else:
        return False


def verify(maze, start, end, path):
    if path[0][0]!=start[0] or path[0][1]!=start[1] or path[-1][0]!=end[0] or path[-1][1]!=end[1]:
        print('Start or end incorrect')
        return False
    last = None
    for pos in path:
        if maze[pos] == 1:
            print('Path cross a wall')
            return False
        if last==None:
            last = pos
            continue

    diffX = abs(pos[0] - last[0])
    diffY = abs(pos[1] - last[1])
    if diffX > 1 or diffY > 1 or diffX + diffY > 1:
        print('Path not consecutive')
        return False
    last = pos
    print('Correct path')
    return True

# def displayMatrix(mat):
#     plt.matshow(mat)
#     plt.show()

# def displayPath(mat, path):
#     if path != None:
#         for (y, x) in path:
#             mat[y, x] = math.inf
#     plt.matshow(mat)
#     plt.show()



start = (4,0)
end = (4,9)

while(True):
    result = Test_MAK1.acquisition()
    array = makeArray(result)
    X,Y= filtre(array)
    # X = [int((x)/200) for x in X]
    # Y = [int((y)/200) for y in Y]
    final_array = np.zeros((10,10))
    final_array= remplire(X ,Y, final_array)
    coord = int(final_array.shape[0]/2)
    start = (coord,0)
    end = (coord,9)
    #maze = np.asarray(final_array)
    print(final_array)
    path = astar(final_array, start, end)
    # displayMatrix(maze)
    # displayPath(maze,path)
    robot.instruction(path[0], path[1])
    print(path)
        





