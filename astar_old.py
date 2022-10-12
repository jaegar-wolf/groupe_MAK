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

            
def remplire(X,Y,new_array):
    for i,x in enumerate(X):
        y=Y[i]
        y_coord = int(y/200)
        x_coord= int(x/200)
        if(y_coord+1<new_array.shape[0] and x_coord+1<new_array.shape[1]):
            new_array[y_coord,x_coord]=1.
            new_array[y_coord,x_coord+1]=1. 
            new_array[y_coord,x_coord-1]=1.  
            new_array[y_coord+1,x_coord]=1.
            new_array[y_coord-1,x_coord]=1. 
    return(new_array)    

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

  # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue


            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

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
    X,Y=filtre(array)
    final_array = np.zeros((10,10))
    final_array= remplire(X,Y,final_array)
    maze = np.asarray(final_array)
    if np.array_equal(maze,np.zeros((10,10))):
        continue
    print(maze)
    path = astar(maze, start, end)
    test = path[:2]
    # displayMatrix(maze)
    # displayPath(maze,path)
    robot.instruction(test)
    print(path)
        





