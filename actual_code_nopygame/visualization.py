from queue import PriorityQueue
import numpy as np
import networkx as nx
import numpy as np

# constants declaration
WIDTH = 800
ROWS = 20

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)
TEAL = (2, 253, 252)

"""
Spots = Nodes = Cubes
This is to keep track of our grid, the node's color and the coordinates it actually is at
"""
class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.heading = None
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col, self.heading

    # this method tells us if this node has been already visited / considered
    def is_close(self):
        return self.color == RED

    # this method tells us that this node is a frontier node
    def is_open(self):
        return self.color == GREEN

    # is this node an obstacle
    def is_obstacle(self):
        return self.color == BLACK or self.color == GREY

    def is_start(self):
        return self.color == ORANGE

    def is_path(self):
        return self.color == PURPLE

    def is_goal(self):
        return self.color == TEAL

    def is_end(self):
        return self.color == TURQUOISE

    def is_closed(self):
        return self.color == RED

    def make_close(self):
        self.color = RED

    # this method tells us that this node is a frontier node
    def make_open(self):
        self.color = GREEN

    # is this node an obstacle
    def make_obstacle(self):
        self.color = BLACK

    def make_obstacle_light(self):
        self.color = GREY

    def make_start(self, start_h):
        self.heading = start_h
        self.color = ORANGE

    def make_path(self):
        self.color = PURPLE

    def make_goal(self, goal_h):
        self.heading = goal_h
        self.color = TEAL

    def start_area(self):
        self.color = YELLOW

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_obstacle(): # NORTH
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_obstacle(): # SOUTH
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_obstacle(): # EAST
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_obstacle(): # WEST
            self.neighbors.append(grid[self.row][self.col - 1])

    # compares 2 different spots (grids)
    def __lt__(self, other):
        return False

def calc_TSP(node_c):
    g1 = nx.complete_graph(len(node_c))

    node_c_no_h = []
    for x in node_c:
        node_c_no_h.append(x[:2])

    for node, node2 in g1.edges:
        xy = np.subtract(node_c_no_h[node], node_c_no_h[node2])
        xy = np.abs(xy)
        xy = np.sum(xy)
        g1[node][node2]['weight'] = xy
    tsp_path = nx.approximation.traveling_salesman_problem(g1, weight = 'weight', cycle = True, method = nx.algorithms.approximation.traveling_salesman.greedy_tsp)
    return tsp_path

# our A* heuristic, h(x), in euclidean distance
def h(p1, p2):
    x1, y1, h1 = p1
    x2, y2, h2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, current):
    path = []
    path_f = []
    path.append(current.get_pos())
    while current in came_from:
        current = came_from[current]
        current_pos = current.get_pos()
        path.append(current_pos)
        if not current.is_goal():
            current.make_path()
    
    path_r = list(reversed(path))

    for c in range(len(path_r)):
        cx, cy, cheading = path_r[c]
        if c+1 < len(path_r):
            path_r[c+1] = list(path_r[c+1])
            nx, ny, nheading = path_r[c+1]
            if nx > cx:
                cheading = 0
            elif nx < cx:
                cheading = 180
            elif ny > cy:
                cheading = 270
            elif ny < cy:
                cheading = 90
        path_f.append(str(cx)+","+str(abs(cy-20))+","+str(cheading))
    
    return path_f
    # runRobotMove(path_f, draw)

'''
def runRobotMove(path, draw):
    global car_dir
    print(car_dir)
    print(path)
    straightCounter = 0
    i = 0

    firstTheta = path[0][2] - car_dir
    if firstTheta == 180 or firstTheta == -180:
        carTurn180()
    elif firstTheta == 90 or firstTheta == -270:
        carReverse(1)
        i = i + carTurnLeft(i-1, path)
    elif firstTheta == -90 or firstTheta == 270:
        carReverse(1)
        i = i + carTurnRight(i-1, path)

    coord = path[i]
    car_dir = coord[2]
    draw()
    pygame.time.wait(30)

    while i < len(path):
        if i == len(path)-1: # last turn
            straightCounter = 0
            firstTheta = path[i][2] - car_dir
            if firstTheta == 180 or firstTheta == -180:
                carTurn180()
        
        if i+1 < len(path):
            turnTheta = path[i+1][2] - car_dir
            if turnTheta == 0:
                straightCounter = straightCounter + 1
            else:
                if straightCounter != 0:
                    carStraight(straightCounter)
                straightCounter = 0
                if turnTheta == 90 or turnTheta == -270:
                    i = i + carTurnLeft(i, path)
                elif turnTheta == -90 or turnTheta == 270:
                    i = i + carTurnRight(i, path)
        
        coord = path[i]
        car_dir = coord[2]
        i = i + 1
        print(straightCounter)

    if straightCounter != 0:
        carStraight(straightCounter)

    return

def carReverse(n):
    print("reverse for", n)
    # TODO ADD INSTRUCTION TO REVERSE n*10 cm
    return

def carStraight(n):
    print("straight for", n)
    # TODO ADD INSTRUCTION TO GO FORWARD n*10 cm
    return

def carTurnRight(index, path):
    print("turn right")
    # TODO ADD INSTRUCTION TO GO TURN RIGHT
    # TODO CHECK IF NEED GO BACK
    count = 0
    if index+1 < len(path):
        dir = path[index+1][2]
        
        for i in range(2, 6):
            checkingIndex = index + i
            if checkingIndex < len(path):
                if path[checkingIndex][2] == dir:
                    count = count + 1
                elif path[checkingIndex][2] - dir == 180 or path[checkingIndex][2] - dir == -180:
                    count = count + 1
                    break
                else:
                    break
        if 4-count != 0:
            carReverse(4-count)
    return count

def carTurnLeft(index, path):
    print("turn left")
    # TODO ADD INSTRUCTION TO GO TURN LEFT
    # TODO CHECK IF NEED GO BACK
    count = 0
    if index+1 < len(path):
        dir = path[index+1][2]

        for i in range(2, 6):
            checkingIndex = index + i
            if checkingIndex < len(path):
                if path[checkingIndex][2] == dir:
                    count = count + 1
                elif path[checkingIndex][2] - dir == 180 or path[checkingIndex][2] - dir == -180:
                    count = count + 1
                    break
                else:
                    break
        if 4-count != 0:
            carReverse(4-count)
    return count

def carTurn180():
    print("turn 180")
    # TODO ADD INSTRUCTION TO TURN 180
    return
'''

def algorithm(grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start)) # Add the start node to the priority queue
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(start.get_pos(), end.get_pos())

    #check items in PQ
    open_set_hash = {start}

    while not open_set.empty():
        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            return reconstruct_path(came_from, end)

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    if not neighbor.is_path() and not neighbor.is_goal() and not neighbor.is_start() and not neighbor.is_close():
                        neighbor.make_open()

        if current != start and not current.is_path() and not current.is_goal() and not current.is_start():
            current.make_close()

    return False

def make_grid(rows, width):
    grid = []
    # integer division
    gap = width // rows

    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)

    return grid

def visualize(width, and_inputs):
    grid = make_grid(ROWS, width)

    # start / end pos
    start = None
    end = None
    # have we started the algorithm
    goal_nodes = []
    shp = []

    input_for_algo = []

    for androidInput in and_inputs:
        coords = androidInput.split(",")
        coords.pop(0)
        coords[0] = int(coords[0])
        coords[1] = abs(int(coords[1]) - 20)
        if coords[2] == 'N':
            coords[2] = 90
        elif coords[2] == 'E':
            coords[2] = 0
        elif coords[2] == 'S':
            coords[2] = 270
        elif coords[2] == 'W':
            coords[2] = 180
        input_for_algo.append(coords)

    for i in range(4):
        for j in range(16, 20):
            spot = grid[i][j]
            spot.start_area()

    for i in range(len(input_for_algo)):
        if i == 0:
            col, row = input_for_algo[0][1], input_for_algo[0][0]
            spot = grid[row][col]
            if input_for_algo[0][2] == 90:
                if not start and row < 3 and col > 15:
                    start = spot
                    start.make_start(90)
                    goal_nodes.append((row, col, 90))
            elif input_for_algo[0][2] == 0:
                if not start and row < 3 and col > 15:
                    start = spot
                    start.make_start(0)
                    goal_nodes.append((row, col, 0))
        else:
            col, row = input_for_algo[i][1], input_for_algo[i][0]
            if input_for_algo[i][2] == 0:
                for x in range(-1, 2, 1):
                    if col+x >= 0 and col+x < ROWS:
                        for y in range(-1, 2, 1):
                            if row+y >= 0 and row+y < ROWS:
                                spot = grid[row+y][col+x]
                                spot.make_obstacle_light()
                spot = grid[row][col]
                spot.make_obstacle()
                spot = grid[row+4][col]
                if spot != start and spot.is_obstacle() == False:
                    if goal_nodes[len(goal_nodes)-1] != (row+4, col):
                        spot.make_goal(180)
                        goal_nodes.append((row+4, col, 180))
            elif input_for_algo[i][2] == 90:
                for x in range(-1, 2, 1):
                    if col+x >= 0 and col+x < ROWS:
                        for y in range(-1, 2, 1):
                            if row+y >= 0 and row+y < ROWS:
                                spot = grid[row+y][col+x]
                                spot.make_obstacle_light()
                spot = grid[row][col]
                spot.make_obstacle()
                spot = grid[row][col-4]
                if spot != start and spot.is_obstacle() == False:
                    if goal_nodes[len(goal_nodes)-1] != (row, col-4):
                        spot.make_goal(270)
                        goal_nodes.append((row, col-4, 270))
            elif input_for_algo[i][2] == 180:
                for x in range(-1, 2, 1):
                    if col+x >= 0 and col+x < ROWS:
                        for y in range(-1, 2, 1):
                            if row+y >= 0 and row+y < ROWS:
                                spot = grid[row+y][col+x]
                                spot.make_obstacle_light()
                spot = grid[row][col]
                spot.make_obstacle()
                spot = grid[row-4][col]
                if spot != start and spot.is_obstacle() == False:
                    if goal_nodes[len(goal_nodes)-1] != (row-4, col):
                        spot.make_goal(0)
                        goal_nodes.append((row-4, col, 0))
            elif input_for_algo[i][2] == 270:
                for x in range(-1, 2, 1):
                    if col+x >= 0 and col+x < ROWS:
                        for y in range(-1, 2, 1):
                            if row+y >= 0 and row+y < ROWS:
                                spot = grid[row+y][col+x]
                                spot.make_obstacle_light()
                spot = grid[row][col]
                spot.make_obstacle()
                spot = grid[row][col+4]
                if spot != start and spot.is_obstacle() == False:
                    if goal_nodes[len(goal_nodes)-1] != (row, col+4):
                        spot.make_goal(90)
                        goal_nodes.append((row, col+4, 90))


    if not start:
        for i in range(4):
            for j in range(16, 20):
                spot = grid[i][j]
                spot.start_area()

    full_path = []
    shp = calc_TSP(goal_nodes)
    shp2 = shp.copy()
    shp2.pop()
    shp3 = [str(x) for x in shp2]

    for row in grid:
        for spot in row:
            spot.update_neighbors(grid)
    
    while len(shp2) != 1:
        n = shp2[0]
        row, col, heading = goal_nodes[n]
        start = grid[row][col]
        shp2.pop(0)
        end_node = shp2[0]
        row, col, heading = goal_nodes[end_node]
        end = grid[row][col]
        full_path.append(algorithm(grid, start, end))
    return shp3, full_path

seq, path = visualize(WIDTH, ["0,2,2,E", "2,14,13,N", "2,7,12,W", "3,11,7,S"])
print(seq)
print(path)