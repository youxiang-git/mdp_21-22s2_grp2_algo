from queue import PriorityQueue
import numpy as np
import networkx as nx
import numpy as np
import math

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

car_dir = 0

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

    def get_neighbors(self):
        return self.neighbors

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

    def is_possible_goal(self):
        return self.color == BLUE

    def make_possible_goal(self):
        self.color = BLUE

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

    def change_heading(self, heading):
        self.heading = heading

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

class Image:
    def __init__(self, row, col, image_dir):
        self.row = row
        self.col = col
        self.image_dir = image_dir
        self.goals = []

    def get_goal(self):
        if len(self.goals) > 0:
            return self.goals[0]
        else:
            return None

    def get_all_goal(self):
        return self.goals

    def get_coord(self):
        return self.row, self.col

    def get_dir(self):
        return self.image_dir

    def add_goal(self, row, col, dir):
        self.goals.append((row, col, dir))

    def remove_goal(self, row, col):
        for goal in self.goals:
            if goal[0] == row and goal[1] == col:
                self.goals.remove(goal)
        return
    
    def remove_all_except(self, coord):
        self.goals = [coord]

class RobotStart:
    def __init__(self, row, col, direction):
        self.row = row
        self.col = col
        self.direction = direction

    def get_goal(self):
        return (self.row, self.col, self.direction)

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

def runRobotMove(path):
    global car_dir
    path_i = []
    straightCounter = 0
    i = 0

    while i < len(path):

        turnTheta = path[i][2] - car_dir
        if turnTheta == 0:
            straightCounter = straightCounter + 1
        else:
            if straightCounter != 0:
                carStraight(path_i, straightCounter)
            straightCounter = 0
            if turnTheta == 180 or turnTheta == -180:
                carTurn180(path_i)
            elif turnTheta == 90 or turnTheta == -270:
                carPointTurnLeft(path_i)
            elif turnTheta == -90 or turnTheta == 270:
                carPointTurnRight(path_i)
            if i != len(path)-1:
                straightCounter = straightCounter + 1

        car_dir = path[i][2]
        i = i + 1
    
    if straightCounter != 0:
        carStraight(path_i, straightCounter)

    return path_i

def carStraight(path_i, n):
    path_i.append("1"+numToLetter(n))
    return

def carReverse(path_i, n):
    path_i.append("2"+numToLetter(n))
    return

def carPointTurnLeft(path_i):
    path_i.append("30")
    return

def carPointTurnRight(path_i):
    path_i.append("40")
    return

def carTurn180(path_i):
    path_i.append("30")
    path_i.append("30")
    return

def numToLetter(num):
    options = {1 : "A", 2 : "B", 3 : "C",
            4 : "D", 5 : "E", 6 : "F",
            7 : "G", 8 : "H", 9 : "I",
            10 : "J", 11 : "K", 12 : "L",
            13 : "M", 14 : "N", 15 : "O",
            16 : "P", 17 : "Q", 18 : "R",
            19 : "S", 20 : "T"}
    return options[num]

def algorithm_handler(grid, start_images, shp, graph):
    full_path = []
    full_ins = []
    goal_nodes = [start_images[0].get_goal()]

    for n in range(1, len(shp)):
        start = goal_nodes[-1]
        end_image = start_images[shp[n]]

        # find overlapped goal node
        if n+1 < len(shp):
            found = False
            next_image = start_images[shp[n+1]]
            goal_nodes1 = end_image.get_all_goal()
            goal_nodes2 = next_image.get_all_goal()
            for i in goal_nodes1:
                for j in goal_nodes2:
                    if i[:2] == j[:2]:
                        end_image.remove_all_except(i)
                        next_image.remove_all_except(j)
                        found = True
                        break
                if found == True:
                    break

        # run algo to find all possible path and return the least turns path & ins
        add_end, input_f, input_i = algorithm(grid, start, end_image, graph)
        goal_nodes.append(add_end)
        full_path.append(input_f)
        full_ins.append(input_i)

    return goal_nodes, full_path, full_ins

def algorithm(grid, start, end_image, graph):
    row, col, heading = start
    start = grid[row][col]
    start.make_goal(heading)
    source = start.get_pos()[:2]
    sheading = start.get_pos()[-1]
    eheading = None

    # find shortest path length goal nodes
    end_list = end_image.get_all_goal()
    map_target_cutoff = {}
    least_cutoff = None
    for index in range(len(end_list)):
        row, col, eheading = end_list[index]
        end = grid[row][col]
        target = end.get_pos()[:2]

        cutoff = nx.shortest_path_length(graph, source=source, target=target)
        map_target_cutoff[target] = cutoff
        if least_cutoff == None or cutoff < least_cutoff:
            least_cutoff = cutoff

    # remove long goal nodes
    for target in map_target_cutoff.keys():
        if map_target_cutoff[target] != least_cutoff:
            end_image.remove_goal(target[0], target[1])

    # nx all simple path to get all least path
    end_list = end_image.get_all_goal()
    all_shortest_path = []
    for possible_goal in end_list:
        row, col, eheading = possible_goal
        end = grid[row][col]
        target = end.get_pos()[:2]
        all_shortest_path.extend(list(nx.all_shortest_paths(graph, source=source, target=target)))
    
    # all_shortest_path = []
    # end = end_image.get_goal()
    # target = end[:2]
    # eheading = end[-1]
    # all_shortest_path.extend(list(nx.all_shortest_paths(graph, source=source, target=target)))

    # find the least turn path
    best_turn_no = None
    best_path = None
    best_path_f = None

    for path in all_shortest_path:
        path_ins = []
        path_f = []
        cur_car_dir = sheading
        turn_count = 0

        for index in range(len(path)):
            cx, cy = path[index]
            cheading = 0
            if index+1 < len(path):
                path[index+1] = list(path[index+1])
                tx, ty = path[index+1]
                if tx > cx:
                    cheading = 0
                elif tx < cx:
                    cheading = 180
                elif ty > cy:
                    cheading = 270
                elif ty < cy:
                    cheading = 90
            else:
                cheading = eheading
            path_ins.append((cx, cy, cheading))
            path_f.append(str(cx)+","+str(abs(cy-19))+","+str(cheading))
            
            turnTheta = cheading - cur_car_dir
            if turnTheta == 180 or turnTheta == -180:
                turn_count = turn_count + 2
            elif turnTheta == 90 or turnTheta == -270 or turnTheta == -90 or turnTheta == 270:
                turn_count = turn_count + 1
            cur_car_dir = cheading

            if best_turn_no != None and turn_count > best_turn_no:
                break
        
        if best_turn_no == None or turn_count <= best_turn_no:
            best_path = path_ins
            best_path_f = path_f
            best_turn_no = turn_count

    path_i = runRobotMove(best_path)

    last_coord = best_path[-1]

    return last_coord, best_path_f, path_i

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

def create_avoidance_area(row, col, grid, start_images):
    reset_nodes = []
    for x in range(-1, 2, 1):
        if col+x >= 0 and col+x < ROWS:
            for y in range(-1, 2, 1):
                if row+y >= 0 and row+y < ROWS:
                    if grid[row+y][col+x].is_possible_goal() == True:
                        reset_nodes.append((row+y, col+x))
                    spot = grid[row+y][col+x]
                    spot.make_obstacle_light()
    spot = grid[row][col]
    spot.make_obstacle()

    # remove overlapped goal
    for coord in reset_nodes:
        for i in range(1, len(start_images), 1):
            start_images[i].remove_goal(coord[0], coord[1])

    return

def create_possible_goal(newImage, grid):
    image_direction = newImage.get_dir()
    row, col = newImage.get_coord()
    if image_direction == 0:
        for increment in range(5, 2, -1):
            if row+increment < ROWS:
                spot = grid[row+increment][col]
                if spot.is_obstacle() == False:
                    spot.make_possible_goal()
                    newImage.add_goal(row+increment, col, 180)
                # if increment == 4:
                #     for jincrement in range(1, -2, -2):
                #         if col+jincrement >= 0 and col+jincrement < ROWS:
                #             spot = grid[row+increment][col+jincrement]
                #             if spot.is_obstacle() == False:
                #                 spot.make_possible_goal()
                #                 newImage.add_goal(row+increment, col+jincrement, 180)
    elif image_direction == 90:
        for increment in range(5, 2, -1):
            if col-increment >= 0:
                spot = grid[row][col-increment]
                if spot.is_obstacle() == False:
                    spot.make_possible_goal()
                    newImage.add_goal(row, col-increment, 270)
                    # if increment == 4:
                    #     for jincrement in range(1, -2, -2):
                    #         if row+jincrement >= 0 and row+jincrement < ROWS:
                    #             spot = grid[row+jincrement][col-increment]
                    #             if spot.is_obstacle() == False:
                    #                 spot.make_possible_goal()
                    #                 newImage.add_goal(row+jincrement, col-increment, 270)
    elif image_direction == 180:
        for increment in range(5, 2, -1):
            if row-increment >= 0:
                spot = grid[row-increment][col]
                if spot.is_obstacle() == False:
                    spot.make_possible_goal()
                    newImage.add_goal(row-increment, col, 0)
                # if increment == 4:
                #     for jincrement in range(1, -2, -2):
                #         if col+jincrement >= 0 and col+jincrement < ROWS:
                #             spot = grid[row-increment][col+jincrement]
                #             if spot.is_obstacle() == False:
                #                 spot.make_possible_goal()
                #                 newImage.add_goal(row-increment, col+jincrement, 0)
    elif image_direction == 270:
        for increment in range(5, 2, -1):
            if col+increment < ROWS:
                spot = grid[row][col+increment]
                if spot.is_obstacle() == False:
                    spot.make_possible_goal()
                    newImage.add_goal(row, col+increment, 90)

def visualize(and_inputs):
    grid = make_grid(ROWS, WIDTH)

    # start / end pos
    start = None
    end = None
    # have we started the algorithm
    goal_nodes = []
    start_images = []
    global car_dir

    input_for_algo = []

    for i in range(4):
        for j in range(16, 20):
            spot = grid[i][j]
            spot.start_area()

    # border avoidance zone
    for i in (0, 19):
        for j in range(20):
            spot = grid[i][j]
            spot.make_obstacle_light()
            spot = grid[j][i]
            spot.make_obstacle_light()

    # translating input from android to algo-friendly inputs
    for androidInput in and_inputs:
        coords = androidInput.split(",")
        coords.pop(0)
        coords[0] = int(coords[0])
        coords[1] = abs(int(coords[1]) - 19)
        if coords[2] == 'N':
            coords[2] = 90
        elif coords[2] == 'E':
            coords[2] = 0
        elif coords[2] == 'S':
            coords[2] = 270
        elif coords[2] == 'W':
            coords[2] = 180
        input_for_algo.append(coords)

    for i in range(len(input_for_algo)):
        if i == 0:
            col, row = input_for_algo[0][1], input_for_algo[0][0]
            spot = grid[row][col]
            if input_for_algo[0][2] == 90:
                if not start and row < 3 and col > 15:
                    car_dir = 90
                    start = spot
                    start.make_start(90)
                    start_images.append(RobotStart(row, col, 90))
            elif input_for_algo[0][2] == 0:
                if not start and row < 3 and col > 15:
                    car_dir = 0
                    start = spot
                    start.make_start(0)
                    start_images.append(RobotStart(row, col, 0))
        else:
            col, row = input_for_algo[i][1], input_for_algo[i][0]
            # image facing right
            if input_for_algo[i][2] == 0:
                create_avoidance_area(row, col, grid, start_images)
                newImage = Image(row, col, 0)
                start_images.append(newImage)
                create_possible_goal(newImage, grid)
            # image facing top
            elif input_for_algo[i][2] == 90:
                create_avoidance_area(row, col, grid, start_images)
                newImage = Image(row, col, 90)
                start_images.append(newImage)
                create_possible_goal(newImage, grid)
            # image facing left
            elif input_for_algo[i][2] == 180:
                create_avoidance_area(row, col, grid, start_images)
                newImage = Image(row, col, 180)
                start_images.append(newImage)
                create_possible_goal(newImage, grid)
            # image facing bottom
            elif input_for_algo[i][2] == 270:
                create_avoidance_area(row, col, grid, start_images)
                newImage = Image(row, col, 270)
                start_images.append(newImage)
                create_possible_goal(newImage, grid)

    goal_nodes_tsp = []
    images_to_remove = []
    shp = []
    graph = nx.Graph()

    # get goal_nodes
    for item in start_images:
        temp = item.get_goal()
        if temp != None:
            goal_nodes_tsp.append(temp)
        else:
            images_to_remove.append(item)

    # remove images from start_images that have no goal nodes
    for removing_item in images_to_remove:
        start_images.remove(removing_item)

    # update neighbors and create graph
    for row in grid:
        for spot in row:
            if spot.is_obstacle() == False:
                spot.update_neighbors(grid)
                current_xy = spot.get_pos()[:2]
                graph.add_node(current_xy)
                for neighbors in spot.get_neighbors():
                    neighbors_xy = neighbors.get_pos()[:2]
                    graph.add_edge(current_xy, neighbors_xy)

    shp = calc_TSP(goal_nodes_tsp)
    shp2 = shp.copy()
    shp2.pop()
    # index sequence for rpi
    shp3 = [str(x) for x in shp2]
    shp3.pop(0)

    goal_nodes, full_path, full_ins = algorithm_handler(grid, start_images, shp2, graph)

    return shp3, full_path, full_ins

# seq, path, ins = visualize( ['0,1,1,N', '1,10,7,W', '2,13,2,E', '3,19,9,W', '4,15,16,S', '5,6,12,N', '6,1,18,S'] )#["0,2,2,E", "1,14,13,N", "2,7,12,W", "3,11,7,S"])
# print("Index sequnce:", seq)
# print("Full path:", path)
# print("Full instructions:", ins)