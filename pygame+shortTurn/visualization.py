from asyncio.windows_events import NULL
from operator import ne

import pygame
from queue import PriorityQueue
import numpy as np
import networkx as nx
import numpy as np
import math

pygame.init()

# constants declaration
WIDTH = 800
ROWS = 20
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Path Finding Visualization")

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

font = pygame.font.SysFont('arial.ttf', 48)

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
        # TEST ONLY
        if self.heading == 0:
            print("Facing East")
        elif self.heading == 90:
            print("Facing North")
        elif self.heading == 180:
            print("Facing West")
        elif self.heading == 270:
            print("Facing South")

    def make_path(self):
        self.color = PURPLE

    def make_goal(self, goal_h):
        self.heading = goal_h
        self.color = TEAL

    def change_heading(self, heading):
        self.heading = heading

    def start_area(self):
        self.color = YELLOW

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_obstacle(): # NORTH
            self.neighbors.append(grid[self.row + 1][self.col])
            # grid[self.row + 1][self.col].change_heading(90)
            # print(grid[self.row + 1][self.col].get_pos())

        if self.row > 0 and not grid[self.row - 1][self.col].is_obstacle(): # SOUTH
            self.neighbors.append(grid[self.row - 1][self.col])
            # grid[self.row - 1][self.col].change_heading(270)
            # print(grid[self.row - 1][self.col].get_pos())

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_obstacle(): # EAST
            self.neighbors.append(grid[self.row][self.col + 1])
            # grid[self.row][self.col + 1].change_heading(0)
            # print(grid[self.row][self.col + 1].get_pos())

        if self.col > 0 and not grid[self.row][self.col - 1].is_obstacle(): # WEST
            self.neighbors.append(grid[self.row][self.col - 1])
            # grid[self.row][self.col - 1].change_heading(180)
            # print(grid[self.row][self.col - 1].get_pos())

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

# our A* heuristic, h(x), in euclidean distance
def h(p1, p2):
    add_cost = 0
    x1, y1, h1 = p1
    x2, y2, h2 = p2
    h1 = math.radians(h1)
    h2 = math.radians(h2)

    cost = abs(x1 - x2) + abs(y1 - y2) + abs(h1 - h2)
    return cost

def reconstruct_path(came_from, current, draw):
    path = []
    path_f = []
    path_ins = []
    path.append(current.get_pos())
    while current in came_from:
        current = came_from[current]
        current_pos = current.get_pos()
        path.append(current_pos)
        if not current.is_goal():
            current.make_path()
        draw()
    
    path_r = list(reversed(path))

    for c in range(len(path_r)):
        cx, cy, cheading = path_r[c]
        # print("cheading = " +str(cheading))
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
        path_ins.append((cx, cy, cheading))
        path_f.append(str(cx)+","+str(abs(cy-19))+","+str(cheading))
            
    path_i = runRobotMove(path_ins, draw)
    return path_f, path_i

def runRobotMove(path, draw):
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
        draw()
        i = i + 1
    
    if straightCounter != 0:
        carStraight(path_i, straightCounter)

    return path_i

def carStraight(path_i, n):
    # print("straight for", n)
    path_i.append("1"+numToLetter(n))
    return

def carReverse(path_i, n):
    # print("reverse for", n)
    path_i.append("2"+numToLetter(n))
    return

def carPointTurnLeft(path_i):
    # print("point turn left")
    path_i.append("30")
    return

def carPointTurnRight(path_i):
    # print("point turn right")
    path_i.append("40")
    return

def carTurn180(path_i):
    # print("turn 180")
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

def algorithm(draw, grid, start, end):
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
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        

        current = open_set.get()[2]
        # print(current.get_pos())
        open_set_hash.remove(current)

        if current == end:
            return reconstruct_path(came_from, end, draw)

        for neighbor in current.neighbors:
            temp_h = -1
            add_cost = 0
            cx, cy, ch = current.get_pos()
            nx, ny, nh = neighbor.get_pos()
            if nx > cx:
                temp_h = 0
            elif nx < cx:
                temp_h = 180
            elif ny > cy:
                temp_h = 270
            elif ny < cy:
                temp_h = 90

            

            if neighbor != end:
                neighbor.change_heading(temp_h)
            # else:
                # print("neighbor is end")
            
            ta = abs(ch - temp_h)

            # print("turning angle " + str(ta))
            if ta >= 90:
                # print("90deg")
                add_cost = add_cost + 1

            temp_g_score = g_score[current] + 1 + add_cost

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                # print("ch is " + str(ch))
                if neighbor != end:
                    f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                else:
                    ex, ey, eh = neighbor.get_pos()[0], neighbor.get_pos()[1], temp_h
                    f_score[neighbor] = temp_g_score + h((ex, ey, eh), end.get_pos())

                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    if not neighbor.is_path() and not neighbor.is_goal() and not neighbor.is_start() and not neighbor.is_close():
                        neighbor.make_open()
        pygame.time.delay(10)
        draw()

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

# draw grid lines
def draw_grid(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))

def draw(win, grid, rows, width, shp, gn):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

    for x in shp:
        text = font.render(str(x), True, BLACK)
        textRect = text.get_rect()
        textRect.topleft = np.array(gn[shp[x]][:2]) * (WIDTH // ROWS)
        win.blit(text, textRect)

    draw_grid(win, rows, width)

    pygame.display.update()

# pos == mouse position, help function
def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos
    row = y // gap
    col = x // gap

    return row, col

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
        for increment in range(4, 2, -1):
            if row+increment < ROWS:
                spot = grid[row+increment][col]
                if spot.is_obstacle() == False:
                    spot.make_possible_goal()
                    newImage.add_goal(row+increment, col, 180)
                if increment == 4:
                    for jincrement in range(1, -2, -2):
                        if col+jincrement >= 0 and col+jincrement < ROWS:
                            spot = grid[row+increment][col+jincrement]
                            if spot.is_obstacle() == False:
                                spot.make_possible_goal()
                                newImage.add_goal(row+increment, col+jincrement, 180)
    elif image_direction == 90:
        for increment in range(4, 2, -1):
            if col-increment >= 0:
                spot = grid[row][col-increment]
                if spot.is_obstacle() == False:
                    spot.make_possible_goal()
                    newImage.add_goal(row, col-increment, 270)
                if increment == 4:
                    for jincrement in range(1, -2, -2):
                        if row+jincrement >= 0 and row+jincrement < ROWS:
                            spot = grid[row+jincrement][col-increment]
                            if spot.is_obstacle() == False:
                                spot.make_possible_goal()
                                newImage.add_goal(row+jincrement, col-increment, 270)
    elif image_direction == 180:
        for increment in range(4, 2, -1):
            if row-increment >= 0:
                spot = grid[row-increment][col]
                if spot.is_obstacle() == False:
                    spot.make_possible_goal()
                    newImage.add_goal(row-increment, col, 0)
                if increment == 4:
                    for jincrement in range(1, -2, -2):
                        if col+jincrement >= 0 and col+jincrement < ROWS:
                            spot = grid[row-increment][col+jincrement]
                            if spot.is_obstacle() == False:
                                spot.make_possible_goal()
                                newImage.add_goal(row-increment, col+jincrement, 0)
    elif image_direction == 270:
        for increment in range(4, 2, -1):
            if col+increment < ROWS:
                spot = grid[row][col+increment]
                if spot.is_obstacle() == False:
                    spot.make_possible_goal()
                    newImage.add_goal(row, col+increment, 90)
                if increment == 4:
                    for jincrement in range(1, -2, -2):
                        if row+jincrement >= 0 and row+jincrement < ROWS:
                            spot = grid[row+jincrement][col+increment]
                            if spot.is_obstacle() == False:
                                spot.make_possible_goal()
                                newImage.add_goal(row+jincrement, col+increment, 90)

# TEST ONLY
def android_add_input(android_input, row, col, direction):
    android_input.append(str(len(android_input))+","+str(row)+","+str(abs(col-19))+","+direction)
    return

def main(win):
    grid = make_grid(ROWS, WIDTH)

    # start / end pos
    start = None
    end = None
    # have we started the algorithm
    run = True
    started = False
    goal_nodes = []
    goal_nodes_tsp = []
    start_images = []
    shp = []
    global car_dir

    draw(win, grid, ROWS, WIDTH, shp, goal_nodes)

    # TEST ONLY variable to get input from android
    android_input = []

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

    while run:
        draw(win, grid, ROWS, WIDTH, shp, goal_nodes)

        if not start:
            for i in range(4):
                for j in range(16, 20):
                    spot = grid[i][j]
                    spot.start_area()

        for event in pygame.event.get():
            # always check if user has quit the game first
            if event.type == pygame.QUIT:
                run = False
            
            if started:
                continue
            
            # check if mouse was clicked
            # 0 == left, 1 == middle, 2 == right mouse buttons

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, WIDTH)
                    android_add_input(android_input, row, col, "E")
                    spot = grid[row][col]
                    if not start and row < 3 and col > 15:
                        car_dir = 0
                        start = spot
                        start.make_start(0)
                        start_images.append(RobotStart(row, col, 0))
                elif event.key == pygame.K_f:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, WIDTH)
                    android_add_input(android_input, row, col, "N")
                    spot = grid[row][col]
                    if not start and row < 3 and col > 15:
                        car_dir = 90
                        start = spot
                        start.make_start(90)
                        start_images.append(RobotStart(row, col, 90))

                elif event.key == pygame.K_w:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, WIDTH)
                    android_add_input(android_input, row, col, "N")
                    create_avoidance_area(row, col, grid, start_images)
                    newImage = Image(row, col, 90)
                    start_images.append(newImage)
                    create_possible_goal(newImage, grid)

                elif event.key == pygame.K_a:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, WIDTH)
                    android_add_input(android_input, row, col, "W")
                    create_avoidance_area(row, col, grid, start_images)
                    newImage = Image(row, col, 180)
                    start_images.append(newImage)
                    create_possible_goal(newImage, grid)

                elif event.key == pygame.K_s:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, WIDTH)
                    android_add_input(android_input, row, col, "S")
                    create_avoidance_area(row, col, grid, start_images)
                    newImage = Image(row, col, 270)
                    start_images.append(newImage)
                    create_possible_goal(newImage, grid)

                elif event.key == pygame.K_d:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, WIDTH)
                    android_add_input(android_input, row, col, "E")
                    create_avoidance_area(row, col, grid, start_images)
                    newImage = Image(row, col, 0)
                    start_images.append(newImage)
                    create_possible_goal(newImage, grid)

                elif event.key == pygame.K_q:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, WIDTH)
                    create_avoidance_area(row, col, grid, start_images)

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not started:
                    images_to_remove = []
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

                    print("start_images array:", start_images)
                    print("Fake goal nodes:", goal_nodes_tsp)

                    full_path = []
                    full_ins = []
                    shp = calc_TSP(goal_nodes_tsp)
                    shp2 = shp.copy()
                    shp2.pop()
                    # index sequence for rpi
                    shp3 = [str(x) for x in shp2]

                    # populating goal_nodes
                    index = 0
                    while index < len(shp2):
                        if shp2[index] == 0: # first sequence
                            goal_nodes.append(start_images[0].get_goal())
                        else:
                            found_overlapped = False
                            sequence = shp2[index]
                            goal_nodes1 = start_images[sequence].get_all_goal()
                            if index != len(shp2)-1:
                                # check if any overlapping nodes with next goal
                                next_sequence = shp2[index+1]
                                goal_nodes2 = start_images[next_sequence].get_all_goal()
                                for n in goal_nodes1:
                                    for m in goal_nodes2:
                                        if n[:2] == m[:2]:
                                            goal_nodes.append(n)
                                            goal_nodes.append(m)
                                            found_overlapped = True
                                            index = index + 1
                                            break
                                    if found_overlapped == True:
                                        break
                            if found_overlapped == False:
                                # find goal_nodes based on shortest distance
                                current_x, current_y = goal_nodes[-1][:2]
                                shortest_node = None
                                shortest_length = None
                                for next_goal_node in goal_nodes1:
                                    next_x, next_y = next_goal_node[:2]
                                    cal_dist = math.sqrt(pow((current_x - next_x), 2) + pow((current_y - next_y), 2))
                                    if shortest_length == None:
                                        shortest_node = next_goal_node
                                        shortest_length = cal_dist
                                    elif cal_dist < shortest_length:
                                        shortest_node = next_goal_node
                                        shortest_length = cal_dist
                                goal_nodes.append(shortest_node)
                        index = index + 1

                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    
                    for n in range(0, len(goal_nodes)-1):
                        row, col, heading = goal_nodes[n]
                        start = grid[row][col]
                        start.make_goal(heading)
                        row, col, heading = goal_nodes[n+1]
                        end = grid[row][col]
                        end.make_goal(heading)
                        input_f, input_i = algorithm(lambda: draw(win, grid, ROWS, WIDTH, shp, goal_nodes), grid, start, end)
                        full_path.append(input_f)
                        full_ins.append(input_i)
                    
                    print("Android input:", android_input)
                    print("Goal nodes:", goal_nodes)
                    print("Index sequence:", shp3)
                    print("Full path:", full_path)
                    print("Full instructions:", full_ins)
    pygame.quit()

main(WIN)