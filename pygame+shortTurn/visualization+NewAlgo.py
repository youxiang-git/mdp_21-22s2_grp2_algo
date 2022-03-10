import pygame
import numpy as np
import networkx as nx
import numpy as np

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

def algorithm_handler(draw, grid, start_images, shp, graph):
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
        add_end, input_f, input_i = algorithm(draw, grid, start, end_image, graph)
        goal_nodes.append(add_end)
        full_path.append(input_f)
        full_ins.append(input_i)

    return goal_nodes, full_path, full_ins

def algorithm(draw, grid, start, end_image, graph):
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

    path_i = runRobotMove(best_path, draw)

    last_coord = best_path[-1]
    row, col, heading = last_coord
    goal_node = grid[row][col]
    goal_node.make_goal(heading)

    for i in range(1, len(best_path)-1):
        row, col, heading = best_path[i]
        path_node = grid[row][col]
        if path_node.is_goal() == False:
            path_node.make_path()
            draw()

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

# draw grid lines
def draw_grid(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))

def draw(win, grid, rows, width, gn):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

    for x in range(len(gn)):
        text = font.render(str(x), True, BLACK)
        textRect = text.get_rect()
        textRect.topleft = np.array(gn[x][:2]) * (WIDTH // ROWS)
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
        # for increment in range(4, 2, -1):
        increment = 4
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
        # for increment in range(4, 2, -1):
        increment = 4
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
        # for increment in range(4, 2, -1):
        increment = 4
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
        # for increment in range(4, 2, -1):
        increment = 4
        if col+increment < ROWS:
            spot = grid[row][col+increment]
            if spot.is_obstacle() == False:
                spot.make_possible_goal()
                newImage.add_goal(row, col+increment, 90)
            # if increment == 4:
            #     for jincrement in range(1, -2, -2):
            #         if row+jincrement >= 0 and row+jincrement < ROWS:
            #             spot = grid[row+jincrement][col+increment]
            #             if spot.is_obstacle() == False:
            #                 spot.make_possible_goal()
            #                 newImage.add_goal(row+jincrement, col+increment, 90)

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
    start_images = []
    global car_dir

    draw(win, grid, ROWS, WIDTH, goal_nodes)

    # TEST ONLY variable to get input from android
    android_input = []

    # border avoidance zone
    for i in (0, 19):
        for j in range(20):
            spot = grid[i][j]
            spot.make_obstacle_light()
            spot = grid[j][i]
            spot.make_obstacle_light()

    while run:
        draw(win, grid, ROWS, WIDTH, goal_nodes)

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

                    print("start_images array:", start_images)
                    print("Fake goal nodes:", goal_nodes_tsp)

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

                    goal_nodes, full_path, full_ins = algorithm_handler(lambda: draw(win, grid, ROWS, WIDTH, goal_nodes), grid, start_images, shp2, graph)

                    print("Android input:", android_input)
                    print("Goal nodes:", goal_nodes)
                    print("Index sequence:", shp3)
                    print("Full path:", full_path)
                    print("Full instructions:", full_ins)
    pygame.quit()

main(WIN)