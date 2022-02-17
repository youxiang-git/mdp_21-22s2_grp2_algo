import pygame
import math
from queue import PriorityQueue
from tsp_approx import calc_TSP
import numpy as np
import os
from RobotCar import RobotCar

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

font = pygame.font.SysFont('arial.ttf', 48)
base_path = os.path.dirname(__file__)
car_img_path = os.path.join(base_path, "car.png")

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

    def get_heading(self):
        return self.heading

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
        if self.heading == 0:
            print("On West")
        elif self.heading == 90:
            print("On South")
        elif self.heading == 180:
            print("On East")
        elif self.heading == 270:
            print("On North")

    def start_area(self):
        self.color = YELLOW

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def set_heading(self, p):
        self.heading = p
        
    def update_neighbors(self, grid, pose):

        print(pose)

        print(f"self row, self col = {self.row}, {self.col}")

        if pose == 0:
            if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_obstacle(): # UP
                self.neighbors.append(grid[self.row + 1][self.col])
                self.neighbors[-1].set_heading(pose) 

            if self.row > 0 and not grid[self.row - 1][self.col].is_obstacle(): # DOWN
                self.neighbors.append(grid[self.row - 1][self.col])
                self.neighbors[-1].set_heading(pose) 
        
        elif pose == 180:
            if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_obstacle(): # UP
                self.neighbors.append(grid[self.row + 1][self.col])
                self.neighbors[-1].set_heading(pose)

            if self.row > 0 and not grid[self.row - 1][self.col].is_obstacle(): # DOWN
                self.neighbors.append(grid[self.row - 1][self.col])
                self.neighbors[-1].set_heading(pose) 

        elif pose == 90:
            if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_obstacle(): # RIGHT
                self.neighbors.append(grid[self.row][self.col + 1])
                self.neighbors[-1].set_heading(pose)

            if self.col > 0 and not grid[self.row][self.col - 1].is_obstacle(): # LEFT
                self.neighbors.append(grid[self.row][self.col - 1])
                self.neighbors[-1].set_heading(pose) 

        elif pose == 270:
            if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_obstacle(): # RIGHT
                self.neighbors.append(grid[self.row][self.col + 1])
                self.neighbors[-1].set_heading(pose)

            if self.col > 0 and not grid[self.row][self.col - 1].is_obstacle(): # LEFT
                self.neighbors.append(grid[self.row][self.col - 1])
                self.neighbors[-1].set_heading(pose)

        if self.col >= 3 and self.row < self.total_rows - 3 and not grid[self.row + 3][self.col - 3].is_obstacle(): #FR
            # print("FR true")
            self.neighbors.append(grid[self.row + 3][self.col - 3])
            if pose == 0:
                self.neighbors[-1].set_heading((pose + 90) % 360)
            if pose == 90:
                self.neighbors[-1].set_heading((pose - 90) % 360)
            if pose == 180:
                self.neighbors[-1].set_heading((pose + 90) % 360)
            if pose == 270:
                self.neighbors[-1].set_heading((pose - 90) % 360)
            
        if self.col < self.total_rows - 3 and self.row < self.total_rows - 3 and not grid[self.row + 3][self.col + 3].is_obstacle(): # BR
            # print("BR true")
            self.neighbors.append(grid[self.row + 3][self.col + 3])
            if pose == 0:
                self.neighbors[-1].set_heading((pose - 90) % 360)
            if pose == 90:
                self.neighbors[-1].set_heading((pose + 90) % 360)
            if pose == 180:
                self.neighbors[-1].set_heading((pose - 90) % 360)
            if pose == 270:
                self.neighbors[-1].set_heading((pose + 90) % 360)

        if self.col >= 3 and self.row >= 3 and not grid[self.row - 3][self.col - 3].is_obstacle(): #FL
            # print("FL true")
            self.neighbors.append(grid[self.row - 3][self.col - 3])
            if pose == 0:
                self.neighbors[-1].set_heading((pose - 90) % 360)
            if pose == 90:
                self.neighbors[-1].set_heading((pose + 90) % 360)
            if pose == 180:
                self.neighbors[-1].set_heading((pose - 90) % 360)
            if pose == 270:
                self.neighbors[-1].set_heading((pose + 90) % 360)

        if self.col < self.total_rows -3 and self.row >= 3 and not grid[self.row][self.col - 1].is_obstacle(): # BL
            # print("BL true")
            self.neighbors.append(grid[self.row - 3][self.col + 3]) 
            if pose == 0:
                self.neighbors[-1].set_heading((pose + 90) % 360)
            if pose == 90:
                self.neighbors[-1].set_heading((pose - 90) % 360)
            if pose == 180:
                self.neighbors[-1].set_heading((pose + 90) % 360)
            if pose == 270:
                self.neighbors[-1].set_heading((pose - 90) % 360)

        # if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_obstacle(): # SOUTH-WEST
        #     self.neighbors.append(grid[self.row - 1][self.col - 1])

        # if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_obstacle(): # SOUTH-EAST
        #     self.neighbors.append(grid[self.row - 1][self.col + 1])

        # if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_obstacle(): # NORTH-WEST
        #     self.neighbors.append(grid[self.row + 1][self.col - 1])

        # if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].is_obstacle(): # NORTH-EAST
        #     self.neighbors.append(grid[self.row + 1][self.col + 1])

        


    # compares 2 different spots (grids)
    def __lt__(self, other):
        return False

# our A* heuristic, h(x), in euclidean distance
def h(p1, p2):
    x1, y1, h1 = p1
    x2, y2, h2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, current, draw, car):
    path = []
    path.append(current.get_pos())
    while current in came_from:
        current = came_from[current]
        current_pos = current.get_pos()
        path.append(current_pos)
        if not current.is_goal():
            current.make_path()
        draw()
        pygame.time.wait(200)
    
    path_r = list(reversed(path))

    for c in range(len(path_r)):
        cx, cy, cheading = path_r[c]
        print(f"cx = {cx}, cy = {cy}, cheading = {cheading}")
        if c+1 < len(path_r):
            path_r[c+1] = list(path_r[c+1])
            nx, ny, nheading = path_r[c+1]
            # print(f"cx = {cx}, cy = {cy}, nx = {nx}, ny = {ny}, cheading = {cheading}")
            # if nx > cx:
            #     cheading = 0
            # elif nx < cx:
            #     cheading = 180
            # elif ny > cy:
            #     cheading = 270
            # elif ny < cy:
            #     cheading = 90
        
        car.set_pos((cx, cy, cheading))
        draw()
        pygame.time.wait(1000)
    
    # for c in path:
    #     print(c)

    # return path

def algorithm(draw, grid, start, end, e_pose, car):

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
    sx, sy, s_pose = start.get_pos()
    print(f"car starting pose = {s_pose}")
    start.update_neighbors(grid, s_pose)


    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        
        current = open_set.get()[2]
        # print(current)
        open_set_hash.remove(current)

        if current == end and current.get_heading() == e_pose:
            reconstruct_path(came_from, end, draw, car)
            return True

        for neighbor in current.neighbors:
            nx, ny, n_pose = neighbor.get_pos()
            if not neighbor == end:
                neighbor.update_neighbors(grid, n_pose)
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

        # pygame.time.wait(10)
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

def draw(win, grid, rows, width, shp, gn, car):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

        if len(shp):
            car_x = car.x + ((WIDTH // ROWS) / 2)
            car_y = car.y + ((WIDTH // ROWS) / 2)
            car_heading = car.theta
            # print(car_heading)
            drawn_car = car.rotate_self(car_heading)
            drawn_car_rect = drawn_car.get_rect()
            drawn_car_rect.center = (car_x, car_y)
            win.blit(drawn_car, drawn_car_rect)

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

def main(win, width):
    grid = make_grid(ROWS, width)

    # start / end pos
    start = None
    end = None
    # have we started the algorithm
    run = True
    started = False
    goal_nodes = []
    shp = []
    car = RobotCar(2.1, 3.0, car_img_path, (-100, -100, 0), (WIDTH // ROWS))

    while run:
        draw(win, grid, ROWS, width, shp, goal_nodes, car)
        # pygame.time.Clock().tick(60)

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
                    row, col = get_clicked_pos(pos, ROWS, width)
                    print(row, col)
                    spot = grid[row][col]
                    if not start and row < 3 and col > 15:
                        start = spot
                        start.make_start(0)
                        goal_nodes.append((row, col, 0))
                elif event.key == pygame.K_f:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, width)
                    print(row, col)
                    spot = grid[row][col]
                    if not start and row < 4 and col > 15:
                        start = spot
                        start.make_start(90)
                        goal_nodes.append((row, col, 90))
            
                elif event.key == pygame.K_w:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, width)
                    for x in range(-1, 2, 1):
                        if col+x >= 0 and col+x < ROWS:
                            for y in range(-1, 2, 1):
                                if row+y >= 0 and row+y < ROWS:
                                    print(x, y)
                                    spot = grid[row+y][col+x]
                                    spot.make_obstacle_light()
                    spot = grid[row][col]
                    spot.make_obstacle()
                    spot = grid[row][col-4]
                    if spot != start and spot.is_obstacle() == False:
                        if goal_nodes[len(goal_nodes)-1] != (row, col-4):
                            spot.make_goal(270)
                            goal_nodes.append((row, col-4, 270))
                            print(goal_nodes)

                elif event.key == pygame.K_a:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, width)
                    for x in range(-1, 2, 1):
                        if col+x >= 0 and col+x < ROWS:
                            for y in range(-1, 2, 1):
                                if row+y >= 0 and row+y < ROWS:
                                    print(x, y)
                                    spot = grid[row+y][col+x]
                                    spot.make_obstacle_light()
                    spot = grid[row][col]
                    spot.make_obstacle()
                    spot = grid[row-4][col]
                    if spot != start and spot.is_obstacle() == False:
                        if goal_nodes[len(goal_nodes)-1] != (row-4, col):
                            spot.make_goal(0)
                            goal_nodes.append((row-4, col, 0))
                            print(goal_nodes)


                elif event.key == pygame.K_s:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, width)
                    for x in range(-1, 2, 1):
                        if col+x >= 0 and col+x < ROWS:
                            for y in range(-1, 2, 1):
                                if row+y >= 0 and row+y < ROWS:
                                    print(x, y)
                                    spot = grid[row+y][col+x]
                                    spot.make_obstacle_light()
                    spot = grid[row][col]
                    spot.make_obstacle()
                    spot = grid[row][col+4]
                    if spot != start and spot.is_obstacle() == False:
                        if goal_nodes[len(goal_nodes)-1] != (row, col+4):
                            spot.make_goal(90)
                            goal_nodes.append((row, col+4, 90))
                            print(goal_nodes)

                elif event.key == pygame.K_d:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, width)
                    for x in range(-1, 2, 1):
                        if col+x >= 0 and col+x < ROWS:
                            for y in range(-1, 2, 1):
                                if row+y >= 0 and row+y < ROWS:
                                    print(x, y)
                                    spot = grid[row+y][col+x]
                                    spot.make_obstacle_light()
                    spot = grid[row][col]
                    spot.make_obstacle()
                    spot = grid[row+4][col]
                    if spot != start and spot.is_obstacle() == False:
                        if goal_nodes[len(goal_nodes)-1] != (row+4, col):
                            spot.make_goal(180)
                            goal_nodes.append((row+4, col, 180))
                            print(goal_nodes)

            if pygame.mouse.get_pressed()[0] and pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                print(row, col)
                for x in range(-1, 2, 1):
                    if col+x >= 0 and col+x < ROWS:
                        for y in range(-1, 2, 1):
                            if row+y >= 0 and row+y < ROWS:
                                print(x, y)
                                spot = grid[row+y][col+x]
                                spot.make_obstacle_light()
                spot = grid[row][col]
                spot.make_obstacle()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not started:
                    # pass
                    shp = calc_TSP(goal_nodes)
                    shp2 = shp.copy()
                    shp2.pop()
                    print(shp2)
                    print(shp)
                    
                    while len(shp2) != 1:
                        n = shp2[0]
                        row, col, heading = goal_nodes[n]
                        print("start is " + str(row), str(col))
                        start = grid[row][col]
                        print("popping " + str(shp2.pop(0)))
                        end_node = shp2[0]
                        row, col, heading = goal_nodes[end_node]
                        print(row, col, heading)
                        end = grid[row][col]
                        end_pose = heading
                        algorithm(lambda: draw(win, grid, ROWS, width, shp, goal_nodes, car), grid, start, end, end_pose, car)
    
    pygame.quit()

main(WIN, WIDTH)