from tkinter import E
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
car_path = os.path.join(base_path, "car.png")

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
        return self.row, self.col

    # this method tells us if this node has been already visited / considered
    def is_close(self):
        return self.color == RED

    # this method tells us that this node is a frontier node
    def is_open(self):
        return self.color == GREEN

    # is this node an obstacle
    def is_obstacle(self):
        return self.color == BLACK

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

    def reset(self):
        self.color = WHITE

    def make_close(self):
        self.color = RED

    # this method tells us that this node is a frontier node
    def make_open(self):
        self.color = GREEN

    # is this node an obstacle
    def make_obstacle(self):
        self.color = BLACK

    def make_start(self, start_h):
        self.heading = start_h
        self.color = BLUE
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
            print("Facing East")
        elif self.heading == 90:
            print("Facing North")
        elif self.heading == 180:
            print("Facing West")
        elif self.heading == 270:
            print("Facing South")

    def start_area(self):
        self.color = YELLOW

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_obstacle(): # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_obstacle(): # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_obstacle(): # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_obstacle(): # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

    # compares 2 different spots (grids)
    def __lt__(self, other):
        return False

# our A* heuristic, h(x)
def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    eu_d = abs(x1 - x2)
    return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()

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
        # print(current)
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            start.make_start(0)
            end.make_goal(0)
            return True

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
                    if not neighbor.is_path() and not neighbor.is_goal() and not neighbor.is_closed():
                        neighbor.make_open()

        pygame.time.wait(20)
        draw()

        if current != start and not current.is_path() and not current.is_goal():
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
        car = RobotCar(2.1, 3.0, car_path, gn[0], (WIDTH // ROWS))
        carRect = car.rect
        car_heading = car.theta
        if car_heading == 90:
            car_x = car.x + ((WIDTH // ROWS) / 2)
            car_y = car.y + (WIDTH // ROWS)
            # car.print_state()
            carRect.midbottom = (car_x, car_y)
            car.rotated.set_alpha(48)
            win.blit(car.rotated, carRect)

        elif car_heading == 0:
            car_x = car.x
            car_y = car.y + ((WIDTH // ROWS) / 2)
            carRect.midleft = (car_x, car_y)
            car.car_img.set_alpha(48)
            win.blit(car.car_img, carRect)

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

    while run:
        draw(win, grid, ROWS, width, shp, goal_nodes)
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
                    if not start and row < 3 and col > 15:
                        start = spot
                        start.make_start(90)
                        goal_nodes.append((row, col, 90))
            
                elif event.key == pygame.K_w:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, width)
                    spot = grid[row][col]
                    if spot != start and spot.is_obstacle() == False:
                        if goal_nodes[len(goal_nodes)-1] != (row, col):
                            spot.make_goal(90)
                            goal_nodes.append((row, col, 90))
                            print(goal_nodes)

                elif event.key == pygame.K_a:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, width)
                    spot = grid[row][col]
                    if spot != start and spot.is_obstacle() == False:
                        if goal_nodes[len(goal_nodes)-1] != (row, col):
                            spot.make_goal(180)
                            goal_nodes.append((row, col, 180))
                            print(goal_nodes)


                elif event.key == pygame.K_s:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, width)
                    spot = grid[row][col]
                    if spot != start and spot.is_obstacle() == False:
                        if goal_nodes[len(goal_nodes)-1] != (row, col):
                            spot.make_goal(270)
                            goal_nodes.append((row, col, 270))
                            print(goal_nodes)

                elif event.key == pygame.K_d:
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, ROWS, width)
                    spot = grid[row][col]
                    if spot != start and spot.is_obstacle() == False:
                        if goal_nodes[len(goal_nodes)-1] != (row, col):
                            spot.make_goal(0)
                            goal_nodes.append((row, col, 0))
                            print(goal_nodes)

            if pygame.mouse.get_pressed()[0] and pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                print(row, col)
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

                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    
                    while len(shp2) != 1:
                        n = shp2[0]
                        row, col = goal_nodes[n][:2]
                        print("start is " + str(row), str(col))
                        start = grid[row][col]
                        print("popping " + str(shp2.pop(0)))
                        end_node = shp2[0]
                        row, col = goal_nodes[end_node][:2]
                        print(row, col)
                        end = grid[row][col]
                        algorithm(lambda: draw(win, grid, ROWS, width, shp, goal_nodes), grid, start, end)


    
    pygame.quit()

main(WIN, WIDTH)