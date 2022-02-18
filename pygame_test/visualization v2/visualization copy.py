import os
import pygame
import math
from queue import PriorityQueue
from tsp_approx import calc_TSP
from rrt_star_reeds_shepp import findPath
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

font = pygame.font.SysFont('arial.ttf', 48)

class Robot:
    def __init__(self, robot_path, width):
        self.w = width
        self.x = 0
        self.y = 0
        self.theta = 0
        self.img = pygame.image.load(robot_path)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
    
    def setStartPosition(self, coord):
        self.x = coord[0]
        self.y = coord[1]

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def move(self, coord):
        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(coord[2]), 1)
        self.rect = self.rotated.get_rect(center = (coord[0], abs(coord[1]-799)))

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

    def is_end(self):
        return self.color == TURQUOISE

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

    def make_start(self):
        self.color = ORANGE

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def make_goal(self):
        self.color = TEAL

    def start_area(self):
        self.color = YELLOW

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        pass

    # compares 2 different spots (grids)
    def __lt__(self, other):
        return False


# our A* heuristic, h(x)
def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2, y1 - y2)

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
        text = font.render(str(shp[x]), True, BLACK)
        textRect = text.get_rect()
        textRect.topleft = np.array(gn[x]) * (WIDTH // ROWS)
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

def drawPath(win, coord, robot):
    if len(coord) >= 3:
        drawPoint = [(coord[0]), (abs(coord[1]-799))]
        pygame.draw.circle(win, RED, drawPoint, 5, 5)
        robot.move(coord)
        robot.draw(win)
        pygame.display.update()

def main(win, width):
    grid = make_grid(ROWS, width)

    # start / end pos
    start = None
    end = None
    # have we started the algorithm
    run = True
    started = False
    goal_nodes = []
    obstacle_nodes = [] # with inverted y
    goal_nodes_with_dir = [] # with inverted y
    shp = []
    allPath = []
    robot_path = os.path.join(os.path.dirname(__file__), "DDR.png")
    robot = Robot(robot_path, 300)
    iteration = -1

    while run:
        pygame.display.update()
        draw(win, grid, ROWS, width, shp, goal_nodes)
        iteration = iteration + 1
        if iteration >= len(allPath):
            iteration = -1
        else:
            drawPath(win, allPath[iteration], robot)

        pygame.time.Clock().tick(1000)

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
            if pygame.mouse.get_pressed()[0] and pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                print(row, col)
                spot = grid[row][col]
                if not start and row < 3 and col > 15:
                    start = spot
                    start.make_start()
                    goal_nodes.append((row, col))
                    goal_nodes_with_dir.append([row*40+20, abs(col-19)*40+20, np.deg2rad(90.0)])

                # elif not end and spot != start:
                #     end = spot
                #     end.make_end()

                elif spot != start: 
                    spot.make_obstacle()
                    obstacle_nodes.append((row*40+20, abs(col-19)*40+20)+(2,))

            elif pygame.mouse.get_pressed()[1] and pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                if spot != start and spot.is_obstacle() == False:
                    if goal_nodes[len(goal_nodes)-1] != (row, col):
                        spot.make_goal()
                        goal_nodes.append((row, col))

                        if win.get_at((row*41, (col+1)*41)) == BLACK:
                            goal_nodes_with_dir.append([row*40+20, abs(col-2-19)*40+20, np.deg2rad(-90)])
                        elif win.get_at((row*41, (col-1)*41)) == BLACK:
                            goal_nodes_with_dir.append([row*40+20, abs(col+2-19)*40+20, np.deg2rad(90)])
                        elif win.get_at(((row-1)*41, col*41)) == BLACK:
                            goal_nodes_with_dir.append([(row+2)*40+20, abs(col-19)*40+20, np.deg2rad(180)])
                        else:
                            goal_nodes_with_dir.append([(row-2)*40+20, abs(col-19)*40+20, np.deg2rad(0)])

                        print(goal_nodes)


            elif pygame.mouse.get_pressed()[2] and pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None

                elif spot == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not started:
                    # pass
                    shp = calc_TSP(goal_nodes)
                    robot.setStartPosition(goal_nodes[0])

                    print("Hamiltonian path:", shp)
                    print("Goal nodes with direction:", goal_nodes_with_dir)                    
                    print("Obstable list:", obstacle_nodes)

                    for i in range(len(shp)-2):
                        print(i)
                        print(goal_nodes_with_dir[shp[i]])
                        print(goal_nodes_with_dir[shp[i+1]])
                        path = findPath(goal_nodes_with_dir[shp[i]], goal_nodes_with_dir[shp[i+1]], obstacle_nodes, 10)
                        print(path)
                        print(path[::-1])
                        allPath.extend(path[::-1])
                    iteration = 0

    pygame.quit()

main(WIN, WIDTH)