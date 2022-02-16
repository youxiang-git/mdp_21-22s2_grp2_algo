import math
import numpy as np
import pygame

class RobotCar:
    def __init__(self, l, w, carImg, startpos, grid_width):
        self.d1 = math.sqrt(grid_width ** 2 + grid_width ** 2)
        self.d2 = grid_width
        self.x = (startpos[0] * grid_width)
        self.y = (startpos[1] * grid_width)
        self.theta = startpos[2]
        self.g_width = grid_width
        self.width = w * self.g_width
        self.length = l * self.g_width
        self.car_img = pygame.image.load(carImg)
        self.max_angle = math.radians(30)
        self.rotated = pygame.transform.rotate(self.car_img.copy(), -90)
        self.rect = self.car_img.get_rect()

    def print_state(self):
        print(f"state is : {self.x}, {self.y}, {self.theta}")


def main():

    dist = round(math.sqrt(40 ** 2 + 40 ** 2))
    print("dist is " + str(dist))
    dist_grid = dist // 40
    print(dist_grid)

main()