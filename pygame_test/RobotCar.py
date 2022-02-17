import math
import numpy as np
import pygame

RED = (255, 0, 0)
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
        self.max_angle = math.radians(24)


    def print_state(self):
        print(f"state is : {self.x}, {self.y}, {self.theta}")


    def rotate_self(self, angle):
        arrow_l = 50
        img_w, img_h = self.car_img.get_size()
        carImageCopy = self.car_img.copy()
        rect2 = pygame.Surface((img_w*2, img_h*2), pygame.SRCALPHA)
        rect_2_center = (rect2.get_width() / 2, rect2.get_height() / 2)
        rect_2_arrow_end = (rect2.get_width() / 2 + arrow_l, rect2.get_height() / 2)
        rect2.blit(carImageCopy, (img_w-25, img_h-60))
        pygame.draw.line(rect2,RED, rect_2_center, rect_2_arrow_end, 3)
        pygame.draw.polygon(rect2, RED, [(rect2.get_width() / 2 + arrow_l, rect2.get_height() / 2 - 10), (rect2.get_width() / 2 + arrow_l + 10, rect2.get_height() / 2), (rect2.get_width() / 2 + arrow_l, rect2.get_height() / 2 + 10)], 5)
        return pygame.transform.rotate(rect2, angle)

    def set_pos(self, pos):
        self.x = (pos[0] * self.d2)
        self.y = (pos[1] * self.d2)
        self.theta = pos[2]

# def main():

#     dist = round(math.sqrt(40 ** 2 + 40 ** 2))
#     print("dist is " + str(dist))
#     dist_grid = dist // 40
#     print(dist_grid)



# main()