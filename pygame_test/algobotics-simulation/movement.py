import pygame
import math

class Environ:
    def __init__(self, dimensions):
        self.black = (0, 0 ,0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.blue = (0, 0, 255)
        self.yellow = (255, 255, 0)

        self.height = dimensions[0]
        self.width = dimensions[1]

        pygame.display.set_caption("Differential Drive Robot")
        self.map=pygame.display.set_mode((self.width, self.height))

pygame.init()

start = (200,200)

dims = (600, 1200)

running = True

environment = Environ(dims)

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.update()
    environment.map.fill(environment.black)