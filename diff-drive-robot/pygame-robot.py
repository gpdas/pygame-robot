#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 12 00:39:07 2021

@author: gpdas
@based on: https://www.youtube.com/watch?v=zHboXMY45YU by Algobotics
"""

import pygame
import math
import os


class Envir:
    def __init__ (self, dimensions):
        # colours
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.yellow = (255, 255, 0)
        
        # map dimensions
        self.height = dimensions[0]
        self.width = dimensions[1]
        
        # window setting
        pygame.display.set_caption('Differential drive robot')
        self.map = pygame.display.set_mode((self.width, 
                                            self.height))
        
        # text variables
        self.font = pygame.font.Font('freesansbold.ttf', 32)
        self.text = self.font.render('default', True, self.white, self.black)
        self.text_rect = self.text.get_rect()
        self.text_rect.center = (self.width - 600, self.height - 100)
        
        self.trail_set = []


    def write_info(self, vl, vr, heading):
        txt = f"VLeft = {vl} VRight = {vr} Heading = {int(math.degrees(heading))}"
        self.text = self.font.render(txt, True, self.white, self.black)
        self.map.blit(self.text, self.text_rect)
        
    
    def trail(self, pos):
        for i in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.map, self.yellow, 
                             (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]))
        
        if self.trail_set.__sizeof__() > 300000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)
    
    
class Robot:
    def __init__(self, start_pos, robot_img, width):
        
        self.metre_to_pixels = 3779.52
        
        # dimensions
        self.width = width
        self.x = start_pos[0]
        self.y = start_pos[1]
        self.heading = 0.
        
        self.img = pygame.image.load(robot_img)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        
        self.vel_left = 0.0
        self.vel_right = 0.0
        
        self.max_speed = 0.02 * self.metre_to_pixels
        self.min_speed = -0.02 * self.metre_to_pixels
        
        
    def draw(self, map):
        map.blit(self.rotated, self.rect)
        
    def move(self, dt, event=None):
        if event is not None:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_KP4:
                    self.vel_left += 0.001 * self.metre_to_pixels
                elif event.key == pygame.K_KP1:
                    self.vel_left -= 0.001 * self.metre_to_pixels
                elif event.key == pygame.K_KP6:
                    self.vel_right += 0.001 * self.metre_to_pixels
                elif event.key == pygame.K_KP3:
                    self.vel_right -= 0.001 * self.metre_to_pixels
        
        self.x += (((self.vel_left + self.vel_right)/2) * math.cos(self.heading)) * dt
        self.y -= (((self.vel_left + self.vel_right)/2) * math.sin(self.heading)) * dt
        self.heading += ((self.vel_right - self.vel_left) / self.width) * dt
        
        # reset heading
        if self.heading > 2*math.pi or self.heading < - 2 * math.pi:
            self.heading = 0
        
        # max speed
        self.vel_left = min(self.vel_left, self.max_speed)
        self.vel_right = min(self.vel_right, self.max_speed)
        # min speed
        self.vel_left = max(self.vel_left, self.min_speed)
        self.vel_right = max(self.vel_right, self.min_speed)
        
        
        self.rotated = pygame.transform.rotozoom(self.img, math.degrees(self.heading), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

if __name__ == '__main__':
    pygame.init()
    
    meter_to_pixel = 3779.52 # magic number to convert from meters to pixels
    # dimensions 
    dimensions = (800, 1200) # of environment / canvas
    
    # running status
    running = True
    
    # the environment object
    envir = Envir(dimensions)

    # start position
    start = (200, 200)
    robot_img = os.path.abspath(os.path.join(os.curdir, 'diff-robot-100.png'))
    robot = Robot(start, robot_img, 0.1 * meter_to_pixel)
    
    last_time = pygame.time.get_ticks()
    # simulation loop
    while running:
        curr_time = pygame.time.get_ticks()
        dt = (curr_time - last_time) / 1000
        last_time = curr_time
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            robot.move(dt, event)
        
        pygame.display.update()
        envir.map.fill(envir.black)
        robot.move(dt)
        robot.draw(envir.map)
        envir.trail((robot.x, robot.y))
        envir.write_info(int(robot.vel_left), int(robot.vel_right), int(math.degrees(robot.heading)))