import world
import json

import pygame
from pygame import freetype

import math
from pygame.locals import (QUIT, KEYDOWN, KEYUP, K_ESCAPE, K_SPACE)


import Box2D

from Box2D.b2 import (polygonShape, circleShape, staticBody, dynamicBody, edgeShape)

colors = {
    staticBody: (50, 100, 50, 190),
    dynamicBody: (127, 127, 127, 200),
    'line': (0, 0, 0, 100),
}

PPM = 30.0
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('RL Bike Pumper')
clock = pygame.time.Clock()

stx = [SCREEN_WIDTH/2, SCREEN_HEIGHT/2]
camera_center = Box2D.b2Vec2(0, 0)

def transform_vert(body, vertices):
    vertices = [(body.transform * v - camera_center) * PPM for v in vertices]
    return [(v[0] + stx[0], SCREEN_HEIGHT - v[1] - stx[1]) for v in vertices]


def draw_polygon(polygon, body, fixture):
    pygame.draw.polygon(screen, colors[body.type],
                        transform_vert(body, polygon.vertices))


def draw_circle(circle, body, fixture):
    vert = transform_vert(body, [circle.pos, circle.pos + Box2D.b2Vec2(0, circle.radius)])
    position = vert[0]
    pygame.draw.circle(screen, colors[body.type], [int(
        x) for x in position], int(circle.radius * PPM))
    pygame.draw.circle(screen, colors['line'], [int(
        x) for x in position], int(circle.radius * PPM), width=2)
    pygame.draw.line(screen, colors['line'], start_pos=vert[0], end_pos=vert[1])


def draw_line(edge, body, fixture):
    vertices = transform_vert(body, edge.vertices)
    pygame.draw.line(screen, colors[body.type], start_pos=vertices[0], end_pos=vertices[1])

def draw_lines(body, edges):
    for edge in edges:
        vertices = transform_vert(body, edge)
        pygame.draw.line(screen, colors['line'], start_pos=vertices[0], end_pos=vertices[1], width=4)
    
    
polygonShape.draw = draw_polygon
edgeShape.draw = draw_line
circleShape.draw = draw_circle
running = True
font = freetype.Font("Vera.ttf", 16)

def process_events():
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            running = False

def draw(w, fps=TARGET_FPS, action_str=''):
    screen.fill((200, 200, 225, 0))

    # Draw the world
    draw_lines(w.bodies["Plane"], w.edges["Plane"])
    for body_name, body in w.bodies.items():
        for fixture in body.fixtures:
            fixture.shape.draw(body, fixture)


    camera_center[0] = (w.bodies["Bike"].transform.position.x * 0.1 + camera_center[0] * 0.9)
    camera_center[1] = w.bodies["Bike"].transform.position.y * 0.1


    if action_str:
        font.render_to(screen, (SCREEN_WIDTH - 120, 10), action_str, (0, 0, 0))

    pygame.display.flip()
    clock.tick(fps)
