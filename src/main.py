import world
import json

import pygame
from pygame.locals import (QUIT, KEYDOWN, KEYUP, K_ESCAPE, K_SPACE)
import math

import Box2D  # The main library
# Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
from Box2D.b2 import (polygonShape, circleShape, staticBody, dynamicBody, edgeShape)

colors = {
    staticBody: (255, 255, 255, 255),
    dynamicBody: (127, 127, 127, 255),
}



PPM = 20.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()

stx = [SCREEN_WIDTH/2, SCREEN_HEIGHT/2]
camera_center = Box2D.b2Vec2(0, 0)

def transform_vert(body, vertices):
    vertices = [(body.transform * v - camera_center) * PPM for v in vertices]
    return [(v[0] + stx[0], SCREEN_HEIGHT - v[1] - stx[1]) for v in vertices]


def draw_polygon(polygon, body, fixture):
    pygame.draw.polygon(screen, colors[body.type],
                        transform_vert(body, polygon.vertices))
polygonShape.draw = draw_polygon


def draw_circle(circle, body, fixture):
    vert = transform_vert(body, [circle.pos])
    position = vert[0]
    pygame.draw.circle(screen, colors[body.type], [int(
        x) for x in position], int(circle.radius * PPM))
circleShape.draw = draw_circle

def my_draw_line(edge, body, fixture):
    vertices = transform_vert(body, edge.vertices)
    pygame.draw.line(screen, colors[body.type], start_pos=vertices[0], end_pos=vertices[1])
edgeShape.draw = my_draw_line


w = world.World()
model = json.loads(open('models/model.json', 'r').read())
print(model)
w.load(model)
running = True
i = 0.0
jump = False
down = False
while running:
    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            # The user closed the window or pressed escape
            running = False
        if  (event.type == KEYDOWN and event.key == ord('a')):
            jump = True
            i = 0
        if  (event.type == KEYUP and event.key == ord('a')):
            jump = False
        if  (event.type == KEYDOWN and event.key == ord('z')):
            down = True
            i = 0
        if  (event.type == KEYUP and event.key == ord('z')):
            down = False
            
    screen.fill((0, 0, 0, 0))

    # Draw the world
    for body_name, body in w.bodies.items():
        for fixture in body.fixtures:
            fixture.shape.draw(body, fixture)

    # Make Box2D simulate the physics of our world for one step.
    w.world.Step(TIME_STEP, 10, 10)

    if down:
        print(dir(w.bodies["Body"]))
        if (joint.translation >= joint.limits[0]):
            w.bodies["Body"].ApplyForceToCenter(Box2D.b2Vec2(0, -1), True)
    if jump:
        print(dir(w.bodies["Body"]))
        joint = w.joints["RearMotor"]
        body = w.bodies["Body"]

        if jump and (joint.translation >= joint.limits[1] - 0.01):
            body.ApplyForceToCenter(Box2D.b2Vec2(0, body.mass * 9.81), True)
        else:
            body.ApplyForceToCenter(Box2D.b2Vec2(0, 2 * body.mass * 9.81), True)

        joint = w.joints["RearMotor"]
        joint.motorEnabled = True
        joint.enableLimit = True
        print(w.joints["RearMotor"])
    elif not down:
        body.ApplyForceToCenter(Box2D.b2Vec2(0, 0.5 * body.mass * 9.81), True)
        
        
    if False and (jump or i >= -1): # and (round(i / 1)) % 1 == 0:
        if "FrontMotor" in w.joints:
            w.joints["FrontMotor"].motorEnabled = True
            w.joints["FrontMotor"].enableLimit = True
            w.joints["FrontMotor"].motorSpeed = 10 * (i - 0.5) #math.sin(2 * 3.14 * i*2)
            w.joints["FrontMotor"].maxMotorForce = 4
            w.joints["FrontMotor"].maxMotorSpeed = 10
        if True:
            w.joints["RearMotor"].motorEnabled = True
            w.joints["RearMotor"].enableLimit = True
            w.joints["RearMotor"].motorSpeed =  10 * math.sin(i * 3.14/2) #math.cos(3.14 * i*2)
            w.joints["RearMotor"].maxMotorForce = 30
            w.joints["RearMotor"].maxMotorSpeed = 10

        print(jump, i)
        if jump:
            i += 0.1
        else:
            i -= 0.1
        if i >= 1:
            i = 1
        
    camera_center[0] = w.bodies["Body"].transform.position.x
    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
print('Done!')
