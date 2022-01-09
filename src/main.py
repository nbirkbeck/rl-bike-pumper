import world
import json
import collections
import window
import pygame
from pygame.locals import (QUIT, KEYDOWN, KEYUP, K_ESCAPE, K_SPACE)
import math

import Box2D
from Box2D import *
from Box2D.b2 import (polygonShape, circleShape, staticBody, dynamicBody, edgeShape)


w = world.World()
model = json.loads(open('models/model.json', 'r').read())
print(model)
w.load(model)

jump = False
down = False

ground = w.bodies["Plane"]
fast_intersection = world.FastIntersection(ground)
joint = w.joints["RearMotor"]
joint.motorEnabled = True
joint.enableLimit = True

while window.running:
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            window.running = False
        if  (event.type == KEYDOWN and event.key == ord('a')):
            jump = True
        if  (event.type == KEYUP and event.key == ord('a')):
            jump = False
        if  (event.type == KEYDOWN and event.key == ord('z')):
            down = True
        if  (event.type == KEYUP and event.key == ord('z')):
            down = False

    if down:
        if (joint.translation > joint.limits[0]):
            w.bodies["Body"].ApplyForceToCenter(Box2D.b2Vec2(0, -1), True)
    if jump:
        body = w.bodies["Body"]

        if jump and (joint.translation >= joint.limits[1] - 0.01):
            body.ApplyForceToCenter(Box2D.b2Vec2(0, body.mass * 9.81), True)
        else:
            body.ApplyForceToCenter(Box2D.b2Vec2(0, 2 * body.mass * 9.81), True)

    window.draw(w, action_str='Pulling up' if jump else 'Pushing down' if down else '')

    w.world.Step(window.TIME_STEP, 10, 10)

pygame.quit()
