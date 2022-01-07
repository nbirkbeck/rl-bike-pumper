import world
import json
import collections
import window
import pygame
from pygame.locals import (QUIT, KEYDOWN, KEYUP, K_ESCAPE, K_SPACE)
import math

import Box2D  # The main library
from Box2D import *
from Box2D.b2 import (polygonShape, circleShape, staticBody, dynamicBody, edgeShape)


w = world.World()
model = json.loads(open('models/model.json', 'r').read())
print(model)
w.load(model)
i = 0.0
jump = False
down = False

ground = w.bodies["Plane"]
fast_intersection = world.FastIntersection(ground)

while window.running:
    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            # The user closed the window or pressed escape
            window.running = False
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

        if  (event.type == KEYUP and event.key == ord(' ')):
            rear_hub = w.bodies["RearHub"]
            print(fast_intersection.intersect_ray(rear_hub.position))


    if down:
        if (joint.translation > joint.limits[0]):
            w.bodies["Body"].ApplyForceToCenter(Box2D.b2Vec2(0, -1), True)
    if jump:
        joint = w.joints["RearMotor"]
        joint.motorEnabled = True
        joint.enableLimit = True

        body = w.bodies["Body"]

        if jump and (joint.translation >= joint.limits[1] - 0.01):
            body.ApplyForceToCenter(Box2D.b2Vec2(0, body.mass * 9.81), True)
        else:
            body.ApplyForceToCenter(Box2D.b2Vec2(0, 2 * body.mass * 9.81), True)
        
        
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

            
    window.draw(w)

    # Make Box2D simulate the physics of our world for one step.
    w.world.Step(window.TIME_STEP, 10, 10)

pygame.quit()
print('Done!')
