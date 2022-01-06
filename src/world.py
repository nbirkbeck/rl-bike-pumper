import Box2D
from Box2D.b2 import world
from Box2D import *

def hasRigidContact(contact):
    while (contact and contact.e):
        c = contact.get_contact()
        if not c: break;
        if (c.IsEnabled() and c.IsTouching()):
            ba = c.GetFixtureA().GetBody()
            bb = c.GetFixtureB().GetBody()
            if (ba.GetType() != Box2D.b2_dynamicBody or
                bb.GetType() != Box2D.b2_dynamicBody):
                return True

        contact = contact.get_next()
    return False

def createPolygonShape(vertices):
    shape = Box2D.b2PolygonShape(vertices=vertices)
    return shape



class World:
    def __init__(self):
        self.bodies = {}
        self.joints = {}
        self.world = None
        self.init_()
        
    def reset(self):
        if self.world:
            Box2D.destroy(self.world)
        self.bodies = {}
        self.joints = {}
        self.init_()

    def init_(self):
        self.world = Box2D.b2World(Box2D.b2Vec2(0.0, -10.0))
  
        bd_ground = b2BodyDef()
        groundBody = self.world.CreateBody(bd_ground)

        #groundBody.CreateFixture(shape=b2EdgeShape(vertices=[b2Vec2(-10.0, -1), b2Vec2(100.0, -1)]))
        groundBody.CreateFixture(shape=b2EdgeShape(vertices=[b2Vec2(-10.0, -2.0), b2Vec2(50.0, -20.0)]))
        groundBody.CreateFixture(shape=b2EdgeShape(vertices=[b2Vec2(-10.0, -7), b2Vec2(60.0, -7)]))

        groundBody.CreateFixture(shape=b2EdgeShape(vertices=[b2Vec2(10.0, -7), b2Vec2(11.0, -6.4)]))

        groundBody.CreateFixture(shape=b2EdgeShape(vertices=[b2Vec2(15, -7), b2Vec2(16, -6.4)]))
        groundBody.CreateFixture(shape=b2EdgeShape(vertices=[b2Vec2(21, -7), b2Vec2(22, -6.4)]))
        print(groundBody)
        self.bodies = {"ground": groundBody}

    def load(self, model):
        for i in range(0, len(model["objects"])):
            obj = model["objects"][i]
            print("body %d, %s" % (i, obj["name"]))


            if "fixtures" not in obj or len(obj["fixtures"]) != 1:
                print("no fixtures")
                print(obj)
                continue
                threeObject.position = (obj["pos"][0], obj["pos"][1], obj["pos"[2]])
                threeObject.rotation.set(0, 0, obj.rot, 'XYZ')
                debugObjects.push(threeObject)
                continue

            if obj["fixtures"][0]["type"] == "mesh":
                body = self.world.CreateBody()
            else:
                body = self.world.CreateDynamicBody(position=b2Vec2(0, 0))
                body.angularDamping = 0.25
                body.linearDamping = 0.001
                print(body)
                
            
            
            if obj["fixtures"][0]:
                fixture = obj["fixtures"][0]
                categoryBits = 0

                filter = Box2D.b2Filter()
                #filter.set_categoryBits(categoryBits)
      
                if fixture["type"] == 'circle':
                    fix = body.CreateCircleFixture(radius=fixture["r"], density=fixture["density"], friction=obj["friction"])
                    fix.friction = obj["friction"]
                    fix.restitution = obj["restitution"]
                elif fixture["type"] == 'box':
                    fix = body.CreatePolygonFixture(box=(fixture["bounds"][0], fixture["bounds"][1]),
                                                    radius=fixture["r"],
                                                    density=fixture["density"])
                    fix.friction = obj["friction"]
                    fix.restitution = obj["restitution"]
                elif fixture["type"] == 'mesh':
                    mesh = fixture["mesh"]
                    for t in mesh["tris"]:
                        vert = [
                            b2Vec2(mesh["verts"][t["v"][0]][0], mesh["verts"][t["v"][0]][1]),
                            b2Vec2(mesh["verts"][t["v"][1]][0], mesh["verts"][t["v"][1]][1]),
                            b2Vec2(mesh["verts"][t["v"][2]][0], mesh["verts"][t["v"][2]][1])
                        ]
                        print(len(vert))
                        fix = body.CreatePolygonFixture(vertices=vert, density=fixture["density"])
                        fix.friction = obj["friction"]
                        fix.restitution = obj["restitution"]

            body.transform = b2Vec2(obj["pos"][0], obj["pos"][1]), obj["rot"]
            self.bodies[obj["name"]] = body
        
        for joint in model["joints"]:
            if "type" not in joint:
                print(joint)
                continue
            if joint["type"] == 'revolute':
                jd = b2RevoluteJointDef()
                jd.Initialize(self.bodies[joint["object1"]], self.bodies[joint["object2"]],
                              b2Vec2(joint["pos"][0], joint["pos"][1]))
                jd.collideConnected = False
                jd.enableLimit = False
                print(jd)
                if len(joint["limits"]) == 2:
                    jd.enableLimit = True
                    jd.lowerAngle = joint.limits[0]
                    jd.upperAngle = joint.limits[1]
                self.joints[joint["name"]] = self.world.CreateJoint(jd)
                print(self.joints[joint["name"]])
            elif joint["type"] == 'weld':
                jd = b2WeldJointDef()
                jd.Initialize(self.bodies[joint["object1"]], self.bodies[joint["object2"]],
                              b2Vec2(joint["pos"][0], joint["pos"][1]))
                jd.collideConnected = False
                self.joints[joint["name"]] = self.world.CreateJoint(jd)
                print(self.joints[joint["name"]])
            elif joint["type"] == 'prismatic':
                jd = b2PrismaticJointDef()
                jd.Initialize(self.bodies[joint["object1"]], self.bodies[joint["object2"]],
                              b2Vec2(joint["pos"][0], joint["pos"][1]),
                              b2Vec2(joint["axis"][0], joint["axis"][1]))
                jd.collideConnected = False
                jd.enableLimit = True
                jd.lowerTranslation = joint["limits"][0]
                jd.upperTranslation = joint["limits"][1]
                jd.damping = 20
                jd.stiffness = 2
                print(jd.lowerTranslation)
                print(jd.upperTranslation)
                self.joints[joint["name"]] = self.world.CreateJoint(jd)
            elif joint["type"] == 'spring':
                jd = b2DistanceJointDef()
                jd.Initialize(self.bodies[joint["object1"]], self.bodies[joint["object2"]],
                              b2Vec2(joint["pos"][0] - joint["axis"][0], joint["axis"][1] - joint["axis"][1]),
                              b2Vec2(joint["pos"][0] + joint["axis"][0], joint["pos"][1] + joint["axis"][1]))
                jd.damping = 20
                jd.stiffness = 1
                jd.collideConnected = False
            self.world.CreateJoint(jd)

  
