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


class FastIntersection:
    def __init__(self, ground):
        self.indexes = collections.defaultdict(set)
        self.sc = 10
        for i in range(0, len(ground.fixtures)):
            f = ground.fixtures[i]
            for v in f.shape.vertices:
                self.indexes[int(v[0] * self.sc)].add(i)

    def set_body(self, ground):
        self.body = ground
        
    def intersect_ray(self, pos):
        input = b2RayCastInput(p1=pos, p2=b2Vec2(pos.x, pos.y-2))
        input.maxFraction = 1.0
 
        output = b2RayCastOutput()

        for i in self.indexes[int(pos.x * self.sc)]:
            hit = self.body.fixtures[i].RayCast(input=input, output=output, childIndex=0)
            if hit:
                return output.fraction
        return 2


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

        groundBody.CreateFixture(shape=b2EdgeShape(vertices=[b2Vec2(-100.0, -100), b2Vec2(100.0, -100)]))
        self.bodies = {"ground": groundBody}
        self.edges = {}

    def load(self, model):
        for i in range(0, len(model["objects"])):
            obj = model["objects"][i]
            # print("body %d, %s" % (i, obj["name"]))

            if "fixtures" not in obj or len(obj["fixtures"]) != 1:
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

                    external_edges = {}
                    nverts = len(mesh["verts"])
                    for t in mesh["tris"]:
                        for i in range(0, 3):
                            v1 = t["v"][i]
                            v2 = t["v"][(i + 1) % 3]
                            external_edges[v1 * nverts + v2] = 1
                    edges = []
                    for t in mesh["tris"]:
                        for i in range(0, 3):
                            v1 = t["v"][i]
                            v2 = t["v"][(i + 1) % 3]
                            if (v2 * nverts + v1) not in external_edges:
                                vertices = [b2Vec2(mesh["verts"][v1][0:2]),
                                            b2Vec2(mesh["verts"][v2][0:2])]
                                edges.append(vertices)
                    self.edges[obj["name"]] = edges
                    for t in mesh["tris"]:
                        vert = [
                            b2Vec2(mesh["verts"][t["v"][0]][0], mesh["verts"][t["v"][0]][1]),
                            b2Vec2(mesh["verts"][t["v"][1]][0], mesh["verts"][t["v"][1]][1]),
                            b2Vec2(mesh["verts"][t["v"][2]][0], mesh["verts"][t["v"][2]][1])
                        ]
                        fix = body.CreatePolygonFixture(vertices=vert, density=fixture["density"])
                        fix.friction = obj["friction"]
                        fix.restitution = obj["restitution"]

            body.transform = b2Vec2(obj["pos"][0], obj["pos"][1]), obj["rot"]
            self.bodies[obj["name"]] = body
        
        for joint in model["joints"]:
            if "type" not in joint:
                continue
            if joint["type"] == 'revolute':
                jd = b2RevoluteJointDef()
                jd.Initialize(self.bodies[joint["object1"]], self.bodies[joint["object2"]],
                              b2Vec2(joint["pos"][0], joint["pos"][1]))
                jd.collideConnected = False
                jd.enableLimit = False
                if len(joint["limits"]) == 2:
                    jd.enableLimit = True
                    jd.lowerAngle = joint.limits[0]
                    jd.upperAngle = joint.limits[1]
                self.joints[joint["name"]] = self.world.CreateJoint(jd)

            elif joint["type"] == 'weld':
                jd = b2WeldJointDef()
                jd.Initialize(self.bodies[joint["object1"]], self.bodies[joint["object2"]],
                              b2Vec2(joint["pos"][0], joint["pos"][1]))
                jd.collideConnected = False
                self.joints[joint["name"]] = self.world.CreateJoint(jd)

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

  
