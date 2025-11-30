import agxUtil

import agx_helper
import agx
import agxSDK
import agxRender
import agxCollide
import math


backwheels_offset = 0.00
chassis_shape = agx_helper.load_shape('objects/simbot_simbot v1_chasis_1_Body1.obj')
bogie_l_shape = agx_helper.load_shape('objects/simbot_simbot v1_bogie_1_Body1.obj')
bogie_r_shape = agx_helper.load_shape('objects/simbot_simbot v1_bogie_r_1_Body1.obj')
rocker_l_shape = agx_helper.load_shape('objects/simbot_simbot v1_rocker_1_Body1.obj')
rocker_r_shape = agx_helper.load_shape('objects/simbot_simbot v1_rocker_r_1_Body1.obj')
wheel_lm_shape = agx_helper.load_shape('objects/simbot_simbot v1_wheel_1_Body1.obj')
wheel_lf_shape = agx_helper.load_shape('objects/simbot_simbot v1_wheel_2_Body1.obj')
wheel_lb_shape = agx_helper.load_shape('objects/simbot_simbot v1_wheel_3_Body1.obj')
wheel_rm_shape = agx_helper.load_shape('objects/simbot_simbot v1_wheel_r_1_Body1.obj')
wheel_rf_shape = agx_helper.load_shape('objects/simbot_simbot v1_wheel_r_2_Body1.obj')
wheel_rb_shape = agx_helper.load_shape('objects/simbot_simbot v1_wheel_r_3_Body1.obj')
wheelflap_lm_shape = agx_helper.load_shape('objects/simbot_simbot v1_wheel flap_1_Body1.obj')
wheelflap_lf_shape = agx_helper.load_shape('objects/simbot_simbot v1_wheel flap_2_Body1.obj')
wheelflap_rm_shape = agx_helper.load_shape('objects/simbot_simbot v1_wheel flap_r_2_Body1.obj')
wheelflap_rf_shape = agx_helper.load_shape('objects/simbot_simbot v1_wheel flap_r_3_Body1.obj')
bogies = (bogie_l_shape, bogie_r_shape)
rockers = (rocker_l_shape, rocker_r_shape)
wheels = (wheel_lm_shape, wheel_lf_shape, wheel_lb_shape, wheel_rm_shape, wheel_rf_shape, wheel_rb_shape)
wheelflaps = (wheelflap_lm_shape, wheelflap_lf_shape, wheelflap_rm_shape, wheelflap_rf_shape)

# joint positions:
w_pos = (agx.Vec3(-64+44+5, 255, 55),
        agx.Vec3(-64+44+5, 55, 55),
        agx.Vec3(-64, 455+backwheels_offset*1000, 55),
        agx.Vec3(-255, 255, 55),
        agx.Vec3(-255, 55, 55),
        agx.Vec3(-206, 455+backwheels_offset*1000, 55))

s_pos = (agx.Vec3(39, 255, 144.4-2),
        agx.Vec3(-39, 55, 144.4-2),
        agx.Vec3(-231, 255, 144.4-2),
        agx.Vec3(-231, 55, 144.4-2))

b_pos = (agx.Vec3(-51-8, 155, 95),
        agx.Vec3(-203-8, 155, 95))

r_pos = (agx.Vec3(-73, 365, 154),
        agx.Vec3(-197, 365, 154))

for pos in [w_pos,s_pos,b_pos,r_pos]:
    for p in pos:
        p *= 0.001  # mm -> meters

class StairBot(agxSDK.Assembly):
    def __init__(self, app):
        super().__init__()
        self.app = app

        self.bodies = {"chassis":[],
                       "bogies":[],
                       "wheels":[],
                       "wheelflaps":[],
                       "rockers":[]}

        self.joints = {"bogies": [],
                       "rockers": [],
                       "wheels": [],
                       "wheelflaps": []}

        self.spawn_rover_geometry()
        self.get_mass()
        self.motorize_wheels()
        self.add_to_sim()

    def spawn_rover_geometry(self):

        material = agx.Material('material')
        self.app.add(material)
        material.getBulkMaterial().setDensity(4000)

        body_shapes = [[chassis_shape], bogies, wheels, wheelflaps]
        colors = [agxRender.Color.Blue(), agxRender.Color.Red(), agxRender.Color.Black(), agxRender.Color.Green()]
        i = 0
        for shapes in body_shapes:
            for shape in shapes:
                body = agx.RigidBody(agxCollide.Geometry(shape.deepCopy()))
                self.app.create_visual(body, colors[i])
                self.bodies[list(self.bodies.keys())[i]].append(body)
            i+=1

        agxUtil.setBodyMaterial(self.bodies["chassis"][0], material)
        self.make_rockers(backwheels_offset)

        #for b in self.bodies["rockers"]:
            #b.setRotation(b.getRotation() + agx.Quat(0.7, 0.7, 0.0, 0.0))
        #    b.setPosition(b.getPosition() + agx.Vec3(0.0, 0.0, 0.1))


        #Offset back wheels
        for b in [self.bodies["wheels"][2], self.bodies["wheels"][5]]:
            b.setPosition(b.getPosition() + agx.Vec3(0.0, backwheels_offset, 0.0))

        # specify masses
        for b in self.bodies["rockers"] + self.bodies["chassis"]:
            b.setCmLocalTranslate(agx.Vec3(0, -0.05, 0))

        # Joints
        jr0 = agx_helper.create_constraint(pos=r_pos[0],
                                                axis=agx.Vec3(-1, 0, 0),
                                                rb1=self.bodies["chassis"][0],
                                                rb2=self.bodies["rockers"][0], c=agx.Hinge)
        jr1 = agx_helper.create_constraint(pos=r_pos[1],
                                                axis=agx.Vec3(-1, 0, 0),
                                                rb1=self.bodies["chassis"][0],
                                                rb2=self.bodies["rockers"][1], c=agx.Hinge)
        self.joints["rockers"] += [jr0, jr1]
        jb0 = agx_helper.create_constraint(pos=b_pos[0],
                                             axis=agx.Vec3(-1, 0, 0),
                                             rb1=self.bodies["rockers"][0],
                                             rb2=self.bodies["bogies"][0], c=agx.Hinge)
        jb1 = agx_helper.create_constraint(pos=b_pos[1],
                                                axis=agx.Vec3(-1, 0, 0),
                                                rb1=self.bodies["rockers"][1],
                                                rb2=self.bodies["bogies"][1], c=agx.Hinge)
        self.joints["bogies"] += [jb0, jb1]
        # Wheel joints
        wj0 = agx_helper.create_constraint(pos=w_pos[0],
                                                axis=agx.Vec3(-1, 0, 0),
                                                rb1=self.bodies["wheelflaps"][0],
                                                rb2=self.bodies["wheels"][0], c=agx.Hinge)
        wj1 = agx_helper.create_constraint(pos=w_pos[1],
                                                axis=agx.Vec3(-1, 0, 0),
                                                rb1=self.bodies["wheelflaps"][1],
                                                rb2=self.bodies["wheels"][1], c=agx.Hinge)
        wj2 = agx_helper.create_constraint(pos=w_pos[2],
                                                axis=agx.Vec3(-1, 0, 0),
                                                rb1=self.bodies["rockers"][0],
                                                rb2=self.bodies["wheels"][2], c=agx.Hinge)
        wj3 = agx_helper.create_constraint(pos=w_pos[3],
                                                axis=agx.Vec3(-1, 0, 0),
                                                rb1=self.bodies["wheelflaps"][2],
                                                rb2=self.bodies["wheels"][3], c=agx.Hinge)
        wj4 = agx_helper.create_constraint(pos=w_pos[4],
                                                axis=agx.Vec3(-1, 0, 0),
                                                rb1=self.bodies["wheelflaps"][3],
                                                rb2=self.bodies["wheels"][4], c=agx.Hinge)
        wj5 = agx_helper.create_constraint(pos=w_pos[5],
                                                axis=agx.Vec3(-1, 0, 0),
                                                rb1=self.bodies["rockers"][1],
                                                rb2=self.bodies["wheels"][5], c=agx.Hinge)
        self.joints["wheels"] += [wj0, wj1, wj2, wj3, wj4, wj5]

        sj0 = agx_helper.create_constraint(pos=s_pos[0],
                                                axis=agx.Vec3(0, 0, 1),
                                                rb1=self.bodies["bogies"][0],
                                                rb2=self.bodies["wheelflaps"][0], c=agx.LockJoint)
        sj1 = agx_helper.create_constraint(pos=s_pos[1],
                                                axis=agx.Vec3(0, 0, 1),
                                                rb1=self.bodies["bogies"][0],
                                                rb2=self.bodies["wheelflaps"][1], c=agx.Hinge)
        sj2 = agx_helper.create_constraint(pos=s_pos[2],
                                                axis=agx.Vec3(0, 0, 1),
                                                rb1=self.bodies["bogies"][1],
                                                rb2=self.bodies["wheelflaps"][2], c=agx.LockJoint)
        sj3 = agx_helper.create_constraint(pos=s_pos[3],
                                                axis=agx.Vec3(0, 0, 1),
                                                rb1=self.bodies["bogies"][1],
                                                rb2=self.bodies["wheelflaps"][3], c=agx.Hinge)
        self.add(sj0)
        self.add(sj2)
        self.joints["wheelflaps"] += [sj1, sj3] #[sj0, sj1, sj2, sj3]

        for j in self.joints["bogies"] + self.joints["rockers"]:
            j.getRange1D().setEnable(True)
            j.getRange1D().setRange(agx.RangeReal(-3.14 / 4, 3.14 / 4))

    def motorize_wheels(self):
        self.servo_pos = 0.0
        for j in self.joints["wheels"]:
            j.setEnableComputeForces(False)
            j.setCompliance(1E-12)
            j.getMotor1D().setCompliance(1E-10)
            j.getMotor1D().setEnable(True)
            j.getMotor1D().setSpeed(0.0)
            j.getMotor1D().setForceRange(-2.0, 2.0)
            j.getLock1D().setEnable(False)

        for s in self.joints["wheelflaps"]:
            s.setCompliance(1E-12)
            s.getLock1D().setCompliance(1E-10)
            s.getMotor1D().setEnable(False)
            s.getLock1D().setPosition(self.servo_pos)
            s.getLock1D().setEnable(True)

    def add_to_sim(self):
        for joint_group in self.joints.values():
            for joint in joint_group:
                self.add(joint)
        for body_group in self.bodies.values():
            for body in body_group:
                self.add(body)

    def get_mass(self):
        for bodies in self.bodies.keys():
            print(bodies)
            for body in self.bodies[bodies]:
                print(f"{body.calculateMass():.3f} kg")

    def make_rockers(self, offset):
        w_r = [-9, 90 + offset*1000, -99]
        b_r = [-14, -210, -59]
        bog_roc = math.sqrt(47777)*0.001*1.05
        w_roc = math.sqrt(9 ** 2 + (90 + offset*1000) ** 2 + 99 ** 2) * 0.001*1.05

        af = math.atan2(b_r[2], b_r[1])
        ab = math.atan2(w_r[2], w_r[1])

        rocker1 = agx.RigidBody()
        rocker2 = agx.RigidBody()
        rocf_shape = agxCollide.Box(0.001, bog_roc*0.5, 0.01)
        rocb_shape = agxCollide.Box(0.001, w_roc*0.5, 0.01)
        rocf_geom = agxCollide.Geometry(rocf_shape)
        rocb_geom = agxCollide.Geometry(rocb_shape)
        rocf_shape2 = agxCollide.Box(0.001, bog_roc * 0.5, 0.01)
        rocb_shape2 = agxCollide.Box(0.001, w_roc * 0.5, 0.01)
        rocf_geom2 = agxCollide.Geometry(rocf_shape2)
        rocb_geom2 = agxCollide.Geometry(rocb_shape2)

        rocf_geom.setPosition(b_pos[1] + (r_pos[1]-b_pos[1])*0.5)
        rocf_geom.setRotation( agx.EulerAngles(af, 0, 0) )
        rocb_geom.setPosition(w_pos[5] + (r_pos[1] - w_pos[5]) * 0.5)
        rocb_geom.setRotation(agx.EulerAngles(ab, 0, 0))

        rocf_geom2.setPosition(b_pos[0] + (r_pos[0] - b_pos[0]) * 0.5)
        rocf_geom2.setRotation(agx.EulerAngles(af, 0, 0))
        rocb_geom2.setPosition(w_pos[2] + (r_pos[0] - w_pos[2]) * 0.5)
        rocb_geom2.setRotation(agx.EulerAngles(ab, 0, 0))

        rocker1.add(rocf_geom)
        rocker1.add(rocb_geom)
        rocker2.add(rocf_geom2)
        rocker2.add(rocb_geom2)

        self.app.create_visual(rocker1, agxRender.Color.Yellow())
        self.app.create_visual(rocker2, agxRender.Color.Yellow())

        self.bodies["rockers"] += [rocker2, rocker1]


class MyKeyboardEvent(agxSDK.GuiEventListener):
    def __init__(self, object: StairBot):
        super().__init__()
        self.obj = object
        self.v = 0.0

    def keyboard(self, key, alt, x, y, down):
        handled = False
        if key == 65362: #up
            self.v+=0.5
            for wheel in self.obj.joints["wheels"]:
                wheel.getMotor1D().setSpeed(self.v)
            handled = True
        elif key == 65364: #down
            self.v-=0.5
            for wheel in self.obj.joints["wheels"]:
                wheel.getMotor1D().setSpeed(self.v)
            handled = True
        elif key == 65361: #left
            for servo in self.obj.joints["wheelflaps"]:
                self.obj.servo_pos -=0.1
                servo.getLock1D().setPosition(self.obj.servo_pos)
            handled = True
        elif key == 65363: #right
            for servo in self.obj.joints["wheelflaps"]:
                self.obj.servo_pos +=0.1
                servo.getLock1D().setPosition(self.obj.servo_pos)
            handled = True
        print(f"AngVel: {self.v} rads/s, Steer: {self.obj.servo_pos} rads")
        return handled

class TorqueLogger(agxSDK.StepEventListener):
    def __init__(self, assembly):
        super().__init__()
        self.assembly = assembly
        self.i = 0
        self.prevTime = 0
        self.prevAngMom = [x.getAngularMomentum()[0] for x in self.assembly.bodies["wheels"]]

    def post(self, tt):
        self.increment()
        #if self.i%1==0:
        self.get_torque(tt)

    def get_torque(self, time):
        AngMom = [x.getAngularMomentum()[0] for x in self.assembly.bodies["wheels"]]
        dt = time - self.prevTime
        for i in range(len(AngMom)):
            torque = (AngMom[i]-self.prevAngMom[i])/dt
            print(f"Wheel {i}: {torque:.6f} Nm")
        self.prevTime = time
        self.prevAngMom = AngMom

class MotorTorqueLogger(agxSDK.StepEventListener):
    def __init__(self, app, assembly):
        super().__init__()
        self.assembly = assembly
        self.app = app
        self.i = 0

    def post(self, tt):
        self.increment()
        if self.i%10==0:
            self.get_torque()
            for i in range(len(self.torques)):
                self.app.app.getSceneDecorator().setText(i, "Torque {}: {:3.2f} Ncm".format(i, self.torques[i]*100))

    def get_torque(self):
        self.torques = []
        for wheel in self.assembly.joints["wheels"]:
            # Get the current force and torque.
            force = agx.Vec3()
            torque = agx.Vec3()
            wheel.getLastForce(0, force, torque)
            self.torques.append(torque[0])

    def increment(self):
        self.i += 1


class ForceLogger(agxSDK.StepEventListener):
    def __init__(self, assembly):
        super().__init__()
        self.assembly = assembly
        self.i = 0

    def post(self, tt):
        self.increment()
        if self.i%5==0:
            self.get_force()

    def get_force(self):
        for bodies in self.assembly.bodies.keys():
            print(bodies)
            for body in self.assembly.bodies[bodies]:
                print(f"{body.getAngularMomentum():.4f} N")

    def increment(self):
        self.i += 1
