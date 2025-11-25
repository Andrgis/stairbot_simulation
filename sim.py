import agx_helper
import agx
import agxSDK
import agxRender
import agxCollide
import agxUtil
import math

from stairbot_V1 import StairBot, MyKeyboardEvent, MotorTorqueLogger, ForceLogger


class SimulationEnvironment(agxSDK.Assembly):
    def __init__(self, app):
        super().__init__()
        self.app = app
        self.stairs = []  # Store stairs

        self.plane_body = agx.RigidBody(
            agxCollide.Geometry(agxCollide.Box(1.5, 1.5, -0.1), agx.AffineMatrix4x4.translate(0, 0, -0.1 / 2)))
        self.plane_body.setMotionControl(agx.RigidBody.STATIC)
        app.create_visual(self.plane_body, diffuse_color=agxRender.Color.Gray())
        app.add(self.plane_body)

    def add_stairs(self, height: float, depth: float, num_stairs: int):
        for i in range(num_stairs + 1):
            stair = agx.RigidBody(agxCollide.Geometry(agxCollide.Box(1.0, depth, height * (i + 1))))
            stair.setMotionControl(agx.RigidBody.STATIC)
            stair.setPosition(0, -0.5 - depth * i, 0.05 + height * (i + 1))
            self.stairs.append(stair)  # Store the stair
            self.app.create_visual(stair, diffuse_color=agxRender.Color.LightGray())
            self.app.add(stair)


class FrictionSetter:
    def __init__(self, app, rb1: StairBot, bodies_with_material2, coeff):
        self.app = app

        material1 = agx.Material('material1')
        material2 = agx.Material('material2')
        app.add(material1)
        app.add(material2)

        contact_material = agx.ContactMaterial(material1, material2)
        app.add(contact_material)

        frictionModel = agx.ScaleBoxFrictionModel()
        frictionModel.setSolveType(agx.FrictionModel.DIRECT)
        contact_material.setFrictionModel(frictionModel)

        contact_material.setYoungsModulus(0.1E9)
        contact_material.setRestitution(0.6)
        contact_material.setFrictionCoefficient(coeff)

        for wheel_body in rb1.bodies["wheels"]:
            agxUtil.setBodyMaterial(wheel_body, material1)

        # Apply material2 to all provided bodies
        for body in bodies_with_material2:
            agxUtil.setBodyMaterial(body, material2)


if __name__ == "__main__":
    def build_scene():
        app = agx_helper.AgxApp(num_threads=6)
        stairbot = StairBot(app)
        app.add(stairbot)
        stairbot.setPosition(-0.1, 0.1, 0.2)
        environment = SimulationEnvironment(app)
        environment.add_stairs(height=0.075, depth=0.28, num_stairs=3)

        # Pass both plane and stairs to FrictionSetter
        FrictionSetter(app, stairbot, [environment.plane_body] + environment.stairs, 0.8)

        app.init_camera(eye=agx.Vec3(1, 1-0.3, 0.5), center=stairbot.getPosition()+agx.Vec3(0.0, -0.3, 0.0))
        app.add(MotorTorqueLogger(app, stairbot))
        app.add(MyKeyboardEvent(stairbot))
        app.set_simulation_dt(0.0035)