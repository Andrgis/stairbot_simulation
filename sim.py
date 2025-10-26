import agx_helper
import agx
import agxSDK
import agxRender
import agxCollide
import math

from stairbot_V1 import StairBot, MyKeyboardEvent

if __name__ == "__main__":
    def build_scene():
        app = agx_helper.AgxApp(num_threads=6)
        stairbot = StairBot(app)
        app.add(stairbot)
        stairbot.setPosition(0, 0, 0.6)
        plane_body = agx.RigidBody(agxCollide.Geometry(agxCollide.Box(1, 1, 0.1), agx.AffineMatrix4x4.translate(0, 0, -0.1/2)))
        plane_body.setMotionControl(agx.RigidBody.STATIC)
        app.create_visual(plane_body, diffuse_color=agxRender.Color.Green())
        app.add(plane_body)
        app.init_camera(eye=agx.Vec3(-1, -1, 0.5), center=stairbot.getPosition())
        app.add(MyKeyboardEvent(stairbot))