from pxr import Usd, UsdGeom
import omni.usd
import omni.graph as og

stage = omni.usd.get_context().get_stage()

ogn = og.core.get_node_by_path("/World/Cube_01/SurfaceGripperActionGraph/on_impulse_event")

attr = ogn.get_attribute("state:enableImpulse")
attr.set_value(True)
ogn.request_compute()