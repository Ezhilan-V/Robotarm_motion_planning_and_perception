"""Bootstrap ScriptNode — runs once on the first OnPlaybackTick to create the
four ROS2 bridges that ``setup_cameras.py`` used to create via ``--exec``:

  1. /Graph/ROS_Camera         — eye-in-hand RGB + depth + camera_info publishers
  2. /Graph/ROS_Clock          — /clock publisher (required for use_sim_time=true)
  3. /Graph/GraspAttachGraph   — subscribes /isaac_attach_cube, drives
                                  attach_detach_fixed_joint.py FixedJoint grasp.
  4. /Graph/ExternalCamera     — fixed overhead camera above the table (creates
                                  /World/ExternalCamera prim if missing) with its
                                  own /external_cam/* topic set. Lets cup_detector
                                  find the cup without needing the arm to move.

Packaged as a ScriptNode compute() entry point so it can be embedded in scene.usda
(/Graph/Bootstrap) and triggered by the existing Play button — no --exec flag needed.
"""
import asyncio
import carb
import omni.kit.app
import omni.usd
import omni.graph.core as og
from pxr import Sdf, UsdGeom, Gf

GRASP_SCRIPT_PATH = "/scene/omni_graph_script_node_usda/attach_detach_fixed_joint.py"
EXTERNAL_CAMERA_PATH = "/World/ExternalCamera"

# External camera placement, WORLD coordinates.
# Overhead view centered over both cups (x≈0.236) and bin (x≈-0.083), covering
# y range [-0.31, -0.118]. Camera at z=2.0 looks straight down (USD default
# Camera points along local -Z → world -Z with identity rotation).
# TF side (static_transform_publisher in bringup_full.launch.py) must match.
EXTERNAL_CAMERA_WORLD_TRANSLATE = (0.1, -0.2, 3.0)
EXTERNAL_CAMERA_WORLD_ROTATE_XYZ = (0.0, 0.0, 0.0)

_FIRED = False


def _graph_exists(path: str) -> bool:
    try:
        return bool(og.Controller.graph(path))
    except Exception:
        return False


def _create_camera_graph():
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/Graph/ROS_Camera", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick",      "omni.graph.action.OnPlaybackTick"),
                ("context",   "isaacsim.ros2.bridge.ROS2Context"),
                ("sim_time",  "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("rp_rgb",    "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("pub_rgb",   "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("pub_info",  "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("rp_depth",  "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("pub_depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick",                  "rp_rgb.inputs:execIn"),
                ("tick.outputs:tick",                  "rp_depth.inputs:execIn"),
                ("context.outputs:context",            "pub_rgb.inputs:context"),
                ("context.outputs:context",            "pub_info.inputs:context"),
                ("context.outputs:context",            "pub_depth.inputs:context"),
                ("rp_rgb.outputs:execOut",             "pub_rgb.inputs:execIn"),
                ("rp_rgb.outputs:execOut",             "pub_info.inputs:execIn"),
                ("rp_rgb.outputs:renderProductPath",   "pub_rgb.inputs:renderProductPath"),
                ("rp_rgb.outputs:renderProductPath",   "pub_info.inputs:renderProductPath"),
                ("rp_depth.outputs:execOut",           "pub_depth.inputs:execIn"),
                ("rp_depth.outputs:renderProductPath", "pub_depth.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                # cameraPrim is a rel (not an attribute) and `type` is a token --
                # both are set post-edit via USD API below.
                ("rp_rgb.inputs:width",        640),
                ("rp_rgb.inputs:height",       480),
                ("rp_depth.inputs:width",      640),
                ("rp_depth.inputs:height",     480),
                ("pub_rgb.inputs:topicName",   "/camera/color/image_raw"),
                ("pub_rgb.inputs:frameId",     "camera_color_optical_frame"),
                ("pub_info.inputs:topicName",  "/camera/color/camera_info"),
                ("pub_info.inputs:frameId",    "camera_color_optical_frame"),
                ("pub_depth.inputs:topicName", "/camera/depth/image_raw"),
                ("pub_depth.inputs:frameId",   "camera_depth_optical_frame"),
            ],
        },
    )

    # rel targets + token values that og.Controller.Keys.SET_VALUES can't set
    stage = omni.usd.get_context().get_stage()

    # Both rp_rgb and rp_depth point at the same Camera prim — depth is synthesized
    # by RTX from the same viewpoint so RGB and depth are pixel-aligned for perception.
    for rp_name, cam_path in [
        ("rp_rgb",   "/World/so101_new_calib/gripper_link/RGB_Camera"),
        ("rp_depth", "/World/so101_new_calib/gripper_link/RGB_Camera"),
    ]:
        rp_prim = stage.GetPrimAtPath(f"/Graph/ROS_Camera/{rp_name}")
        cam_prim = stage.GetPrimAtPath(cam_path)
        if not rp_prim or not rp_prim.IsValid():
            carb.log_error(f"[bootstrap_graphs] {rp_name} prim missing -- node not created?")
            continue
        if not cam_prim or not cam_prim.IsValid():
            carb.log_error(f"[bootstrap_graphs] Camera prim missing at {cam_path}")
            continue
        rel = rp_prim.CreateRelationship("inputs:cameraPrim")
        rel.SetTargets([Sdf.Path(cam_path)])
        got = rel.GetTargets()
        carb.log_warn(f"[bootstrap_graphs] {rp_name}.cameraPrim -> {list(got)}")

    # pub_info is ROS2CameraInfoHelper -- no `type` input; pub_rgb/pub_depth only.
    for node_name, type_value in [("pub_rgb", "rgb"), ("pub_depth", "depth")]:
        prim = stage.GetPrimAtPath(f"/Graph/ROS_Camera/{node_name}")
        attr = prim.GetAttribute("inputs:type")
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute("inputs:type", Sdf.ValueTypeNames.Token)
        attr.Set(type_value)


def _ensure_external_camera_prim(stage):
    """Create /World/ExternalCamera if missing. Idempotent — safe to call multiple times."""
    prim = stage.GetPrimAtPath(EXTERNAL_CAMERA_PATH)
    if prim and prim.IsValid() and prim.GetTypeName() == "Camera":
        return
    cam = UsdGeom.Camera.Define(stage, Sdf.Path(EXTERNAL_CAMERA_PATH))
    # RealSense-ish intrinsics; match the eye-in-hand camera so back-projection math
    # in cup_detector.py stays stable.
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))
    cam.CreateFocalLengthAttr(24.0)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(15.2908)
    cam.CreateProjectionAttr("perspective")
    # xform ops
    cam_prim = cam.GetPrim()
    xf = UsdGeom.Xformable(cam_prim)
    xf.ClearXformOpOrder()
    tx = xf.AddTranslateOp()
    tx.Set(Gf.Vec3d(*EXTERNAL_CAMERA_WORLD_TRANSLATE))
    rx = xf.AddRotateXYZOp()
    rx.Set(Gf.Vec3f(*EXTERNAL_CAMERA_WORLD_ROTATE_XYZ))
    carb.log_warn(
        f"[bootstrap_graphs] Created {EXTERNAL_CAMERA_PATH} at translate={EXTERNAL_CAMERA_WORLD_TRANSLATE}"
    )


def _create_external_camera_graph():
    """/Graph/ExternalCamera publishes /external_cam/color/image_raw, camera_info, and depth
    from the fixed overhead camera. Mirrors _create_camera_graph() but with different
    cameraPrim target + topic namespace."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/Graph/ExternalCamera", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick",      "omni.graph.action.OnPlaybackTick"),
                ("context",   "isaacsim.ros2.bridge.ROS2Context"),
                ("sim_time",  "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("rp_rgb",    "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("pub_rgb",   "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("pub_info",  "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("rp_depth",  "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("pub_depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick",                  "rp_rgb.inputs:execIn"),
                ("tick.outputs:tick",                  "rp_depth.inputs:execIn"),
                ("context.outputs:context",            "pub_rgb.inputs:context"),
                ("context.outputs:context",            "pub_info.inputs:context"),
                ("context.outputs:context",            "pub_depth.inputs:context"),
                ("rp_rgb.outputs:execOut",             "pub_rgb.inputs:execIn"),
                ("rp_rgb.outputs:execOut",             "pub_info.inputs:execIn"),
                ("rp_rgb.outputs:renderProductPath",   "pub_rgb.inputs:renderProductPath"),
                ("rp_rgb.outputs:renderProductPath",   "pub_info.inputs:renderProductPath"),
                ("rp_depth.outputs:execOut",           "pub_depth.inputs:execIn"),
                ("rp_depth.outputs:renderProductPath", "pub_depth.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("rp_rgb.inputs:width",        640),
                ("rp_rgb.inputs:height",       480),
                ("rp_depth.inputs:width",      640),
                ("rp_depth.inputs:height",     480),
                ("pub_rgb.inputs:topicName",   "/external_cam/color/image_raw"),
                ("pub_rgb.inputs:frameId",     "external_cam_optical_frame"),
                ("pub_info.inputs:topicName",  "/external_cam/color/camera_info"),
                ("pub_info.inputs:frameId",    "external_cam_optical_frame"),
                ("pub_depth.inputs:topicName", "/external_cam/depth/image_raw"),
                ("pub_depth.inputs:frameId",   "external_cam_optical_frame"),
            ],
        },
    )

    # Same post-edit USD tweaks as ROS_Camera: rel targets + token values.
    stage = omni.usd.get_context().get_stage()
    for rp_name in ("rp_rgb", "rp_depth"):
        rp_prim = stage.GetPrimAtPath(f"/Graph/ExternalCamera/{rp_name}")
        cam_prim = stage.GetPrimAtPath(EXTERNAL_CAMERA_PATH)
        if not rp_prim or not rp_prim.IsValid():
            carb.log_error(f"[bootstrap_graphs] ext {rp_name} prim missing")
            continue
        if not cam_prim or not cam_prim.IsValid():
            carb.log_error(f"[bootstrap_graphs] external camera prim missing at {EXTERNAL_CAMERA_PATH}")
            continue
        rel = rp_prim.CreateRelationship("inputs:cameraPrim")
        rel.SetTargets([Sdf.Path(EXTERNAL_CAMERA_PATH)])

    for node_name, type_value in [("pub_rgb", "rgb"), ("pub_depth", "depth")]:
        prim = stage.GetPrimAtPath(f"/Graph/ExternalCamera/{node_name}")
        attr = prim.GetAttribute("inputs:type")
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute("inputs:type", Sdf.ValueTypeNames.Token)
        attr.Set(type_value)


def _create_clock_graph():
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/Graph/ROS_Clock", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick",      "omni.graph.action.OnPlaybackTick"),
                ("context",   "isaacsim.ros2.bridge.ROS2Context"),
                ("sim_time",  "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("pub_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick",               "pub_clock.inputs:execIn"),
                ("context.outputs:context",         "pub_clock.inputs:context"),
                ("sim_time.outputs:simulationTime", "pub_clock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("pub_clock.inputs:topicName", "/clock"),
            ],
        },
    )


def _create_grasp_attach_graph():
    """Create GraspAttachGraph in two passes: (1) build nodes + set the Bool
    message type so the subscriber's outputs:data attribute gets materialized;
    (2) wire the connections. A single og.Controller.edit runs SET_VALUES AFTER
    CONNECT, which means sub_attach.outputs:data doesn't exist yet when the
    connection is attempted -- this splits the phases to avoid that race.
    """
    keys = og.Controller.Keys

    og.Controller.edit(
        {"graph_path": "/Graph/GraspAttachGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick",         "omni.graph.action.OnPlaybackTick"),
                ("context",      "isaacsim.ros2.bridge.ROS2Context"),
                ("sub_attach",   "isaacsim.ros2.bridge.ROS2Subscriber"),
                ("grasp_script", "omni.graph.scriptnode.ScriptNode"),
            ],
            keys.CREATE_ATTRIBUTES: [
                ("grasp_script.inputs:attach_cmd", "bool"),
            ],
            keys.SET_VALUES: [
                ("sub_attach.inputs:topicName",      "/isaac_attach_cube"),
                ("sub_attach.inputs:messageName",    "Bool"),
                ("sub_attach.inputs:messagePackage", "std_msgs"),
                ("grasp_script.inputs:usePath",      True),
                ("grasp_script.inputs:scriptPath",   GRASP_SCRIPT_PATH),
            ],
        },
    )

    base = "/Graph/GraspAttachGraph"
    og.Controller.connect(f"{base}/tick.outputs:tick",          f"{base}/sub_attach.inputs:execIn")
    og.Controller.connect(f"{base}/context.outputs:context",    f"{base}/sub_attach.inputs:context")
    og.Controller.connect(f"{base}/sub_attach.outputs:execOut", f"{base}/grasp_script.inputs:execIn")
    og.Controller.connect(f"{base}/sub_attach.outputs:data",    f"{base}/grasp_script.inputs:attach_cmd")


async def _ensure_all():
    app = omni.kit.app.get_app()
    for _ in range(60):
        await app.next_update_async()

    # External camera prim must exist before /Graph/ExternalCamera references it.
    stage = omni.usd.get_context().get_stage()
    try:
        _ensure_external_camera_prim(stage)
    except Exception as exc:
        carb.log_error(f"[bootstrap_graphs] ERROR creating ExternalCamera prim: {exc}")

    for label, graph_path, factory in [
        ("/Graph/ROS_Camera",       "/Graph/ROS_Camera",       _create_camera_graph),
        ("/Graph/ExternalCamera",   "/Graph/ExternalCamera",   _create_external_camera_graph),
        ("/Graph/ROS_Clock",        "/Graph/ROS_Clock",        _create_clock_graph),
        ("/Graph/GraspAttachGraph", "/Graph/GraspAttachGraph", _create_grasp_attach_graph),
    ]:
        if _graph_exists(graph_path):
            carb.log_warn(f"[bootstrap_graphs] {label} already present -- skipping.")
            continue
        carb.log_warn(f"[bootstrap_graphs] Creating {label} ...")
        try:
            factory()
            carb.log_warn(f"[bootstrap_graphs] {label} created successfully.")
        except Exception as exc:
            carb.log_error(f"[bootstrap_graphs] ERROR creating {label}: {exc}")


def setup(db):
    pass


def compute(db):
    global _FIRED
    if _FIRED:
        return True
    _FIRED = True
    asyncio.ensure_future(_ensure_all())
    return True


def cleanup(db):
    pass
