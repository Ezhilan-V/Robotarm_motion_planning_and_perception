import carb
import omni.usd
from pxr import Sdf, UsdPhysics, Gf

# -----------------------------
# CONFIG
# -----------------------------

# Cup rigid body (Body1)
CUP_BODY_PATH_STR = "/World/SM_Mug_A2_red/SM_Mug_A2"

# Jaw rigid body (Body0)
JAW_BODY_PATH_STR = "/World/so101_new_calib/moving_jaw_so101_v1_link"

# Where joint will live
JOINTS_XFORM_STR = "/World/so101_new_calib/GraspJoints"
JOINT_PATH_STR   = "/World/so101_new_calib/GraspJoints/cup_grasp_fixed_joint"

# Grasp frame on the jaw (jaw-local)
GRASP_POS0 = Gf.Vec3d(-0.01962, -0.04449, 0.02761)

# Rotation handling (pick one mode)
GRASP_ROT_IS_QUAT = False
GRASP_QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)

GRASP_AXIS = Gf.Vec3d(0.0, 1.0, 0.0)
GRASP_DEG  = 0.0

# Internal state
_ATTACHED = False

# -----------------------------
# HELPERS
# -----------------------------

def _stage():
    return omni.usd.get_context().get_stage()


def _valid(stage, p: str) -> bool:
    prim = stage.GetPrimAtPath(p) if stage else None
    return bool(prim) and prim.IsValid()


def _ensure_container(stage):
    if not _valid(stage, JOINTS_XFORM_STR):
        stage.DefinePrim(JOINTS_XFORM_STR, "Xform")


def _quatd_to_quatf(qd: Gf.Quatd) -> Gf.Quatf:
    im = qd.GetImaginary()
    return Gf.Quatf(float(qd.GetReal()),
                    Gf.Vec3f(float(im[0]), float(im[1]), float(im[2])))


def _make_grasp_rot_quatd() -> Gf.Quatd:
    if GRASP_ROT_IS_QUAT:
        w, x, y, z = GRASP_QUAT_WXYZ
        return Gf.Quatd(float(w), Gf.Vec3d(float(x), float(y), float(z)))
    # axis-angle
    axis = GRASP_AXIS
    if axis.GetLength() < 1e-9:
        axis = Gf.Vec3d(0.0, 1.0, 0.0)
    else:
        axis = axis / axis.GetLength()
    rot = Gf.Rotation(axis, float(GRASP_DEG))
    return rot.GetQuat()


def _xf_from_tr_quat(t: Gf.Vec3d, q: Gf.Quatd) -> Gf.Matrix4d:
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(q))
    m.SetTranslate(t)
    return m

# -----------------------------
# ATTACH / DETACH
# -----------------------------

def _attach(stage):
    _ensure_container(stage)

    jaw_prim  = stage.GetPrimAtPath(JAW_BODY_PATH_STR)
    cup_prim  = stage.GetPrimAtPath(CUP_BODY_PATH_STR)

    if not jaw_prim.IsValid() or not cup_prim.IsValid():
        carb.log_error("Jaw body or cup prim missing/invalid")
        return False

    if _valid(stage, JOINT_PATH_STR):
        return True

    # World transforms
    jaw_xf = omni.usd.get_world_transform_matrix(jaw_prim)
    cup_xf = omni.usd.get_world_transform_matrix(cup_prim)

    # Build the joint frame in JAW LOCAL
    grasp_q0 = _make_grasp_rot_quatd()
    joint_in_jaw = _xf_from_tr_quat(GRASP_POS0, grasp_q0)

    # Convert to WORLD
    J_world = jaw_xf * joint_in_jaw

    # Compute joint frame in each body's local space
    local0 = J_world * jaw_xf.GetInverse()
    local1 = J_world * cup_xf.GetInverse()

    t0 = local0.ExtractTranslation()
    q0 = local0.ExtractRotationQuat()
    t1 = local1.ExtractTranslation()
    q1 = local1.ExtractRotationQuat()

    # Create FixedJoint
    joint = UsdPhysics.FixedJoint.Define(stage, Sdf.Path(JOINT_PATH_STR))
    joint.CreateBody0Rel().SetTargets([Sdf.Path(JAW_BODY_PATH_STR)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(CUP_BODY_PATH_STR)])

    joint.CreateLocalPos0Attr(Gf.Vec3f(float(t0[0]), float(t0[1]), float(t0[2])))
    joint.CreateLocalRot0Attr(_quatd_to_quatf(q0))

    joint.CreateLocalPos1Attr(Gf.Vec3f(float(t1[0]), float(t1[1]), float(t1[2])))
    joint.CreateLocalRot1Attr(_quatd_to_quatf(q1))

    carb.log_warn("✅ ATTACH: Fixed joint created at jaw grasp frame for cup")
    return True


def _detach(stage):
    if not _valid(stage, JOINT_PATH_STR):
        return True
    stage.RemovePrim(Sdf.Path(JOINT_PATH_STR))
    carb.log_warn(f"✅ DETACH: Removed fixed joint {JOINT_PATH_STR}")
    return True

# -----------------------------
# SCRIPT NODE ENTRY POINT
# -----------------------------

def _read_attach_cmd(db) -> bool:
    for name in ("attach_cmd", "data", "value"):
        try:
            return bool(getattr(db.inputs, name))
        except Exception:
            pass
    carb.log_error(
        "Missing bool input. Add Script Node bool input 'attach_cmd' and wire Subscriber.data -> attach_cmd"
    )
    return False


def compute(db):
    global _ATTACHED

    stage = _stage()
    if stage is None:
        carb.log_error("No USD stage")
        return True

    attach_cmd = _read_attach_cmd(db)

    if attach_cmd and not _ATTACHED:
        if _attach(stage):
            _ATTACHED = True
    elif (not attach_cmd) and _ATTACHED:
        if _detach(stage):
            _ATTACHED = False

    return True
