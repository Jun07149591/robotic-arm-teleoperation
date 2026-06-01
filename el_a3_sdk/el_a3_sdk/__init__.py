from el_a3_sdk.data_types import (
    MotorFeedback,
    ArmStatus,
    ArmJointStates,
    ArmEndPose,
    MotorHighSpdInfo,
    MotorLowSpdInfo,
    MotorAngleLimitMaxVel,
    DynamicsInfo,
    TrajectoryResult,
)
from el_a3_sdk.protocol import (
    MotorType,
    RunMode,
    ControlMode,
    MoveMode,
    ArmState,
    LogLevel,
)
from el_a3_sdk.realsense import (
    CameraIntrinsics,
    BookSpineMatchConfig,
    BookSpineMatcher,
    BookSpineMatchResult,
    CenterAlignment,
    DepthUnit,
    FeatureBackend,
    PointCloud,
    RGBDFrame,
    RealSenseD435,
    RigidTransform,
    ColorOrder,
    draw_book_spine_overlay,
    evaluate_center_alignment,
    import_open3d,
    import_pyrealsense2,
    load_template_image,
    make_display_cloud,
    match_target,
    match_with_fallback,
    pick_point,
    pick_point_index,
    pick_target,
    point_cloud_to_open3d,
    prepare_view_cloud,
    polygon_to_bbox,
    polygon_to_roi,
    resize_keep_aspect,
    rpy_to_matrix,
    save_selected_point,
    save_selected_target,
    save_point_cloud,
    update_stable_state,
    SelectedPoint,
    SelectedTarget,
)

__version__ = "1.0.0"


def __getattr__(name):
    if name == "ELA3Interface":
        from el_a3_sdk.interface import ELA3Interface

        return ELA3Interface
    if name == "ArmManager":
        from el_a3_sdk.arm_manager import ArmManager

        return ArmManager
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def get_kinematics():
    """延迟导入 ELA3Kinematics（避免无 pinocchio 环境下 import 失败）"""
    from el_a3_sdk.kinematics import ELA3Kinematics
    return ELA3Kinematics


__all__ = [
    "ELA3Interface",
    "ArmManager",
    "get_kinematics",
    "MotorFeedback",
    "ArmStatus",
    "ArmJointStates",
    "ArmEndPose",
    "MotorHighSpdInfo",
    "MotorLowSpdInfo",
    "MotorAngleLimitMaxVel",
    "DynamicsInfo",
    "TrajectoryResult",
    "MotorType",
    "RunMode",
    "ControlMode",
    "MoveMode",
    "ArmState",
    "LogLevel",
    "CameraIntrinsics",
    "BookSpineMatchConfig",
    "BookSpineMatcher",
    "BookSpineMatchResult",
    "CenterAlignment",
    "ColorOrder",
    "DepthUnit",
    "FeatureBackend",
    "PointCloud",
    "RGBDFrame",
    "RealSenseD435",
    "RigidTransform",
    "draw_book_spine_overlay",
    "evaluate_center_alignment",
    "import_open3d",
    "import_pyrealsense2",
    "load_template_image",
    "make_display_cloud",
    "match_target",
    "match_with_fallback",
    "pick_point",
    "pick_point_index",
    "pick_target",
    "point_cloud_to_open3d",
    "prepare_view_cloud",
    "polygon_to_bbox",
    "polygon_to_roi",
    "resize_keep_aspect",
    "rpy_to_matrix",
    "save_selected_point",
    "save_selected_target",
    "save_point_cloud",
    "update_stable_state",
    "SelectedPoint",
    "SelectedTarget",
]
