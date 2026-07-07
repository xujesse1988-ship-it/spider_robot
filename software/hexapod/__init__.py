"""爬墙六足机器人控制包（树莓派 5 大脑）。"""
from .config import RobotConfig, DEFAULT_CONFIG, LEG_NAMES
from .driver import Servo2040Driver, MockDriver
from .gait import GaitEngine, TRIPOD, WAVE, CLIMB
from .kinematics import leg_ik, leg_fk, WorkspaceError
from .robot import Hexapod
from .adhesion import AdhesionController, MockVacuumIO, FootState

__all__ = [
    "RobotConfig", "DEFAULT_CONFIG", "LEG_NAMES",
    "Servo2040Driver", "MockDriver",
    "GaitEngine", "TRIPOD", "WAVE", "CLIMB",
    "leg_ik", "leg_fk", "WorkspaceError",
    "Hexapod",
    "AdhesionController", "MockVacuumIO", "FootState",
]
