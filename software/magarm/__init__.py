"""MaGaRM: 6-DOF arm + gripper on Damiao and RobStride motors (CAN, MIT control)."""
from .arm import Arm
from .config import JOINTS

__all__ = ['Arm', 'JOINTS']
