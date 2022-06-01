"""
Simple Dynamical System for Generating the Correct Dynamics
"""
# Author: Lukas Huber
# Created: 2022-05-31
# Github: hubernikus
from dataclasses import dataclass

from scipy.spatial.transform import Rotation
import numpy as np
from numpy import linalg as LA


@dataclass
class AngularDynamics:
    target_orientation: Rotation
    gain: float = 1
    clamp: float = 0.25

    def evaluate(self, orientation: Rotation) -> np.ndarray:
        diff_vec = (self.target_orientation * orientation.inv()).as_rotvec()
        diff_vec *= self.gain

        if self.clamp is not None:
            diff_norm = LA.norm(diff_vec)

            if diff_norm > self.clamp:
                diff_vec *= self.clamp / diff_norm

        return diff_vec


@dataclass
class LinearDynamics:
    target_position: np.ndarray
    gain: float = 1
    clamp: float = 0.25

    def evaluate(self, position: np.ndarray) -> np.ndarray:
        diff_vec = (self.target_position - position)
        diff_vec *= self.gain

        if self.clamp is not None:
            diff_norm = LA.norm(diff_vec)

            if diff_norm > self.clamp:
                diff_vec *= self.clamp / diff_norm

        return diff_vec
