"""Vendor ONNX policy, evaluated at a fixed cadence on the physics thread.

Observation/action scaling matches Lite3TestPolicyRunnerONNX. The policy is
unchanged. This is a simulation controller, not the factory gait-mode API.
"""
import math
from pathlib import Path
import mujoco
import numpy as np
import onnxruntime as ort


class Policy:
    def __init__(self, model, maximum_tilt_deg=0.):
        path = Path(__file__).resolve().parents[1]/'vendor/lite3/policy.onnx'
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), options, providers=['CPUExecutionProvider'])
        self.torso = model.body('TORSO').id
        self.gyro = int(model.sensor_adr[np.flatnonzero(model.sensor_type == mujoco.mjtSensor.mjSENS_GYRO)[0]])
        self.default = np.tile([0., -.65, 1.3], 4)
        self.scale = np.tile([.125, .25, .25], 4)
        self.last_action = np.zeros(12)
        self.target = self.default.copy()
        self.fault = False
        self.maximum_tilt_deg = maximum_tilt_deg

    def torque(self, model, data, velocity, step):
        matrix = data.xmat[self.torso].reshape(3, 3)
        if self.maximum_tilt_deg > 0 and matrix[2, 2] < math.cos(math.radians(self.maximum_tilt_deg)):
            self.fault = True
        if self.fault:
            return -1.5*data.qvel[6:18]
        if step >= 1000 and step % 12 == 0:
            observation = np.concatenate((data.sensordata[self.gyro:self.gyro+3]*.25,
                                          matrix.T @ [0., 0., -1.], velocity,
                                          data.qpos[7:19]-self.default,
                                          data.qvel[6:18]*.05, self.last_action)).astype(np.float32)
            action = self.session.run(None, {'obs': observation[None]})[0].reshape(12)
            if not np.isfinite(action).all():
                self.fault = True
                return -1.5*data.qvel[6:18]
            self.last_action = action
            self.target = self.default + action*self.scale
        return 30.*(self.target-data.qpos[7:19])-data.qvel[6:18]
