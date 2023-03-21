"""
    # TrackedObjectArray.msg

    std_msgs/Header header

    uint64 frame_seq
    eflatun_msgs/TrackedObject[] detections

=================================================
    
    TrackedObject.msg

    std_msgs/Header header

    uint32 unique_id
    uint32 missing_age
    uint32 age

    uint32 center_x
    uint32 center_y

    uint32 width
    uint32 height
"""

from filterpy.kalman import KalmanFilter
from eflatun_msgs.msg import TrackedObject
import numpy as np
from typing import List, Tuple
from scipy.optimize import minimize


class Object:

    def __init__(self, msg: TrackedObject, dt: float = 1/30, kf_params: dict=None) -> None:

        self.unique_id = msg.unique_id
        self.age = msg.age
        self.missing_age = msg.missing_age
        self.center_x = msg.center_x
        self.center_y = msg.center_y
        self.width = msg.width
        self.height = msg.height

        self.predicted_x = self.center_x
        self.predicted_y = self.center_y
        self.predicted_width = self.width
        self.predicted_height = self.height


        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # Update Kalman filter parameters if provided
        if kf_params:
            for param, value in kf_params.items():
                setattr(self.kf, param, value)
        else:
            self.kf.x = np.array([self.center_x, self.center_y, 0, 0])
            self.kf.F = np.array([[1, 0, dt, 0],
                                [0, 1, 0, dt],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
            self.kf.H = np.array([[1, 0, 0, 0],
                                [0, 1, 0, 0]])
            self.kf.P = np.diag([60, 60, 10, 10])
            self.kf.R = np.diag([20, 20])
            self.kf.Q = np.diag([0.1, 0.1, 0.01, 0.01])

    def update(self, center_x: int, center_y: int, width: int, height: int) -> None:
        self.missing_age = 0
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height


        # Predict next state using Kalman filter
        self.predict()
        self.kf.update(np.array([center_x, center_y]))

    def predict(self) -> None:
        # Predict next position using Kalman filter
        self.predicted_state = self.kf.predict()
        self.age += 1
        self.predicted_x, self.predicted_y = self.kf.x[0], self.kf.x[1]
        self.predicted_width, self.predicted_height = self.width, self.height  # Assuming fixed plane size

    def to_ros_message(self) -> TrackedObject:
        msg = TrackedObject()

        msg.unique_id = self.unique_id
        msg.age = self.age
        msg.center_x = self.center_x
        msg.center_y = self.center_y
        msg.width = self.width
        msg.height = self.height
        return msg
    
    def from_prediction_to_ros_message(self) -> TrackedObject:
        self.center_x, self.center_y = self.predicted_x, self.predicted_y
        msg = TrackedObject()
        msg.unique_id = self.unique_id
        msg.age = self.age
        msg.center_x = self.predicted_x
        msg.center_y = self.predicted_y
        msg.width = self.predicted_width
        msg.height = self.predicted_height
        
        return msg
    
    def has_desired_size(self, min_width: int, min_height: int) -> bool:
        return self.width >= min_width and self.height >= min_height

    def is_within_range(self, x_range: Tuple[int, int], y_range: Tuple[int, int]) -> bool:
        return (x_range[0] <= self.center_x <= x_range[1]) and (y_range[0] <= self.center_y <= y_range[1])

    def __repr__(self):
        return f"\nObject(\n    id={self.unique_id}, age={self.age}, missing age={self.missing_age}\n    x={self.center_x} <|> {self.predicted_x},\n    y={self.center_y} <|> {self.predicted_y},\n    w={self.width}, h={self.height}\n)"

    def __str__(self):
        return self.__repr__()

    def __eq__(self, other):
        return self.unique_id == other.unique_id

    def __lt__(self, other):
        return self.age < other.age
    
class KalmanParameterOptimizer: #TODO use this code find parameters and save them as .yaml file.
 
    def __init__(self, measurements: List[Tuple[float, float]], initial_guess: List[float] = None) -> None:
        self.measurements = measurements
        self.initial_guess = initial_guess or [100, 100, 10, 10, 50, 50, 0.1, 0.1, 0.01, 0.01]

    def cost_function(self, params: List[float]) -> float:
        dim_x = 4
        dim_z = 2
        dt = 1/30

        kf = KalmanFilter(dim_x=dim_x, dim_z=dim_z)

        kf.x = np.array([self.measurements[0][0], self.measurements[0][1], 0, 0])
        kf.F = np.array([[1, 0, dt, 0],
                         [0, 1, 0, dt],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
        kf.H = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0]])
        kf.R = np.diag([params[4], params[5]])
        kf.Q = np.diag([params[6], params[7], params[8], params[9]])

        kf.P = np.diag([params[0], params[1], params[2], params[3]])

        predictions = []
        for measurement in self.measurements:
            kf.predict()
            kf.update(np.array(measurement))
            predictions.append(kf.x)

        predicted_states = np.array(predictions)

        errors = predicted_states[:, :2] - np.array(self.measurements)

        return np.mean(np.square(errors))

    def optimize(self) -> Tuple[List[float], float]:
        result = minimize(self.cost_function, self.initial_guess, method='Powell')
        return result.x, result.fun
    
def main(args=None):
    pass


if __name__ == '__main__':
    main()