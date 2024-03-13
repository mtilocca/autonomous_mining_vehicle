import math

class BicycleModel3D:
    def __init__(self, length: float = 2.0, velocity: float = 0.0, heading: float = 0.0, pitch: float = 0.0, roll: float = 0.0, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """
        Initializes the 3D Bicycle Model.

        :param length: The length between the front and rear axles.
        :param velocity: The initial velocity of the vehicle.
        :param heading: The initial yaw (heading) direction of the vehicle (in radians).
        :param pitch: The initial pitch angle of the vehicle (in radians).
        :param roll: The initial roll angle of the vehicle (in radians).
        :param x: The initial x-position of the vehicle.
        :param y: The initial y-position of the vehicle.
        :param z: The initial z-position (altitude) of the vehicle.
        """
        self.length = length
        self.velocity = velocity
        self.heading = heading
        self.pitch = pitch
        self.roll = roll
        self.x = x
        self.y = y
        self.z = z
        self.delta = 0.0  # Steering angle

    def update(self, delta: float, velocity: float, pitch_rate: float, roll_rate: float, dt: float = 1.0) -> None:
        """
        Updates the vehicle's state.

        :param delta: The steering angle (in radians).
        :param velocity: The velocity of the vehicle.
        :param pitch_rate: The rate of change of pitch angle (in radians per second).
        :param roll_rate: The rate of change of roll angle (in radians per second).
        :param dt: The time step for the update.
        """
        self.delta = delta
        self.velocity = velocity
        self.pitch += pitch_rate * dt
        self.roll += roll_rate * dt

        # Updating x, y based on heading and velocity
        self.x += self.velocity * math.cos(self.heading) * math.cos(self.pitch) * dt
        self.y += self.velocity * math.sin(self.heading) * math.cos(self.pitch) * dt
        # Updating z based on pitch
        self.z += self.velocity * math.sin(self.pitch) * dt
        
        # Update heading (yaw) based on the steering angle and velocity
        self.heading += self.velocity / self.length * math.tan(self.delta) * dt

    def get_state(self) -> dict:
        """
        Returns the current state of the vehicle.

        :return: The current state as a dictionary.
        """
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "velocity": self.velocity,
            "heading": self.heading,
            "pitch": self.pitch,
            "roll": self.roll,
            "delta": self.delta
        }
