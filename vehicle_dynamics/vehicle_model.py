import math

class BicycleModel:
    def __init__(self, length:float =2.0, velocity:float =0.0, heading:float=0.0, x:float=0.0, y:float=0.0)-> None:
        """
        Initializes the Bicycle Model.

        :param length: The length between the front and rear axles.
        :param velocity: The initial velocity of the vehicle.
        :param heading: The initial heading direction of the vehicle (in radians).
        :param x: The initial x-position of the vehicle.
        :param y: The initial y-position of the vehicle.
        """
        self.length = length
        self.velocity = velocity
        self.heading = heading
        self.x = x
        self.y = y
        self.delta = 0.0  # Steering angle

    def update(self, delta:float, velocity:float, dt:float=1.0)-> None:
        """
        Updates the vehicle's state.

        :param delta: The steering angle (in radians).
        :param velocity: The velocity of the vehicle.
        :param dt: The time step for the update.
        """
        self.delta = delta
        self.velocity = velocity
        self.x += self.velocity * math.cos(self.heading) * dt
        self.y += self.velocity * math.sin(self.heading) * dt
        self.heading += self.velocity / self.length * math.tan(self.delta) * dt

    def get_state(self)-> dict:
        """
        Returns the current state of the vehicle.

        :return: The current state as a dictionary.
        """
        return {
            "x": self.x,
            "y": self.y,
            "velocity": self.velocity,
            "heading": self.heading,
            "delta": self.delta
        }
