class PIDController:
    def __init__(self, kp, ki, kd, max_integral, max_output):
        """
        Initializes the PID controller.

        :param kp: Proportional gain.
        :param ki: Integral gain.
        :param kd: Derivative gain.
        :param max_integral: Maximum value for the integral term to prevent wind-up.
        :param max_output: Maximum output value (to limit the control signal).
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.max_output = max_output

        self.setpoint = 0.0
        self.last_error = 0.0
        self.integral = 0.0

    def update(self, current_value, dt):
        """
        Updates the PID controller.

        :param current_value: The current value of the parameter being controlled.
        :param dt: Time step.
        :return: Control signal.
        """
        error = self.setpoint - current_value
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)  # Anti-windup

        derivative = (error - self.last_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.max_output), -self.max_output)  # Limit output

        self.last_error = error
        return output

    def set_setpoint(self, setpoint):
        """
        Sets the desired setpoint for the PID controller.

        :param setpoint: The desired setpoint value.
        """
        self.setpoint = setpoint
