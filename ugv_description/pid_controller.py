import time
from collections import deque

class PIDController:
    def __init__(self, Kp, Ki, Kd, ticks_per_revolution, wheel_circumference,
                 pwm_neutral=1500, pwm_min=1100, pwm_max=1900, 
                 buffer_size=20, filter_alpha=0.3):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.pwm_neutral = pwm_neutral
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        
        self.filter_alpha = filter_alpha
        
        # Conversion factors
        self.ticks_per_revolution = ticks_per_revolution
        self.wheel_circumference = wheel_circumference
        
        self.error_integral = 0.0
        self.prev_measured_vel = 0.0
        self.prev_host_time = None
        
        self.pwm = pwm_neutral
        
        # Velocity estimation buffers
        self.pos_buffer = deque(maxlen=buffer_size)
        self.time_buffer = deque(maxlen=buffer_size)
        self.filtered_vel_ticks = 0.0
    
    def update(self, target_vel, pos, t_us):
        """
        Update PID controller and return PWM value
        
        target_vel: desired velocity (m/s)
        pos: encoder position (ticks)
        t_us: timestamp from Arduino (microseconds)
        """
        current_host_time = time.time()
        
        # Buffer management - store position and time data
        self.pos_buffer.append(pos)
        self.time_buffer.append(t_us / 1_000_000.0)
        
        # Wait for minimum buffer size before calculating velocity
        if len(self.pos_buffer) < 2:
            self.prev_host_time = current_host_time
            return self.pwm_neutral
        
        # Velocity estimation over buffer window (ticks/sec)
        dt_buffer = self.time_buffer[-1] - self.time_buffer[0]
        dpos_buffer = self.pos_buffer[-1] - self.pos_buffer[0]
        
        if dt_buffer <= 0:
            return self.pwm
        
        raw_vel_ticks = dpos_buffer / dt_buffer
        
        # Low-pass filter on velocity (in ticks/sec)
        self.filtered_vel_ticks = (self.filter_alpha * raw_vel_ticks + 
                                    (1 - self.filter_alpha) * self.filtered_vel_ticks)
        
        # Convert filtered velocity to m/s
        measured_vel = (self.filtered_vel_ticks / self.ticks_per_revolution) * self.wheel_circumference
        
        # Time delta calculation for PID
        if self.prev_host_time is None:
            self.prev_host_time = current_host_time
            self.prev_measured_vel = measured_vel
            return self.pwm
        
        dt = current_host_time - self.prev_host_time
        if dt <= 0 or dt > 1.0:
            self.prev_host_time = current_host_time
            return self.pwm
        
        # Zero velocity handling - reset integral and return neutral
        # NOTE: might need revisiting if rover is on a hill - neutral PWM could cause sliding
        if abs(target_vel) < 0.01:
            self.error_integral = 0.0
            self.pwm = self.pwm_neutral
            self.prev_measured_vel = measured_vel
            self.prev_host_time = current_host_time
            return self.pwm
        
        # PID Control Law
        error = target_vel - measured_vel
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup - only accumulate if not saturated
        if not (self.pwm >= self.pwm_max or self.pwm <= self.pwm_min):
            self.error_integral += error * dt
        self.error_integral = max(-100, min(100, self.error_integral))
        I = self.Ki * self.error_integral
        
        # Derivative term (on measurement to avoid derivative kick)
        D = -self.Kd * (measured_vel - self.prev_measured_vel) / dt
        
        # PWM calculation
        pwm_change = P + I + D
        self.pwm = max(self.pwm_min, min(self.pwm_max, self.pwm + pwm_change))
        
        # Update state variables
        self.prev_measured_vel = measured_vel
        self.prev_host_time = current_host_time
        
        return self.pwm
