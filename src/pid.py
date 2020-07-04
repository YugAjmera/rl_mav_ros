class PIDController(object):

    def __init__(self, kp, kd=0, ki=0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_z = 0.0
        self.last_error_rot_z = 0.0
        self.last_iterm_x = 0.0
        self.last_iterm_y = 0.0
        self.last_iterm_z = 0.0
        self.last_iterm_rot_z = 0.0
        self.time_delta = 0.00833

    def run(self, current_x, goal_x, current_y, goal_y, current_z, goal_z, current_rot_z, goal_rot_z):
        error_x = goal_x - current_x
        self.last_iterm_x += self.ki*self.last_iterm_x*self.time_delta
        error_x_delta = error_x - self.last_error_x
        u_x = self.kp*error_x + (self.kd*error_x_delta)*self.time_delta + self.last_iterm_x
        self.last_error_x = error_x

        error_y = goal_y - current_y
        self.last_iterm_y += self.ki*self.last_iterm_y*self.time_delta
        error_y_delta = error_y - self.last_error_y
        u_y = self.kp*error_y + (self.kd*error_y_delta)*self.time_delta + self.last_iterm_y
        self.last_error_y = error_y

        error_z = goal_z - current_z
        self.last_iterm_z += self.ki*self.last_iterm_z*self.time_delta
        error_z_delta = error_z - self.last_error_z
        u_z = self.kp*error_z + (self.kd*error_z_delta)*self.time_delta + self.last_iterm_z
        self.last_error_z = error_z

        error_rot_z = goal_rot_z - current_rot_z
        self.last_iterm_rot_z += self.ki*self.last_iterm_rot_z*self.time_delta
        error_rot_z_delta = error_rot_z - self.last_error_rot_z
        u_rot_z = self.kp*error_rot_z + (self.kd*error_rot_z_delta)*self.time_delta  + self.last_iterm_rot_z
        self.last_error_rot_z = error_rot_z
        
        return u_x, u_y, u_z, u_rot_z

