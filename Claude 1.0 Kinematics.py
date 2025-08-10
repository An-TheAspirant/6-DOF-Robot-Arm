import numpy as np
import math
import time
try:
    import RPi.GPIO as GPIO # type: ignore
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("RPi.GPIO not available - servo control disabled")

class RobotArm6DOF:
    def __init__(self, dh_params, servo_pins=None):
        """
        Initialize 6-DOF robot arm controller
        
        Args:
            dh_params: List of [a, alpha, d, theta_offset] for each joint
            servo_pins: List of GPIO pins for servo control (optional)
        """
        self.dh_params = np.array(dh_params)
        self.num_joints = len(dh_params)
        self.joint_angles = np.zeros(self.num_joints)
        self.servo_pins = servo_pins
        
        # Joint limits (degrees)
        self.joint_limits = [
            [-180, 180],  # Base rotation
            [-90, 90],    # Shoulder
            [-90, 90],    # Elbow
            [-180, 180],  # Wrist roll
            [-90, 90],    # Wrist pitch
            [-180, 180]   # Wrist yaw
        ]
        
        # Initialize servo control
        if GPIO_AVAILABLE and servo_pins:
            self.init_servos()
    
    def init_servos(self):
        """Initialize GPIO pins for servo control"""
        if not GPIO_AVAILABLE:
            return
            
        GPIO.setmode(GPIO.BCM)
        self.pwm_objects = []
        
        for pin in self.servo_pins:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, 50)  # 50Hz frequency
            pwm.start(0)
            self.pwm_objects.append(pwm)
    
    def dh_transform(self, a, alpha, d, theta):
        """
        Create DH transformation matrix
        
        Args:
            a: link length
            alpha: link twist (radians)
            d: link offset
            theta: joint angle (radians)
        
        Returns:
            4x4 transformation matrix
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d   ],
            [0,   0,      0,     1   ]
        ])
    
    def forward_kinematics(self, joint_angles=None):
        """
        Calculate forward kinematics
        
        Args:
            joint_angles: Joint angles in degrees (optional, uses current if None)
        
        Returns:
            4x4 transformation matrix from base to end-effector
        """
        if joint_angles is None:
            joint_angles = self.joint_angles
        
        # Convert to radians
        angles_rad = np.radians(joint_angles)
        
        # Initialize with identity matrix
        T = np.eye(4)
        
        # Multiply transformation matrices
        for i in range(self.num_joints):
            a, alpha, d, theta_offset = self.dh_params[i]
            theta = angles_rad[i] + theta_offset
            T_i = self.dh_transform(a, alpha, d, theta)
            T = np.dot(T, T_i)
        
        return T
    
    def get_end_effector_pose(self, joint_angles=None):
        """
        Get end-effector position and orientation
        
        Returns:
            position: [x, y, z]
            orientation: [roll, pitch, yaw] in degrees
        """
        T = self.forward_kinematics(joint_angles)
        
        
        # Extract position
        position = T[:3, 3]
        
        # Extract orientation (roll, pitch, yaw)
        roll = math.atan2(T[2, 1], T[2, 2])
        pitch = math.atan2(-T[2, 0], math.sqrt(T[2, 1]**2 + T[2, 2]**2))
        yaw = math.atan2(T[1, 0], T[0, 0])
        
        orientation = np.degrees([roll, pitch, yaw])
        
        return position, orientation
    
    def jacobian(self, joint_angles=None):
        """
        Calculate Jacobian matrix
        
        Returns:
            6x6 Jacobian matrix
        """
        if joint_angles is None:
            joint_angles = self.joint_angles
        
        # Small perturbation for numerical differentiation
        eps = 1e-6
        J = np.zeros((6, self.num_joints))
        
        # Get current end-effector pose
        T0 = self.forward_kinematics(joint_angles)
        pos0, rot0 = self.get_end_effector_pose(joint_angles)
        
        # Calculate partial derivatives
        for i in range(self.num_joints):
            # Perturb joint angle
            angles_perturbed = joint_angles.copy()
            angles_perturbed[i] += eps
            
            # Calculate perturbed pose
            pos_p, rot_p = self.get_end_effector_pose(angles_perturbed)
            
            # Position derivatives
            J[:3, i] = (pos_p - pos0) / eps
            
            # Orientation derivatives
            J[3:, i] = (rot_p - rot0) / eps
        
        return J
    
    def inverse_kinematics(self, target_pos, target_rot=None, max_iter=100, tolerance=1e-3):
        """
        Solve inverse kinematics using Jacobian method
        
        Args:
            target_pos: Target position [x, y, z]
            target_rot: Target orientation [roll, pitch, yaw] in degrees (optional)
            max_iter: Maximum iterations
            tolerance: Convergence tolerance
        
        Returns:
            joint_angles: Solution joint angles in degrees
            success: True if converged
        """
        if target_rot is None:
            target_rot = [0, 0, 0]
        
        target = np.concatenate([target_pos, target_rot])
        angles = self.joint_angles.copy()
        
        for iteration in range(max_iter):
            # Current pose
            current_pos, current_rot = self.get_end_effector_pose(angles)
            current = np.concatenate([current_pos, current_rot])
            
            # Error
            error = target - current
            
            # Check convergence
            if np.linalg.norm(error) < tolerance:
                return angles, True
            
            # Calculate Jacobian
            J = self.jacobian(angles)
            
            # Pseudo-inverse for redundant/singular cases
            try:
                J_inv = np.linalg.pinv(J)
            except np.linalg.LinAlgError:
                return angles, False
            
            # Update joint angles
            delta_angles = np.dot(J_inv, error)
            angles += np.degrees(delta_angles) * 0.1  # Step size factor
            
            # Apply joint limits
            for i in range(len(angles)):
                angles[i] = np.clip(angles[i], 
                                  self.joint_limits[i][0], 
                                  self.joint_limits[i][1])
        
        return angles, False
    
    def angle_to_pwm_duty_cycle(self, angle):
        """
        Convert servo angle to PWM duty cycle
        
        Args:
            angle: Servo angle in degrees (0-180)
        
        Returns:
            duty_cycle: PWM duty cycle percentage
        """
        # MG996R: 1ms = 0°, 1.5ms = 90°, 2ms = 180°
        # At 50Hz: 20ms period, so 1ms = 5% duty, 2ms = 10% duty
        angle = np.clip(angle + 90, 0, 180)  # Offset for servo range
        pulse_width = 1.0 + (angle / 180.0)  # 1-2ms pulse width
        duty_cycle = (pulse_width / 20.0) * 100  # Convert to percentage
        return duty_cycle
    
    def move_to_angles(self, joint_angles, duration=1.0):
        """
        Move servos to specified joint angles
        
        Args:
            joint_angles: Target joint angles in degrees
            duration: Movement duration in seconds
        """
        if not (GPIO_AVAILABLE and self.servo_pins):
            print(f"Would move to angles: {joint_angles}")
            self.joint_angles = np.array(joint_angles)
            return
        
        # Validate joint limits
        for i, angle in enumerate(joint_angles):
            if not (self.joint_limits[i][0] <= angle <= self.joint_limits[i][1]):
                print(f"Warning: Joint {i+1} angle {angle}° exceeds limits {self.joint_limits[i]}")
                return
        
        # Interpolate movement
        start_angles = self.joint_angles.copy()
        steps = int(duration * 20)  # 20 steps per second
        
        for step in range(steps + 1):
            progress = step / steps
            current_angles = start_angles + (np.array(joint_angles) - start_angles) * progress
            
            # Set servo positions
            for i, angle in enumerate(current_angles):
                if i < len(self.pwm_objects):
                    duty_cycle = self.angle_to_pwm_duty_cycle(angle)
                    self.pwm_objects[i].ChangeDutyCycle(duty_cycle)
            
            time.sleep(duration / steps)
        
        self.joint_angles = np.array(joint_angles)
    
    def move_to_pose(self, target_pos, target_rot=None, duration=1.0):
        """
        Move to target end-effector pose
        
        Args:
            target_pos: Target position [x, y, z]
            target_rot: Target orientation [roll, pitch, yaw] in degrees
            duration: Movement duration in seconds
        
        Returns:
            success: True if inverse kinematics solved successfully
        """
        joint_angles, success = self.inverse_kinematics(target_pos, target_rot)
        
        if success:
            self.move_to_angles(joint_angles, duration)
            print(f"Moved to position: {target_pos}")
        else:
            print(f"Could not reach target position: {target_pos}")
        
        return success
    
    def cleanup(self):
        """Clean up GPIO resources"""
        if GPIO_AVAILABLE and hasattr(self, 'pwm_objects'):
            for pwm in self.pwm_objects:
                pwm.stop()
            GPIO.cleanup()

# Example usage
if __name__ == "__main__":
    # Define DH parameters for a typical 6-DOF arm
    # [a, alpha, d, theta_offset] for each joint
    dh_parameters = [
        [0,     np.pi/2,  0.1,  0],      # Base joint
        [0.2,   0,        0,    -np.pi/2], # Shoulder
        [0.15,  0,        0,    0],      # Elbow
        [0,     np.pi/2,  0.1,  0],      # Wrist roll
        [0,     -np.pi/2, 0,    0],      # Wrist pitch
        [0,     0,        0.05, 0]       # Wrist yaw
    ]
    
    # GPIO pins for servos (if using Raspberry Pi)
    servo_pins = [18, 19, 20, 21, 22, 23]
    
    # Create robot arm instance
    robot = RobotArm6DOF(dh_parameters, servo_pins)
    
    try:
        # Test forward kinematics
        test_angles = [0, 30, -45, 0, 60, 0]
        position, orientation = robot.get_end_effector_pose(test_angles)
        print(f"End-effector position: {position}")
        print(f"End-effector orientation: {orientation}")
        
        # Test inverse kinematics
        target_position = [0.2, 0.1, 0.3]
        target_orientation = [0, 0, 45]
        
        solution, success = robot.inverse_kinematics(target_position, target_orientation)
        if success:
            print(f"IK solution: {solution}")
            robot.move_to_angles(solution)
        
        # Example movement sequence
        positions = [
            [0.3, 0, 0.2],
            [0.2, 0.2, 0.25],
            [0.1, -0.1, 0.15]
        ]
        
        for pos in positions:
            robot.move_to_pose(pos, duration=2.0)
            time.sleep(1)
    
    finally:
        robot.cleanup()