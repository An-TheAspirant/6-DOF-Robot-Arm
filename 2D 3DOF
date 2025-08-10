import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math

class RobotArm2D:
    """
    2D Robot Arm Forward Kinematics Implementation
    
    This class demonstrates the fundamental concepts of forward kinematics
    for a 2-joint (2-DOF) robot arm in 2D space.
    """
    
    def __init__(self, link1_length=1.0, link2_length=0.8):
        """
        Initialize the robot arm with link lengths.
        
        Args:
            link1_length (float): Length of first link (shoulder to elbow)
            link2_length (float): Length of second link (elbow to end-effector)
        """
        self.L1 = link1_length  # Length of link 1
        self.L2 = link2_length  # Length of link 2
        
        # Current joint angles (in radians)
        self.theta1 = 0.0  # Shoulder angle
        self.theta2 = 0.0  # Elbow angle
        
        # Joint positions for visualization
        self.joint_positions = None
        self.end_effector_position = None
        
    def forward_kinematics(self, theta1, theta2):
        """
        Calculate forward kinematics for 2D robot arm.
        
        This is the core function that calculates the end-effector position
        given joint angles using transformation matrices.
        
        Args:
            theta1 (float): Shoulder joint angle in radians
            theta2 (float): Elbow joint angle in radians
            
        Returns:
            tuple: (end_effector_x, end_effector_y, joint_positions)
        """
        # Base position (shoulder joint)
        base_x, base_y = 0.0, 0.0
        
        # Elbow position (end of link 1)
        elbow_x = self.L1 * math.cos(theta1)
        elbow_y = self.L1 * math.sin(theta1)
        
        # End-effector position (end of link 2)
        # Note: theta2 is relative to link 1, so we add theta1 + theta2
        end_x = elbow_x + self.L2 * math.cos(theta1 + theta2)
        end_y = elbow_y + self.L2 * math.sin(theta1 + theta2)
        
        # Store positions for visualization
        joint_positions = {
            'base': (base_x, base_y),
            'elbow': (elbow_x, elbow_y),
            'end_effector': (end_x, end_y)
        }
        
        return end_x, end_y, joint_positions
    
    def forward_kinematics_matrix(self, theta1, theta2):
        """
        Alternative implementation using transformation matrices.
        
        This demonstrates the matrix-based approach that scales to 6 DOF.
        Each joint has a transformation matrix that describes its position
        and orientation relative to the previous joint.
        """
        # Transformation matrix for joint 1 (shoulder)
        T1 = np.array([
            [math.cos(theta1), -math.sin(theta1), self.L1 * math.cos(theta1)],
            [math.sin(theta1),  math.cos(theta1), self.L1 * math.sin(theta1)],
            [0,                 0,                1]
        ])
        
        # Transformation matrix for joint 2 (elbow) relative to joint 1
        T2 = np.array([
            [math.cos(theta2), -math.sin(theta2), self.L2 * math.cos(theta2)],
            [math.sin(theta2),  math.cos(theta2), self.L2 * math.sin(theta2)],
            [0,                 0,                1]
        ])
        
        # Combined transformation from base to end-effector
        T_total = T1 @ T2
        
        # Extract position from transformation matrix
        end_x = T_total[0, 2]
        end_y = T_total[1, 2]
        
        # Calculate intermediate positions
        elbow_x = T1[0, 2]
        elbow_y = T1[1, 2]
        
        joint_positions = {
            'base': (0.0, 0.0),
            'elbow': (elbow_x, elbow_y),
            'end_effector': (end_x, end_y)
        }
        
        return end_x, end_y, joint_positions
    
    def set_joint_angles(self, theta1_deg, theta2_deg):
        """
        Set joint angles in degrees (converts to radians internally).
        
        Args:
            theta1_deg (float): Shoulder angle in degrees
            theta2_deg (float): Elbow angle in degrees
        """
        self.theta1 = math.radians(theta1_deg)
        self.theta2 = math.radians(theta2_deg)
        
        # Update positions
        end_x, end_y, positions = self.forward_kinematics(self.theta1, self.theta2)
        self.joint_positions = positions
        self.end_effector_position = (end_x, end_y)
        
    def get_workspace_boundary(self, num_points=1000):
        """
        Calculate the workspace boundary of the robot arm.
        
        The workspace is the set of all points the end-effector can reach.
        """
        # Outer boundary (both links extended)
        outer_radius = self.L1 + self.L2
        outer_angles = np.linspace(0, 2*np.pi, num_points)
        outer_x = outer_radius * np.cos(outer_angles)
        outer_y = outer_radius * np.sin(outer_angles)
        
        # Inner boundary (links in opposite directions)
        inner_radius = abs(self.L1 - self.L2)
        inner_x = inner_radius * np.cos(outer_angles)
        inner_y = inner_radius * np.sin(outer_angles)
        
        return (outer_x, outer_y), (inner_x, inner_y)
    
    def plot_arm(self, ax=None, show_workspace=True):
        """
        Plot the current arm configuration.
        
        Args:
            ax: Matplotlib axis object (optional)
            show_workspace (bool): Whether to show the workspace boundary
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 8))
        
        if self.joint_positions is None:
            # Initialize with default position
            self.set_joint_angles(30, 45)
        
        # Extract positions
        base_pos = self.joint_positions['base']
        elbow_pos = self.joint_positions['elbow']
        end_pos = self.joint_positions['end_effector']
        
        # Plot workspace boundary
        if show_workspace:
            outer_boundary, inner_boundary = self.get_workspace_boundary()
            ax.plot(outer_boundary[0], outer_boundary[1], 'g--', alpha=0.5, label='Outer Workspace')
            if inner_boundary[0][0] > 0:  # Only plot if inner boundary exists
                ax.plot(inner_boundary[0], inner_boundary[1], 'r--', alpha=0.5, label='Inner Workspace')
        
        # Plot links
        ax.plot([base_pos[0], elbow_pos[0]], [base_pos[1], elbow_pos[1]], 
                'b-', linewidth=6, label='Link 1')
        ax.plot([elbow_pos[0], end_pos[0]], [elbow_pos[1], end_pos[1]], 
                'r-', linewidth=6, label='Link 2')
        
        # Plot joints
        ax.plot(base_pos[0], base_pos[1], 'ko', markersize=12, label='Base')
        ax.plot(elbow_pos[0], elbow_pos[1], 'bo', markersize=10, label='Elbow')
        ax.plot(end_pos[0], end_pos[1], 'ro', markersize=10, label='End Effector')
        
        # Add coordinate frame at base
        ax.arrow(0, 0, 0.3, 0, head_width=0.05, head_length=0.05, fc='black', ec='black')
        ax.arrow(0, 0, 0, 0.3, head_width=0.05, head_length=0.05, fc='black', ec='black')
        ax.text(0.35, 0, 'X', fontsize=12, fontweight='bold')
        ax.text(0, 0.35, 'Y', fontsize=12, fontweight='bold')
        
        # Formatting
        ax.set_xlim(-2.5, 2.5)
        ax.set_ylim(-2.5, 2.5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title(f'2D Robot Arm\n'
                    f'θ₁ = {math.degrees(self.theta1):.1f}°, θ₂ = {math.degrees(self.theta2):.1f}°\n'
                    f'End Effector: ({end_pos[0]:.3f}, {end_pos[1]:.3f})')
        
        return ax

def demonstrate_forward_kinematics():
    """
    Demonstrate forward kinematics with various configurations.
    """
    # Create robot arm
    robot = RobotArm2D(link1_length=1.0, link2_length=0.8)
    
    # Test different configurations
    configurations = [
        (0, 0, "Straight out"),
        (90, 0, "Straight up"),
        (45, 45, "45° both joints"),
        (30, -60, "Elbow bent back"),
        (135, 90, "Reaching back")
    ]
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()
    
    for i, (theta1, theta2, description) in enumerate(configurations):
        robot.set_joint_angles(theta1, theta2)
        
        ax = axes[i]
        robot.plot_arm(ax, show_workspace=(i == 0))  # Show workspace only on first plot
        ax.set_title(f'{description}\nθ₁={theta1}°, θ₂={theta2}°')
        
        # Print forward kinematics results
        end_x, end_y = robot.end_effector_position
        print(f"{description}: End effector at ({end_x:.3f}, {end_y:.3f})")
    
    # Remove empty subplot
    fig.delaxes(axes[5])
    
    plt.tight_layout()
    plt.show()

def interactive_demo():
    """
    Create an interactive demonstration of forward kinematics.
    """
    robot = RobotArm2D()
    
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Animation function
    def animate(frame):
        ax.clear()
        
        # Create smooth motion
        t = frame * 0.1
        theta1 = 30 * math.sin(t) + 45  # Oscillate around 45°
        theta2 = 60 * math.cos(t * 0.7) + 30  # Different frequency
        
        robot.set_joint_angles(theta1, theta2)
        robot.plot_arm(ax, show_workspace=True)
        
        # Add trajectory trace
        if hasattr(animate, 'trajectory'):
            animate.trajectory.append(robot.end_effector_position)
            if len(animate.trajectory) > 50:  # Keep last 50 points
                animate.trajectory.pop(0)
            
            # Plot trajectory
            if len(animate.trajectory) > 1:
                traj_x = [point[0] for point in animate.trajectory]
                traj_y = [point[1] for point in animate.trajectory]
                ax.plot(traj_x, traj_y, 'm-', alpha=0.6, linewidth=2, label='Trajectory')
        else:
            animate.trajectory = []
    
    # Create animation
    anim = FuncAnimation(fig, animate, frames=200, interval=100, repeat=True)
    plt.show()
    
    return anim

def workspace_analysis():
    """
    Analyze and visualize the robot arm's workspace.
    """
    robot = RobotArm2D()
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot 1: Workspace boundary
    outer_boundary, inner_boundary = robot.get_workspace_boundary()
    
    ax1.plot(outer_boundary[0], outer_boundary[1], 'g-', linewidth=2, label='Outer Workspace')
    if inner_boundary[0][0] > 0:
        ax1.plot(inner_boundary[0], inner_boundary[1], 'r-', linewidth=2, label='Inner Workspace')
    
    ax1.fill_between(outer_boundary[0], outer_boundary[1], alpha=0.2, color='green')
    if inner_boundary[0][0] > 0:
        ax1.fill_between(inner_boundary[0], inner_boundary[1], alpha=0.2, color='white')
    
    ax1.set_xlim(-2.5, 2.5)
    ax1.set_ylim(-2.5, 2.5)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_title('Robot Arm Workspace')
    
    # Plot 2: Reachable points sampling
    theta1_range = np.linspace(0, 2*np.pi, 36)  # 10° increments
    theta2_range = np.linspace(0, 2*np.pi, 36)  # 10° increments
    
    reachable_x = []
    reachable_y = []
    
    for theta1 in theta1_range:
        for theta2 in theta2_range:
            end_x, end_y, _ = robot.forward_kinematics(theta1, theta2)
            reachable_x.append(end_x)
            reachable_y.append(end_y)
    
    ax2.scatter(reachable_x, reachable_y, c='blue', s=1, alpha=0.6)
    ax2.set_xlim(-2.5, 2.5)
    ax2.set_ylim(-2.5, 2.5)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Reachable Points (Dense Sampling)')
    
    plt.tight_layout()
    plt.show()

# Main execution
if __name__ == "__main__":
    print("2D Forward Kinematics Demonstration")
    print("=" * 40)
    
    # Demonstrate basic forward kinematics
    print("\n1. Basic Forward Kinematics:")
    demonstrate_forward_kinematics()
    
    # Analyze workspace
    print("\n2. Workspace Analysis:")
    workspace_analysis()
    
    # Interactive demo (uncomment to run)
    print("\n3. Interactive Demo:")
    print("Uncomment the next line to see animated motion")
    interactive_demo()
    
    print("\nKey Learning Points:")
    print("- Forward kinematics calculates end-effector position from joint angles")
    print("- Workspace is the set of all reachable points")
    print("- Transformation matrices provide a systematic approach")
    print("- This 2D concept extends directly to 6 DOF arms")
    