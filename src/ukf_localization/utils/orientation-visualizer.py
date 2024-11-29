import matplotlib.pyplot as plt
import numpy as np
import math
import tf
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D  # Ensure this import is present

def create_rotation_matrix_and_quaternion(rpy_angles):
    """
    Create a rotation matrix and compute the quaternion for a given RPY angles.
    Args:
        rpy_angles (tuple): A tuple of (roll, pitch, yaw) in degrees.
    Returns:
        np.ndarray: A 3x3 rotation matrix.
        list: Quaternion [x, y, z, w].
    """
    roll, pitch, yaw = rpy_angles
    # Convert to radians and compute quaternion using the provided snippet
    quaternion = tf.transformations.quaternion_from_euler(
        math.radians(roll), math.radians(pitch), math.radians(yaw)
    )
    # Convert quaternion to rotation matrix
    rotation = R.from_quat(quaternion)
    return rotation.as_matrix(), quaternion

def plot_orientation_vectors_with_console_output(rpy_angles_list, show_enu=True):
    """
    Plot orientation vectors for a given list of roll, pitch, yaw angles, and output console details.
    Args:
        rpy_angles_list (list): List of tuples, where each tuple contains (roll, pitch, yaw) in degrees.
        show_enu (bool): Whether to show the ENU coordinate system for reference.
    """
    # Prepare the 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Define a list of colors for quivers
    colors = plt.cm.viridis(np.linspace(0, 1, len(rpy_angles_list)))  # Colormap for distinct colors

    # Plot ENU coordinate system if required
    if show_enu:
        ax.quiver(0, 0, 0, 1, 0, 0, color='red', label='East (X)', linewidth=1)
        ax.quiver(0, 0, 0, 0, 1, 0, color='green', label='North (Y)', linewidth=1)
        ax.quiver(0, 0, 0, 0, 0, 1, color='blue', label='Up (Z)', linewidth=1)

    # Plot each orientation vector and print console output
    for i, angles in enumerate(rpy_angles_list):
        roll, pitch, yaw = angles
        rotation_matrix, quaternion = create_rotation_matrix_and_quaternion(angles)
        direction_vector = rotation_matrix @ np.array([1, 0, 0])  # Get direction vector

        # Print details to console
        print(f"Orientation {i+1}:")
        print(f"  RPY Angles (degrees): Roll = {roll}, Pitch = {pitch}, Yaw = {yaw}")
        print(f"  Quaternion: [x = {quaternion[0]}, y = {quaternion[1]}, z = {quaternion[2]}, w = {quaternion[3]}]")
        print()

        # Plot orientation vector with unique color
        ax.quiver(
            0, 0, 0,
            direction_vector[0], direction_vector[1], direction_vector[2],
            color=colors[i], label=f'Orientation {i+1}', linewidth=2, arrow_length_ratio=0.1
        )

    # Set plot limits
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    # Label axes
    ax.set_xlabel('East (X)')
    ax.set_ylabel('North (Y)')
    ax.set_zlabel('Up (Z)')

    # Add grid and legend
    ax.grid(True)
    ax.legend()

    # Set title
    ax.set_title('3D Orientation Vectors in ENU Frame')

    # Show plot
    plt.show()

# Example usage
if __name__ == "__main__":
    # Define a list of Roll, Pitch, Yaw angles (in degrees)
    # roll,pitch,yaw = 1.035, 0.176, 248.449
    roll,pitch,yaw =   248.449, 0.176, 1.035
    transformed_roll = pitch  # New roll = old pitch
    transformed_pitch = roll  # New pitch = old roll
    transformed_yaw = -yaw  # Shift yaw for ENU frame (Y-forward)


    rpy_angles_list = [
        (roll, pitch, yaw),
        #(yaw, pitch, roll),
    ]

    plot_orientation_vectors_with_console_output(rpy_angles_list)
