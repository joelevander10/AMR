import time
import board
import busio
import adafruit_bno055
import math
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import csv

# Open a CSV file for writing
csv_file = open('sensor_data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Timestamp', 'Sampling Rate (Hz)', 'IMU_Pitch', 'IMU_Roll', 'IMU_Yaw', 'IMU_LinAccel_X', 'IMU_LinAccel_Y', 'IMU_LinAccel_Z', 'IMU_AngVel_X', 'IMU_AngVel_Y', 'IMU_AngVel_Z', 'Lidar_X', 'Lidar_Y', 'Lidar_Z'])

# Variables to track scan count and timing
scan_count = 0
start_time = 0
point_count = 0

# Desired sampling rate (Hz)
desired_sampling_rate = 20

# Initialize I2C and IMU
i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_bno055.BNO055_I2C(i2c)

# Callback function for PointCloud2 data
def callback(data):
    global scan_count, start_time, point_count
    
    # Convert PointCloud2 message to a list of points
    points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    # Get the initial orientation
    initial_euler = imu.euler
    initial_pitch, initial_roll, initial_yaw = initial_euler[1], initial_euler[2], initial_euler[0]

    # Process points in the current scan
    for point in points:
        x, y, z = point
        point_count += 1
        
        # Check if the desired time has elapsed
        current_time = time.time()
        time_diff = current_time - start_time
        
        if time_diff >= (1 / desired_sampling_rate):
            # Read IMU data
            euler = imu.euler
            pitch, roll, yaw = euler[1], euler[2], euler[0]
            # Calculate the relative orientation
            relative_pitch = pitch - initial_pitch
            relative_roll = roll - initial_roll
            relative_yaw = yaw - initial_yaw

            accel_x, accel_y, accel_z = imu.linear_acceleration
            gyro_x, gyro_y, gyro_z = imu.gyro
            
            # Calculate sampling rate
            sampling_rate = point_count / time_diff
            
            # Print the data
            print(f"Timestamp: {current_time:.4f}")
            print(f"Sampling Rate: {sampling_rate:.2f} Hz")
            print(f"IMU Pitch: {relative_pitch:.2f} deg")
            print(f"IMU Roll: {relative_roll:.2f} deg")
            print(f"IMU Yaw: {relative_yaw:.2f} deg")
            print(f"IMU Linear Acceleration: ({accel_x:.4f}, {accel_y:.4f}, {accel_z:.4f})")
            print(f"IMU Angular Velocity: ({gyro_x:.4f}, {gyro_y:.4f}, {gyro_z:.4f})")
            print(f"Lidar Point: ({x:.4f}, {y:.4f}, {z:.4f})")
            print("---")
            
            # Write sensor data to the CSV file
            csv_writer.writerow([current_time, sampling_rate, math.degrees(pitch), math.degrees(roll), math.degrees(yaw), accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, x, y, z])
            
            # Reset point count and start time for the next scan
            point_count = 0
            start_time = current_time
            scan_count += 1

            # Delay to achieve the desired sampling rate
            time.sleep(1 / desired_sampling_rate)

# Function to listen for PointCloud2 data
def listener():
    rospy.init_node('sensor_fusion_node', anonymous=True)
    rospy.Subscriber('/lslidar_point_cloud', PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    start_time = time.time()  # Set the initial start time
    listener()

# Close the CSV file when the program exits
csv_file.close()
