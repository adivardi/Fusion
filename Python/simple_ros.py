import imufusion
import numpy

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped


def callback(imu_msg: Imu):
    gyroscope = numpy.array((imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z))
    accelerometer = numpy.array(
        (imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z))

    # convert from rad/s to deg/s
    gyroscope *= 180 / numpy.pi

    # convert the linear acceleration from m/s^2 to g's
    g_to_ms2 = 9.80665
    accelerometer /= g_to_ms2
    # accelerometer.y /= g_to_ms2
    # accelerometer.z /= g_to_ms2

    period = 1 / imu_sample_rate  # 200 Hz sample rate

    gyroscope_bias_removed = offset.update(gyroscope)

    ahrs.update_no_magnetometer(gyroscope, accelerometer, period)
    quat = ahrs.quaternion
    euler = quat.to_euler()

    # print(f"quat: {quat.w}, {quat.x}, {quat.y}, {quat.z}")
    # print(f"euler: {euler}")

    imu_orientation_msg = imu_msg
    imu_orientation_msg.orientation.w = quat.w
    imu_orientation_msg.orientation.x = quat.x
    imu_orientation_msg.orientation.y = quat.y
    imu_orientation_msg.orientation.z = quat.z

    # stddev: 10 - still has map noise
    # stddev: 30 - still more map noise than imu_tools with 50
    # stddev: 50 - very little map noise
    orientation_stddev = 50
    orientation_variance_ = orientation_stddev * orientation_stddev

    imu_orientation_msg.orientation_covariance = [0.0] * 9
    imu_orientation_msg.orientation_covariance[0] = orientation_variance_
    imu_orientation_msg.orientation_covariance[4] = orientation_variance_
    imu_orientation_msg.orientation_covariance[8] = orientation_variance_

    pub.publish(imu_orientation_msg)

    rpy_msg = Vector3Stamped()
    rpy_msg.header = imu_msg.header
    rpy_msg.vector.x = euler[0] * numpy.pi / 180.0
    rpy_msg.vector.y = euler[1] * numpy.pi / 180.0
    rpy_msg.vector.z = euler[2] * numpy.pi / 180.0

    pub_rpy.publish(rpy_msg)

    # ahrs_internal_states = numpy.array([ahrs.internal_states.acceleration_error,
    #                                     ahrs.internal_states.accelerometer_ignored,
    #                                     ahrs.internal_states.acceleration_recovery_trigger,
    #                                     ahrs.internal_states.magnetic_error,
    #                                     ahrs.internal_states.magnetometer_ignored,
    #                                     ahrs.internal_states.magnetic_recovery_trigger])

    internal_states_msg = Vector3Stamped()
    internal_states_msg.header = imu_msg.header
    internal_states_msg.vector.x = ahrs.internal_states.acceleration_error
    internal_states_msg.vector.y = 0.1 if ahrs.internal_states.accelerometer_ignored > 0 else 0.0
    internal_states_msg.vector.z = ahrs.internal_states.acceleration_recovery_trigger

    pub_internal_states.publish(internal_states_msg)

    gyro_bias_removed_msg = Vector3Stamped()
    gyro_bias_removed_msg.header = imu_msg.header
    gyro_bias_removed_msg.vector.x = gyroscope_bias_removed[0]
    gyro_bias_removed_msg.vector.y = gyroscope_bias_removed[1]
    gyro_bias_removed_msg.vector.z = gyroscope_bias_removed[2]
    pub_gyro_bias_removed.publish(gyro_bias_removed_msg)


if __name__ == '__main__':
    rospy.init_node('imu_fusion', anonymous=True)

    imu_sample_rate = 200  # 200 Hz sample rate
    ahrs = imufusion.Ahrs()

    gain = 0.5
    # without bias removal: g0.5 still drifts. g0.7 only drifts on short term. g1.0 has more noise with only small reduction in drift
    # with fixed bias removal g0.5 it still diverges without bias removal
    print(f"gain: {gain}")

    ahrs.settings = imufusion.Settings(imufusion.CONVENTION_ENU,  # convention
                                       gain,  # gain
                                       2000,  # gyroscope range in deg/sec. TDK datasheet also shows 2000
                                       5,  # acceleration rejection in deg
                                       10,  # magnetic rejection in deg
                                       5 * imu_sample_rate)  # recovery trigger period in number of samples

    # gyroscope offset correction
    offset = imufusion.Offset(imu_sample_rate)

    # imu_name = "back_left"
    imu_name = "back_right"
    print(f"imu_name: {imu_name}")
    # Create a publisher for the filtered transforms
    pub = rospy.Publisher(f'/sensors/inertial/{imu_name}/imu_orientation_fusion', Imu, queue_size=1)
    pub_rpy = rospy.Publisher(f'/sensors/inertial/{imu_name}/imu_orientation_fusion_rpy', Vector3Stamped, queue_size=1)
    pub_internal_states = rospy.Publisher(
        f'/sensors/inertial/{imu_name}/imu_orientation_fusion_debug', Vector3Stamped, queue_size=1)
    pub_gyro_bias_removed = rospy.Publisher(
        f'/sensors/inertial/{imu_name}/imu_bias_removed_fusion', Vector3Stamped, queue_size=1)

    # Subscribe to the unfiltered transform messages
    rospy.Subscriber(f'/sensors/inertial/{imu_name}/imu_corrected', Imu, callback)

    # Spin
    rospy.spin()
