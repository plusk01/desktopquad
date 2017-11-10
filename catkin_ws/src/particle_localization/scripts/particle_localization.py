#!/usr/bin/env python
import rospy
import time, tf
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from dynamics import ConstVel as Dynam
from aruco_localization.msg import MarkerMeasurement, MarkerMeasurementArray

class ParticleLocalization:
    """
    Class for particle filter based localization
    """
    def __init__(self):
        """
        Constructor
        """
        try:
            param_namespace = '/desktopquad'
            self.param = rospy.get_param(param_namespace)
        except KeyError:
            rospy.logfatal('Params not set in /desktopquad namespace')
            rospy.signal_shutdown('Params not set')

        self.map = self.param['landmarks']
        #M_acc = self.param['M_acc']
        #M_gyro = self.param['M_gyro']
        #self.R = self.param['R_aruco']
        Q_pos = np.diag(3*[1e-3])
        Q_att = np.diag(3*[1e-3])

        #n_particles = self.param['n_particles']
        n_particles = 1

        # PARKER ARUCO STUFF
        rospy.Subscriber("chiny/aruco/measurements", MarkerMeasurementArray, self.aruco_callback)
        # ROSFLIGHT ACCEL & GYRO STUFF
        rospy.Subscriber("chiny/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("chiny/imu/acc_bias", Vector3Stamped, self.imu_acc_callback)
        rospy.Subscriber("chiny/imu/gyro_bias", Vector3Stamped, self.imu_gyro_callback)
        self.tmp_pub_ = rospy.Publisher("tmp/", Odometry)

        self.init_particles(n_particles)

        self.acc_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.dynamics = Dynam(Q_pos, Q_att)
        self.particle_update = False
        self.particle_propogate = False
        self.prev_time = rospy.Time.now()
        self.measurements = []
        self.init_particles(n_particles)
        self.run()

    def init_particles(self, n_particles):
        """
        These particles are stored in columns representing
        positions, velocities, attitude and attitude rates
        as described below.

        0 - p_n intertial north
        1 - p_e inertial east
        2 - p_d intertial down
        3 - u body frame velocity along i^b
        4 - v body frame velocity along j^b
        5 - body frame velocity along k^b
        6 - phi roll angle
        7 - theta pitch angle
        8 - yaw angle
        9 - p roll rate along i^b
        10 - q pitch rate along i^b
        11 - r yaw rate along k^b
        """
        mins = self.param['mins']
        maxs = self.param['maxs']
        self.particles = np.zeros((12, n_particles))
        for i in xrange(len(mins)//2):
            self.particles[i] = np.random.uniform(low=float(mins[i]),
                                high=float(maxs[i]), size=n_particles)
            self.particles[6+i] = np.random.uniform(low=float(mins[3+i]),
                                high=float(maxs[3+i]), size = n_particles)

    def localize(self, msg):
        """
        """
        pass

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()

    def aruco_callback(self, msg):
        """
        """
        self.particle_update = True
        self.localize(msg)
        self.particle_update = False

    def imu_callback(self, msg):
        """
        """
        if not self.particle_update :
            dt = (rospy.Time.now() - self.prev_time).to_sec()
            if dt > .1:
                self.prev_time = rospy.Time.now()
                # store accel & gyro for dynamics
                u = np.zeros(6)
                u[0:3] = [msg.linear_acceleration.x,msg.linear_acceleration.y, -msg.linear_acceleration.z]
                #u[3:] = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]

                # correct for imu bias
                u[0:3] -= self.acc_bias
                #u[3:] -= self.gyro_bias

                # update particles
                self.particles = self.dynamics.propogate(
                                self.particles, u, dt)

                x_hat = np.mean(self.particles, axis=1) 
                print x_hat
                o_msg = Odometry()
                o_msg.header.frame_id = "world"
                o_msg.pose.pose.position.x = x_hat[0]
                o_msg.pose.pose.position.y = x_hat[1]
                o_msg.pose.pose.position.z = x_hat[2]
                o_msg.pose.pose.orientation.x = 1.
                self.tmp_pub_.publish(o_msg)

    def imu_acc_callback(self, msg):
        self.acc_bias[:] = [msg.vector.x, msg.vector.y, msg.vector.z]

    def imu_gyro_callback(self, msg):
        self.gyro_bias[:] = [msg.vector.x, msg.vector.y, msg.vector.z]

def main():
    # initialize node
    rospy.init_node("particle_localization")

    pf = ParticleLocalization()
    pf.run()

if __name__=="__main__":
    main()
