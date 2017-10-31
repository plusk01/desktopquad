#!/usr/bin/env python
import rospy
import time, tf
import numpy as np
from sensor_msgs.imu import Imu
from dynamics import ConstVel as Dynam

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

        n_particles = self.param['n_particles']
        mins = self.param['mins'] # mins of states for pose and pose_d
        maxs = self.param['maxs'] # maxes of states for pose and pose_d

        # PARKER ARUCO STUFF
        rospy.Subscriber("aruco_measurements", MSG_TYPE, self.aruco_callback)
        # ROSFLIGHT ACCEL & GYRO STUFF
        rospy.Subscriber("accel_gyro_stuff", Imu, self.imu_callback)

        self.dynamics = Dynam()
        self.particle_update = False
        self.prev_time = rospy.Time.now()
        self.measurements = []
        self.init_particles(n_particles)
        self.run()

    def init_particles(self, n_particles):
        # need to think about how to efficiently initialize particles
        self.particles = 0

    def propagate(self, dt):
        # call dynamic model on particles
        dynam.propogate(self.particles, self.imu, dt)

    def localize(self):
        self.particle_update = False

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.localize()
            rate.sleep()

    def aruco_callback(self, msg)
        # coordinate w/ parker on this one
        self.particle_update = True
        self.localize()

    def imu_callback(self, msg)
        # store accel & gyro for dynamics
        u = np.zeros(6)
        u[0:3] = msg.linear_acceleration
        u[4:] = msg.angular_velocity
        if not self.particle_update
            dt = self.prev_time - rospy.Time.now()
            self.prev_time = rospy.Time.now()
            self.particles = self.dynamics.propogate(
                                self.particles,u, dt)


def main():
    # initialize node
    rospy.init_node("particle_localization")

    pf = ParticleLocalization()
    pf.run()
