#!/usr/bin/env python
import rospy
import time, tf
import numpy as np
from sensor_msgs.imu import Imu
from dynamics import ConstVel as Dynam
from aruco_localization import MarkerMeasurement, MarkerMeasurementArray

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

        self.map = self.param['map']
        M_acc = self.param['M_acc']
        M_gyro = self.param['M_gyro']
        self.R = self.param['R_aruco']

        n_particles = self.param['n_particles']

        # PARKER ARUCO STUFF
        rospy.Subscriber("measurements", MarkerMeasurementArray, self.aruco_callback)
        # ROSFLIGHT ACCEL & GYRO STUFF
        rospy.Subscriber("imu/data", Imu, self.imu_callback)

        self.init_particles(n_particles)

        self.dynamics = Dynam(M_acc, M_gyro)
        self.particle_update = False
        self.prev_time = rospy.Time.now()
        self.measurements = []
        self.init_particles(n_particles)
        self.run()

    def init_particles(self, n_particles):
        # need to think about how to efficiently initialize particles
        mins = self.param['mins']
        maxs = self.param['maxs']
        self.particles = np.zeros((12, n_particles))
        for i in xrange(len(mins)):
            self.particles[i] = np.random.uniform(low=mins[i],
                                high=maxs[i], size=n_particles)

        self.particles[6:] = np.random.randn(3,n_particles)

    def propagate(self, dt):
        # call dynamic model on particles
        self.particles = dynam.propogate(self.particles, self.imu, dt)

    def localize(self, msg):
        for aruco_m in msg:
            sig = aruco_m.aruco_id

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.localize()
            rate.sleep()

    def aruco_callback(self, msg)
        # coordinate w/ parker on this one
        self.particle_update = True
        self.localize(msg)
        self.particle_update = False

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
