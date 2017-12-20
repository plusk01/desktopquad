#!/usr/bin/env python
import copy
import rospy
import time, tf
import numpy as np
import pyqtgraph as pg
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf import TransformListener

## Switch to using white background and black foreground
# pg.setConfigOption('background', 'w')
# pg.setConfigOption('foreground', 'k')

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

class Plotter:
    """
    Class for plotting methods.
    """
    def __init__(self):
        # get parameters from server
        self.t_win = rospy.get_param('~time_window', 12.0)
        self.hw = rospy.get_param('~hw', False)

        self.first_time_secs = None

        self.tf = TransformListener()

        # setup subsribers
        rospy.Subscriber('/chiny/ground_truth/odometry/NED', Odometry, self.truthCallback)
        rospy.Subscriber('/chiny/mcl/estimate', PoseStamped, self.mclCallback)
        rospy.Subscriber('/chiny/aruco/estimate', PoseStamped, self.arucoCallback)

        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])
        self.w = pg.GraphicsWindow(title='States vs Time')
        self.w.resize(1000, 800)

        # initialize plots in one window
        self.p_pn = self.w.addPlot()
        self.p_pn.addLegend(size=(1,1), offset=(1,1))
        self.p_pe = self.w.addPlot()
        self.p_pd = self.w.addPlot()
        self.w.nextRow()
        self.p_phi = self.w.addPlot()
        self.p_theta = self.w.addPlot()
        self.p_psi = self.w.addPlot()

        # label the plots
        self.p_pn.setLabel('left', 'x')
        self.p_pe.setLabel('left', 'y')
        self.p_pd.setLabel('left', 'z')
        self.p_phi.setLabel('left', 'phi')
        self.p_theta.setLabel('left', 'theta')
        self.p_psi.setLabel('left', 'psi')

        # create curves to update later
        self.c_pn_t = self.p_pn.plot(name='truth')
        self.c_pe_t = self.p_pe.plot()
        self.c_pd_t = self.p_pd.plot()
        self.c_phi_t = self.p_phi.plot()
        self.c_theta_t = self.p_theta.plot()
        self.c_psi_t = self.p_psi.plot()

        self.c_pn_mcl = self.p_pn.plot(name='mcl')
        self.c_pe_mcl = self.p_pe.plot()
        self.c_pd_mcl = self.p_pd.plot()
        self.c_phi_mcl = self.p_phi.plot()
        self.c_theta_mcl = self.p_theta.plot()
        self.c_psi_mcl = self.p_psi.plot()

        self.c_pn_aruco = self.p_pn.plot(name='aruco')
        self.c_pe_aruco = self.p_pe.plot()
        self.c_pd_aruco = self.p_pd.plot()
        self.c_phi_aruco = self.p_phi.plot()
        self.c_theta_aruco = self.p_theta.plot()
        self.c_psi_aruco = self.p_psi.plot()

        # initialize state variables
        self.time_t = 0
        self.pn_t = 0
        self.pe_t = 0
        self.pd_t = 0
        self.phi_t = 0
        self.theta_t = 0
        self.psi_t = 0

        self.time_mcl = 0
        self.pn_mcl = 0
        self.pe_mcl = 0
        self.pd_mcl = 0
        self.phi_mcl = 0
        self.theta_mcl = 0
        self.psi_mcl = 0

        self.time_aruco = 0
        self.pn_aruco = 0
        self.pe_aruco = 0
        self.pd_aruco = 0
        self.phi_aruco = 0
        self.theta_aruco = 0
        self.psi_aruco = 0

        # truth/estimate storage lists
        self.truths = []
        self.mcls = []
        self.arucos = []

        # curve lists
        self.c_list_t       = [self.c_pn_t, self.c_pe_t, self.c_pd_t, self.c_phi_t, self.c_theta_t, self.c_psi_t]
        self.c_list_mcl     = [self.c_pn_mcl, self.c_pe_mcl, self.c_pd_mcl, self.c_phi_mcl, self.c_theta_mcl, self.c_psi_mcl]
        self.c_list_aruco   = [self.c_pn_aruco, self.c_pe_aruco, self.c_pd_aruco, self.c_phi_aruco, self.c_theta_aruco, self.c_psi_aruco]

    # method for updating each states
    def update(self):
        # pack stored data into lists
        self.truths.append([self.time_t, self.pn_t, self.pe_t, self.pd_t, self.phi_t, self.theta_t, self.psi_t])
        self.mcls.append([self.time_mcl, self.pn_mcl, self.pe_mcl, self.pd_mcl, self.phi_mcl, self.theta_mcl, self.psi_mcl])
        self.arucos.append([self.time_aruco, self.pn_aruco, self.pe_aruco, self.pd_aruco, self.phi_aruco, self.theta_aruco, self.psi_aruco])

        # discard data outside desired plot time window
        for i in range(0,1000):
            if self.truths[0][0] < self.truths[-1][0] - self.t_win:
                self.truths.pop(0)
            if self.mcls[0][0] < self.mcls[-1][0] - self.t_win:
                self.mcls.pop(0)
            if self.arucos[0][0] < self.arucos[-1][0] - self.t_win:
                self.arucos.pop(0)
        
        # stack the data lists
        truths_array = np.vstack(self.truths)
        time_t_array = truths_array[:,0]

        mcl_array = np.vstack(self.mcls)
        time_mcl_array = mcl_array[:,0]

        aruco_array = np.vstack(self.arucos)
        time_aruco_array = aruco_array[:,0]

        # set the truth states
        if not self.hw:
            for i in xrange(0,len(self.c_list_t)):
    	        self.c_list_t[i].setData(time_t_array, truths_array[:,i+1], pen=pg.mkPen(color=(255,0,0), width=2))

        # set the estimated states
        for i in xrange(0,len(self.c_list_mcl)):
            self.c_list_mcl[i].setData(time_mcl_array, mcl_array[:,i+1], pen=pg.mkPen(color=(0,255,0), width=2))

        # set the desired states
        for i in xrange(0,len(self.c_list_aruco)):
            self.c_list_aruco[i].setData(time_aruco_array, aruco_array[:,i+1], pen=pg.mkPen(color=(0,0,255), width=2))

        # update the plotted data
        self.app.processEvents()


    def truthCallback(self, msg):
        # convert from world_NED to aruco frame
        world_NED = PoseStamped()
        world_NED.header = copy.deepcopy(msg.header)
        world_NED.header.stamp = rospy.Time(0)
        world_NED.pose = msg.pose.pose
        aruco = self.tf.transformPose('aruco', world_NED)

        # unpack positions and linear velocities
        self.pn_t = aruco.pose.position.x
        self.pe_t = aruco.pose.position.y
        self.pd_t = aruco.pose.position.z

        # orientation in quaternion form
        quaternion = (
            aruco.pose.orientation.x,
            aruco.pose.orientation.y,
            aruco.pose.orientation.z,
            aruco.pose.orientation.w)

        # Use ROS tf to convert to Euler angles from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # unpack angles and angular velocities
        self.phi_t = euler[0]*180/np.pi
        self.theta_t = euler[1]*180/np.pi
        self.psi_t = euler[2]*180/np.pi

        if self.phi_t < 0:
            self.phi_t += 360
        if self.theta_t < 0:
            self.theta_t += 360
        if self.psi_t < 0:
            self.psi_t += 360

        if self.first_time_secs is None:
            self.first_time_secs = msg.header.stamp.to_sec() - 0.1

        # unpack time
        self.time_t = msg.header.stamp.to_sec() - self.first_time_secs


    def mclCallback(self, msg):
        # unpack positions and linear velocities
        self.pn_mcl = msg.pose.position.x
        self.pe_mcl = msg.pose.position.y
        self.pd_mcl = msg.pose.position.z

        # orientation in quaternion form
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)

        # Use ROS tf to convert to Euler angles from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # unpack angles and angular velocities
        self.phi_mcl = euler[0]*180/np.pi
        self.theta_mcl = euler[1]*180/np.pi
        self.psi_mcl = euler[2]*180/np.pi

        if self.phi_mcl < 0:
            self.phi_mcl += 360
        if self.theta_mcl < 0:
            self.theta_mcl += 360
        if self.psi_mcl < 0:
            self.psi_mcl += 360

        if self.first_time_secs is None:
            self.first_time_secs = msg.header.stamp.to_sec()

        # unpack time
        self.time_mcl = msg.header.stamp.to_sec() - self.first_time_secs


    def arucoCallback(self, msg):
        # unpack positions and linear velocities
        self.pn_aruco = msg.pose.position.y
        self.pe_aruco = msg.pose.position.x
        self.pd_aruco = msg.pose.position.z

        # orientation in quaternion form
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)

        # Use ROS tf to convert to Euler angles from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # unpack angles and angular velocities
        self.phi_aruco = euler[0]*180/np.pi
        self.theta_aruco = euler[1]*180/np.pi
        self.psi_aruco = euler[2]*180/np.pi

        if self.phi_aruco < 0:
            self.phi_aruco += 360
        if self.theta_aruco < 0:
            self.theta_aruco += 360
        if self.psi_aruco < 0:
            self.psi_aruco += 360

        if self.first_time_secs is None:
            self.first_time_secs = msg.header.stamp.to_sec()

        # unpack time
        self.time_aruco = msg.header.stamp.to_sec() - self.first_time_secs


################################################################################
################################################################################
################################################################################


def main():
    # initialize node
    rospy.init_node('state_plotter', anonymous=True)

    # initialize plotter class
    plotter = Plotter()

    # listen for messages and plot
    while not rospy.is_shutdown():
        try:
            # plot the local positions of each vehicle
            plotter.update()

            # let it rest a bit
            time.sleep(0.01)
        except rospy.ROSInterruptException:
            print "exiting...."
            return

if __name__ == '__main__':
    main()