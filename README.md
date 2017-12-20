The DesktopQuad
===============

The DesktopQuad project aims to put a multirotor on every table. The platform consists of a tethered micro quad, built using the frame of an Inductrix FPV quad. The multirotor is equipped with an upward facing USB camera, allowing it to localize itself using an ArUco marker map. The DestopQuad autopilot is built on top of the [ROSflight](http://rosflight.org/) stack.

## System Overview ##

The DesktopQuad connects to a desktop computer, where all the high level processing is done. As shown in the system architecture diagram below, the onboard sensors are processed on the computer to provide state estimation and position control. Attitude control and stabilization is done onboard the flight controller in ROSflight.

<div>
    <img src="https://raw.githubusercontent.com/wiki/plusk01/desktopquad/figures/sysarch.png" width="65%" />
</div>