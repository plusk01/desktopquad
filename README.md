The DesktopQuad
===============

The DesktopQuad project aims to put a multirotor on every table. The platform consists of a tethered micro quad, built using the frame of an Inductrix FPV quad. The multirotor is equipped with an upward facing USB camera, allowing it to localize itself using an ArUco marker map. The DestopQuad autopilot is built on top of the [ROSflight](https://rosflight.org/) stack.

<p align="center">
    <img src="https://raw.githubusercontent.com/wiki/plusk01/desktopquad/figures/platform_sim.png" width="30.75%" />
    <img src="https://raw.githubusercontent.com/wiki/plusk01/desktopquad/figures/platform.jpg" width="35%" />
</p>

## System Overview ##

The DesktopQuad connects to a desktop computer, where all the high level processing is done. As shown in the system architecture diagram below, the onboard sensors are processed on the computer to provide state estimation and position control. Attitude control and stabilization is done onboard the flight controller in ROSflight.

<p align="center">
    <img src="https://raw.githubusercontent.com/wiki/plusk01/desktopquad/figures/sysarch.png" width="65%" />
</p>

## Hardware Overview ##

The DesktopQuad is a custom built platform based on the Inductrix FPV frame and using a [BetaFPV F3 EVO](https://betafpv.com/products/f3-brushed-fc) flight controller with [ROSflight](https://rosflight.org/) firmware ported to run on the onboard STM32 F3 microprocessor. The camera comes from a commonly available [USB endoscope camera](https://www.amazon.com/gp/product/B00N4L5N4A/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1). Fitted with an adapted camera mount, the camera is rigidly attached to the quadcopter's frame which minimizes motion blur. For the wire tether, 30AWG silicone wire is used for the camera and flight controller USB connection (2 wires each) and 22AWG is used for supplying power, resulting in 6 wires.

<p align="center">
    <img src="https://raw.githubusercontent.com/wiki/plusk01/desktopquad/figures/quad_side.jpg" width="15%" />
    <img src="https://raw.githubusercontent.com/wiki/plusk01/desktopquad/figures/quad_top.jpg" width="15%" />
</p>

## See it in Action ##

- [Maiden flight (RC)](https://www.youtube.com/watch?v=8vroYsu1chs&index=5&list=PLQc_kHscNTeguc7sT1ASR-9vpAyASg_8j)
- [Particle filter sim results](https://www.youtube.com/watch?v=7iTq9rV5ex4&index=1&t=4s&list=PLQc_kHscNTeguc7sT1ASR-9vpAyASg_8j)
- [Particle filter hardware results](https://www.youtube.com/watch?v=tHIVad2cPMA&index=2&list=PLQc_kHscNTeguc7sT1ASR-9vpAyASg_8j)
