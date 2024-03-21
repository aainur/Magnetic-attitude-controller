# **Magnetic attitude controller** 

This research project presents a magnetic attitude controller specifically designed for educational purposes, aiming to enhance the learning experience within the Aerospace Engineering program.

The hardware controller consists of two coils, a Raspberry Pi Pico microcontroller, a magnetometer, and a Wi-Fi module. The proposed system serves as a learning tool to demonstrate the functionality of the Attitude Determination and Control System (ADCS) on a satellite. Measurements and tests can be conducted within a Helmholtz coil and on a floating table, where the device should be positioned.

The system also includes real-time visualization of received measurements in a 3D plot, facilitating the understanding of control mechanisms.

### Key Features of the Magnetic Attitude Controller:

- The controller incorporates two electromagnets with ferromagnetic cores.
- It utilizes a magnetometer (MMC5983MA) for attitude determination.
- The PCB fits within a 1U CubeSat form factor.
- The device can orient itself around one axis with a tolerance of 10 degrees.
- Calibration minimizes the electromagnets' impact on magnetometer readings.
- The device is powered by a LiPo battery.

The design and manufacturing of the PCB layout are accomplished using open-source CAD software, KiCAD. 
Additionally, the control code is designed to detumble the rotation of the device when placed on a floating table using the Bdot algorithm and to orient itself to a required degree using PID control.


![controller](https://github.com/aainur/Magnetic-attitude-controller/assets/105110202/6cee8e0f-1d2b-4d7b-b9ff-a0ac3b668f04)




