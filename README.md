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

Block Scheme of the system: 

![BlockScheme](https://github.com/aainur/Magnetic-attitude-controller/assets/105110202/eca3d830-e87f-4729-bd57-57f36a5e48f1)

PCB Schematic: 

<img width="801" alt="PCB Schematic" src="https://github.com/user-attachments/assets/597bffc5-2ee1-483a-97c2-18365b35c93c" />

PCB Layout:

<img width="656" alt="PCB Layout" src="https://github.com/user-attachments/assets/ee83e35a-044b-44b7-a540-cd6643d579f2" />

Device: 

![controller](https://github.com/aainur/Magnetic-attitude-controller/assets/105110202/6cee8e0f-1d2b-4d7b-b9ff-a0ac3b668f04)

Measurement setup: 
![helmholtz](https://github.com/user-attachments/assets/f959f23e-d401-4639-aac7-93a593b86533)

Demagnetization of Coils: 
<img width="1270" alt="demag" src="https://github.com/user-attachments/assets/74c8143f-984e-4982-94c4-6bb81deb768c" />

XZ plane before calibration: 

<img width="1440" alt="XZplaneBeforeCal" src="https://github.com/aainur/Magnetic-attitude-controller/assets/105110202/e12d7565-3fb5-47ad-92bb-a49bff50073a">

XZ plane after calibration: 

<img width="1440" alt="XZplaneAfterCal" src="https://github.com/aainur/Magnetic-attitude-controller/assets/105110202/f5407a83-475a-40cb-99c1-8d86b22af6c4">


Bdot detumbling measurements: 

<img width="1440" alt="bdot" src="https://github.com/aainur/Magnetic-attitude-controller/assets/105110202/c04ec077-ebd2-44e6-87f0-1c76d643b7b5">








