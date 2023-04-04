# Embedded Systems Project
## Control System for an Autonomous Underwater Glider
A microcontroller board is connected to three motors. One is the main propeller, pushing the glider forward. The second motor actuates the rudder of the glider, allowing it to have a yaw motion. The third one is a linear motor that moves the battery pack, which in turns allows to statically change the pitch of the vehicle.
The microcontroller receives desired reference values for the forward speed, the pitch, and the rudder angle. These reference signals are sent through a serial interface. The microcontroller sends feedback messages back to the control PC to report a few status information.

## Hardware Specifications
The main motor can run from -10000 to 10000 RPMs.
The second and third motors can run from -100 to 100 RPMs.
The motors are controlled through a PWM signal.
The frequency must be 1 kHz.
For motor 1, 50% duty cycle corresponds to 0 RPM, 0% corresponds to -11000 RPM and 100% corresponds to 11000 RPMs.
For motor 2 and 3, 50% duty cycle corresponds to 0 RPM, 0% corresponds to -110 RPM and 100% corresponds to 110 RPMs.
A dead time of at least 3 microseconds should be used to prevent problems with the H-bridge controlling the motors.
Running the motors above their limits can damage them and should be avoided.
Ignoring dynamics, when motor 1 runs at 10000 RPMs the vehicle moves with a forward speed of 2 m/s. The speed is approximated to be linear with the RPMs.
When motor 2 runs at 100 RPMs, the battery pack moves with 5 mm/s. The battery pack can move from position 0 to a maximum of 10 cm. The pitch is linear with the position of the battery pack: again, ignoring dynamics, when the battery is at position 5 cm, the pitch is 0 degrees. When the battery is at position 10 cm, the pitch is +20 degrees, and when it is at 0 cm, the pitch is 0 degrees.
When motor 3 runs at 100 RPMs, the rudder moves with 5 deg/s. The rudder can move from -30 to 30 deg.
