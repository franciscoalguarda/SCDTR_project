# Distributed Real-Time Control Systems Project
## This repository contains the software for the distributed illumination system project.

Project Structure
main.ino: Initial configuration and concurrent execution cycles.

hardware.h / hardware.cpp: Functions for the illuminance measurement system and LED actuation system. Storage and computation of energy and comfort performance metrics.

can_comms.h / can_comms.cpp: Functions for sending and receiving CAN bus messages asynchronously.

can_protocol.h: Definitions of constants and message types.

calibration.h / calibration.cpp: State machines for the network boot sequence and distributed calibration.

hub.h / hub.cpp: Processing of serial interface commands and routing of information between the PC and the control nodes.

pid.h / pid.cpp: C++ class for the individual luminaire controller.

distributed_control.h / distributed_control.cpp: Implementation of the cooperative distributed control algorithm.
