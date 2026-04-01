# Distributed Real-Time Control Systems Project

This repository contains the software for a distributed illumination control system. The project has been consolidated into a streamlined architecture optimized for the **Raspberry Pi Pico** (RP2040), utilizing hardware dual-core processing and CAN-bus communication.

## Project Structure

The codebase has been refactored from a multi-file architecture into a more cohesive structure:

* **`part2.ino`**: The core application file containing the complete system logic. It integrates all the functional modules:
  * **Auto-Addressing & Boot**: Dynamic node addressing using the Raspberry Pi Pico's hardware Unique ID. The node with the lowest ID automatically becomes the Hub (Master).
  * **Distributed Calibration**: State machines for background illuminance measurement and cross-gain (K-matrix) calibration across the network.
  * **CAN Communication & Protocol**: Asynchronous CAN message passing, network routing, and software loopback for seamless local/network command execution.
  * **PID & Control Loop**: A dedicated `PIDController` class with anti-windup, coupled with a cooperative distributed control algorithm that factors in external illuminance from neighboring nodes.
  * **Dual-Core Execution**: 
    * **Core 0** (`setup()` / `loop()`): Handles CAN messages, state machines (boot/calibration), and the Serial User Interface.
    * **Core 1** (`setup1()` / `loop1()`): Runs the strict 100Hz real-time control loop (sensor reading, PID computation, LED actuation).
    * *Concurrency*: Utilizes hardware mutexes (`pico/mutex.h`) for thread-safe variable sharing between cores, preventing race conditions.
  * **Metrics & UI**: Real-time computation of comfort and performance metrics (Energy, Visibility, and Flicker), accessible via a robust serial command menu at the Hub.

* **`mcp2515.h`**: Header library file for the MCP2515 CAN controller, responsible for SPI communication and CAN physical layer configuration.

## Features overview
* **Plug & Play Network**: Nodes auto-discover each other, share Unique IDs, and assign addresses automatically.
* **Safe Concurrency**: Strict mutex-protected data access between the asynchronous network/UI workloads and the 100Hz synchronous control loop.
* **Cooperative Control**: Dynamically adjusts LED PWM based on local LDR feedback and feed-forward data coming from neighbors over the CAN bus.
