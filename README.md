# Autonomous Line-Following Robot – PSoC Embedded System

## Overview

This project is a hardware-software co-design project for a two-wheeled autonomous robot using a Cypress PSoC development kit. The robot was designed to navigate a projected maze, follow line paths, detect junctions, and collect target points in a predefined sequence.

The system combines phototransistor sensors, analogue signal conditioning, ADC-based sensor reading, FSM-based turning logic, and motor control to produce autonomous movement on physical hardware.

This repository is a portfolio case study based on a university group project. It focuses on my individual contributions in ADC reading, sensor processing, filter design, and FSM-based turning logic.

---

## Demo

### Robot Walking Demo

[Watch the robot demo video](media/robot_demo.mp4)


---

## Hardware Photos

### Robot Body

![Robot Body](media/robot.jpg)


---

## System Highlights

This project demonstrates:

* Embedded control logic on Cypress PSoC
* ADC-based sensor reading
* Phototransistor line detection
* FSM-based turning control
* Hardware-software integration
* Real-world debugging of sensor and movement behaviour
* Iterative testing on physical hardware

---

## System Design

The robot used multiple phototransistor sensors to detect projected light and dark regions on the maze path. Sensor readings were processed through analogue filtering and ADC threshold detection before being used by the control logic.

The overall control flow can be summarised as:

```text
Phototransistor Sensors
        ↓
Signal Conditioning / Filtering
        ↓
ADC Reading on PSoC
        ↓
Threshold-Based Line Detection
        ↓
FSM Turning Logic
        ↓
Motor Control
        ↓
Robot Movement
```

The robot relied on front sensors for junction detection and inner sensors for path correction. This allowed the robot to both follow straight paths and detect when it needed to turn.

---

## Sensor and ADC Processing

Phototransistor sensors were used to detect the difference between projected line regions and background regions. Because ambient lighting affected the raw sensor signal, analogue filtering was used to reduce noise and improve signal reliability.

The processed sensor signals were then sampled through the PSoC ADC. Threshold-based detection was used to classify whether each sensor was over the projected line or not.

### Sensor Reading Graph

![Sensor Reading Graph](media/sensor_reading_graph.png)

> Insert the sensor voltage reading graph here.
> Suggested filename: `media/sensor_reading_graph.png`

---

## FSM-Based Turning Logic

The robot movement controller was implemented using a finite state machine. The FSM determined the robot's behaviour based on sensor readings and movement instructions.

Main movement states included:

* Decision state
* Forward state
* Left turn
* Right turn
* U-turn
* Final stopping behaviour

The decision state selected the next movement instruction, while the turning states controlled motor behaviour until the robot reacquired the path line.

### Turning FSM Diagram

![Turning FSM](media/turning_fsm.png)

> Insert the turning logic FSM diagram here.
> Suggested filename: `media/turning_fsm.png`

---

## My Contributions

My main contributions focused on embedded sensing and control logic.

* Contributed to filter design for reducing ambient lighting interference in sensor readings.
* Worked on ADC reading and threshold-based detection for phototransistor sensors.
* Designed and implemented FSM-based turning logic for robot navigation.
* Integrated sensor input with motor control decisions for path correction and junction handling.
* Tested and refined turning behaviour to reduce overshooting and improve movement reliability.
* Collaborated with teammates responsible for pathfinding, PCB design, soldering, and motor control integration.

---

## Testing and Debugging

Testing was performed on physical hardware under realistic lighting and movement conditions. Key debugging areas included:

* Sensor readings affected by ambient lighting
* ADC threshold tuning
* Robot drifting away from the path
* Overshooting during turns
* Junction detection reliability
* Coordination between sensor input and motor response

One important improvement was refining the turning logic to reduce overshooting. Instead of relying only on the centre sensor, additional sensors were used to detect when the robot had correctly realigned with the path.

---

## Relevance to Firmware Engineering

This project is highly relevant to firmware and embedded systems because it required software to interact directly with real hardware.

Key firmware-related concepts demonstrated:

* Reading and interpreting sensor data
* Working with ADC input
* Designing deterministic FSM control logic
* Integrating software decisions with physical motor behaviour
* Debugging hardware-dependent issues
* Testing and refining behaviour on a real embedded platform

The project strengthened my understanding of how firmware connects sensors, control logic, and actuators in a physical system.


```
```
