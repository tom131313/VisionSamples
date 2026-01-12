# FRC AprilTag Vision Examples

Quick start for FRC teams learning AprilTag detection and robot pose estimation. Detects AprilTags, computes robot position, and drives to game targets.

## Architecture

`VisionContainer` wraps three vision system implementations:

-   `ControllerVision` - Free software based on WPILib, requires a camera, and a roboRIO or a simulation PC
-   `PhotonVision` - Free software, requires a camera, roboRIO and a coprocessor or a simulation PC handles both roboRIO and coprocessor 
-   `LimelightVision` - Purchased commercial hardware and integrated software requires a roboRIO or a simulation PC

## Example Commands

Three targeting strategies are implemented as commands:

### 1. Turn and Drive to Distance

Turn to target and drive to specified distance using yaw/pitch relative to AprilTag.

### 2. Drive to Field Position

Drive to a 3D field pose. Two implementations:

-   **PID-based**: Three PID controllers with stub drivetrain
-   **PathPlanner**: On-the-fly command (partial example untested in this package)

### 3. Drive to Target-Relative Position

Drive to position relative to target using `Transform3d` instead of field coordinates.

> [!NOTE]
> Examples target AprilTag #14 and any of the paired tags (FRC 2026 RbuilT). You'll need to write your own game-specific targeting code.

## Setup

Read the comments in [Robot.java](src/main/java/frc/robot/Robot.java) and each vision system wrapper class for configuration details.

**Required to run this project:**
-   To run simulation of a roboRIO, a DriverStation PC is needed.
-   To run with a real roboRIO, a roboRIO v1 (or v2) is needed.
-   This project runs as is with a provided crude drivetrain and gyro.
-   To test PhotonVision, that needs to be downloaded to at least the DriverStation PC.
-   To test LimelightVision, that needs to be purchased (beg, borrow, ...).

**Required if advancing to competitive usage:**
-   Your drivetrain implementation
-   Your implementation of where to drive to scoring positions
For 3-D pose usage the following are strongly recommended for accuracy and continuous driving even if the AprilTag is not in view:
-   Odometry
-   Gyro
-   Pose Estimator combining odometry, gyro, and AprilTag pose estimation

**Defaults:**
-   AprilTag #10 (change in code or print tag #10)
-   Xbox controller (easily changed to other inputs)

> [!WARNING]
> Tune PID and timeout parameters for your specific robot. Current values are for hand-driven camera testing.

## Hardware Requirements

**Minimum (simulation/testing):**

-   DriverStation PC
-   Optional ControllerVision (included in this project) is required to test that function
-   Optional PhotonVision software is required to test that function
-   USB camera (~$50) for ControllerVision or PhotonVision
-   Optional LimelightVision device is required to test that function
-   A roboRIO can be used to avoid simulation and implement vision on a robot

**For competition:**

-   Accurate vision systems require calibrated cameras. PhotonVision and LimelightVision provide for calibration and PV can be used to calibrate cameras for ControllerVision
-   roboRIO v1 for ControllerVision turn and drive to target (yaw/pitch)
-   roboRIO v2 for ControllerVision 3-D pose usage
-   A dedicated coprocessor for PhotonVision or LimelightVision is recommended instead of using ControllerVision software on the roboRIO

> [!NOTE]
> Running vision processing directly on the roboRIO is **discouraged**. Substantial CPU and timing tradeoffs may make it unsuitable for competition.

## Vision System Options

| System                  | Cost              | Hardware                               |
| ----------------------- | ----------------- | -------------------------------------- |
| ControllerVision        | Free              | USB camera only                        |
| PhotonVision            | Free              | USB camera + coprocessor for real mode |
| LimelightVision         | Purchase required | All-in-one device                      |

PhotonVision on a coprocessor offers good quality at low cost. LimelightVision is simpler to set up but more expensive. ControllerVision software is free and runs on your roboRIO; it may suffice for "zero budget" teams.

## Running
This project runs on a roboRIO v1 with all three vision systems.

Simulation of a roboRIO mode works with all three vision systems.

> [!NOTE]
> LimelightVision in simulation requires connecting your PC and Limelight to the same network.
