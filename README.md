# FRC AprilTag Vision Examples

Quick start for FRC teams learning AprilTag detection and robot pose estimation. Detects AprilTags, computes robot position, and drives to game targets.

## Architecture

`VisionContainer` wraps three vision system implementations:

-   `ControllerVision` - Free software based on WPILib, requires a camera, and a roboRIO or a simulation PC
-   `PhotonVision` - Free software, requires a camera, roboRIO and a coprocessor or a simulation PC handles both roboRIO and coprocessor 
-   `LimelightVision` - Purchased commercial hardware and integrated software requires a roboRIO or a simulation PC

## Example Commands

Four targeting strategies are implemented as commands:

### 1. Drive to Field Position - 2026 Paired HUB Tags

Drive to a 3D field pose relative to the left HUB tag.

### 2. Drive to Relative Position - Either Single Tag of a Pair

Drive to a 3D pose relative to either tag of a pair. Two implementations:

-   **PID-based**: Three PID controllers with stub drivetrain
-   **PathPlanner**: On-the-fly command (partial example untested in this package)

### 3. Drive to Single-Target Relative Position

Drive to position relative to target using `Transform3d` instead of field coordinates.

### 4. Turn and Drive to Distance

Turn to target and drive to specified distance using yaw/pitch relative to AprilTag.

> [!NOTE]
> Different alignment command examples target AprilTag #12, 9-10 and 19-20 paired HUB tags, or any of the paired tags (FRC 2026 RbuilT). You'll need to write your own game-specific targeting code.

## Setup

Read the comments in [Robot.java](src/main/java/frc/robot/Robot.java) and each vision system wrapper class for configuration details.

**Required to run this project:**
-   To run simulation of a roboRIO, a DriverStation PC is needed.
-   To run with a real roboRIO, a roboRIO v1 (or v2) is needed.
-   This project runs as is with a provided crude drivetrain and gyro.
-   To test PhotonVision, that needs to be downloaded to at least the DriverStation PC (or a co-processor may be used).
-   To test LimelightVision, that needs to be purchased (beg, borrow, ...).

**Required if advancing to competitive usage:**
-   Your drivetrain implementation
-   Your implementation of where to drive to scoring positions
For 3-D pose usage the following are strongly recommended for accuracy and continuous driving even if the AprilTag is not in view:
-   Odometry
-   Gyro
-   Pose Estimator combining odometry, gyro, and AprilTag pose estimation

**Defaults:**
-   AprilTag #12 (change in code or print tag #12)
-   Paired HUB tags #9 and #10 work for the default fake gyro for Limelight to match the heading
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
-   AprilTags #14, #9, #10

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

There are a few other viable Vision Systems.

## Running
This project runs on a roboRIO v1 with all three vision systems.

Simulation of a roboRIO mode works with all three vision systems.

> [!NOTE]
> LimelightVision in simulation requires connecting your PC and Limelight to the same network.
