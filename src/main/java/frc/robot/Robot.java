package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AlignToTag12FieldRelativePose3D;
import frc.robot.commands.AlignToTag12RelativeArcade2D;
import frc.robot.commands.AlignToPairedTagTargetRelativeTransform3D;
import frc.robot.commands.AlignToHUBPairedTagFieldRelativePose3D;
import frc.robot.vision.ControllerVision;
import frc.robot.vision.LimelightVision;
import frc.robot.vision.PhotonVision;
import frc.robot.vision.VisionContainer;

/**
 * <p>
 * This Project utilizes vision systems to detect and decode AprilTags in order to compute the
 * robot position.
 * <p>
 * {@link Config#} and {@link Constants#} have values to customize this project for team use. 
 * <p>
 * Four example uses of Vision using different targeting strategies are included as commands:
 * <p>
 * 1. Turn the robot to a target and drive to a specified distance from the target (yaw and pitch
 * of the robot wrt the AprilTag) {@link AlignToTag12RelativeArcade2D} (activated by XBox
 * controller B button {@link RobotContainer#AlignToTag12RelativeArcade2D()})
 * <p>
 * Heed the caution in this example targeting command about the limitations of the distance
 * calculation and a circumvention.
 * <p>
 * 2. Drive the robot to a specified field position (3-D pose of the robot in the field)
 * {@link AlignToTag12FieldRelativePose3D} (activated by XBox controller A button {@link
 * RobotContainer#AlignToTag12FieldRelativePose3D()})
 * <p>
 * This (swerve) drive command is presented in two forms:
 * <p>
 * a. use of three PID controllers to achieve the target position goal with a stub drivetrain
 * receiving the motor commands
 * <p>
 * b. proposed, but untested as packaged herein, PathPlanner on the fly command to drive to position
 * <P>
 * 3. Drive the robot to a specified position relative to the target (Transform3d). This is similar
 * to example #2 but instead of using field coordinates it uses a displacement from a target and
 * utilizes either tag of a pair of tags.
 * {@link AlignToPairedTagTargetRelativeTransform3D} (activated by XBox controller X button {@link
 * RobotContainer#AlignToPairedTagTargetRelativeTransform3D()})
 * <p>
 * 4. Drive the robot to a specified field position (3-D pose of the robot in the field). This a
 * sort of hybrid of other commands in its utilization of driving to a relative position to a 3-D
 * pose and it is using a single robot pose assuming it's the best pose based on a pair of HUB tags.
 * {@link AlignToHUBPairedTagFieldRelativePose3D}(activated  by XBox controller Y button
 * {@link RobotContainer#AlignToHUBPairedTagFieldRelativePose3D()})
 * <p>
 * The variations of these commands all assume a good robot pose and for LL and PV the multi-tag
 * features can be used. The main difference is how the desired tag for targeting is validated.
 * <p>
 * These example commands variously are limited to aligning with a single selected target near
 * AprilTag #12, any of the paired tags of 2026 RebuilT, and 9-10 and 19-20 of the paired HUB tags.
 * <p>
 * Completed team code should include the robot drivetrain, odometry and a gyro is extremely useful
 * for pose accuracy and drivability of the robot.
 * <p>
 * PID and timeout parameters must be tuned for each robot/drivetrain. Examples are for the a
 * hand-held camera.
 * <p>
 * This project is a wrapper called {@link VisionContainer} for three lower level wrappers
 * {@link ControllerVision} {@link PhotonVision} {@link LimelightVision} of three complex vision
 * systems - WPILib, PhotonVision, and LimelightVision.
 * <p>
 * After selecting and configuring the desired vision system as described below, use the robot pose
 * as in an example commands {@link AlignToTag12FieldRelativePose3D} or {@link
 * AlignToTag12RelativeArcade2D} or {@link AlignToPairedTagTargetRelativeTransform3D}
 * <p>
 * See {@link VisionContainer} as an example of code to instantiate each of the three vision
 * systems and use the RobotPose provided by each of the vision classes.
 * <p>
 * Each of the three wrappers for the three vision system also may be used without benefit of the
 * simple VisionContainer wrapper. Doing so allows use of more methods in the individual vision
 * system wrapper. And there is no requirement to use any of these wrappers. Each vision system has
 * a complex set of methods to perform a lot of functions. These wrappers make it easier to get
 * started quickly.
 * <p>
 * PhotonVision and LimelightVision each have examples of their usage on their web sites. Most
 * teams have Vision code; there are hundreds of varieties. This code helps beginner or novice
 * teams get started quicker and gives hints of some possibilities to use vision and commands.
 * <p>
 * Presentation of three Vision systems:
 * <p>
 * WPILib Vision example (known herein as ControllerVision) with an example wrapper consistent with
 * the other two wrappers
 * <p>
 * Example wrapper for PhotonVision
 * <p>
 * Example wrapper for LimelightVision
 * <p>
 * Each system puts robot pose information to the NetworkTables and that can be viewed in
 * AdvantageScope or other viewers.
 * <p>
 * These three Vision routines each can detect AprilTags and compute a robot pose. Select the one
 * to run in {@link Config}
 * <p>
 * This project may be used in whole by selecting which one of the three vision systems to be
 * demonstrated.
 * <p>
 * Each of the Visions Systems must be configured for the "camera" name and minimum parameters
 * needed for each system.
 * <p>
 * Each has its mounted camera location wrt the robot that must be provided by the team (example
 * default provided). {@link Config}
 * <p>
 * The ControllerVision (completely run on the roboRIO) must also have the chosen camera
 * calibration parameters specified herein (a few examples provided).
 * {@link Constants.ControllerVision}
 * <p>
 * ControllerVision arbitrarily selects to use the robot pose of the nearest AprilTag, if there are
 * multiple tags in view in a frame. That may be wrong sometimes. Some smarts need to be added so
 * targeting goes to the right target; see {@link ControllerVision#getPose3d()} and below it.
 * <p>
 * Use of PhotonVision or LimelightVision require that those devices and software be setup as
 * instructed in their respective documentation. They are mostly configured through their
 * respective "dashboards."
 * <p>
 * This project demonstrates some PV capabilities for single camera, single tag poses estimation and
 * single camera, multiple tag pose estimation. See
 * https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html
 * <p>
 * PhotonVision has available more sophisticated pose estimation algorithms than are demonstrated
 * in this project. They include multiple cameras and use of the robot heading (gyro). See
 * https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
 * <p>
 * Use of Limelight MegaTag2 requires the robot heading (gyro value) be sent to LL on each
 * iteration. {@link LimelightVision#update()}
 * <p>
 * Selection of the best pose from LimelightVision requires filtering. Simple example is presented.
 * {@link LimelightVision#update()}
 * <p>
 * Selection of the very best pose estimate from PhotonVision requires using {@link
 * PhotonVision#getEstimatedGlobalPose(Pose2d)}. A reference is given to the PV documentation.
 * Otherwise, the simpler "bestTarget" as used in this project is a good approximation.
 * <p>
 * Use of the WPILib PoseEstimator requires that values from this vision targeting project be
 * merged with the gyro and wheel odometry positions.
 * <p>
 * This demonstration project may be run in simulation mode or on a roboRIO. The cameras are not
 * simulated. That is, no roboRIO is required but the roboRIO simulation may be used with one of
 * the following attached (USB or network) to the PC running simulation:
 * <p>
 * a camera attached to the simulation PC,
 * <p>
 * a LimelightVision box attached to the same network as the simulation PC,
 * <p>
 * PhotonVision running on some device - the simulation PC or a coprocessor attached to the same
 * network as the simulation PC.
 * <p>
 * This code has been tested with Limelight 2+ limelight2_2025_1_release,
 * photonvision-v2026.1.1 on Raspberry Pi 4 Model B Rev 1.2, photonvision-v2026.1.1 on
 * Windows 11, and WPILib 2026.2.1; all with both a roboRIO v1 and DriverStation PC simulation.
 * <p>
 * Targeting commands were driven by a hand-held camera without benefit of a robot.
 * <p>
 * Cameras used were Microsoft LifeCam HD-3000 (USB interface) and ArduCam UC-844 Rev. B (OV9281
 * camera sensor; USB interface).
 * <p>
 * Gyro was simulated with a "180" heading and "0" rate.
 * <p>
 * Drivetrain was crudely simulated by printing the commanded velocity and voltage values and PWM
 * motor controllers.
 * <p>
 * Note that a significant amount of logging is turned on including @Logged annotation, datalog to
 * a file and terminal output. These are examples and not required for correct vision operation.
 * There are some helpful examples of debugging prints in the code most but not all of which are
 * commented out. Command execution summary logging and spike removal are printed as are fake
 * drivetrain data. Pose alignment discrepancies are printed so the human can pretend to drive (the
 * camera) to the target.
 * <p>
 * --------------------------------------------------------
 * <p>
 * ---  Setting up simulation of the roboRIO on the PC  ---
 * <p>
 * Get the ip address of the PC running the simulation by command prompt program "ipconfig" (linux
 * command is "ifconfig"). If the IP address isn't correct initially causing a connection issue in
 * simulation or real, no harm is done and this program provides some guidance to correctly enter
 * the team number and lists the IP address on the terminal.
 * <p>
 * Optional for the curious: Start Outline Viewer on the PC and change Options/settings/team/IP
 * 127.0.0.1 (or localhost or the PC IP address) - client nt4 mode and apply
 * <p>
 * For PhotonVision start the device/program and change Settings Team Number/NetworkTables Server
 * Address to the PC IP address PV needs to see the Server
 * <p>
 * For LimelightVision start the device and change Settings Custom NT Server IP: to the PC IP
 * address LL needs to see the Server
 * <p>
 * For ControllerVision plugin the camera into the PC
 * <p>
 * AdvantageScope start the program and select File / Connect to Simulator
 * <p>
 * Start the "Simulate Robot Code" on the PC
 * <p>
 * This is not simulating the camera device; that is not supported in this project. A real camera
 * device must be used. Only the roboRIO is being simulated.
 * <p>
 * Example of camera simulation is available (with limitations) in PhotonVision only.
 * <p>
 * --------------------------------------------------------
 * <p>
 * Cameras that allow manual focus must be checked and focused. It's easy to focus a camera by eye
 * or use the PhotonVision focus function or the LimelightVision focus pipeline for its own camera.
 * A Siemens Star can be used or any object at about the typical distance that the camera is to be
 * used. Mark the lens holder where the camera is focused and glue into place if it's prone to
 * moving.
 * <p>
 * Autofocus sounds like a good idea but some inexpensive cameras jitter violently trying to focus
 * continuously especially while the camera is moving (with the robot, of course). Autofocus likely
 * is more harmful than helpful.
 * <p>
 * Cameras need to be calibrated - creating a camera matrix at the desired resolution. Calibrate
 * using PhotonVision and copy its camera matrix from the downloaded setting file or try this
 * online calibration database and calibrator https://www.calibdb.net/ . LimelightVision includes
 * a calibration process for its own camera.
 * <p>
 * For team's customized use, it is fairly easy to copy the code of one or more of the vision
 * systems into a team's own project.
 * <p>
 * If desired, unneeded vision code may be stripped out based on each file's use:
 * <p>
 * AcquireAprilTag.java CV
 * <p>
 * AcquireRobotPose.java CV
 * <p>
 * AlignToHUBPairedTagFieldRelativePose3D.java CV, PV, LV (neutral example command)
 * <p>
 * AlignToTag12FieldRelativePose3D.java CV, PV, LV (neutral example command)
 * <p>
 * AlignToPairedTagTargetRelativeTransform3D.java CV, PV, LV (neutral example command)
 * <p>
 * AlignToTag12RelativeArcade2D.java CV, PV, LV (neutral example command)
 * <p>
 * AprilTagsLocations.java CV, PV, LV (LV only by association with the example alignment commands
 * that may use the tag locations)
 * <p>
 * CameraBase.java CV, PV, LV
 * <p>
 * CommandSchedulerLog.java CV, PV, LV
 * <p>
 * Config.java CV, PV, LV
 * <p>
 * Constants CV
 * <p>
 * ControllerVision.java CV
 * <p>
 * Drivetrain.java CV, PV, LV (fake for drivetrain and gyro for alignment commands)
 * <p>
 * Image.java CV
 * <p>
 * LimelightHelpers.java LV
 * <p>
 * LimelightVision.java LV
 * <p>
 * Network.java PV, LV
 * <p>
 * Main.java CV, PV, LV
 * <p>
 * PhotonVision.java PV
 * <p>
 * Robot.java CV, PV, LV
 * <p>
 * RobotContainer.java CV, PV, LV
 * <p>
 * RobotPose.java CV, PV, LV
 * <p>
 * SpikeFilter.java CV
 * <p>
 * VisionContainer.java CV, PV, LV (four cases of the two undesired vision processes may be
 * removed)
 * <p>
 * {@summary AprilTag Vision Wrappers for "snippets" for WPILib, PhotonVision, and LimelightVision}
 * @author Mr. Thomas
 * @author Gerthworm (reviewer)
 * @author Sticks (reviewer)
 * @version 1/23/2026
 */

@Logged
public class Robot extends TimedRobot {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    public Robot() {
        new RobotContainer();
    }

    @Override
    public void robotPeriodic() // called last in the periodic loop despite always seen written first in Robot.java
    {
        RobotContainer.runBeforeCommands();
        CommandScheduler.getInstance().run();
    }
}
