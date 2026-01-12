package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Fake drivetrain including gyro
 */
public class Drivetrain extends SubsystemBase {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    // almost fake drive motors to show how it might be done
    // the PWM ports are live with this
    private final DifferentialDrive robotDrive;
    private final PWMTalonFX leftMotor = new PWMTalonFX(0);
    private final PWMTalonFX rightMotor = new PWMTalonFX(1);

    // essentially fake gyro to show how it might be done
    @SuppressWarnings("unused")
    private AnalogGyro gyro; // unused because of getHeading and getRate

    public Drivetrain() {
        robotDrive = new DifferentialDrive((speed) -> leftMotor.set(speed), (speed) -> rightMotor.set(speed));
        robotDrive.setSafetyEnabled(false);

        gyro = new AnalogGyro(0); // the analog port is live with this
    }

    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        robotDrive.arcadeDrive(xSpeed, zRotation, squareInputs);
    }

    /**
     * dummy
     * 
     * @param chassisSpeeds
     *            the desired robot chassis speed
     */
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        // goal current speeds
        SmartDashboard.putString("ChassisSpeeds", chassisSpeeds.toString());
    }

    /**
     * dummy zeros
     */
    public Pose2d getPose() {
        return getState().pose;
    }

    /**
     * dummy zeros
     */
    public SwerveDriveState getState() {
        return new SwerveDriveState();
    }

    /**
     * dummy zeros
     */
    public class SwerveDriveState {
        public Pose2d pose = new Pose2d(); // The current pose of the robot
        public ChassisSpeeds Speeds = new ChassisSpeeds(); // The current robot-centric velocity
    }

    /**
     * dummy - constant value degrees
     * 
     * @return 0. // FIXME fake heading good for 2026 RebuilT AprilTags 1-6, 13-16, 23-28
     * @return 180. // FIXME fake heading good for 2026 RebuilT AprilTags 7, 9-10, 12, 17, 19-20, 22, 29-32
     */
    public double getHeading() {
        // return gyro.getAngle();
        return 180.;
    }

    /**
     * dummy zero
     * 
     * @return 0. // FIXME fake rate
     */
    public double getRate() {
        // return gyro.getRate();
        return 0.;
    }

    /**
     * Sample of PathPlanner followPath to drive to target position. It is essentially the PID
     * controllers similar to the above command controllers and would replace the above command.
     * <p>
     * Drives autonomously from the given pose to the target pose using PathPlanner
     * <p>
     * PathPlanner AutoBuilder must be configured and tuned before usage.
     * <p>
     * This code has not been verified but came from functioning team code.
     * 
     * @param targetPose
     * @param currentPose
     * @return Command to follow path on the fly
     * @author Biggie Cheese
     */
    public /* static */ Command driveToPositionCommand(Pose2d targetPose, Pose2d currentPose) {
        PathConstraints constraints = new PathConstraints(2.0, 1.0, Units.degreesToRadians(360),
                Units.degreesToRadians(360));
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPose.getTranslation(), currentPose.getRotation()),
                new Pose2d(targetPose.getTranslation(), targetPose.getRotation()));

        double vxMetersPerSecond = getState().Speeds.vxMetersPerSecond;
        double vyMetersPerSecond = getState().Speeds.vyMetersPerSecond;

        double velocity = Math.hypot(vxMetersPerSecond, vyMetersPerSecond);

        Rotation2d rotation = getPose().getRotation();

        IdealStartingState idealStartingState = new IdealStartingState(velocity, rotation);

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                idealStartingState, // set this to null if not working
                new GoalEndState(0.0, targetPose.getRotation()));
        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }
}
/*
2026 RebuilT welded
AprilTag(ID: 1, pose: Pose3d(Translation3d(X: 11.88, Y: 7.42, Z: 0.89), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0))))    0.0,    0.0,  180.0 [degrees]
AprilTag(ID: 2, pose: Pose3d(Translation3d(X: 11.92, Y: 4.64, Z: 1.12), Rotation3d(Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476))))    0.0,    0.0,   90.0 [degrees]
AprilTag(ID: 3, pose: Pose3d(Translation3d(X: 11.31, Y: 4.39, Z: 1.12), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0))))    0.0,    0.0,  180.0 [degrees]
AprilTag(ID: 4, pose: Pose3d(Translation3d(X: 11.31, Y: 4.03, Z: 1.12), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0))))    0.0,    0.0,  180.0 [degrees]
AprilTag(ID: 5, pose: Pose3d(Translation3d(X: 11.92, Y: 3.43, Z: 1.12), Rotation3d(Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476))))    0.0,    0.0,  -90.0 [degrees]
AprilTag(ID: 6, pose: Pose3d(Translation3d(X: 11.88, Y: 0.64, Z: 0.89), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0))))    0.0,    0.0,  180.0 [degrees]
AprilTag(ID: 7, pose: Pose3d(Translation3d(X: 11.95, Y: 0.64, Z: 0.89), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
AprilTag(ID: 8, pose: Pose3d(Translation3d(X: 12.27, Y: 3.43, Z: 1.12), Rotation3d(Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476))))    0.0,    0.0,  -90.0 [degrees]
AprilTag(ID: 9, pose: Pose3d(Translation3d(X: 12.52, Y: 3.68, Z: 1.12), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
AprilTag(ID: 10, pose: Pose3d(Translation3d(X: 12.52, Y: 4.03, Z: 1.12), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
AprilTag(ID: 11, pose: Pose3d(Translation3d(X: 12.27, Y: 4.64, Z: 1.12), Rotation3d(Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476))))    0.0,    0.0,   90.0 [degrees]
AprilTag(ID: 12, pose: Pose3d(Translation3d(X: 11.95, Y: 7.42, Z: 0.89), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
AprilTag(ID: 13, pose: Pose3d(Translation3d(X: 16.53, Y: 7.40, Z: 0.55), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0))))    0.0,    0.0,  180.0 [degrees]
AprilTag(ID: 14, pose: Pose3d(Translation3d(X: 16.53, Y: 6.97, Z: 0.55), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0))))    0.0,    0.0,  180.0 [degrees]
AprilTag(ID: 15, pose: Pose3d(Translation3d(X: 16.53, Y: 4.32, Z: 0.55), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0))))    0.0,    0.0,  180.0 [degrees]
AprilTag(ID: 16, pose: Pose3d(Translation3d(X: 16.53, Y: 3.89, Z: 0.55), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0))))    0.0,    0.0,  180.0 [degrees]
AprilTag(ID: 17, pose: Pose3d(Translation3d(X: 4.66, Y: 0.64, Z: 0.89), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
AprilTag(ID: 18, pose: Pose3d(Translation3d(X: 4.63, Y: 3.43, Z: 1.12), Rotation3d(Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476))))    0.0,    0.0,  -90.0 [degrees]
AprilTag(ID: 19, pose: Pose3d(Translation3d(X: 5.23, Y: 3.68, Z: 1.12), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
AprilTag(ID: 20, pose: Pose3d(Translation3d(X: 5.23, Y: 4.03, Z: 1.12), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
AprilTag(ID: 21, pose: Pose3d(Translation3d(X: 4.63, Y: 4.64, Z: 1.12), Rotation3d(Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476))))    0.0,    0.0,   90.0 [degrees]
AprilTag(ID: 22, pose: Pose3d(Translation3d(X: 4.66, Y: 7.42, Z: 0.89), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
AprilTag(ID: 23, pose: Pose3d(Translation3d(X: 4.59, Y: 7.42, Z: 0.89), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0))))    0.0,    0.0,  180.0 [degrees]
AprilTag(ID: 24, pose: Pose3d(Translation3d(X: 4.27, Y: 4.64, Z: 1.12), Rotation3d(Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476))))    0.0,    0.0,   90.0 [degrees]
AprilTag(ID: 27, pose: Pose3d(Translation3d(X: 4.27, Y: 3.43, Z: 1.12), Rotation3d(Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476))))    0.0,    0.0,  -90.0 [degrees]
AprilTag(ID: 28, pose: Pose3d(Translation3d(X: 4.59, Y: 0.64, Z: 0.89), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0))))    0.0,    0.0,  180.0 [degrees]
AprilTag(ID: 29, pose: Pose3d(Translation3d(X: 0.01, Y: 0.67, Z: 0.55), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
AprilTag(ID: 30, pose: Pose3d(Translation3d(X: 0.01, Y: 1.10, Z: 0.55), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
AprilTag(ID: 31, pose: Pose3d(Translation3d(X: 0.01, Y: 3.75, Z: 0.55), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
AprilTag(ID: 32, pose: Pose3d(Translation3d(X: 0.01, Y: 4.18, Z: 0.55), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))))    0.0,    0.0,    0.0 [degrees]
 */