package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.AprilTagsLocations;
import frc.robot.vision.RobotPose;
import frc.robot.vision.VisionContainer;

/**
 * Robot approach target to desired distance and turn to desired yaw.
 * <p>
 * This example shows both differential drive arcade speed and chassis speed.
 * <p>
 * The units for those two concepts are very different so the PID Kp constants
 * must be very different but they aren't in this example. Don't do both ways -
 * pick what the drivetrain interface needs. 
 * <p>
 * This is an example for only one AprilTag and that is determined in the code. Note that the "best"
 * pose that is found may not be the one that is sought by this command. The wrong tag has to be
 * ignored or logic changed to find the desired tag even if it isn't the "best" tag.
 * <p>
 * Related references:
 * <p>
 * https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-with-visual-servoing
 * <p>
 * https://docs.photonvision.org/en/latest/docs/examples/aimingatatarget.html
 * <p>
 * https://docs.photonvision.org/en/latest/docs/examples/aimandrange.html
 * <p>
 * https://www.chiefdelphi.com/t/how-do-you-line-up-on-the-reef/503904/10 (code pasted below this
 * class; it's worth a look)
 */
public class AlignToTag12RelativeArcade2D extends Command {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    // need to know what target is seen to know target height from floor for distance calculation
    // and where scoring is relative to it
    // FIXME need dynamic determination of the tag to use (and change name of class)
    private int tagIDDesired = 12;
    private double targetHeight = AprilTagsLocations.getTagLocation(tagIDDesired).getZ(); // for distance calculation
    private double scoringDistance = 0.6; // stop a little before the tag; parallel to the floor in meters
    private double scoringAngle = Units.degreesToRadians(5.); // point slightly left of tag - radians

    private double cameraHeight;
    private double cameraPitch;

    private final VisionContainer visionContainer;

    // PID constants
    public static final double DISTANCE_TAG_12_ALIGNMENT_KP = 1.;
    public static final double ROT_TAG_12_ALIGNMENT_KP = 1.;

    public static final double DISTANCE_TOLERANCE_TAG_12_ALIGNMENT = 0.06; // meters
    public static final double ROT_TOLERANCE_TAG_12_ALIGNMENT = Units.degreesToRadians(3.);

    private PIDController distanceController;
    private PIDController rotController;

    // FIXME that's a long time to drive without knowing where to go
    // maybe stop motors after .25 sec but keep the command going for 1.?
    // odometry should take over if there is no vision after a short time like 1 iteration?
    // no vision then use odometry? This example does not include odometry
    public static final double DONT_SEE_TAG_WAIT_TIME = 10.;

    public static final double HOLD_POSE_VALIDATION_TIME = 0.3;

    private Timer dontSeeTagTimer;
    private Timer holdPoseValidationTimer;

    private boolean bail;
    private double distanceSpeed;
    private double rotSpeed;

    private Drivetrain drivetrain;

    public AlignToTag12RelativeArcade2D(VisionContainer visionContainer, Drivetrain drivetrain) {
        this.visionContainer = visionContainer;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        cameraHeight = visionContainer.getRobotToCamera().getZ();
        cameraPitch = visionContainer.getRobotToCamera().getRotation().getY();

        // System.out.println("info " + target + " " + AprilTagsLocations.getTagLocation(10) + " " + targetBranch + " end info");

        distanceController = new PIDController(DISTANCE_TAG_12_ALIGNMENT_KP, 0.0, 0); // +forward/-back translation
        distanceController.setSetpoint(scoringDistance);
        distanceController.setTolerance(DISTANCE_TOLERANCE_TAG_12_ALIGNMENT);

        rotController = new PIDController(ROT_TAG_12_ALIGNMENT_KP, 0, 0); // +CCW/-CW rotation
        rotController.setSetpoint(scoringAngle);
        rotController.setTolerance(ROT_TOLERANCE_TAG_12_ALIGNMENT);
        rotController.enableContinuousInput(0., Math.PI * 2.);
    }

    public void initialize() {
        holdPoseValidationTimer = new Timer();
        holdPoseValidationTimer.start();
        dontSeeTagTimer = new Timer();
        dontSeeTagTimer.start();
        bail = false;
        distanceController.reset();
        rotController.reset();
    }

    public void execute() {
        RobotPose pose;

        if (visionContainer.getRobotPose().isPresent()) { // see a tag so reset countdown to failure timer and process this iteration
            pose = visionContainer.getRobotPose().get();
            dontSeeTagTimer.reset();
        } else { // no tag seen so let the countdown to failure timer run and quit this iteration
            return;
        }

        if (pose.AprilTagId != tagIDDesired) // make sure still looking at the correct tag
        {
            System.out.println("Oops! " + pose.AprilTagId + " is the wrong tag. Point at tag " + tagIDDesired);
            return;
        }

        // get the camera frame pose information
        var targetPitch = pose.pitch;
        var targetYaw = pose.yaw;
        // robot to camera pitch convention in this project is "-" is camera pointing up and "+" is down.
        // the distanceToTarget() uses the opposite sign so flip it here
        var distanceToTarget = distanceToTarget(cameraHeight, targetHeight, -cameraPitch,
                Units.degreesToRadians(targetPitch));
        // controlling the pitch could be used directly instead of converting to distance but best
        // practice is to work in the engineering units that are familiar for the problem being
        // solved: Degrees of tilt? Not obvious and changeable with mounting. Meters of distance?
        // Clear and invariant.
        System.out.println("angle error " + (scoringAngle - targetYaw) +
                ", distance error " + (scoringDistance - distanceToTarget));

        // clamping the outputs to -1 to +1 is good for arcade drive but not so good for chassis speeds
        distanceSpeed = MathUtil.clamp(distanceController.calculate(distanceToTarget), -1.0, 1.0);
        rotSpeed = MathUtil.clamp(rotController.calculate(targetYaw), -1.0, 1.0);
        // This rotation calculate() uses the camera which generally is slow to respond
        // (limelight claims to the contrary not withstanding).
        // Generally it's superior to use the PID controller on the gyro reading. The PID
        // setpoint would be the gyro offset from the vision heading. Although the
        // desired gyro heading won't change much, update that setpoint as new vision data
        // is available.

    }

    @Override
    public void end(boolean interrupted) {
        // Two examples of how to drive - pick one way - maybe not either but what your drivetrain requires
        drivetrain.arcadeDrive(0, 0, false);
        drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));

        if (interrupted) {
            System.out.println("ended by interrupted");
        }
    }

    public boolean isFinished() {
        if (rotController.atSetpoint() &&
                distanceController.atSetpoint()) {
            // at goal pose so stop and see if it settles
            // Two examples of how to drive - pick one way - maybe not either but what your drivetrain requires
            drivetrain.arcadeDrive(0, 0, false);
            drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        } else {
            // Two examples of how to drive - pick one way - maybe not either but what your drivetrain requires
            drivetrain.arcadeDrive(distanceSpeed, rotSpeed, false);
            drivetrain.driveRobotRelative(new ChassisSpeeds(distanceSpeed, 0., rotSpeed));
            holdPoseValidationTimer.reset();
        }

        var dontSeeTag = dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME);
        var holdPose = holdPoseValidationTimer.hasElapsed(HOLD_POSE_VALIDATION_TIME);

        if (bail) {
            System.out.print(" ended by bail " + bail);
        }

        if (dontSeeTag) {
            System.out.print(" ended by don't see tag " + dontSeeTag);
        }

        if (holdPose) {
            System.out.print(" ended by correct pose held " + holdPose);
        }

        return bail || dontSeeTag || holdPose;
    }

    /**
     * Estimation of shortest distance parallel to the floor from camera to the vertical wall that
     * a target is mounted on based on Right Triangle geometry.
     * <p>
     * See https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
     * 
     * <p>
     * Note that the camera must mounted with 0 roll (rotation around the forward axis).
     * 
     * <p>
     * Highest accuracy if the camera is in line side-to-side with target and large difference in
     * height. That's inconsistent with camera positions that are best for 3-D pose estimation
     * where the camera should be at least a small angle to the side of a target in addition to
     * being at different heights.
     * 
     * <p>
     * There must be significant height difference for this to work accurately especially if far
     * from the target. Slight jitter in the pitch from robot movement causes a lot of jitter in
     * the distance calculation if far from the target and little difference between the camera
     * height and the target height (tangent function). Accuracy improves as the robot approaches
     * the target.
     * 
     * For low targets a better distance measurement may come from a proper distance sensor such as
     * the analog mode (at least for 2026) of
     * https://swyftrobotics.com/products/swyft-ranger-distance-sensor .
     * 
     * A distance sensor may or may not respond more quickly than a camera; there's a lot of
     * variation in the speed of cameras and distance sensors. The SWYFT is 2.5 Hz to 15 Hz which
     * is much slower than or about as fast as cameras. PhotonVision and LimelightVision may be
     * substantially faster than this.
     *
     * @param cameraHeight
     *            height of the camera off the floor in meters.
     * @param targetHeight
     *            height of the tag off the floor in meters.
     * @param cameraPitch
     *            pitch of the camera from the horizontal plane in radians.
     *            Positive values are up.
     * @param targetPitch
     *            pitch of the tag to the camera's lens in radians. Positive
     *            values are up.
     * @return estimated ground distance from the camera to the target in meters.
     */
    public static double distanceToTarget(
            double cameraHeight, // fixed
            double targetHeight, // fixed
            double cameraPitch, // fixed
            double targetPitch) // measured each frame
    {
        return (targetHeight - cameraHeight) // distance between heights
                / Math.tan(cameraPitch + targetPitch); // total angle with the floor
    }

    // // Some teams use a look-up-table alternative to using the tangent function in
    // // distanceToTarget().
    // // Since many hand measurements must be made and recorded and changed if the
    // // camera mount is changed, it is much harder to use.
    // // It still suffers if the camera and target are at about the same height.
    // distanceToTarget = new InterpolatingDoubleTreeMap();
    // // example degrees pitch to meters distance
    // // make enough measurements for linear interpolation to be accurate
    // distanceToTarget.put(-6., 1.5);
    // distanceToTarget.put(0., 0.5);
    // distanceToTarget.put(9., 0.25);

    // // Some teams use distance sensors. One example is the SWYFT analog distance sensor
    // AnalogInput distanceSensor = new AnalogInput(0);
    // distanceToTarget = distanceSensor.getVoltage()*32.50930976)-2.695384202);
}

/*
 * [
 * This vision project author found a technique below that was posted on ChiefDelphi. You can give
 * it a try. It hasn't been used here and is completely commented out.
 * 
 * This project author comments:
 * Apparently the vision system is in 2-D mode (not 3-D). In that case the AprilTag isn't returning
 * the rotation of the robot wrt the tag. The "flamekeeper" must somehow relate the robot
 * orientation with the tag orientation while in 2-D mode.
 * 
 * To do that one needs both the gyro reading and the tag rotation in the field from the tags in
 * field list. See any of the alignment commands on how the tag list from
 * {@link AprilTagsLocations)} is used.
 * ]
 * 
 * https://www.chiefdelphi.com/t/how-do-you-line-up-on-the-reef/503904/10
 * flamekeeper: I made the robot move forward based on the pitch of the tag, and combined
 * the rotation needed to get yaw to 0 and make the gyro line up with the tags field position:
 * 
 * if (vision.hasTarget()) {
 * double yaw = vision.getTargetYaw(); // yaw offset
 * double pitch = vision.getTargetPitch(); // vertical offset
 * double roff = vision.getRotOffset();
 * 
 * double turnOutput = MathUtil.clamp(turnPID.calculate(roff, 0.0), -1.0, 1.0);
 * double forwardOutput = MathUtil.clamp(forwardPID.calculate(pitch, desiredPitch), -1.0, 1.0);
 * double strafeOut = MathUtil.clamp(strafePID.calculate(yaw, 0.0), -1.0, 1.0);
 * 
 * xs = drivetrain.getCos() * maxN * -forwardOutput;
 * ys = drivetrain.getSin() * maxN * -forwardOutput;
 * xs += drivetrain.getPose().getRotation().getSin() * maxN * strafeOut;
 * ys += -drivetrain.getPose().getRotation().getCos() * maxN * strafeOut;
 * xs /= 2;
 * ys /= 2;
 * rs = turnOutput; // rotation
 * } else {
 * // Stop if target lost
 * xs = 0.0;
 * ys = 0.0;
 * rs = 0.0;
 * }
 * 
 * // Stop when PID reaches setpoint
 * if (vision.hasTarget() && turnPID.atSetpoint() && forwardPID.atSetpoint()) {
 * xs = 0.0;
 * ys = 0.0;
 * rs = 0.0;
 * isAutoAligning = false;
 * }
 */