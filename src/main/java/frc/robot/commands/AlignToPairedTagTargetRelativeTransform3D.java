package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.RobotPose;
import frc.robot.vision.VisionContainer;

/**
 * Robot approach target and drive to requested pose based on the relative position to the target.
 * That is, this command drives to a Transform3d coordinate location near the target as opposed to
 * the {@link AlignToTag12FieldRelativePose3D} which drives to a field location coordinate.
 * <p>
 * This example uses chassis speed.
 * <p>
 * Pick what the drivetrain interface actually needs.
 * <p>
 * In RebuilT 2026 there are several target positions that have a pair of AprilTags. Validation is
 * done to assure the best tag in view is one of those in the set of paired relative tags.
 * <p>
 * Thus, in use, this command must have a list of (tags, Transform3d) that are the coordinates of
 * the robot's target positions relative to each of the corresponding tags.
 * <p>
 * Related references:
 * Started with a view of BetaBot2025-develop[ElectroBunny].zip
 * 3 PIDs to align robot to reef tag 10 left or right branches
 * https://www.chiefdelphi.com/t/from-zero-to-auto-align-to-reef-tutorial-how-to-simply-auto-align-to-the-reef/494478
 * <p>
 * Other references for 3-D poses at LimelightVision.io and photonvision.org
 */
public class AlignToPairedTagTargetRelativeTransform3D extends Command {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    // PID constants
    private static final double X_PAIRED_TAG_ALIGNMENT_KP = 1.;
    private static final double Y_PAIRED_TAG_ALIGNMENT_KP = 1.;
    private static final double ROT_PAIRED_TAG_ALIGNMENT_KP = 1.;

    private static final double X_TOLERANCE_PAIRED_TAG_ALIGNMENT = 0.06; // meters
    private static final double Y_TOLERANCE_PAIRED_TAG_ALIGNMENT = 0.06; // meters
    private static final double ROT_TOLERANCE_PAIRED_TAG_ALIGNMENT = Units.degreesToRadians(5.);

    private PIDController xController;
    private PIDController yController;
    private PIDController rotController;

    // FIXME that's a long time to drive without knowing where to go
    // maybe stop motors after .25 sec but keep the command going for 1.?
    // odometry should take over if there is no vision after a short time like 1 iteration?
    // no vision then use odometry? This example does not include odometry
    private static final double DONT_SEE_TAG_WAIT_TIME = 10.;
    private static final double HOLD_POSE_VALIDATION_TIME = 0.3;

    private Timer dontSeeTagTimer;
    private Timer holdPoseValidationTimer;
    private Map<Integer, Transform3d> tagIDsPossible = new HashMap<>();
    private Transform3d target;
    private VisionContainer visionContainer;
    private Drivetrain drivetrain;
    private boolean bail;
    private double xSpeed;
    private double ySpeed;
    private double rotValue;
    private int tagDesiredLeft;
    private int tagDesiredRight;

    public AlignToPairedTagTargetRelativeTransform3D(VisionContainer visionContainer, Drivetrain drivetrain) {

        this.visionContainer = visionContainer;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        // Each of these tags is one of a pair of tags near a desired position
        // Either tag should get the robot to the target and the transforms will be similar
        // these offsets are dummy test data and only resemble but are not actual tag pair distances

        // If pose algorithms used here were changed to use heading, these need robot heading of 180
        // For example: I want to lineup straight on tag 10 and 1 m away.
        // If pointing straight on tag 10 the left/right is y = 0 and x is +1 m
        // If pointing straight on tag 9 the left/right is go right so tag 9 appears to the left at y = 0.35 and x is +1 m
        tagIDsPossible.put(9, new Transform3d(1., 0.35, 0., new Rotation3d(0., 0., Math.PI)));
        tagIDsPossible.put(10, new Transform3d(1., 0.0, 0., new Rotation3d(0., 0., Math.PI)));

        tagIDsPossible.put(19, new Transform3d(1., 0.35, 0., new Rotation3d(0., 0., Math.PI)));
        tagIDsPossible.put(20, new Transform3d(1., 0.0, 0., new Rotation3d(0., 0., Math.PI)));

        tagIDsPossible.put(29, new Transform3d(1., 0.35, 0., new Rotation3d(0., 0., Math.PI)));
        tagIDsPossible.put(30, new Transform3d(1., 0.0, 0., new Rotation3d(0., 0., Math.PI)));

        tagIDsPossible.put(31, new Transform3d(1., 0.35, 0., new Rotation3d(0., 0., Math.PI)));
        tagIDsPossible.put(32, new Transform3d(1., 0.0, 0., new Rotation3d(0., 0., Math.PI)));

        // If pose algorithms used here were changed to use heading, these need robot heading of 0
        tagIDsPossible.put(3, new Transform3d(1., 0.35, 0., new Rotation3d(0., 0., Math.PI))); // left (odd)
        tagIDsPossible.put(4, new Transform3d(1., 0.0, 0., new Rotation3d(0., 0., Math.PI))); // right (even)

        tagIDsPossible.put(13, new Transform3d(1., 0.35, 0., new Rotation3d(0., 0., Math.PI)));
        tagIDsPossible.put(14, new Transform3d(1., 0.0, 0., new Rotation3d(0., 0., Math.PI)));

        tagIDsPossible.put(15, new Transform3d(1., 0.35, 0., new Rotation3d(0., 0., Math.PI)));
        tagIDsPossible.put(16, new Transform3d(1., 0.0, 0., new Rotation3d(0., 0., Math.PI)));

        tagIDsPossible.put(25, new Transform3d(1., 0.35, 0., new Rotation3d(0., 0., Math.PI)));
        tagIDsPossible.put(26, new Transform3d(1., 0.0, 0., new Rotation3d(0., 0., Math.PI)));
 
        // The robot can score at a position to the right or left of the HUB AprilTags
        xController = new PIDController(X_PAIRED_TAG_ALIGNMENT_KP, 0.0, 0); // +forward/-back translation
        xController.setTolerance(X_TOLERANCE_PAIRED_TAG_ALIGNMENT);

        yController = new PIDController(Y_PAIRED_TAG_ALIGNMENT_KP, 0.0, 0); // +left/-right translation
        yController.setTolerance(Y_TOLERANCE_PAIRED_TAG_ALIGNMENT);

        rotController = new PIDController(ROT_PAIRED_TAG_ALIGNMENT_KP, 0, 0); // +CCW/-CW rotation
        rotController.setTolerance(ROT_TOLERANCE_PAIRED_TAG_ALIGNMENT);
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {

        holdPoseValidationTimer = new Timer();
        holdPoseValidationTimer.start();
        dontSeeTagTimer = new Timer();
        dontSeeTagTimer.start();
        bail = false;
        xController.reset();
        yController.reset();
        rotController.reset();
        tagDesiredLeft = -1; // initially haven't determined tag; will set in first iteration
        tagDesiredRight = -1;
    }

    @Override
    public void execute() {

        RobotPose pose;

        if (visionContainer.getRobotPose().isPresent()) { // see a tag so reset countdown to failure timer and process this iteration
            pose = visionContainer.getRobotPose().get();
            dontSeeTagTimer.reset();
        } else { // no tag seen so let the countdown to failure timer run and quit this iteration
            return;
        }

        if (pose.pose3D.equals(Pose3d.kZero)) // make sure there is 3-D vision; this is a cheat that works most of the time
        {
            System.out.println("Oops! 3-D processing mode not activated in selected vision system");
            bail = true;
            return;
        }

        if (!tagIDsPossible.containsKey(pose.AprilTagId)) // make sure still looking at a possibly correct tag
        {
            System.out.println("Oops! " + pose.AprilTagId + " is not a paired tag");
            return;
        }

        // determine which pair of tags to seek by the first ones seen
        // this algorithm works only if all the odds are lower values than rights
        if (tagDesiredLeft < 0 || tagDesiredRight < 0) {
            if (pose.AprilTagId % 2 == 0) {
                tagDesiredLeft = pose.AprilTagId -1;
                tagDesiredRight = pose.AprilTagId;
            }
            else {
                tagDesiredLeft = pose.AprilTagId;
                tagDesiredRight = pose.AprilTagId + 1;
            }
        }

        if (pose.AprilTagId != tagDesiredLeft && pose.AprilTagId != tagDesiredRight) // make sure still looking at a possibly correct tag
        {
            System.out.println("Oops! " + pose.AprilTagId + " is not in the correct pair");
            return;
        }

        target = tagIDsPossible.get(pose.AprilTagId);
        xController.setSetpoint(target.getX());
        yController.setSetpoint(target.getY());
        rotController.setSetpoint(target.getRotation().getZ());

        // Because the robot pose is transform3d from robot to tag, the PID controller must be in reverse
        // Too big robot x position means drive forward at +speed to reduce the -delta which is Setpoint-Measurement

        System.out.println("pose error " + (-target.getX() + pose.cameraToTarget.getX()) + " " +
                (-target.getY() + pose.cameraToTarget.getY()) + " " +
                (-target.getRotation().getZ() + pose.cameraToTarget.getRotation().getZ()) + 
                " using tag " + pose.AprilTagId + " of (" + tagDesiredLeft + ", " + tagDesiredRight + ")");

        xSpeed = -xController.calculate(pose.cameraToTarget.getX());
        ySpeed = -yController.calculate(pose.cameraToTarget.getY());
        rotValue = -rotController.calculate(pose.cameraToTarget.getRotation().getZ());
        // These pose calculate() use the camera which generally is slow to respond
        // (limelight claims to the contrary not withstanding).
        // Generally it's superior to use the robot pose as the best estimate using
        // odometry, gyro, and vision.
        // Also, odometry would be available if vision wasn't available because of slow
        // updates or no longer in view.
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));

        if (interrupted) {
            System.out.println(" ended by interrupted ");
        }
    }

    @Override
    public boolean isFinished() {

        if (rotController.atSetpoint() &&
                yController.atSetpoint() &&
                xController.atSetpoint()) {
            // at goal pose so stop and see if it settles
            drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        } else {
            drivetrain.driveRobotRelative(new ChassisSpeeds(xSpeed, ySpeed, rotValue));
            holdPoseValidationTimer.reset();
        }

        var dontSeeTag = dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME);
        var holdPose = holdPoseValidationTimer.hasElapsed(HOLD_POSE_VALIDATION_TIME);

        if (bail) {
            System.out.print(" ended by bail ");
        }

        if (dontSeeTag) {
            System.out.print(" ended by don't see tag ");
        }

        if (holdPose) {
            System.out.print(" ended by correct pose held ");
        }

        return bail || dontSeeTag || holdPose;
    }
}
