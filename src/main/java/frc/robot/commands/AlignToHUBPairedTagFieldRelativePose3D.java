package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.AprilTagsLocations;
import frc.robot.vision.RobotPose;
import frc.robot.vision.VisionContainer;

/**
 * Robot approach target and drive to requested pose based on the first HUB paired tag seen.
 * <P>
 * This command uses robot Pose3d which is based on LL MegaTag2 that requires the correct robot heading.
 * <p>
 * Assume all scoring positions the same relative transform from the above determined left AprilTag
 * "scoringPositionFromLeftTag" is set in this code
 * <p>
 * This example uses chassis speed.
 * <p>
 * Pick what the drivetrain interface actually needs.
 * <p>
 * This is an example for only the HUB paired AprilTags and that is determined in the code.
 * <p>
 * Related references:
 * Started with a view of BetaBot2025-develop[ElectroBunny].zip
 * 3 PIDs to align robot to reef tag 10 left or right branches
 * https://www.chiefdelphi.com/t/from-zero-to-auto-align-to-reef-tutorial-how-to-simply-auto-align-to-the-reef/494478
 * 
 * <p>
 * Other references for 3-D poses at LimelightVision.io and photonvision.org
 */
public class AlignToHUBPairedTagFieldRelativePose3D extends Command {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    // PID constants
    private static final double X_HUB_ALIGNMENT_KP = 1.;
    private static final double Y_HUB_ALIGNMENT_KP = 1.;
    private static final double ROT_HUB_ALIGNMENT_KP = 1.;

    private static final double X_TOLERANCE_HUB_ALIGNMENT = 0.06; // meters
    private static final double Y_TOLERANCE_HUB_ALIGNMENT = 0.06; // meters
    private static final double ROT_TOLERANCE_HUB_ALIGNMENT = Units.degreesToRadians(3.);

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
    private List<Integer> tagIDsPossible = new ArrayList<>();
    private Pose3d target;
    private VisionContainer visionContainer;
    private Drivetrain drivetrain;
    private boolean bail;
    private double xSpeed;
    private double ySpeed;
    private double rotValue;
    private int tagDesiredLeft;
    private int tagDesiredRight;

    public AlignToHUBPairedTagFieldRelativePose3D(VisionContainer visionContainer, Drivetrain drivetrain) {

        this.visionContainer = visionContainer;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        // Each of these tags is one of a pair of tags near a desired position of the HUB.

        // If using the fake robot heading of 0 for LL MegaTag2, only tags 25-26 and 3-4 make sense.
        // If using the fake robot heading of 180 for LL MegaTag2, only tags 9-10 and 19-20 make sense.

        // Either tag should get the robot to the target and the transforms will be similar
        // these offsets are dummy test data and only resemble but are not actual tag pair distances.

        // This command considers the robot in field position and so assume there is only one
        // scoring position for the pair. Each pair is at a different location, though, maybe
        // scoring position is the same relative to the tag pair. Fix this if not true.

        // This is self defining on which tag is nearby the scoring position so the driver
        // doesn't have to tell the robot where to go - driver just has to get the target in view.

        // these tags require a fake heading of 180, if using LL MegaTag2
        tagIDsPossible.add(10); // right (even tag, even index)
        tagIDsPossible.add(9); // left (odd tag, odd index)

        tagIDsPossible.add(20);
        tagIDsPossible.add(19);

        // these tags are at other angles and require different headings, if using LL MegaTag2
        // tagIDsPossible.add(4);
        // tagIDsPossible.add(3);
        // tagIDsPossible.add(8);
        // tagIDsPossible.add(5);
        // tagIDsPossible.add(2);
        // tagIDsPossible.add(11);
        // tagIDsPossible.add(26);
        // tagIDsPossible.add(25);
        // tagIDsPossible.add(24);
        // tagIDsPossible.add(21);
        // tagIDsPossible.add(18);
        // tagIDsPossible.add(27);

        xController = new PIDController(X_HUB_ALIGNMENT_KP, 0.0, 0); // +forward/-back translation
        xController.setTolerance(X_TOLERANCE_HUB_ALIGNMENT);

        yController = new PIDController(Y_HUB_ALIGNMENT_KP, 0.0, 0); // +left/-right translation
        yController.setTolerance(Y_TOLERANCE_HUB_ALIGNMENT);

        rotController = new PIDController(ROT_HUB_ALIGNMENT_KP, 0, 0); // +CCW/-CW rotation
        rotController.setTolerance(ROT_TOLERANCE_HUB_ALIGNMENT);
        rotController.enableContinuousInput(0., Math.PI * 2.);
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

        if (!tagIDsPossible.contains(pose.AprilTagId)) // make sure still looking at a possibly correct tag
        {
            System.out.println("Oops! " + pose.AprilTagId + " is not a paired HUB tag");
            return;
        }
        
        // first iteration determines which pair of tags to seek by the first one tag seen
        if (tagDesiredLeft < 0 || tagDesiredRight < 0) {
            // this algorithm works only if all the even indices are the right tag of the pair; that is add(right) then add(left)
            var tagSeenIndex = tagIDsPossible.indexOf(pose.AprilTagId);
            
            if (tagSeenIndex % 2 == 0) {
                tagDesiredLeft = tagIDsPossible.get(tagSeenIndex  + 1);
                tagDesiredRight = tagIDsPossible.get(tagSeenIndex);
            }
            else {
                tagDesiredLeft = tagIDsPossible.get(tagSeenIndex);
                tagDesiredRight = tagIDsPossible.get(tagSeenIndex - 1);
            }

        // assume all HUB scoring positions the same relative transform from the above determined left AprilTag
        var scoringPositionFromLeftTag = new Transform3d(1., -0.17, 0., new Rotation3d(0., 0., Math.PI));
        target = AprilTagsLocations.getTagLocation(tagDesiredLeft).plus(scoringPositionFromLeftTag);
        xController.setSetpoint(target.getX());
        yController.setSetpoint(target.getY());
        rotController.setSetpoint(target.getRotation().getZ());
        }

        // make sure still looking at a possibly correct tag.
        // if the robot pose is correct, it doesn't matter if it's from another tag or odometry.
        // checking for correct tag might help assure better accuracy being near the correct tag.
        if (pose.AprilTagId != tagDesiredLeft && pose.AprilTagId != tagDesiredRight)
        {
            System.out.println("Oops! " + pose.AprilTagId + " is not in the correct pair");
            return;
        }

        System.out.println("pose error " + (target.getX() - pose.pose3D.getX()) + " " +
                (target.getY() - pose.pose3D.getY()) + " " +
                (target.getRotation().getZ() - pose.pose3D.getRotation().getZ()) + 
                " using at least tag " + pose.AprilTagId + " of (" + tagDesiredLeft + ", " + tagDesiredRight + ")");

        xSpeed = xController.calculate(pose.pose3D.getX());
        ySpeed = yController.calculate(pose.pose3D.getY());
        rotValue = rotController.calculate(pose.pose3D.getRotation().getZ());
        // These pose calculate() use the camera which generally is slow to respond
        // (limelight claims to the contrary not withstanding).
        // Generally it's superior to use the robot pose as the best estimate using
        // odometry, gyro, and vision.
        // Also, odometry would be available if vision wasn't available because of slow
        // updates or no longer in view.
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveFieldRelative(new ChassisSpeeds(0, 0, 0));

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
            drivetrain.driveFieldRelative(new ChassisSpeeds(0, 0, 0));
        } else {
            drivetrain.driveFieldRelative(new ChassisSpeeds(xSpeed, ySpeed, rotValue));
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
