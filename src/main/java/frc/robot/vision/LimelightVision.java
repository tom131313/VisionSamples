package frc.robot.vision;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.utils.Network.getMyIPAddress;

import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

/**
 * Limelight requires significant setup using the dashboard and, somewhat, code can be used.
 * 
 * <p>
 * The LimeLight fmap (field map) is at
 * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-map-specification
 * 
 * <p>
 * The Limelight coordinate systems are at
 * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems
 */

/**
 * Implementation of LimelightVision. This class can be used with the VisionContainer class or
 * without that wrapper by the example usage described below.
 * 
 * <p>
 * Simple validation/filtering of MegaTag and MegaTag2 are presented in {@link #update()} and can
 * be changed as determined by game and team use.
 * 
 * <p>
 * This class acquires the Limelight target data and the MegaTag and MegaTag2 robot poses in field.
 * 
 * <p>
 * In addition to what Limelight publishes in NetworkTables this program publishes Limelight
 * MegaTag and MegaTag2 and filtered robot Pose2d and Pose3d to table limelightName + "Logged".
 * 
 * <p>
 * In part it is a stripped-down version of the LimelightHelpers, somewhat uses more efficient
 * access to the Limelight, uses some LimelightHelpers, and some featured examples to access and
 * process Limelight data.
 * <p>
 * Example usage NOT using {@link VisionContainer}:
 * 
 * <pre>
 * <code>
    // construct one limelight object; instantiate more objects if multiple cameras
    boolean useLL1 = true; // make using LL1 an option
    CameraLL LL1; // first LimeLight Vision camera
    String limelightName1 = "limelight";
    if (useLL1 && LimelightVision.isAvailable(limelightName1))
    {
        LL1 = new LimelightVision(limelightName1, RobotContainer.getDrivetrain());

        // put additional LL setup as needed
        LimelightHelpers.setStreamMode_PiPSecondary(limelightName1); // if there is a driver camera, where to put the image - pick your favorite
    }
    else
    {
        DriverStation.reportWarning("No " + limelightName1 + " connection", false);

        LL1 = null;
    }

    // periodically set and get data
    if (LL1 != null)
    {
        // for MegaTag2 set the robot orientation from the gyro heading and rate in the Limelight.
        LimelightHelpers.SetRobotOrientation(limelightName1, getHeading(), getRate(), 0., 0.,0., 0.);

        LL1.update();

        // some methods to use:
        if (LL1.isFresh())
        {
            // --- TARGET  STATISTICS ---
            System.out.println(LL1.getTagID() + " tag id");
            System.out.println(LL1.getTX() + " yaw");
            System.out.println(LL1.getTXNC() + " yaw offset");
            System.out.println(LL1.getTY() + " pitch");
            System.out.println(LL1.getTYNC() + " pitch offset");
            System.out.println(LL1.getTA() + " target area");
            System.out.println(LL1.getCameraToTarget() + " camera to tag transform");
            System.out.println(LL1.getTimestampSeconds() + " Time of the pose");
            System.out.println(LL1.getCameraToTarget() + " camera to tag transform\n"); 
            System.out.println(LL1.getPose2dMT1() + " pose 2-D MegaTag");
            System.out.println(LL1.getPose2dMT2() + " pose 2-D MegaTag2");
            System.out.println(LL1.getPose2d() + " best estimate pose 2-D from MT or MT2");
            System.out.println(LL1.isMegaTag2() + " true=> pose from MT2; false=> pose from MT");
            System.out.println(LL1.getLatency() + " total latency of the pose");
            System.out.println(LL1.getTagCount() + " number of tags seen to make the pose");
            System.out.println(LL1.getSuggestResetOdometry() + " validation suggests highly reliable vision pose")

            System.out.println(LL1); // prints same data as above plus more statistics

            // simplistic data usage example for robot pose update.
            if (LL1.getSuggestResetOdometry())
            { // Believe completely the vision pose - it passed validation.
                var pose = LL1.getPose2d();
                poseEstimator.resetPose(pose);
                odometry.resetPose(pose);
            }
            else
            { // Merge somewhat uncertain vision pose with odometry to make an estimate.

                // Standard deviations for MT1 and MT2 can be found in the "stddevs" with
                // sample code herein commented out.

                // Put the optional gyro on the robot and use it for field oriented driving and greatly
                // improved pose estimation with MegaTag2 and then don't use vision headings (yaw).
                if (isMegaTag2())
                {
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9_999_999.)); // MT2
                }
                else
                {
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9_999_999.)); // MT
                }
                poseEstimator.addVisionMeasurement(LL1.getPose2d(), LL1.getTimestampSeconds());
            }
        }
    }
</code>
 * </pre>
 */
public class LimelightVision extends CameraBase {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    private final String limelightName; // name of the limelight
    private static final NetworkTableInstance NTinstance = NetworkTableInstance.getDefault();
    private final NetworkTable LLtable;
    private final NetworkTable LLtableLog;
    private final DoubleArraySubscriber t2d; // LL t2d array containing several values for matched-timestamp statistics
    private final StructPublisher<Pose2d> publishRobotPoseMT1;
    private final StructPublisher<Pose2d> publishRobotPoseMT2;
    private final StructPublisher<Pose2d> publishRobotPose2d;
    private final StructPublisher<Pose3d> publishRobotPose3d;

    private boolean suggestResetOdometry = false; // indicates excellent LL pose estimation; initialize in case used before an acquire
    private boolean isMegaTag2 = false;
    // fields associated with statistics and target
    private TimestampedDoubleArray stats;
    private int targetCount;
    private double latency;
    private double tx;
    private double ty;
    private double txnc;
    private double tync;
    private double ta;
    private int tid;
    private boolean isFresh;
    private long previousTimestamp = 0; // arbitrary initial time to start
    private double timestampSeconds;
    private Pose2d robotInField = Pose2d.kZero;
    private Transform3d cameraToTarget;

    private PoseEstimate MT1;
    private PoseEstimate MT2;
    private Drivetrain drivetrain;

    public LimelightVision(String limelightName, Drivetrain drivetrain) {
        this.limelightName = limelightName;
        this.drivetrain = drivetrain;

        LLtable = NTinstance.getTable(limelightName); // Get the limelight table
        LLtableLog = NTinstance.getTable(limelightName + "Logged"); // new table for output from this class

        // Roll our own access to t2d instead of using LimelightHelpers
        // This assures all the data is a single consistent transaction and can use getAtomic for
        // the associated timestamp
        t2d = LLtable.getDoubleArrayTopic("t2d").subscribe(new double[] {}); // default is no data (array length = 0)

        publishRobotPoseMT1 = LLtableLog.getStructTopic("robotPose2DMT1", Pose2d.struct).publish();
        publishRobotPoseMT2 = LLtableLog.getStructTopic("robotPose2DMT2", Pose2d.struct).publish();
        publishRobotPose2d = LLtableLog.getStructTopic("robotPose2d", Pose2d.struct).publish();
        publishRobotPose3d = LLtableLog.getStructTopic("robotPose3d", Pose3d.struct).publish();
    }

    /**
     * getter for validity of last acquisition
     * <p>
     * data must be valid and different timestamp than the previous data
     * 
     * @return freshness of the last acquisition
     */
    public boolean isFresh() {
        return isFresh;
    }

    /**
     * 
     * GETTERS FOR t2d DATA
     * 
     */

    /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     * 
     * @return Horizontal offset angle in degrees
     */
    public double getTX() {
        return tx;
    }

    /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     * 
     * @return Vertical offset angle in degrees
     */
    public double getTY() {
        return ty;
    }

    /**
     * Gets the horizontal offset from the principal pixel/point to the target in degrees.
     * <p>
     * This is the most accurate 2d metric if you are using a calibrated camera and you don't need
     * adjustable crosshair functionality.
     * 
     * @return Horizontal offset angle in degrees
     */
    public double getTXNC() {
        return txnc;
    }

    /**
     * Gets the vertical offset from the principal pixel/point to the target in degrees.
     * <p>
     * This is the most accurate 2d metric if you are using a calibrated camera and you don't need
     * adjustable crosshair functionality.
     * 
     * @return Vertical offset angle in degrees
     */
    public double getTYNC() {
        return tync;
    }

    /**
     * Gets the area of the target
     * 
     * @return area of the target
     */
    public double getTA() {
        return ta;
    }

    /**
     * Primary targeted tag id for use with getTX, getTXNC, getTY, getTYNC
     * <p>
     * Beware of usage with pose2d as a tag id is not an inherent property of the pose
     * 
     * @return tag id
     */
    public int getTagID() {
        return tid;
    }

    /**
     * Camera-Space Transform3d of how a tag has moved away from the camera that is facing the
     * camera
     * 
     * @return Transform from camera to tag
     */
    public Transform3d getCameraToTarget() {
        return cameraToTarget;
    }

    /**
     * User specified validation and filtering suggests pose is good enough
     * to use to reset the PoseEstimator with the value in getPose2d().
     * 
     * @return
     */
    public boolean getSuggestResetOdometry() {
        return suggestResetOdometry;
    }

    /**
     * MegaTag1 or MegaTag2 Pose2d as determined by user filtering
     * <p>
     * use for PoseEstimator.addVisionMeasurement
     * <p>
     * see {@link #isMegaTag2()} and {@link #getSuggestResetOdometry()}
     * 
     * @return robot position
     */
    public Pose2d getPose2d() {
        return robotInField;
    }

    /**
     * MegaTag1 or MegaTag2 Pose2d as determined by user filtering
     * converted to Pose3d with z = 0 and x angle and y angle = 0
     * <p>
     * use for PoseEstimator.addVisionMeasurement
     * <p>
     * see {@link #getPose2d()}
     * 
     * @return robot position clamped to floor
     */
    public Pose3d getPose3d() {
        return new Pose3d(getPose2d());
    }

    /**
     * Timestamp getter from pose (includes latency)
     * <p>
     * use for PoseEstimator.addVisionMeasurement
     * 
     * @return
     */
    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    /**
     * Total latency getter
     * 
     * @return
     */
    public double getLatency() {
        return latency;
    }

    /**
     * Tag count getter
     * 
     * @return
     */
    public int getTargetCount() {
        return targetCount;
    }

    /**
     * 
     * @return MegaTag
     */
    public Pose2d getPose2dMT1() {
        return MT1.pose;
    }

    /**
     * 
     * @return MegaTag2
     */
    public Pose2d getPose2dMT2() {
        return MT2.pose;
    }

    /**
     * Basis of the robot pose [see {@link #getPose2d()}]
     * 
     * @return true => MegaTag2 pose; false => MegaTag pose
     */
    public boolean isMegaTag2() {
        return isMegaTag2;
    }

    /*
     * t2d
     * doubleArray containing several values for matched-timestamp statistics:
     * targetValid, [0]
     * targetCount, [1]
     * targetLatency, [2]
     * captureLatency, [3]
     * tx, [4]
     * ty, [5]
     * txnc, [6]
     * tync, [7]
     * ta, [8]
     * tid, [9]
     * targetClassIndexDetector, [10]
     * targetClassIndexClassifier, [11]
     * targetLongSidePixels, [12]
     * targetShortSidePixels, [13]
     * targetHorizontalExtentPixels, [14]
     * targetVerticalExtentPixels, [15]
     * targetSkewDegrees [16]
     */

    /**
     * Read the latest Limelight values.
     * <p>
     * Call this method in the robot iterative loop.
     */
    public void update() {
        // for MegaTag2 need the robot orientation - at least the yaw.
        LimelightHelpers.SetRobotOrientation(limelightName, drivetrain.getHeading(), drivetrain.getRate(), 0., 0., 0.,
                0.);

        // initialize each iteration to origin or something invalid for anything that could be
        // returned to user; invalid data leaves the previously published poses unchanged but the
        // poses are zeros if used
        robotInField = Pose2d.kZero;
        tid = -1; // initially indicate no tag
        suggestResetOdometry = false; // believe pose is better than odometry, if true

        // get LL t2d data needed to determine validity
        stats = t2d.getAtomic();

        // check if new and valid data
        isFresh = stats.timestamp != previousTimestamp && stats.value.length >= 17 && stats.value[0] == 1.0;
        previousTimestamp = stats.timestamp;
        timestampSeconds = ((double) (stats.timestamp) / 1_000_000.0) - (latency / 1_000.0); // Convert server timestamp from microseconds to seconds and adjust for latency

        if (isFresh()) {
            // example use of interpreting the JSON string from LL - it's a bit slow; not
            // implemented here yet unless wanted
            // use of JSON results must be activated on the LL dashboard
            // LimelightLib has a json parser if desired. https://github.com/LimelightVision/limelightlib-wpijava
            // var JSONdump = LimelightHelpers.getLatestResults(limelightName); // 0.2 milliseconds (1 tag) to 0.3 milliseconds (2 tags), roughly
            // System.out.println(JSONdump);

            targetCount = (int) (stats.value[1]);
            latency = stats.value[2] + stats.value[3];
            tx = stats.value[4];
            ty = stats.value[5];
            txnc = stats.value[6];
            tync = stats.value[7];
            ta = stats.value[8];
            tid = (int) stats.value[9];

            // OpenCV and WPILib estimator and likely LL layout of axes is EDN and field WPILib is NWU;
            // need x -> -y , y -> -z , z -> x and same for differential rotations
            var cameraToTarget3D = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);

            cameraToTarget = new Transform3d(
                    new Translation3d(
                            cameraToTarget3D.getX(),
                            cameraToTarget3D.getY(),
                            cameraToTarget3D.getZ()),
                    new Rotation3d(
                            -cameraToTarget3D.getRotation().getX() - Math.PI,
                            -cameraToTarget3D.getRotation().getY(),
                            cameraToTarget3D.getRotation().getZ() - Math.PI));
            cameraToTarget = CoordinateSystem.convert(cameraToTarget, CoordinateSystem.EDN(), CoordinateSystem.NWU());

            // MEGATAG1 process
            MT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
            publishRobotPoseMT1.set(MT1.pose);

            // MEGATAG2 process
            MT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            publishRobotPoseMT2.set(MT2.pose);

            // Hints below on how Standard Deviations might be managed to get really fancy.

            // Using reset the pose for 100% sure (essentially 0 deviation?).
            // Using some other fixed number recommended by WPILib or LimelightVision docs is probably good enough.

            // If used, then stddev of x, y, yaw for either MT or MT2 are the pertinent values to
            // consider for poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(x, y, yaw));
            /*
             * stddevs
             * doubleArray MegaTag Standard Deviations
             * MT1x,
             * MT1y,
             * MT1z,
             * MT1roll,
             * MT1pitch,
             * MT1Yaw,
             * MT2x,
             * MT2y,
             * MT2z,
             * MT2roll,
             * MT2pitch,
             * MT2yaw
             */
            // private final DoubleArraySubscriber stddevs; // make this a class field variable

            // stddevs = LLtable.getDoubleArrayTopic("stddevs").subscribe(
            // new double[]{ // default stddevs huge number so as not to be used but validation should have prevented that anyway
            // 9999., 9999., 9999., 9999., 9999., 9999.,
            // 9999., 9999., 9999., 9999., 9999., 9999.
            // });

            // var sd = stddevs.get();
            // System.out.println("standard deviations - x, y, z, roll, pitch, yaw");
            // System.out.println("for poseEstimator.setVisionMeasurementStdDevs(x, y, yaw) use x, y, and 9_999_999 for yaw")
            // if (isMegaTag2())
            // {
            // System.out.format("MegaTag2 x %6.2f, y %6.2f, z %6.2f, r %6.2f, p %6.2f, yaw %6.2f%n",
            // sd[6], sd[7], sd[8], sd[9], sd[10], sd[11]);
            // }
            // else
            // {
            // System.out.format("MegaTag1 x %6.2f, y %6.2f, z %6.2f, r %6.2f, p %6.2f, yaw %6.2f%n",
            // sd[0], sd[1], sd[2], sd[3], sd[4], sd[5]);
            // }

            // Team specified validation or filtering of MT1 and MT2 poses to suggest
            // 1. pick the best pose
            // 2. determine if it's good enough to reset odometry
            // 3. Otherwise add the pose to the PoseEstimator:
            // with some reasonable setVisionMeasurementStdDevs(VecBuilder.fill(sd x, sd y, sd yaw))
            // or use the example stddevs
            // with addVisionMeasurement(LL1.getPose2d(), LL1.getTimestampSeconds())

            // MT1 works better than MT2 close in.
            // MT1 is too jittery beyond 1.1 meters and MT2 isn't as accurate less than 1 meter
            // MT1 has good range but the heading is flakey so use gyro for heading
            // MT2 quits when close to target and MT1 keeps on working
            // MT2 lag for gyro may be tricky to get right
            // MT2 goes to pose at field center when it's invalid - not checking for that here - just keeping the faith

            // FIXME filter poses
            // Teams can put whatever validation and selection they deem reasonable for using LL poses.
            // Here's a reasonable, trivial example. WPILib suggestions include don't use vision pose
            // if it's somewhat far from the odometry.
            if (MT1.tagCount >= 1 && MT1.avgTagDist < 1.1) {
                robotInField = MT1.pose;
                isMegaTag2 = false;
                suggestResetOdometry = true;
            } else {
                robotInField = MT2.pose;
                isMegaTag2 = true;
                suggestResetOdometry = false;
            }

            publishRobotPose2d.set(getPose2d()); // publish the pose that made it through validation/filtering
            publishRobotPose3d.set(getPose3d()); // publish the pose that made it through validation/filtering
        } else {
            if (stats.value.length == 0) {
                System.out.println(limelightName + " not connected");
            }
        }
    }

    /**
     * Class toString -- Format some target data to print
     * 
     * @return formatted string of fresh target data or "no fresh data"
     */
    public String toString() {
        StringBuilder sb = new StringBuilder(900);

        if (this.isFresh()) {
            sb.append("\n--- TARGET  STATISTICS ---\n");
            sb.append(stats.timestamp + " local time\n");
            sb.append(stats.serverTime + " server time\n");
            sb.append(stats.value[0] + " valid\n"); // useless print always 1.0 since it's a subset of the check for "isFresh"
            sb.append(getTargetCount() + " count\n");
            sb.append(stats.value[2] + " target latency\n");
            sb.append(stats.value[3] + " capture latency\n");
            sb.append(getTX() + " horizontal offset\n");
            sb.append(getTY() + " vertical offset\n");
            sb.append(getTXNC() + " horizontal offset no crosshair\n");
            sb.append(getTYNC() + " vertical offset no crosshair\n");
            sb.append(getTA() + " area\n");
            sb.append(getTagID() + " tag id\n");
            sb.append(getCameraToTarget() + " camera to tag transform\n");
            sb.append(getPose2dMT1() + " MEGATAG POSE\n");
            sb.append(getPose2dMT2() + " MEGATAG2 POSE\n");
            sb.append(getSuggestResetOdometry() + " suggest reset estimated odometry\n");
            sb.append(getPose2d() + " filtered Pose2d");
            sb.append(isMegaTag2() + " filtered Pose2d from MegaTag2 (else MegaTag)");
        } else {
            sb.append("\nno fresh data\n");
        }

        return sb.toString();
    }

    /**
     * Verify limelight name exists as a table in NT.
     * <p>
     * This check is expected to be run once during robot construction and is not intended to be checked
     * in the iterative loop.
     *
     * @param limelightName
     *            Limelight Name to check for table existence.
     * @return true if an NT table exists with requested LL name.
     *         <p>
     *         false and issues a WPILib Error Alert if requested LL doesn't appear as an NT table.
     */
    @SuppressWarnings("resource")
    public static boolean isAvailable(String limelightName) {
        final var retryDelay = Seconds.of(1);
        final var retryLimit = 20;

        // Get the limelight table
        // duplicates the constructor so constructor can be private and makeCamera can return a null
        var LLtable = NTinstance.getTable(limelightName);

        // LL sends key "getpipe" if it's on so check that
        // put in a delay if needed to help assure NT has latched onto the LL if it is transmitting

        for (int i = 1; i <= retryLimit; i++) {
            if (LLtable.containsKey("getpipe")) {
                return true;
            }
            System.out.println("attempt " + i + " of " + retryLimit + " to attach to limelight named " + limelightName);
            try {
                Thread.sleep((long) retryDelay.in(Milliseconds));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        String errMsg = "Your limelight name \"" + limelightName +
                "\" is invalid; doesn't exist on the network (no getpipe key).\n" +
                "These may be available:> " +
                NTinstance.getTable("/").getSubTables().stream()
                        .filter(ntName -> ((String) (ntName)).startsWith("limelight"))
                        .collect(Collectors.joining("\n"));

        errMsg = errMsg + "\nMake sure the team number is entered in the Limelight\n";
        if (Robot.isSimulation()) {
            errMsg = errMsg
                    + "Enter the IP address below on the interface to the Limelight for Dashboard: Settings / Custom NT Server IP:\n";
            errMsg = errMsg + getMyIPAddress();
        } else // it's real roboRIO
        {
            errMsg = errMsg + "Enter 0.0.0.0 for the custom NetworkTables IP address\n";
        }

        new Alert(errMsg, AlertType.kError).set(true);
        DriverStation.reportWarning(errMsg, false);

        return false;
    }
}
