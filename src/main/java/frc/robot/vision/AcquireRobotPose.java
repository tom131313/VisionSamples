package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.ControllerVision.Camera;
import frc.robot.utils.SpikeFilter;
import frc.robot.vision.Image.AcquisitionTime;

/**
 * Pose estimation using WPILib math on previously detected AprilTags (see {@link AcquireAprilTag})
 * <p>
 * The robot is clamped to the floor (Z = 0 and that code could be removed).
 * 
 * This was intended to run in an independent thread but now in this project it shares a thread
 * with AcquireAprilTag and runs sequentially with it.
 *
 * <p>
 * This program publishes Tags and Poses to NetworkTables table "robotsLocations".
 * 
 * <p>
 * Poses are drawn on the image of the AprilTag. (easily disabled)
 * 
 * <p>
 * It is paced by the availability of new detected tags in camera images and its
 * speed of computing poses from the detections.
 */
public class AcquireRobotPose {

    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    private ControllerVision controllerVision;
    private Camera camera;
    private boolean usePose3D;
    private Transform3d cameraInRobotFrame;

    private AcquisitionTime acquisitionTime;
    private int lastFrameRead = 0;
    private ArrayList<Long> tags = new ArrayList<>(AprilTagsLocations.getTagCount()); // tags detected in this camera frame

    // We'll output to NT
    private NetworkTable robotsTable = NetworkTableInstance.getDefault().getTable("robotsLocations");
    private IntegerArrayPublisher pubTagsDetected = robotsTable.getIntegerArrayTopic("tagsDetected").publish(); // tag ids in frame
    private List<StructPublisher<Pose3d>> publishRobotPose = new ArrayList<>(AprilTagsLocations.getTagCount()); // robot pose from every possible tag

    private double tagSize = 0.1651; // meters of the targeted AprilTag

    private ArrayList<RobotPose> poses = new ArrayList<>(30);
    private ArrayList<RobotPose> posesLastFrame = new ArrayList<>(30);
    private Scalar outlineColor = new Scalar(0, 255, 0); // bgr
    private Scalar crossColor = new Scalar(0, 0, 255); // bgr
    private int crossLength = 10;
    private Mat outImage = new Mat(); // this will receive the stored image's Mat
    private CvSource outputStream;
    private double yaw;
    private double pitch;
    private AprilTagPoseEstimator estimator;
    private ArrayList<SpikeFilter> xSpikeFilter = new ArrayList<>(30);
    private ArrayList<SpikeFilter> ySpikeFilter = new ArrayList<>(30);

    public AcquireRobotPose(ControllerVision controllerVision, Camera camera, boolean usePose3D,
            Transform3d cameraInRobotFrame /* robotToCamera */) {
        // Tag positions
        // tag rotation is CCW looking down on field from the ceiling.
        // rotating around Z, 0 degrees is parallel to Y and facing down field or +X. 30 degrees is still
        // facing down field +X and a little facing into the +Y across the field

        this.controllerVision = controllerVision;
        this.camera = camera;
        this.usePose3D = usePose3D;
        this.cameraInRobotFrame = cameraInRobotFrame;

        acquisitionTime = controllerVision.image.new AcquisitionTime(); // this will receive the stored image's acquisition time

        // This will send annotated images back to a Dashboard or browser
        outputStream = CameraServer.putVideo("Detected", camera.width, camera.height); // http://10.42.37.2:1182/ http://roborio-4237-frc.local:1182/?action=stream

        // Set up Pose Estimator
        Config poseEstConfig = new AprilTagPoseEstimator.Config(tagSize, camera.fx, camera.fy,
                camera.cx, camera.cy);

        estimator = new AprilTagPoseEstimator(poseEstConfig);

        Consumer<AprilTag> initializeRobotPosePublishers = tag -> {
            var robotPosePublisher = robotsTable.getStructTopic("robotPose3D_" + tag.ID, Pose3d.struct).publish();
            publishRobotPose.add(robotPosePublisher); // no tag 0 so tag 1 will be index 0 in the list

            // these spike filters remove only the very worst spikes in pose jitter apparent further than about 3
            // meters from the tag and especially with straight-on poses which are poor camera mount positions.
            xSpikeFilter.add(new SpikeFilter(0.05, 9999., 1));
            ySpikeFilter.add(new SpikeFilter(0.05, 9999., 1));
        };

        AprilTagsLocations.getTagsLocations().forEach(initializeRobotPosePublishers);
    }

    /**
     * Acquire Robot Pose from detected AprilTag processing.
     * Since no longer in its own thread it must be iterated in an external loop.
     */
    public void run() {

        if (!controllerVision.image.isFreshImage()) {
            return;
        }

        AprilTagDetection[] detections = controllerVision.image.getImage(outImage, acquisitionTime); // get the stored image w/ detections

        tags.clear(); // have not seen any tags yet
        poses.clear();

        // loop to get all AprilTag detections within current camera frame
        for (AprilTagDetection detection : detections) {
            Pose3d tagInFieldFrame; // pose from WPILib resource or custom pose file

            if (AprilTagsLocations.getTagsLocations().size() >= detection.getId() && // no tag 0; tag 1 is index 0 (-1 ouch!)
                    detection.getDecisionMargin() > 50. && // margin < 20 seems bad; margin > 120 are good
                    detection.getId() != 0) // tag 0 not used - it's invalid for FRC
            {
                tagInFieldFrame = AprilTagsLocations.getTagLocation(detection.getId());
            } else {
                System.out.println(
                        "bad decision margin for id " + detection.getId() + ", " + detection.getDecisionMargin());
                continue;
            }

            tags.add((long) detection.getId()); // remember we saw this tag

            // draw lines around the tag
            for (var i = 0; i <= 3; i++) {
                var j = (i + 1) % 4;
                var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
                var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
                Imgproc.line(outImage, pt1, pt2, outlineColor, 2);
                // corners appear as 3 2
                // 0 1
            }

            var tagCx = detection.getCenterX();
            var tagCy = detection.getCenterY();
            Imgproc.line(outImage, new Point(tagCx - crossLength, tagCy), new Point(tagCx + crossLength, tagCy),
                    crossColor, 2);
            Imgproc.line(outImage, new Point(tagCx, tagCy - crossLength), new Point(tagCx, tagCy + crossLength),
                    crossColor, 2);

            // identify the tag
            Imgproc.putText(
                    outImage,
                    Integer.toString(detection.getId()),
                    new Point(tagCx + crossLength, tagCy),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    1,
                    crossColor,
                    3);

            // Calculates the yaw and pitch this tag's center vs the camera principal point.
            // Yaw and pitch must be calculated together to account for perspective distortion.
            // Yaw is positive right and pitch is positive up.

            /* synchronized(this) */ // no longer needed since there is one for the robotPose
            { // these two values go together as one transaction
              // set the 3-D pose to zeros here initially
                yaw = Math.atan((tagCx - camera.cx) / camera.fx);
                pitch = Math.atan((camera.cy - tagCy) / (camera.fy / Math.cos(yaw)));
                yaw = Units.radiansToDegrees(yaw);
                pitch = Units.radiansToDegrees(pitch);
            }

            if (usePose3D) {
                Transform3d tagFacingCameraFrame = estimator.estimate(detection);

                { // draw a frustum in front of the AprilTag
                  // use the estimated pose from above before any other transforms

                    // camera same as above but different format for OpenCV
                    float[] cameraParm = { (float) camera.fx, 0.f, (float) camera.cx,
                            0.f, (float) camera.fy, (float) camera.cy,
                            0.f, 0.f, 1.f };
                    Mat K = new Mat(3, 3, CvType.CV_32F); // camera matrix
                    K.put(0, 0, cameraParm);

                    // 3D points of ideal, original corners, flat on the tag, scaled to the actual tag size.
                    // Order doesn't matter except must be in same order as the top points so pillars connect right.
                    // We could reuse the corners from detector if we know the order (and we do) and avoid redundant
                    // variable and recalculation but we'll re-specify them for fun and match the detectors corners order.
                    MatOfPoint3f bottom = new MatOfPoint3f(
                            new Point3(-1. * tagSize / 2., 1. * tagSize / 2., 0.),
                            new Point3(1. * tagSize / 2., 1. * tagSize / 2., 0.),
                            new Point3(1. * tagSize / 2., -1. * tagSize / 2., 0.),
                            new Point3(-1. * tagSize / 2., -1. * tagSize / 2., 0.));

                    // 3D points of the ideal, original corners, in front of the tag to make a frustum, scaled to the actual tag size
                    // note that the orientation and size of the face of the box can be controlled by the sign of the "Z"
                    // value of the "top" variable.
                    // "-" (negative) gives larger top facing straight away from the plane of the tag
                    // "+" (positive) gives smaller top facing toward the camera
                    MatOfPoint3f top = new MatOfPoint3f( // order doesn't matter except must be in same order as the bottom points
                            new Point3(-1. * tagSize / 2., 1. * tagSize / 2., -0.7 * tagSize),
                            new Point3(1. * tagSize / 2., 1. * tagSize / 2., -0.7 * tagSize),
                            new Point3(1. * tagSize / 2., -1. * tagSize / 2., -0.7 * tagSize),
                            new Point3(-1. * tagSize / 2., -1. * tagSize / 2., -0.7 * tagSize));

                    // The OpenCV rvec is a rotation vector with three elements representing the axis scaled by
                    // the angle in the EDN coordinate system. (angle = norm, and axis = rvec / norm).
                    // Those three elements are not the same 3 elements of the 3 rotations for the 3 axes that might be used in WPILib Rotation3D.
                    // Those are roll, pitch, and yaw (Euler angles?) that can be converted to the rotation vector (directional vector?)
                    // and vice versa. The roll, pitch, and yaw angles can be recovered from the rotation vector:
                    // Rodrigues conversion of rotation vector to rotation matrix (or vice versa)
                    // Rotation matrix to Euler angles https://learnopencv.com/rotation-matrix-to-euler-angles/
                    // https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation
                    /*
                     * https://www.reddit.com/r/computervision/comments/eek49u/q_how_to_get_the_euler_angles_from_a_rotation/?rdt=45312
                     * I think of the compact representations of a rotation (Euler angles, Rodrigues vector, quaternion, etc) as practical
                     * ways to communicate or store the information about rotation. But when it comes to actually using it for something 99 percent
                     * of the time I use the good ol 3x3 rotation matrix because it's super easy to understand. multiply a vector and you rotate
                     * the vector, invert it and get the rotation the other way round, want to know the base vectors of the world frame of
                     * reference wrt the camera? this are the columns of the rotation matrix.
                     */
                    double[] rotationVector = tagFacingCameraFrame.getRotation().getQuaternion().toRotationVector()
                            .getData(); // 3x1 3 rows 1 col

                    Mat T = new Mat(3, 1, CvType.CV_64FC1);
                    Mat R = new Mat(3, 1, CvType.CV_64FC1);
                    T.put(0, 0, tagFacingCameraFrame.getX(), tagFacingCameraFrame.getY(), tagFacingCameraFrame.getZ());
                    R.put(0, 0, rotationVector[0], rotationVector[1], rotationVector[2]);

                    MatOfPoint2f imagePointsBottom = new MatOfPoint2f();
                    Calib3d.projectPoints(bottom, R, T, K, camera.distortionCoeffs, imagePointsBottom);

                    MatOfPoint2f imagePointsTop = new MatOfPoint2f();
                    Calib3d.projectPoints(top, R, T, K, camera.distortionCoeffs, imagePointsTop);

                    ArrayList<Point> topCornerPoints = new ArrayList<Point>();

                    // draw from bottom points to top points - pillars
                    for (int i = 0; i < 4; i++) {
                        var x1 = imagePointsBottom.get(i, 0)[0];
                        var y1 = imagePointsBottom.get(i, 0)[1];
                        var x2 = imagePointsTop.get(i, 0)[0];
                        var y2 = imagePointsTop.get(i, 0)[1];

                        topCornerPoints.add(new Point(x2, y2));

                        Imgproc.line(outImage,
                                new Point(x1, y1),
                                new Point(x2, y2),
                                outlineColor,
                                2);
                    }

                    MatOfPoint topCornersTemp = new MatOfPoint();
                    topCornersTemp.fromList(topCornerPoints);
                    ArrayList<MatOfPoint> topCorners = new ArrayList<>();
                    topCorners.add(topCornersTemp);

                    Imgproc.polylines(outImage, topCorners, true, outlineColor, 2);
                } /* end draw a frustum in front of the AprilTag */

                /*
                 * This Transform3d from tagFacingCameraFrame to tagInCameraFrame is required for the correct robot pose.
                 * It appears to arise from the tag facing the camera thus Pi radians rotated or CCW/CW flipped from
                 * the mathematically described pose from the estimator. The true rotation has to be used to get the
                 * right robot pose. It seems that the T and R from the estimator could take care of all this (it is
                 * consistent without the extra transform when drawing the tag and orientation box).
                 * 
                 * From PhotonVision this is likely the explanation:
                 * The AprilTag pose rotation outputs are X left, Y down, Z away from the tag
                 * with the tag facing
                 * the camera upright and the camera facing the target parallel to the floor.
                 * But our OpenCV
                 * solvePNP code would have X left, Y up, Z towards the camera with the target
                 * facing the camera
                 * and both parallel to the floor. So we apply a base rotation to the rotation
                 * component of the
                 * apriltag pose to make it consistent with the EDN system that OpenCV uses,
                 * internally a 180
                 * rotation about the X axis
                 */
                var tagInCameraFrame = new Transform3d(
                        new Translation3d(
                                tagFacingCameraFrame.getX(),
                                tagFacingCameraFrame.getY(),
                                tagFacingCameraFrame.getZ()),
                        new Rotation3d(
                                -tagFacingCameraFrame.getRotation().getX() - Math.PI,
                                -tagFacingCameraFrame.getRotation().getY(),
                                tagFacingCameraFrame.getRotation().getZ() - Math.PI));
                /*
                 * from WPILib documentation Drive classes:
                 * Axis Conventions:
                 * The drive classes use the NWU axes convention (North-West-Up as external reference in the world frame).
                 * The positive X axis points ahead, the positive Y axis points left, and the positive Z axis points up.
                 * We use NWU here because the rest of the library, and math in general, use NWU axes convention.
                 * 
                 * Joysticks follow NED (North-East-Down) convention, where the positive X axis points ahead, the
                 * positive Y axis points right, and the positive Z axis points down. However, it’s important to note
                 * that axes values are rotations around the respective axes, not translations. When viewed with each
                 * axis pointing toward you, CCW is a positive value and CW is a negative value. Pushing forward on the joystick is a CW rotation around the Y axis, so you get
                 * a negative value. Pushing to the right is a CCW rotation around the X axis, so you get a positive value.
                 */
                // OpenCV and WPILib estimator layout of axes is EDN and field WPILib is NWU;
                // need x -> -y , y -> -z , z -> x and same for differential rotations
                tagInCameraFrame = CoordinateSystem.convert(tagInCameraFrame, CoordinateSystem.EDN(),
                        CoordinateSystem.NWU());
                // // WPILib CoordinateSystem.convert was wrong for transforms in 2023 so this patch was used
                // { // corrected convert
                // var from = CoordinateSystem.EDN();
                // var to = CoordinateSystem.NWU();
                // tagInCameraFrame = new Transform3d(
                // CoordinateSystem.convert(tagInCameraFrame.getTranslation(), from, to),
                // CoordinateSystem.convert(new Rotation3d(), to, from)
                // .plus(CoordinateSystem.convert(tagInCameraFrame.getRotation(), from, to)));
                // } // end of corrected convert
                // var // transform to camera from robot chassis center at floor level - robot specific!
                // cameraInRobotFrame = new Transform3d(
                // new Translation3d(0., 0., 0.), // camera at center bottom of robot
                // // new Translation3d(0.2, 0., 0.8), // camera in front of center of robot and above ground
                // // new Rotation3d(0.0, Units.degreesToRadians(0.), Units.degreesToRadians(0.0)) // camera in line with robot chassis
                // new Rotation3d(0., Units.degreesToRadians(-25.), 0.) // camera in line with robot chassis, pointing up slightly
                // );
                // x + roll is camera rolling CCW relative to the robot looking facing the robot
                // y + pitch is camera pointing down relative to the robot. -25 camera points up; +25 points down; sign is correct but backwards of LL
                // z + yaw is camera pointing to the left of robot looking down on it (CCW relative to the robot)

                var // robot in field is the composite of 3 pieces
                robotInFieldFrame = ComputerVisionUtil.objectToRobotPose(tagInFieldFrame, tagInCameraFrame,
                        cameraInRobotFrame);

                // the above transforms match LimeLight Vision botpose_wpiblue network tables entries
                // as they display in AdvantageScope 3D Field robotInFieldFrame

                // There is a lot of jitter at longer distances with this algorithm especially if the camera is poorly placed straight-on
                // to the tag. Much of the jitter is in the Pose3d components that are not used in Pose2d which is what is commonly needed
                // for targeting. Zeroing out those "extra" components makes the robot look a lot better in AdvantageScope Field3d. Clamping
                // the robot to the floor (Z, height) is also an option in the LimelightVision product. It is unlikely the Z value is used
                // except maybe for 3-D distance to a target.
                // A spike filter can remove the worst spasms with distant, head-on views.

                // Some teams use the X-Y values and get the rotation from the gyro. This class code does not do any validation or value
                // replacement using the gyro. It could be added here or done downstream if desired.

                var xNoSpikes = xSpikeFilter.get(detection.getId() - 1).calculate(robotInFieldFrame.getX()); // ouch! tags 1 to 22 in list positions 0 to 21
                var yNoSpikes = ySpikeFilter.get(detection.getId() - 1).calculate(robotInFieldFrame.getY());
                var zAngle = robotInFieldFrame.getRotation().getZ();
                // A fallacy here is what if X or Y but not both had spikes removed? Then they don't go together anymore here or in
                // the history of previous value saved in the spike remover. They are re-synched with the next non-spiky iteration.
                // Could save the previous values of all 3 and if one was replaced by previous value, then all should be.
                // Or just skip this tag. Would need to remove it from the tags list.

                var clampedToZero = 0.;

                robotInFieldFrame = new Pose3d(xNoSpikes, yNoSpikes, clampedToZero, // essentially make Pose2d from Pose3d and remove spikes
                        new Rotation3d(clampedToZero, clampedToZero, zAngle));

                // end of transforms to get the robot pose from this vision tag pose
                poses.add(new RobotPose(detection.getId(), yaw, pitch, robotInFieldFrame, tagInCameraFrame));

                // put detection and pose information on dashboards
                // SmartDashboard.putNumber("detectionDecisionMargin " + detection.getId(), detection.getDecisionMargin());

                // put out to NetworkTables tag and robot pose for this tag in AdvantageScope format
                publishRobotPose.get(detection.getId() - 1).set(robotInFieldFrame); // ouch! tags 1 to 22 in list positions 0 to 21
            } else {
                poses.add(new RobotPose(detection.getId(), yaw, pitch, Pose3d.kZero, Transform3d.kZero));
            }

        } // end of all detections

        // update with all tags detected in this camera frame
        // except it's still the same arraylist object
        synchronized (this) { // lock the list from {@link #getPoses()} while it's being completely redone with the accumulated new data
            posesLastFrame.clear();
            for (RobotPose pose : poses) {
                posesLastFrame.add(pose.clone()); // adding deep copy of the data
            }
        }

        // put list of tags seen onto dashboard
        pubTagsDetected.set(tags.stream().mapToLong(Long::longValue).toArray());

        // all the data available at this point.

        outputStream.putFrame(outImage); // Give the output stream a new image to display

        @SuppressWarnings("unused")
        double endFrameTime = Timer.getFPGATimestamp();
        // SmartDashboard.putNumber("theoretical fps (1/frame time)",
        //         1. / (endFrameTime - acquisitionTime.acquisitionTime));
    }

    /**
     * Get the pose of the camera wrt the AprilTag.
     * <p>
     * This is running in an asynchronous thread so what is returned is the last completed
     * calculation and a new calculation that may be in progress does not effect these data.
     * <p>
     * Lock the list while it's being read so updates can't happen simultaneously in {@link #run()}
     * 
     * @return poses in the frame or null if no new poses since last getPoses() call
     */
    public synchronized ArrayList<RobotPose> getPoses() {
        if ((acquisitionTime.frameNumber == lastFrameRead) || (posesLastFrame.size() == 0)) {
            return null;
        } else {
            lastFrameRead = acquisitionTime.frameNumber;
            return new ArrayList<RobotPose>(posesLastFrame); // create new list object for others to use while this one might be updated
        }
    }
}
