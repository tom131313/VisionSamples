package frc.robot.vision;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.utils.Network.getMyIPAddress;
import static frc.robot.vision.AprilTagsLocations.getTagCount;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * PhotonVision requires significant setup using the dashboard and, somewhat, code can be used.
 * <p>
 * In addition to what PhotonVision publishes to NetworkTables this program publishes robot
 * Pose3d and yaw and pitch for each tag detected (one best tag per frame) to table
 * "PhotonVisionLogged".
 * <p>
 * The {@link #getPose3d} is from Multi-Tag localization if activate in PV
 * <p>
 * For multi-tag use PV dashboard Output tab enable "Do Multi-Target Estimation" and enable
 * "Always Do Single-Target Estimation" to keep individual tag info, too. Must have the same
 * AprilTags field map in PV and WPILib.
 * <p>
 * PV always returns a 3-D pose even if it not activated by calibrating the camera and selecting
 * 3-D Processing Mode on the PV dashboard. The 3-D pose is made zero (origin) for this mode. This
 * value may occasionally conflict with a "legitimate" (calculated?) zero pose but that's not
 * suppose to happen as there is a corner of the field there where the center of a robot can't go.
 * {@link #getPose3d()}.
 */
public class PhotonVision extends CameraBase {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    private PhotonCamera camera;
    private String name;
    private boolean targetVisible;
    private PhotonTrackedTarget bestTarget;
    private Transform3d cameraInRobotFrame;
    private Optional<MultiTargetPNPResult> multiTagResult;

    private int maxTagId = getTagCount() + 1; // add 1 for tag ID 0 so indexing matches list
    private NetworkTable RobotPoseTable = NetworkTableInstance.getDefault().getTable("PhotonVisionLogged");

    private List<StructPublisher<Pose3d>> publishRobotPose = new ArrayList<>(maxTagId); // Pose3d
    private List<DoubleArrayPublisher> publishRobotServoing = new ArrayList<>(maxTagId); // yaw and pitch

    // PhotonPipelineResult PVResult = new PhotonPipelineResult();

    public PhotonVision(String name, Transform3d cameraInRobotFrame) {
        this.cameraInRobotFrame = cameraInRobotFrame;
        this.name = name;
        camera = new PhotonCamera(name);

        // make an empty bucket for every possible tag
        for (int tag = 0; tag < maxTagId; tag++) {
            var robotPosePublisher = RobotPoseTable.getStructTopic("robotPose3D_" + tag, Pose3d.struct).publish();
            publishRobotPose.add(robotPosePublisher);

            var robotServoingPublisher = RobotPoseTable.getDoubleArrayTopic("robotServoing_" + tag).publish();
            publishRobotServoing.add(robotServoingPublisher);
        }

        // better poses possible using the PhotonPoseEstimator; lot's of possibilities with it
        // PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        // PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameraInRobotFrame);
    }

    /**
     * Read relevant data from the Camera
     */
    public void update() {

        targetVisible = false;
        var results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1); // Get the last, most current result in the list.
            if (result.hasTargets()) { // At least one AprilTag was seen by the camera
                multiTagResult = result.getMultiTagResult();
                
                // example looping through all targets seen in this frame. An individual tag information
                // could be pulled off here as the PhotonVision example program shows.
                // var countTargets = 0;
                // for (var target : result.getTargets()) {
                // System.out.println("target " + ++countTargets + " " + target);
                // }

                // for this example use the PV designated Best Target
                bestTarget = result.getBestTarget();
                // double area = bestTarget.getArea();
                // double skew = bestTarget.getSkew();
                // List<TargetCorner> corners = bestTarget.getDetectedCorners();
                // double poseAmbiguity = bestTarget.getPoseAmbiguity();
                // Transform3d alternateCameraToTarget = bestTarget.getAlternateCameraToTarget();

                // System.out.println(/*"\nRESULT " + result +*/ "\n\nPV BEST TARGET " + bestTarget);
                var tag = bestTarget.getFiducialId();
                publishRobotPose.get(tag).set(getPose3d());
                publishRobotServoing.get(tag).set(new double[] { getTX(), getTY() });

                targetVisible = true;
            } else {
                System.out.println(Timer.getTimestamp() + " skipping because PV result has no targets");
            }
        }
    }

    @Override
    public boolean isFresh() {
        return targetVisible;
    }

    /**
     * 3-D robot pose getter
     * <p>
     * Use multi-tag if activated otherwise use the single best as determined in {@link #update()}
     */
    @Override
    public Pose3d getPose3d() {

        if (multiTagResult.isPresent()) {
            Transform3d fieldToCamera = multiTagResult.get().estimatedPose.best;
            // if multi-tag do camera to robot and  camera to field
            Transform3d robotInFieldFrame = fieldToCamera.plus(cameraInRobotFrame.inverse());  
            return new Pose3d(robotInFieldFrame.getTranslation(), robotInFieldFrame.getRotation());                    
        } else {
            var tagInCameraFrame = getCameraToTarget();
            if (tagInCameraFrame.equals(Transform3d.kZero)) {
                return Pose3d.kZero;
            } else {
                Pose3d tagInFieldFrame = AprilTagsLocations.getTagLocation(getTagID());
                var robotInFieldFrame = ComputerVisionUtil.objectToRobotPose(tagInFieldFrame, tagInCameraFrame,
                        cameraInRobotFrame);
                return robotInFieldFrame;
            }
        }
    }

    @Override
    public Pose2d getPose2d() {
        return new Pose2d(getPose3d().getX(), getPose3d().getY(), new Rotation2d(getPose3d().getRotation().getZ()));
    }

    @Override
    public double getTX() {
        return bestTarget.getYaw();
    }

    @Override
    public double getTY() {
        return bestTarget.getPitch();
    }

    @Override
    public int getTagID() {
        return bestTarget.getFiducialId();
    }

    public Transform3d getCameraToTarget() {
        return bestTarget.getBestCameraToTarget();
    }

    public boolean isAvailable() {
        
        final var retryDelay = Seconds.of(1); // configurable but unlikely changes needed
        final var retryLimit = 20; // configurable but unlikely changes needed

        boolean vision = false;

        checkConnection: {
            for (int i = 1; i <= retryLimit; i++) {
                if (camera.isConnected()) {
                    vision = true;
                    break checkConnection;
                }
                System.out.println("attempt " + i + " of " + retryLimit + " to attach to PhotonVision named "
                        + name);
                try {
                    Thread.sleep((long) retryDelay.in(Milliseconds));
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            camera.getAllUnreadResults(); // force PV to issue no camera by that name but there are other PV cameras

            if (Robot.isSimulation()) {
                System.out.println(
                        "Enter in the PV Settings Team Number/NetworkTables Server Address the IP address of this simulation device on the interface to the PhotonVision device");
                try {
                    System.out.println(
                            "If PV running on the same device as this simulation, then use any of the IP addresses assign to that device such as 127.0.0.1, localhost, or "
                                    +
                                    InetAddress.getLocalHost().getHostAddress());
                } catch (UnknownHostException e) {
                    e.printStackTrace();
                }
                System.out.println(getMyIPAddress());
            } else // it's real roboRIO
            {
                System.out.println("Make sure the team number " + RobotController.getTeamNumber()
                        + " is entered to attach to NetworkTables");
            }

            DriverStation.reportWarning("No PhotonVision connection", false);
        }

        return vision;
    }
}
