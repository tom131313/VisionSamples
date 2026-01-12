package frc.robot.vision;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Config;
import frc.robot.Config.ControllerVisionSettings;
import frc.robot.Config.LimelightVisionSettings;
import frc.robot.Config.PhotonVisionSettings;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToTag12FieldRelativePose3D;
import frc.robot.commands.AlignToTag12RelativeArcade2D;
import frc.robot.commands.AlignToHUBPairedTagFieldRelativePose3D;
import frc.robot.commands.AlignToPairedTagTargetRelativeTransform3D;

/**
 * This class is a wrapper for the three vision classes {@link ControllerVision}, {@link
 * PhotonVision}, and {@link LimelightVision}
 * <p>
 * ControllerVision (roboRIO) returns team specified robot pose based on the tag ID that is closest
 * to the robot (this needs work; that might not always be appropriate).
 * <p>
 * PhotonVision returns its best single tag pose and multi-tag improved pose if activated (but not
 * the very best pose estimator that includes odometry and gyro that teams could add).
 * <p>
 * LimelightVision returns team specified filtered selection from MegaTag or MegaTag2.
 * (Example is a reasonable default but could use more tuning. See {@link LimelightVision#update()})
 * <p>
 * After selecting and configuring the desired vision system {@link Config}, use the
 * robot pose as in example commands {@link AlignToTag12FieldRelativePose3D},
 * {@link AlignToPairedTagTargetRelativeTransform3D}, {@link AlignToTag12RelativeArcade2D}, and
 * {@link AlignToHUBPairedTagFieldRelativePose3D}
 * <p>
 * Getters are available for each of the three vision systems. This allows use of other low level
 * methods that may be available within the systems' classes. That also helps reveal the following
 * getters which are duplicative with the RobotPose class data so there is no reason to use them.
 * <p>
 * Each vision system extends {@link CameraBase} which provides for
 * 
 * <pre>
 * <code>
 *public abstract void update();
 *public abstract boolean isFresh();
 *public abstract int getTagID();
 *public abstract Pose3d getPose3d();
 *public abstract Pose2d getPose2d();
 *public abstract double getTX();
 *public abstract double getTY();
 *public abstract Transform3d getCameraToTarget();
 * </code>
 * </pre>
 * <p>
 * Usage of these methods requires that after "update()" the "isFresh()" is checked before using
 * any of the getters.
 * <p>
 * Since all of these data are available in the RobotPose there isn't a reason to use these methods
 * in this example project. Other projects may use them.
 */
public class VisionContainer {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());

        System.loadLibrary(org.opencv.core.Core.NATIVE_LIBRARY_NAME); // OpenCV required for all Vision Processes

        try { // likely needed for 3-D pose usage
            Class.forName(frc.robot.vision.AprilTagsLocations.class.getCanonicalName());
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }
    }

    public enum VisionSelector {
        useControllerVision, usePhotonVision, useLimelightVision
    }

    private VisionSelector visionSelector = Config.visionSelector;

    private ControllerVision controllerVision = null; // only for Controller (roboRIO)
    private PhotonVision photonVision = null; // only for PV
    private LimelightVision limelightVision = null; // only for LL
    private AtomicReference<Optional<RobotPose>> robotPose =
        new AtomicReference<Optional<RobotPose>>(Optional.empty());
    private Transform3d robotToCamera;
    private boolean vision = false; // initially not activated

    public VisionContainer() {

        DriverStation.reportWarning("Vision selection " + visionSelector, false);

        switch (visionSelector) {

            case useControllerVision:
                if (ControllerVision.isAvailable(ControllerVisionSettings.cameraDeviceId)) {
                    robotToCamera = ControllerVisionSettings.robotToCamera;
                    controllerVision = new ControllerVision(ControllerVisionSettings.cameraDeviceId,
                            ControllerVisionSettings.camera, ControllerVisionSettings.usePose3D, robotToCamera);

                    // vision runs as a "background" thread to minimize interference with the other
                    // robot actions. This may increase latency. Priority could be experimented with.
                    Thread acquireControllerCamera = new Thread(controllerVision);
                    acquireControllerCamera.setName("roboRIOCameraPose");
                    acquireControllerCamera.setPriority(acquireControllerCamera.getPriority() - 1);
                    acquireControllerCamera.setDaemon(true);
                    acquireControllerCamera.start();
                    vision = true;
                } else {
                    DriverStation.reportWarning("No ControllerVision process", false);
                    System.out.println("Make sure camera is plugged in" +
                            (Robot.isSimulation() ? " and number is correct (try the other one)" : ""));

                    controllerVision = null;
                }

                break;

            case usePhotonVision:
                robotToCamera = PhotonVisionSettings.robotToCamera;

                // PV started connecting when the NT server started but still it's a race to make
                // a connection before this instantiation and check
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                photonVision = new PhotonVision(PhotonVisionSettings.name, robotToCamera);
                if (photonVision.isAvailable()) {
                    vision = true;
                }
                else {
                    photonVision = null; // put it back to null since constructor didn't really make a viable camera
                }

                break;

            case useLimelightVision:
                /*
                 * Add additional Limelight setups as indicated below
                 */
                if (LimelightVision.isAvailable(LimelightVisionSettings.name)) {
                    limelightVision = new LimelightVision(LimelightVisionSettings.name, RobotContainer.getDrivetrain());
                    var c = LimelightHelpers.getCameraPose3d_RobotSpace(LimelightVisionSettings.name); // retrieve camera mount location that was set in LL
                    robotToCamera = new Transform3d(c.getTranslation(),
                            new Rotation3d(c.getRotation().getX(), -c.getRotation().getY(), c.getRotation().getZ())); // - pitch to match CV & PV
                    vision = true;
                    // additional LL setup as needed
                    // LimelightHelpers.setStreamMode_PiPSecondary(LimelightVisionSettings.name); // if there is a driver camera, where to put the image - pick your favorite
                } else {
                    DriverStation.reportWarning("No LimelightVision connection", false);

                    limelightVision = null;
                }

                break;

            default:
                DriverStation.reportWarning("Invalid vision system option selected in VisionContainer constructor, got: "
                        + visionSelector.toString(),
                        true);
                break;
        }
    }

    /**
     * 
     * @return if there is a Vision System active
     */
    public boolean vision() {
        return vision;
    }

    /**
     * periodically executed to update vision information
     */
    public void update() {
        if (!vision()) {
            return;
        }

        switch (visionSelector) {
            
            case useControllerVision:
                controllerVision.update();
                if (controllerVision.isFresh()) {
                    robotPose.set(Optional.of(new RobotPose(
                            controllerVision.getTagID(),
                            controllerVision.getTX(), controllerVision.getTY(),
                            controllerVision.getPose3d(),
                            controllerVision.getCameraToTarget())));
                } else {
                    robotPose.set(Optional.empty());
                }

                break;

            case usePhotonVision:
                photonVision.update();
                if (photonVision.isFresh()) {
                    robotPose.set(Optional.of(new RobotPose(
                            photonVision.getTagID(),
                            photonVision.getTX(), photonVision.getTY(),
                            photonVision.getPose3d(),
                            photonVision.getCameraToTarget())));
                    // System.out.println("PV " + photonVision.getTagID() + ", " + photonVision.getTX() + ", " + photonVision.getTY() + ", " + photonVision.getPose3d());
                } else {
                    robotPose.set(Optional.empty());
                }

                break;

            case useLimelightVision:
                limelightVision.update();
                if (limelightVision.isFresh()) {
                    robotPose.set(Optional.of(new RobotPose(
                            limelightVision.getTagID(),
                            limelightVision.getTX(), limelightVision.getTY(),
                            limelightVision.getPose3d(),
                            limelightVision.getCameraToTarget())));
                    // System.out.println((limelightVision.getSuggestResetOdometry() ?
                    // "reset odometry pose " : "addVisionMeasurement pose ") +
                    // limelightVision.getPose2d() + (limelightVision.isMegaTag2() ? " MegaTag2 pose" : " MegaTag pose"));
                } else {
                    robotPose.set(Optional.empty());
                }

                break;

            default:
                DriverStation.reportWarning("Invalid vision system option selected in VisionContainer update, got: "
                        + visionSelector.toString(),
                        true);
                break;
        }

        // getRobotPose().ifPresentOrElse
        // (
        // (pose)-> {
        // var t = getRobotPose().get().cameraToTarget;
        // System.out.println("tag is forward " + t.getX() + "m, left " + t.getY() + "m, up " + t.getZ() +
        // "m, roll " + Units.radiansToDegrees(t.getRotation().getX()) +
        // "deg, pitch " + Units.radiansToDegrees(t.getRotation().getY()) +
        // "deg, yaw " + Units.radiansToDegrees(t.getRotation().getZ()) + "deg");},
        // ()-> System.out.print("X")
        // );
    }

    /**
     * Robot pose for whomever wants it
     * 
     * @return RobotPose
     */
    public Optional<RobotPose> getRobotPose() {
        return robotPose.get();
    }

    /**
     * 
     * @return location of the camera wrt the robot
     */
    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }

    /**
     * Getter for vision system for CameraBase getters
     * 
     * @return
     */
    public PhotonVision getPhotonVision() {
        return photonVision;
    }

    /**
     * Getter for vision system for CameraBase getters
     * 
     * @return
     */
    public LimelightVision getLimelightVision() {
        return limelightVision;
    }

    /**
     * Getter for vision system for CameraBase getters
     * 
     * @return
     */
    public ControllerVision getControllerVision() {
        return controllerVision;
    }
}
