// Detecting and drawing AprilTags is significantly based on a WPILib example vision project so
// for ControllerVision.java, AcquireAprilTag.java, and AcquireRobotPose.java much is:
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.ArrayList;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import static frc.robot.Constants.ControllerVision.Camera;

/**
 * This is a demo program showing the detection of AprilTags and robot pose calculation using
 * WPILib functions.
 * <p>
 * The image is acquired from the USB camera, then any detected AprilTags are marked up on the
 * image, transformed to the robot on the field pose and sent to NetworkTables.
 * 
 * <p>
 * To remove irritating vertical (Z height) jitter an example of clamping the robot pose to the
 * floor is included in {@link AcquireRobotPose#run()}. That could be removed in AcquireRobotPose.
 * (LimelightVision setup has that option as set in the Limelight.)
 * 
 * <p>
 * Especially at distances > about 1.1 meters there is side-to-side and forward-backward jitter
 * especially when the camera is straight-on to the tag and there is ambiguity on which side of the
 * tag the camera is on. A smoothing or simple averaging of two (or more?) successive poses may
 * yield a better result. Don't smooth out the fact that the robot might actually be moving.
 * 
 * <p>
 * The camera image whiteness (contrast, gamma, brightness, exposure, gain) and sharpness have a
 * huge effect on jitter. These settings all interact so experiment. Often an image slightly
 * darker than what humans may find comfortable is about right. Try high exposure, high brightness,
 * low gain, low gamma, high contrast. Too bad high exposure is the opposite of what is needed for
 * high frame rate.
 * 
 * <p>
 * Included is an example Spike Filter that does give improved performance for both X and Y
 * axes. See {@link AcquireRobotPose} to remove or tune them better for another environment.
 * 
 * <p>
 * AcquireRobotPose has no provision for using the gyro to improve pose estimation. The gyro
 * heading could be used instead of the pose rotation, if desired.
 *
 * <p>
 * Be aware that the performance on a roboRIO (especially v1) is much worse than a coprocessor
 * solution!
 * 
 * <p>
 * The camera view is displayed with additional information of the AprilTag pose to camera.
 * That display is optional and a tiny bit of cpu processing can be saved by not doing it.
 * Since AprilTag view can be in normal light, that camera can also be used by the operator if it's
 * pointing in a good direction. If the operator doesn't need that view, don't display it and
 * the image can be made a little darker and more contrast - whatever can reduce the cpu
 * processing time for an image. Experiment with exposure, contrast, gamma, brightness, etc.
 * 
 * <p>
 * AprilTag has known pose on the field loaded from file (WPILib or your custom file).
 * Detected tag's perspective seen by the camera is used to calculate an estimate of the camera
 * pose relative to the tag. Camera has a known pose relative to the robot chassis. Combine this
 * chain to calculate the robot pose in the field. Camera parameters must be provided from another
 * source such as a related calibration program.
 * 
 * <p>
 * The original design of this vision system was {@link AcquireAprilTag} and
 * {@link AcquireRobotPose} ran in two separate free-wheeling threads that were synchronized as
 * needed by {@link Image}. That scheme has been disabled, however, some remnants remain. The
 * implementation for this project is this {@link ControllerVision} class runs in a separate
 * free-wheeling thread started by a higher level thread. Then this class {@link #run()} runs
 * {@link AcquireAprilTag#run()} and {@link AcquireRobotPose#run()} successively thus no
 * synchronization is needed.
 */
public class ControllerVision extends CameraBase implements Runnable {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    Image image = new Image(); // where a video frame goes for multiple processes to use
    private AcquireAprilTag acquireAprilTag;
    private AcquireRobotPose acquireRobotPose;
    private ArrayList<RobotPose> poses;
    private int bestPoseIndex;
    private RobotPose bestPose;

    public ControllerVision(int cameraDeviceId, Camera camera, boolean usePose3D,
            Transform3d robotToCamera) {
        acquireAprilTag = new AcquireAprilTag(this, cameraDeviceId, camera);
        acquireRobotPose = new AcquireRobotPose(this, camera, usePose3D, robotToCamera);
    }

    /**
     * See if camera by this device number is connected
     * <p>
     * Sometimes linux/camera server says camera is connected when it isn't and then something else
     * often goes wrong.
     * 
     * @param cameraDeviceId
     * @return
     */
    public static boolean isAvailable(int cameraDeviceId) {
        for (UsbCameraInfo cameraInfo : UsbCamera.enumerateUsbCameras()) {
            if (cameraInfo.dev == cameraDeviceId) {
                return true; // found camera so probably okay to try to start auto capture
            }
        }

        System.out.println("camera " + cameraDeviceId + " not found. Other cameras < ");
        for (UsbCameraInfo cameraInfo : UsbCamera.enumerateUsbCameras()) {
            System.out.println(cameraInfo.dev + " " + cameraInfo.name + " " + cameraInfo.path);
        }
        System.out.println(" >");
        return false;
    }

    /**
     * Loop of the thread to acquire camera images and compute the poses of AprilTags
     */
    public void run() {
        // This 'while' cannot be 'true'. The program will never exit if it is. Using
        // interrupted() lets the robot stop this thread when restarting robot code or deploying.
        while (!Thread.interrupted()) {
            acquireAprilTag.run();
            acquireRobotPose.run();
        }
    }

    /**
     * Get latest data - must be first before accessing data
     */
    @Override
    public void update() {
        poses = acquireRobotPose.getPoses();
        // poses.forEach(pose -> System.out.println("CV " + pose)); // show all available data

        if (isFresh()) {
            // FIXME Determine best target in the list; example filter is select closest tag.
            // This needs work because it interacts with what target is being used to align to for
            // scoring. Consider a command asking for a particular tag to be processed or make this
            // a list and the resulting increase in complexity. Or commands can ignore tags they
            // don't want and wait for the desired tag to appear.
            bestPoseIndex = 0;
            // Select the pose with the closest distance to the camera
            // Closer tags generally provide more accurate pose estimates with less jitter
            double minDistance = Double.MAX_VALUE;

            for (int i = 0; i < getPoses().size(); i++) {
                RobotPose pose = getPoses().get(i);
                // Calculate distance from camera to target (Euclidean distance in 3D space)
                Transform3d transform = pose.cameraToTarget;
                double distance = Math.sqrt(
                    Math.pow(transform.getX(), 2) +
                    Math.pow(transform.getY(), 2) +
                    Math.pow(transform.getZ(), 2)
                );

                if (distance < minDistance) {
                    minDistance = distance;
                    bestPoseIndex = i;
                }
            }

            bestPose = getPoses().get(bestPoseIndex).clone();         
        }
    }

    /**
     * check for new data
     * 
     * @return true if new data; false if stale or no data
     */
    @Override
    public boolean isFresh() {
        return poses != null;
    }

    // These are single-valued characteristics in a possibly multi-tag frame.
    // To see all tags detected use the getPoses() method and parse from it anything you want.

    /**
     * @return best 3-D pose
     */
    @Override
    public Pose3d getPose3d() {
        return bestPose.pose3D;
    }

    /**
     * @return best 2-D pose
     */
    @Override
    public Pose2d getPose2d() {
        return new Pose2d(bestPose.pose3D.getX(), bestPose.pose3D.getY(),
                new Rotation2d(bestPose.pose3D.getRotation().getZ()));
    }

    /**
     * @return best robot yaw to tag
     */
    @Override
    public double getTX() {
        return bestPose.yaw;
    }

    /**
     * @return best robot pitch to tag
     */
    @Override
    public double getTY() {
        return bestPose.pitch;
    }

    /**
     * @return best tag id
     */
    @Override
    public int getTagID() {
        return bestPose.AprilTagId;
    }

    /**
     * @return best transform from camera to target
     */
    @Override
    public Transform3d getCameraToTarget() {
        return bestPose.cameraToTarget;
    }

    /**
     * 
     * @return list of poses from all tag ids detected in the frame
     */
    public ArrayList<RobotPose> getPoses() {
        return poses;
    }
}
