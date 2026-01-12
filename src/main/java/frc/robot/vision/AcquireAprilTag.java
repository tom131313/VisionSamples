package frc.robot.vision;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetector.QuadThresholdParameters;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ControllerVision.Camera;

/**
 * WPILib version of Detect AprilTags
 * 
 * This was intended to run in an independent thread but now in this project it shares a thread
 * with AcquireRobotPose.
 * 
 * A USB camera is defined and then this loops forever acquiring an image from the camera
 * and detecting any AprilTags in the image. Detected AprilTags are saved for subsequent
 * processing by pose estimation.
 * 
 * Camera image displayed on port 1181
 * 
 * It was a free-wheeling thread that acquires detected tags from camera images as fast as
 * but now in this project it is paced also by completion of the {@link AcquireRobotPose#run()}.
 */

public class AcquireAprilTag {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    private ControllerVision controllerVision;
    private int frameNumber = 0;
    private AprilTagDetector detector = new AprilTagDetector();
    private CvSink cvSink;
    private Mat mat = new Mat();
    private Mat grayMat = new Mat();

    AcquireAprilTag(ControllerVision controllerVision, int cameraDeviceId, Camera camera) {
        this.controllerVision = controllerVision;

        // Caution: correct 0 or 1 error bits on roboRIO v1 or up to 2 on roboRIO v2
        // error correction is a memory hog so Simulator mode on a PC can go higher but a roboRIO cannot
        var bitsCorrected = 1; // configurable but unlikely changes needed
        detector.addFamily("tag36h11", bitsCorrected);

        // 2025 and 2026 have 300 default which is too large to be useful so use 2024 value of 5
        QuadThresholdParameters qtp = detector.getQuadThresholdParameters();
        qtp.minClusterPixels = 5; // configurable but unlikely changes needed
        // qtp.criticalAngle = 10 * Math.PI / 180.0; // default changed to 45 degrees for 2025 and 2026
        detector.setQuadThresholdParameters(qtp);

        // Get the UsbCamera from CameraServer
        UsbCamera cameraCapture = CameraServer.startAutomaticCapture(cameraDeviceId); // http://10.42.37.2:1181/ http://roborio-4237-frc.local:1181/?action=stream
        // "myCam", "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0"
        cameraCapture.setResolution(camera.width, camera.height);
        cameraCapture.setFPS(camera.fps);

        // cameraCapture.setExposureAuto();

        // Get a CvSink. This will capture Mats from the camera
        cvSink = CameraServer.getVideo(cameraCapture);
    }

    /**
     * Acquire a camera frame. Since no longer in its own thread it must be iterated in an external loop.
     */
    public void run() {

        double acquisitionTime = 0;
        long frameError = 0;

        frameNumber++;
        acquisitionTime = Timer.getFPGATimestamp();

        if (cvSink.grabFrame(mat, 1.) == frameError) {
            // Send the output the error.
            System.out.println("camera error " + cvSink.getError());
            // skip the rest of the current iteration
            return;
        }
        Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_BGR2GRAY); // color camera 3 channels to 1 gray channel

        // Core.extractChannel(mat, grayMat, 0); // monochrome camera on 3 channels to 1 gray channel (isn't significantly better than above)

        AprilTagDetection[] detections = detector.detect(grayMat);

        // new Image with detections available for use so pass it on
        controllerVision.image.setImage(mat, detections, acquisitionTime, frameNumber);
    }
}
