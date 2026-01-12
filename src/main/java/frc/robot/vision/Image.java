package frc.robot.vision;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagDetection;

/**
 * Buffers an image with setImage() method.
 * <p>
 * getImage() method has a thread wait until a new image is set (blocking).
 * [disabled for this single threaded vision set and get use]
 * <p>
 * Multi-Thread-Safe version (overkill if that isn't needed)
 * [disabled - no longer thread safe as its use here is single thread]
 */
public class Image {
    /**
     * image from camera in OpenCV Mat form and detections
     */
    private Mat mat = new Mat();
    private AprilTagDetection[] detections;
    private double acquisitionTime = 0L;
    private int frameNumber = 0;
    private boolean isFreshImage = false; // indicates a new camera image; no implication if that image is more or less useful than the previous image

    public /* synchronized */ void setImage(Mat mat, AprilTagDetection[] detections, double acquisitionTime,
            int frameNumber) {
        mat.copyTo(this.mat);
        this.detections = detections.clone();
        this.acquisitionTime = acquisitionTime;
        this.frameNumber = frameNumber;
        this.isFreshImage = true;
        // notify(); // fresh image so tell whoever is waiting for it
    }

    public /* synchronized */ AprilTagDetection[] getImage(Mat mat, AcquisitionTime acquisitionTime) {
        // try
        // {
        // while( ! this.isFreshImage) // make sure awakened for the right reason
        // {
        // wait(0L, 0); // stale image so thread wait for a new image no timeout
        // }
        // }
        // catch (Exception e)
        // {
        // System.out.println("getImage Exception " + e.toString());
        // throw new RuntimeException(e);
        // }
        this.isFreshImage = false;
        this.mat.copyTo(mat);
        acquisitionTime.acquisitionTime = this.acquisitionTime;
        acquisitionTime.frameNumber = this.frameNumber;
        return this.detections;
    }

    public /* synchronized */ boolean isFreshImage() {
        return this.isFreshImage;
    }

    /**
     * Image information provided by the caller of the setImage() method (not generated herein).
     */
    class AcquisitionTime {
        protected int frameNumber = 0;
        protected double acquisitionTime = 0;
    }
}
