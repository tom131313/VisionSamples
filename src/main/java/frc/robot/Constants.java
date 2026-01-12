package frc.robot;

import org.opencv.core.MatOfDouble;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. These values are not expected to change (often). This class should not be used for any
 * other purpose. All constants should be declared globally (i.e. public static). Do not put
 * anything functional in this class.
 * <p>
 * Variables that are considered user settable to configure a particular use should be set in the
 * {@link Config} interface.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * <p>
 * There are a few user settable constants scattered around this project especially in
 * {@link AcquireAprilTag}. These likely will not be changed and aren't in {@link Config} or
 * {@link Constants}.
 */
public final class Constants {

    // Xbox controller port used to initiate alignment commands
    public static final int driverController = 0;

    /**
     * Vision constants for camera calibration and configuration
     */
    public static final class ControllerVision {

        /**
         * Camera calibration data for a specific camera configuration
         * <p>
         * Commonly used formats of coefficients (PhotonVision and ControllerVision using WPILib)
         * <p>
         * camera matrix format [fx, 0, cx, 0, fy, cy, 0, 0, 1]
         * <p>
         * | fx 0 cx |
         * <p>
         * | 0 fy cy |
         * <p>
         * | 0  0  1 |
         * <p>
         * PhotonVision exports the first 8 distortion coefficients and assumes the last 6 are zeros
         * <p>
         * camera 8 distortion coefficients [k1, k2, p1, p2, k3, k4, k5, k6]
         * <p>
         * assume last 6 distortion coefficients [s1, s2, s3, s4, tx, ty] are all zeros
         * <p>
         * OpenCV can use 4, 5, 8, 12, or 14 distortion coefficients
         * <p>
         * If you don't have camera parameters, then here's a way to estimate them from resolution
         * and camera sensor size (guestimates given). [It's easy to calibrate a camera with 
         * PhotonVision. Download it to your DriverStation and run: java -jar "path_to_the_jar"]
         * <p>
         * Estimating camera parameters:
         * <p>
         * Focal Length (pixels) approx Focal Length (mm) * Resolution (pixels) / Sensor Size (mm)
         * <p>
         * For example, a typical inexpensive USB camera might have:
         * <p>
         * Focal Length (mm): 4mm (common for wide-angle views)
         * <p>
         * Sensor Size (horizontal): ~3.6 mm (for a 1/3" sensor) or ~4.8 mm (for a 1/2.5" sensor)
         * <p>
         * Resolution (horizontal): 640 pixels (VGA) or 1920 pixels (1080p) 
         * <p>
         * Using these approximate values, the focal length in pixels could range from about 600 to
         * 2000 pixels.
         * <p>
         * Principal point: cx about width/2;  cy about height/2
         * <p>
         * Calibrate the camera at the used resolution or scale Fx,Fy,Cx,Cy proportional to what
         * resolution was used for camera calibration. (But apparently recalibrating at various
         * resolutions does yield slightly varying results and is often recommended.)
         */
        public enum Camera {
            /**
             * ArduCam 320x240 calibration
             * Camera: Arducam_OV9281_USB_camera_(B); cameron board 320x240
             */
            ARDUCAM_320x240(320, 240, 100,
                    274.56056711562485,
                    273.97669170487774,
                    154.97945360319505,
                    123.27353823076139,
                    new MatOfDouble(
                            0.08661082744478585, -0.2648606329052293, 0.0011702574017837134, -0.005142425624676014,
                            0.4949799731829983,
                            -0.004071726866234826, 0.009276949314993517, -0.020626159841108008)),

            /**
             * ArduCam 1280x800 calibration
             * Camera: Arducam_OV9281_USB_Camera_(A?); Cameron board 1280x800
             */
            ARDUCAM_1280x800(1280, 800, 100,
                    907.6920444758049,
                    907.1513951038395,
                    604.1750223777503,
                    416.4609913313957,
                    new MatOfDouble(
                            0.040354289830866516, -0.044066115475547216, 6.662818829158613E-4, 9.755603732755772E-4, // k1 k2 p1 p2
                            -0.013630390510289322, // k3
                            -0.0011985508423857224, 0.003370423168524356, 0.0010337869630847195 // k4 k5 k6
                    )), // Assumes s1 s2 s3 s4 tx ty are all zeros)

            /**
             * LifeCam 320x240 calibration from PhotonVision
             */
            LIFECAM_320x240(320, 240, 30,
                    353.74653217742724,
                    340.77624878700817,
                    163.5540798921191,
                    119.8945718300403,
                    new MatOfDouble()), // assume no distortion)

            /**
             * LifeCam 640x480 calibration from WPILib example
             * Source: https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21
             */
            LIFECAM_640x480(640, 480, 30,
                    699.3778103158814,
                    677.7161226393544,
                    345.6059345433618,
                    207.12741326228522,
                    new MatOfDouble()); // assume no distortion)

            public final int width;
            public final int height;
            public final int fps;
            public final double fx; // horizontal focal length, in pixels
            public final double fy; // vertical focal length, in pixels
            public final double cx; // horizontal focal center, in pixels
            public final double cy; // vertical focal center, in pixels
            public final MatOfDouble distortionCoeffs;

            Camera(int width, int height, int fps,
                    double fx, double fy, double cx, double cy,
                    MatOfDouble distortionCoeffs) {
                this.width = width;
                this.height = height;
                this.fps = fps;
                this.fx = fx;
                this.fy = fy;
                this.cx = cx;
                this.cy = cy;
                this.distortionCoeffs = distortionCoeffs;
            }
        };
    }
}
