package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Type class for passing the robot pose around between methods.
 * Deep copy made of the constructor arguments.
 * 
 * <p>
 * cameraToTarget Transform3d orientation:
 * <p>
 * x+ is tag is forward of the camera, y+ is tag is left of the camera, z+ is tag is up from the
 * camera
 * 
 * <p>
 * If the camera and tag are straight-on then yaw is +180 deg or -180 deg since the tag must be
 * pointing at the camera to be seen (facing each other).
 * 
 * <p>
 * with tag facing camera:
 * <p>
 * +roll is tag rotated ccw wrt the camera
 * <p>
 * +pitch is top of the tag moved toward the camera
 * <p>
 * +yaw straight-on -180, -179 ... -> 0 as tag rotates around to the left;
 * +180, +179, ... 0 as tag rotates around to the right of the camera
 */
public class RobotPose {
    public int AprilTagId; // integer encoded in tag
    public double yaw; // degrees Robot yaw from tag
    public double pitch; // degrees Robot pitch from tag
    public Pose3d pose3D; // meters and radians Robot Pose in Field
    public Transform3d cameraToTarget; // meters and radians offset from camera to tag

    public RobotPose(int AprilTagId, double yaw, double pitch, Pose3d pose3D, Transform3d cameraToTarget) {
        // deep copy of the arguments
        this.AprilTagId = AprilTagId;
        this.yaw = yaw;
        this.pitch = pitch;
        this.pose3D = new Pose3d(
                pose3D.getX(), pose3D.getY(), pose3D.getZ(),
                new Rotation3d(pose3D.getRotation().getX(), pose3D.getRotation().getY(), pose3D.getRotation().getZ()));
        this.cameraToTarget = cameraToTarget;

        // var t = getCameraToTarget();
        // System.out.println("tag is forward " + t.getX() + "m, left " + t.getY() + "m, up " + t.getZ() +
        // "m, roll " + Units.radiansToDegrees(t.getRotation().getX()) +
        // "deg, pitch " + Units.radiansToDegrees(t.getRotation().getY()) +
        // "deg, yaw " + Units.radiansToDegrees(t.getRotation().getZ()) + "deg");
    }

    /**
     * deep copy of object based on constructor
     */
    public RobotPose clone() {
        return new RobotPose(AprilTagId, yaw, pitch, pose3D, cameraToTarget);
    }

    /**
     * Formatted robot pose
     * 
     * @return printable string of the robot pose
     */
    public String toString() {
        return String.format("tag %d, yaw [deg] %f, pitch [deg] %f, %s, %s", AprilTagId, yaw, pitch, pose3D,
                cameraToTarget);
    }
}
