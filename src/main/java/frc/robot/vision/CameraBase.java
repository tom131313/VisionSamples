package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Base class of cameras that supply the pose of the robot.
 * 
 * <p>
 * Defines required methods for camera classes to provide 1-D, 2-D, and 3-D poses.
 * 
 * <p>
 * Provides requirement of methods to put the robot 3-D pose from the camera on
 * NetworkTables for display purposes.
 */
public abstract class CameraBase {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    // private CameraBase() {}

    public abstract void update();

    public abstract boolean isFresh();

    public abstract int getTagID();

    public abstract Pose3d getPose3d();

    public abstract Pose2d getPose2d();

    public abstract double getTX();

    public abstract double getTY();

    public abstract Transform3d getCameraToTarget();
}
