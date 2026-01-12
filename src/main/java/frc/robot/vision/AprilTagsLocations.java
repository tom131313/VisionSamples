package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Easy access to the WPILib AprilTag Field Layout
 * <p>
 * Tag locations on the field are used to calculate robot locations on the field given the robot
 * location wrt the tag.
 * 
 * <p>
 * The tag locations are published to NetworkTables and may be displayed with AdvantageScope.
 * <p>
 * Throughout this project Lists assume there is no tag 0 so tag 1 becomes list index 0. Also, it
 * is assumed that the tag numbers are continuously numbered from 1 to size of list. If there are
 * gaps, then a lot of things break in this project.
 */
public class AprilTagsLocations {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    private static AprilTagFieldLayout aprilTagFieldLayout;
    private static NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("AprilTagsLocations");

    static {

        final boolean CustomTagLayout = false; // true is use custom deploy of layout

        // Tag positions
        // tag rotation is CCW looking down on field from the ceiling.
        // rotating around Z, 0 degrees is parallel to Y and facing down field or +X. 30 degrees is
        // still facing down field +X and a little facing into the +Y across the field

        try {
            if (CustomTagLayout)
                aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json"); // custom file example (missing)
            else
                aprilTagFieldLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2026RebuiltWelded);
        } catch (IOException e) {
            e.printStackTrace();
            aprilTagFieldLayout = null;
        }

        List<StructPublisher<Pose3d>> publishTagPose = new ArrayList<>(getTagCount());

        Consumer<AprilTag> initializeTagPoses = tag -> {
            System.out.format("%s %6.1f, %6.1f, %6.1f [degrees]%n",
                    tag.toString(),
                    Units.radiansToDegrees(tag.pose.getRotation().getX()),
                    Units.radiansToDegrees(tag.pose.getRotation().getY()),
                    Units.radiansToDegrees(tag.pose.getRotation().getZ()));
            // assuming there is no tag 0 so indexing starts with tag 1 in list position 0. Ugh!
            var tagPosePublisher = tagsTable.getStructTopic("tagPose3D_" + tag.ID, Pose3d.struct).publish();
            publishTagPose.add(tagPosePublisher);
            publishTagPose.get(tag.ID - 1).set(tag.pose); // no tag 0 so index back 1; ouch! that sequencing trick hurts!
        };

        System.out.println(aprilTagFieldLayout.getTags().size() + " Tags on file");
        aprilTagFieldLayout.getTags().forEach(initializeTagPoses);
    }

    private AprilTagsLocations() {
    }

    public static int getTagCount() {
        return aprilTagFieldLayout.getTags().size();
    }

    public static List<AprilTag> getTagsLocations() {
        return aprilTagFieldLayout.getTags();
    }

    public static Pose3d getTagLocation(int tagID) {
        return aprilTagFieldLayout.getTags().get(tagID - 1).pose; // no tag 0 so index back one; ouch! that sequencing trick hurts!
    }
}
