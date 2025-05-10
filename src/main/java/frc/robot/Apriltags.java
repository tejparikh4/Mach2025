package frc.robot;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Apriltags {

    public static double outOffset = 0.6;
    public static double leftRightOffset = 0.18;

    public static Map<Integer, Pose2d> aprilTagMap = new HashMap<>();

    public static final List<Pose2d> tagLocations = Arrays.asList(
        new Pose2d(13.474, 3.307, Rotation2d.fromDegrees(300)),
        new Pose2d(13.891, 4.025, Rotation2d.fromDegrees(0)),
        new Pose2d(13.474, 4.745, Rotation2d.fromDegrees(60)),
        new Pose2d(12.644, 4.745, Rotation2d.fromDegrees(120)),
        new Pose2d(12.228, 4.025, Rotation2d.fromDegrees(180)),
        new Pose2d(12.644, 3.307, Rotation2d.fromDegrees(240)),
        new Pose2d(4.074, 3.307, Rotation2d.fromDegrees(240)),
        new Pose2d(3.658, 4.025, Rotation2d.fromDegrees(180)),
        new Pose2d(4.074, 4.745, Rotation2d.fromDegrees(120)),
        new Pose2d(4.905, 4.745, Rotation2d.fromDegrees(60)),
        new Pose2d(5.321, 4.025, Rotation2d.fromDegrees(0)),
        new Pose2d(4.905, 3.307, Rotation2d.fromDegrees(300))
    );



    public Apriltags() {
        aprilTagMap.put(6, new Pose2d(13.272, 3.257, Rotation2d.fromDegrees(300)));
        aprilTagMap.put(7, new Pose2d(13.682, 3.965, Rotation2d.fromDegrees(0)));
        aprilTagMap.put(8, new Pose2d(13.272, 4.674, Rotation2d.fromDegrees(60)));
        aprilTagMap.put(9, new Pose2d(12.454, 4.674, Rotation2d.fromDegrees(120)));
        aprilTagMap.put(10, new Pose2d(12.044, 3.965, Rotation2d.fromDegrees(180)));
        aprilTagMap.put(11, new Pose2d(12.454, 3.257, Rotation2d.fromDegrees(240)));
        aprilTagMap.put(17, new Pose2d(4.013, 3.257, Rotation2d.fromDegrees(240)));
        aprilTagMap.put(18, new Pose2d(3.603, 3.965, Rotation2d.fromDegrees(180)));
        aprilTagMap.put(19, new Pose2d(4.013, 4.674, Rotation2d.fromDegrees(120)));
        aprilTagMap.put(20, new Pose2d(4.831, 4.674, Rotation2d.fromDegrees(60)));
        aprilTagMap.put(21, new Pose2d(5.241, 3.965, Rotation2d.fromDegrees(0)));
        aprilTagMap.put(22, new Pose2d(4.831, 3.257, Rotation2d.fromDegrees(300)));

    }

    public static Pose2d getTagLocation(int tag) {
        return aprilTagMap.get(tag);
    }

    public static Pose2d getTargetLocation(int tag, int leftOrRight) {
        Pose2d tagPose = getTagLocation(tag);
        double x = tagPose.getX() + Math.cos(tagPose.getRotation().getRadians()) * outOffset - leftOrRight * Math.sin(tagPose.getRotation().getRadians()) * leftRightOffset;
        double y = tagPose.getY() + Math.sin(tagPose.getRotation().getRadians()) * outOffset + leftOrRight * Math.cos(tagPose.getRotation().getRadians()) * leftRightOffset;
        return new Pose2d(x, y, Rotation2d.fromDegrees(tagPose.getRotation().getDegrees() + 180));
        
    }

    public static Pose2d getTargetLocation(Pose2d tagPose, int leftOrRight) {
        double x = tagPose.getX() + Math.cos(tagPose.getRotation().getRadians()) * outOffset - leftOrRight * Math.sin(tagPose.getRotation().getRadians()) * leftRightOffset;
        double y = tagPose.getY() + Math.sin(tagPose.getRotation().getRadians()) * outOffset + leftOrRight * Math.cos(tagPose.getRotation().getRadians()) * leftRightOffset;
        return new Pose2d(x, y, Rotation2d.fromDegrees(tagPose.getRotation().getDegrees() + 180));
        
    }
}
