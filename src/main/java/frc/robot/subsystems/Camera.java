//pranav and kevin
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera extends SubsystemBase {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");

    // how many degrees back is your limelight rotated from perfectly vertical?
    private final double limelightMountAngleDegrees = 45.0; 

    // distance from the center of the Limelight lens to the floor
    private final double limelightLensHeightInches = 4.5; 

    // distance from the target to the floor
    private final double goalHeightInches = 60.0; 
    private double distanceToGoalInches;
    private double heading;
    double[] botpose;

    public Camera() {
        // table.getEntry("ledMode").setNumber(1);
    }

    public void periodic() {
        heading = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        botpose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

        // if (botpose[5] < 0)
        //     botpose[5] += 360;

        SmartDashboard.putNumber("LimelightX", heading);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightBotposeX", botpose[0]);
        SmartDashboard.putNumber("LimelightBotposeY", botpose[1]);
        SmartDashboard.putNumber("LimelightBotposeZ", botpose[2]);
        SmartDashboard.putNumber("LimelightBotposeRoll", botpose[3]);
        SmartDashboard.putNumber("LimelightBotposePitch", botpose[4]);
        SmartDashboard.putNumber("LimelightBotposeYaw", botpose[5]);


        double angleToGoalDegrees = limelightMountAngleDegrees + y;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
        distanceToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians); //58 inches

        SmartDashboard.putNumber("LimelightDistance", distanceToGoalInches);
        SmartDashboard.putNumber("LimelightHeading", heading);

    }

    public double getDistanceToGoal() {
        return distanceToGoalInches;
    }

    public double getHeading() {
        return heading;
    }

    public double[] getBotpose() {
        return botpose;
    }
}
