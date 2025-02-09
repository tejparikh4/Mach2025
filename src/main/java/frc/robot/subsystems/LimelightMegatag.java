package frc.robot.subsystems;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import swervelib.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Util.RectanglePoseArea;
public class LimelightMegatag {
    swervelib.SwerveDrive drivebase;
    String llName;
    private Boolean enabled = true;
    private Boolean trust = false;
    private double Confidence = 0.0;
    private double compareDistance;
    private Pose2d limelightPostionEstimate;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Pose");
    private final DoubleArrayPublisher limelPublisher = table.getDoubleArrayTopic("llPose").publish();
    // private final RectanglePoseArea fieldBoundry = new RectanglePoseArea()
    public LimelightMegatag(swervelib.SwerveDrive drivebase, String limelightName){
        this.llName = limelightName;
        this.drivebase = drivebase;
        LimelightHelpers.setPipelineIndex(limelightName, 0);    
    }
    @Override
    public void periodic(){
        if((enabled||DriverStation.isEnabled()) && !RobotBase.isSimulation()){
            Confidence =0;
            LimelightHelpers.SetRobotOrientation(llName, drivebase.getYaw().getDegrees(), 0,0,0,0,0);
            LimelightHelpers.PoseEstimate LimelightMesurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);
            LimelightHelpers.PoseEstimate LimelightMesurementNew = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            SmartDashboard.putNumber("Number of Tags Present",LimelightMesurement.tagCount);

            if(LimelightMesurement.tagCount >0 && fieldBoundry.isPoseWithinArena)



        }
    }

}
