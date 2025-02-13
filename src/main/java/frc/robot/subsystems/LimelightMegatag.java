package frc.robot.subsystems;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import swervelib.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
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
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.geometry.Translation2d;
public class LimelightMegatag {
    swervelib.SwerveDrive drivebase;
    SwerveSubsystem swerve;
    String llName;
    public String currentAllience = DriverStation.getAlliance().toString();
    private Boolean enabled = true;
    private Boolean trust = false;
    private double Confidence = 0.0;
    private double compareDistance;
    private Pose2d limelightPostionEstimate;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Pose");
    private final DoubleArrayPublisher limelPublisher = table.getDoubleArrayTopic("llPose").publish();
    private final Rectangle2d fieldBoundary = new Rectangle2d(new Translation2d(0,0),new Translation2d(16.514,8.211));
    // private final RectanglePoseArea fieldBoundry = new RectanglePoseArea()
    public LimelightMegatag(swervelib.SwerveDrive drivebase, String limelightName){
        this.llName = limelightName;
        this.drivebase = drivebase;
        LimelightHelpers.setPipelineIndex(limelightName, 0);    
    }
    public void periodic(){
        if((enabled||DriverStation.isEnabled()) && !RobotBase.isSimulation()){
            Confidence =0;
            LimelightHelpers.SetRobotOrientation(llName, drivebase.getYaw().getDegrees(), 0,0,0,0,0);
            LimelightHelpers.PoseEstimate LimelightMesurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);
            LimelightHelpers.PoseEstimate LimelightMesurementNew = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
            SmartDashboard.putNumber("Number of Tags Present",LimelightMesurement.tagCount);

            if(LimelightMesurement.tagCount >0 && fieldBoundary.contains(LimelightMesurement.pose.getTranslation())){
                if(LimelightMesurement.avgTagDist < Units.feetToMeters(12)){
                    Confidence = 0.5;    
                }else{
                    Confidence = 0.7;
                    LimelightMesurement = LimelightMesurementNew;
                }                
            }
            addPose(LimelightMesurement, Confidence);



        }
    }
    public void addPose(PoseEstimate lEstimate, Double confidence){
        lEstimate.isMegaTag2 = true;
        if(confidence>0){
            SmartDashboard.putString("Pose Update","Succeeded");
            publishToField(lEstimate);
            swerve.checkOdometry(lEstimate.pose,lEstimate.timestampSeconds);
        }else{
            SmartDashboard.putString("Pose Update","Failed");
             publishToField(new PoseEstimate(new Pose2d(), 0., 0., 0, 0, 0., 0., new RawFiducial[0], true));
        }
    }
    private void publishToField(PoseEstimate lMesurements){
        limelPublisher.set(new double[]{
            lMesurements.pose.getX(),
            lMesurements.pose.getY(),
            lMesurements.pose.getRotation().getDegrees()
        });
    }

}