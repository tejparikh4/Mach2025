package frc.robot.subsystems;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import swervelib.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
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
public class Camera {
    swervelib.SwerveDrive drivebase;
    SwerveSubsystem swerve;
    SwerveDrivePoseEstimator poseEstimator;
    String llName;
    
    public Camera(SwerveSubsystem swerve) {
        this.swerve = swerve;
        poseEstimator = swerve.getSwerveDrive().swerveDrivePoseEstimator;
    }

    public void periodic() {
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials("limelight");
        
        // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        // if(Math.abs(m_gyro.getRate()) > 360)
        // {
        //     doRejectUpdate = true;
        // }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            double closestFiducial = Double.POSITIVE_INFINITY;
            for (RawFiducial fiducial : rawFiducials) {
                if (fiducial.distToCamera < closestFiducial) {
                    closestFiducial = fiducial.distToCamera;
                }
            }
            SmartDashboard.putNumber("closestFiducial", closestFiducial);
            double std = 0.3 + closestFiducial * 0.4;
            SmartDashboard.putNumber("sexually transmitted disease", std);
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(std,std,9999999));
            poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
    }

}