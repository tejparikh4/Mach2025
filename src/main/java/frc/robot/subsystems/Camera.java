package frc.robot.subsystems;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import swervelib.SwerveDrive;
public class Camera extends SubsystemBase {
    swervelib.SwerveDrive drivebase;
    SwerveSubsystem swerve;
    SwerveDrive swerveDrive;
    SwerveDrivePoseEstimator poseEstimator;
    String llName;

    StructPublisher<Pose2d> limelightPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("limelightOdometry", Pose2d.struct).publish();
    
    public Camera(SwerveSubsystem swerve) {
        this.swerve = swerve;
        swerveDrive = swerve.getSwerveDrive();
        poseEstimator = swerveDrive.swerveDrivePoseEstimator;
    }

    
    public void periodic() {
        boolean doRejectUpdate = false;
        SmartDashboard.putNumber("Gyro Rotation", swerveDrive.getGyro().getRotation3d().getZ());
        LimelightHelpers.SetRobotOrientation("", swerveDrive.getGyro().getRotation3d().getZ() * 180 / Math.PI/*poseEstimator.getEstimatedPosition().getRotation().getDegrees()*/, 0, 0, 0, 0, 0);
        // LimelightHelpers.SetRobotOrientation("", swerve.getGyroRaw(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials("");

        
        
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
            // double std = 0.3 + closestFiducial * 0.4;

            double[] stds = NetworkTableInstance.getDefault().getTable("limelight").getEntry("stddevs").getDoubleArray(new double[6]);
            SmartDashboard.putNumber("std x", stds[6]);
            SmartDashboard.putNumber("std y", stds[7]);
            // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(std,std,9999999));
            // poseEstimator.addVisionMeasurement(
            //     mt2.pose,
            //     mt2.timestampSeconds);

            limelightPublisher.set(mt2.pose);
        }
    }

}