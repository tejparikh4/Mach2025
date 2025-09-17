package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;

public class photon  extends SubsystemBase{
    SwerveSubsystem swerveDriveBase;
    SwerveDrive swerveDrive;

    PhotonCamera camera;
    PhotonPoseEstimator photonEstimator;
    private Matrix<N3,N1> curStdevs;
    EstimateConsumer estConsumer;


    public photon(SwerveSubsystem swerveDriveBase, EstimateConsumer estConsumer){
        this.swerveDriveBase = swerveDriveBase;
        this.camera = new PhotonCamera("photonvision");
        this.estConsumer = estConsumer;
          photonEstimator =
                new PhotonPoseEstimator(Constants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    public void getResults(){
        var result = camera.getAllUnreadResults();
        boolean hasTargets = !result.isEmpty();
        if (!hasTargets ){
            return ;
        }
        else{
            PhotonPipelineResult target = result.get(0);
            
        }
    }
    
    
    
    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

}
