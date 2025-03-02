package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class limelightDrive{
    private SwerveSubsystem drivebase;
    private boolean completed = false;
    private Timer timer = new Timer();
    private SlewRateLimiter limiter = new SlewRateLimiter(3.0);

    public limelightDrive(SwerveSubsystem driveBase){
        this.drivebase = driveBase;
    }
    public void getToPosition(Pose2d wantedPosition){
        Pose2d currentPostion = drivebase.getPose();
        double xError = wantedPosition.getX()-currentPostion.getX();
        double yError = wantedPosition.getY()-currentPostion.getY();
        Rotation2d thetaError = Rotation2d.fromDegrees(wantedPosition.getRotation().getDegrees()).minus(currentPostion.getRotation());
        Double thetaErrorInDegrees = thetaError.getDegrees();
        DoubleSupplier xSupplier = ()->xError;
        DoubleSupplier ySupplier = ()->yError;
        DoubleSupplier thetaSupplier = ()->thetaErrorInDegrees;
        drivebase.driveCommand(xSupplier, ySupplier, thetaSupplier);
    }
}  
