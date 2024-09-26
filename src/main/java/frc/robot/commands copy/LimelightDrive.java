package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class LimelightDrive extends CommandBase{ 
    private Camera m_camera;
    private Swerve m_swerve;
    private boolean complete = false;
    private double timeout;
    private Timer timer = new Timer();
    private SlewRateLimiter limiter = new SlewRateLimiter(3.0);
    private double[] botpose;
    private double xDistance = Units.inchesToMeters(40);
    private double yDistance = 0;
    private double theta = 0;



    public LimelightDrive(Camera camera, Swerve swerve, double timeout, double xDistance, double yDistance, double theta) {
        m_camera = camera;
        m_swerve  = swerve;
        this.timeout = timeout;
        addRequirements(m_swerve);
        this.xDistance = Units.inchesToMeters(xDistance);
        this.yDistance = Units.inchesToMeters(yDistance);
        this.theta = theta;

    }

    @Override
    public void initialize() {
        complete = false;
        timer.start();
    }

    @Override
    public void execute() {

        botpose = m_camera.getBotpose();
        final double anglekP = 0.005;
        final double distancekP = 0.1;
        if (botpose[0] == 0) {
            complete = true;
            m_swerve.drive(new Translation2d(0, 0), 0, false, false);
            return;
      }
        double xError = xDistance + botpose[2];
        double yError = yDistance - botpose[0];
        double angleError = -botpose[4];
        
        
        // SmartDashboard.putNumber("distanceError", distanceError);
        SmartDashboard.putNumber("LimelightxError", xError);
        SmartDashboard.putNumber("LimelightyError", yError);
        SmartDashboard.putNumber("LimelightangleError", angleError);


        double angularSpeed = MathUtil.clamp(angleError * anglekP, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity*0.5);
        double xSpeed = xError * distancekP;
        double ySpeed = yError * distancekP;

        SmartDashboard.putNumber("LimelightxSpeed", xSpeed);
        SmartDashboard.putNumber("LimelightySpeed", ySpeed);
        SmartDashboard.putNumber("LimelightangleSpeed", angularSpeed);

        

    
        if (Math.abs(angleError) > 6 || Math.abs(xError) > 0.1 || Math.abs(yError) > 0.1 && timer.get() < timeout) {
            m_swerve.drive(new Translation2d(-xSpeed, -ySpeed), angularSpeed, false, true);
        } else {
            complete = true;
        }
    }    

    @Override
    public boolean isFinished() {
        return complete;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(new Translation2d(0,0), 0, false, true);
        timer.stop();
    }
    
}