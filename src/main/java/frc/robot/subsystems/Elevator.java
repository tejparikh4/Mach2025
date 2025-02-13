package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private SparkClosedLoopController leftController;
    private SparkClosedLoopController rightController;
    private static double kDt = 0.02;
    private static double kMaxVelocity = 1.75;
    private static double kMaxAcceleration = 0.75;
    private static double kP = 1.3;
    private static double kI = 0.0;
    private static double kD = 0.7;
    private static double kS = 1.1;
    private static double kG = 1.2;
    private static double kV = 1.3;
    private final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(1.5, 0.75);
    private final ProfiledPIDController m_Controller = new ProfiledPIDController(kP, kI, kD, m_Constraints);
    private final ElevatorFeedforward m_Feedforward =  new ElevatorFeedforward(kS, kG, kV);
    public Elevator() {
        leftMotor = new SparkMax(Constants.elevatorLeftId, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.elevatorRightId, MotorType.kBrushless);

        leftController = leftMotor.getClosedLoopController();
        rightController = rightMotor.getClosedLoopController();
    }

    public Command moveToHeight(double height){
        m_Controller.setGoal(height);
        return run(() -> leftMotor.setVoltage(m_Controller.calculate(leftMotor.getAbsoluteEncoder().getPosition())+m_Feedforward.calculate(m_Controller.getSetpoint().velocity)));

    }
    
    
}
