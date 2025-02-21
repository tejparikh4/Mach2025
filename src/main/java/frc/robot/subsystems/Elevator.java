package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//max height-116 rot
public class Elevator extends SubsystemBase{
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private SparkClosedLoopController leftController;
    private SparkClosedLoopController rightController;
    private double voltage = 0;
    private static double kDt = 0.02;
    private static double kMaxVelocity = 1.75;
    private static double kMaxAcceleration = 0.75;
    private static double kS = 1.1;
    private static double kG = 1.2;
    private static double kV = 1.3;
    private static double kA = 0;
    // private static double kP = 1.3;
    // private static double kI = 0.0;
    // private static double kD = 0.7;
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private final TrapezoidProfile.State ground = new TrapezoidProfile.State(0, 0);
    private final TrapezoidProfile.State L1 = new TrapezoidProfile.State(10, 0);
    private final TrapezoidProfile.State L2 = new TrapezoidProfile.State(20, 0);


    // private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, constraints, kDt);
    private final ElevatorFeedforward feedForward =  new ElevatorFeedforward(kS, kG, kV);
    public Elevator() {
        leftMotor = new SparkMax(Constants.elevatorLeftId, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.elevatorRightId, MotorType.kBrushless);

        leftController = leftMotor.getClosedLoopController();
        rightController = rightMotor.getClosedLoopController();
    }

    public Command moveToHeight(double height){
        // controller.setGoal(height);
        setpoint = profile.calculate(1, ground, new TrapezoidProfile.State(height,0));
        return startEnd(() -> {
            setpoint = profile.calculate(0.2, setpoint, new TrapezoidProfile.State(height,0));
            // voltage = feedForward.calculate(velocity);

            voltage = feedForward.calculateWithVelocities(leftMotor.getEncoder().getVelocity(), setpoint.velocity);
            leftMotor.setVoltage(voltage);
            rightMotor.setVoltage(-voltage);
        },()->{
            leftMotor.setVoltage(0);
            rightMotor.setVoltage(0);
        });
    }

    // positive is raise
    public Command setSpeed(double speed) {
        return startEnd(() ->
            {
                leftMotor.set(speed);
                rightMotor.set(-speed);
            }, () -> {
                leftMotor.set(0);
                rightMotor.set(0);
            }
        );
    }

    

    @Override
    public void periodic(){
        SmartDashboard.putNumber("leftEncoderPos", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("rightEncoderPos", rightMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("feedforward voltage", voltage);
    }
    
    
}
