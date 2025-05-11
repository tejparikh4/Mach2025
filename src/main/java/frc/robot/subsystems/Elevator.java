package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import javax.crypto.KeyGenerator;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;;
//max height-116 rot
public class Elevator extends SubsystemBase{
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private SparkClosedLoopController leftController;
    private SparkClosedLoopController rightController;
    private double voltage = 0;
    private static double kDt = 0.02;
    private static double kMaxVelocity = Constants.kElevatorMaxVelocity;
    private static double kMaxAcceleration = Constants.kElevatorMaxAcceleration;

    // kG + kS = 0.4
    // kG - kS = 0.14
    // kG = 0.27
    // kS = 0.13 OLD

    // kg+ks=0.56
    // kg-ks=-0.16
    // kg=0.2
    // ks=0.36

    private static double kG = 0.2;
    private static double kS = 0.36;
    private static double kV = 0.12;
    private static double kA = 0.00;

    private static double kP = 1;
    // private static double kI = 0.0;
    // private static double kD = 0.7;
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State lastSetpoint = new TrapezoidProfile.State();
    private final TrapezoidProfile.State ground = new TrapezoidProfile.State(0, 0);
    private final TrapezoidProfile.State L1 = new TrapezoidProfile.State(10, 0);
    private final TrapezoidProfile.State L2 = new TrapezoidProfile.State(20, 0);
    private double startTime = 0;
    private double startPos = 0;

    private double setToVoltage = 0;
    
    private boolean hasElevated = false;

    private SysIdRoutine sysId;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe rotational distance values, persisted to avoid reallocation.
    private final MutAngle m_distance = Rotations.mutable(0);
    // Mutable holder for unit-safe rotational velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);
    boolean isFinished = false;



    // private final ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, constraints, kDt);
    private final ElevatorFeedforward feedForward =  new ElevatorFeedforward(kS, kG, kV, kA);
    public Elevator() {
        leftMotor = new SparkMax(Constants.elevatorLeftId, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.elevatorRightId, MotorType.kBrushless);
        
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        zeroEncoders();
        // leftController = leftMotor.getClosedLoopController();
        // rightController = rightMotor.getClosedLoopController();

        sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                // null, Volts.of(1), Time time(10, Seconds)

            ),
            new SysIdRoutine.Mechanism(
                voltage -> {
                    leftMotor.setVoltage(voltage);
                    rightMotor.setVoltage(voltage.unaryMinus());
                },
                log -> {
                    log.motor("liftLeftMotor")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                        .angularPosition(m_distance.mut_replace(leftEncoder.getPosition(), Rotations))
                        .angularVelocity(
                            m_velocity.mut_replace(leftEncoder.getVelocity() / 60, RotationsPerSecond));
                }, this
            )
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }
        
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }
    public BooleanSupplier areDoublesEqual(Double double1, Double double2){
        return () -> {return(double1 == double2);};
    }
    public Command moveToHeight(double height){
        // controller.setGoal(height);
        return startRun(() -> {
                startTime = Timer.getFPGATimestamp();
                startPos = getAverageEncoderPosition();
                isFinished = false;
            },()->{
                if(Arm.pivotEncoder.get() < 0.08){return;}


                // trapezoidal motion profile
                setpoint = profile.calculate(Timer.getFPGATimestamp() - startTime, new TrapezoidProfile.State(startPos, 0), new TrapezoidProfile.State(height, 0));

                // feedforward
                SmartDashboard.putNumber("targetVel", setpoint.velocity);
                SmartDashboard.putNumber("measuredLeftVelRPS", leftEncoder.getVelocity() / 60);
                voltage = feedForward.calculateWithVelocities(lastSetpoint.velocity, setpoint.velocity);
                SmartDashboard.putNumber("ff voltage", voltage);

                // pid
                double error = setpoint.position - getAverageEncoderPosition();
                SmartDashboard.putNumber("elevator error", error);

                double output = kP * error;
                SmartDashboard.putNumber("elevator pid voltage", output);

                voltage += output;

                //position limiter
                if(height > startPos && leftMotor.getEncoder().getPosition() > Constants.L4Height) {
                    System.out.println("top hard limit reached");
                    voltage = 0;                                         
                    isFinished = true;
                }

                if(height < startPos && getAverageEncoderPosition() < Constants.intakeHeight) {
                    System.out.println("bottom hard limit reached");
                    voltage = 0;
                    isFinished = true;
                }

                if(Math.abs(getAverageEncoderPosition() - height) < 0.1){
                    // if (setpoint.position == height) {
                    System.out.println("close enough reached");
                    voltage = 0;
                    isFinished = true;
                    // }
                }

                if(height == setpoint.position) {
                    isFinished = true;
                }
                
                runMotors(voltage);
                lastSetpoint = setpoint;
                setpoint.velocity = 0;
                
                return;

            }).until(() -> isFinished)
            .finallyDo(() -> {
                runMotors(kG);
                System.out.println("FINISHED");
            });
        };

    // public void changekV(double amount) {
    //     kV += amount;
    //     feedForward.set
    // }

    // positive is raise
    public Command setSpeed() {
        return startEnd(() ->
            {
                runMotors(setToVoltage);
                
            }, () -> {
                runMotors(0);
            }
        );
    }

    public Command setSpeed(double speed) {
        return startEnd(() ->
            {
                runMotors(speed);
            }, () -> {
                runMotors(0);
            }
        );
    }

    public void runMotors(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(-voltage);
    }


    public void changeSpeed(double delta) {
        setToVoltage += delta;
    }

    public double getAverageEncoderPosition() {
        return (leftEncoder.getPosition() - rightEncoder.getPosition()) / 2;
    }

    public double getAverageEncoderVelocity() {
        return (leftEncoder.getVelocity() - rightEncoder.getVelocity()) / 2 / 60;
    }

    public void zeroEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getkG() {
        return kG;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("leftEncoderPos", leftEncoder.getPosition());
        SmartDashboard.putNumber("rightEncoderPos", rightEncoder.getPosition());
        SmartDashboard.putNumber("averageElevatorEncoderPos", getAverageEncoderPosition());
        SmartDashboard.putNumber("averageElevatorEncoderVel", getAverageEncoderVelocity());
        SmartDashboard.putNumber("setpoint velocity", setpoint.velocity);
        SmartDashboard.putNumber("total elevator voltage", voltage);
        SmartDashboard.putNumber("setToVoltage", setToVoltage);
        SmartDashboard.putNumber("setpoint position", setpoint.position);
        SmartDashboard.putBoolean("Finished", isFinished);
        SmartDashboard.putNumber("12", 12);
        
    }

    
    
    
}
