package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.TCS34725ColorSensor;
import frc.robot.TCS34725ColorSensor.TCSColor;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
   private SparkMax ArmMotor;
   private RelativeEncoder ArmEncoder;
   private double voltage = 0;
   private SparkMax intakeMotortop;
   private SparkMax intakeMotorbottom;
   private double kS = 0;
   private double kG = 0;
   private double kV = 0;
   private double startTime;
   private static double kDt = 0.02;
   private static double kMaxVelocity = 1.75;
   private static double kMaxAcceleration = 0.75;
   private TCS34725ColorSensor colorSensor;
   private TCSColor color = colorSensor.readColors();
   private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocity,
         kMaxAcceleration);
   private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
   private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
   private TrapezoidProfile.State lastSetpoint = new TrapezoidProfile.State();
   private final TrapezoidProfile.State ground = new TrapezoidProfile.State(0, 0);

   public Arm() {
      ArmMotor = new SparkMax(Constants.armId, MotorType.kBrushless);
      intakeMotortop = new SparkMax(Constants.intakeMotortopId, MotorType.kBrushless);
      intakeMotorbottom = new SparkMax(Constants.intakeMotorbottomId, MotorType.kBrushless);
      colorSensor = new TCS34725ColorSensor();
      ArmEncoder = ArmMotor.getEncoder();
   }

   private final ArmFeedforward feedForward = new ArmFeedforward(kS, kG, kV);

   public Command rotate(DoubleSupplier speedDoubleSupplier) {
      return run(() -> {
         ArmMotor.set(speedDoubleSupplier.getAsDouble());
      });
   }

   public Command intake(Double speedDouble) {
      if (!color.equals(new TCSColor(0, 0, 0, 0))) {
         return run(() -> {
            intakeMotorbottom.set(speedDouble);
            intakeMotortop.set(-speedDouble);
         });

      }
      return run(() -> {
         intakeMotorbottom.set(0);
         intakeMotortop.set(0);
      });

   }

   public Command moveToPositron(double height) {
      // controller.setGoal(height);
      return startRun(() -> {
         startTime = Timer.getFPGATimestamp();
         // leftMotor.setVoltage(voltage);
         // rightMotor.setVoltage(-voltage);
      }, () -> {
         lastSetpoint = setpoint;
         setpoint = profile.calculate(Timer.getFPGATimestamp() - startTime, new TrapezoidProfile.State(0, 0),
               new TrapezoidProfile.State(height, 0));

         // voltage = feedForward.calculate(velocity);
         System.out.println("moving to outaking epic style");
         SmartDashboard.putNumber("target: louis vuiotton", setpoint.velocity);
         SmartDashboard.putNumber("measured: louis vuiotton", ArmEncoder.getVelocity());
         voltage = feedForward.calculateWithVelocities(lastSetpoint.position, lastSetpoint.velocity, setpoint.velocity);
         ArmMotor.setVoltage(voltage);
      }).finallyDo(() -> {
         voltage = 0;
         ArmMotor.setVoltage(0);
      });

   }

   public Command outtake(double outtakeSpeed) {
      return run(() -> {
         intakeMotortop.set(outtakeSpeed);
         intakeMotorbottom.set(-outtakeSpeed);
      });
   }
}
