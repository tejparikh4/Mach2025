package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.TCS34725ColorSensor;
import frc.robot.TCS34725ColorSensor.TCSColor;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
   private SparkMax pivotMotor;
   public static DutyCycleEncoder pivotEncoder;
   private double voltage = 0;
   private SparkMax intakeMotorTop;
   private SparkMax intakeMotorBottom;
   private double kS = 0;
   private double kG = 0;
   private double kV = 0;
   private double startTime;
   private static double kMaxVelocity = (32* Math.PI);
   private static double kMaxAcceleration = (16 * Math.PI);
   boolean endIntake = false;

   private TCS34725ColorSensor colorSensor;
   private TCSColor color;
   private double colorSum;
   private boolean isCoral = false;
   private boolean prevIsCoral = false;
   private boolean isFinishedRotating = false;

   private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocity,
         kMaxAcceleration);
   private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
   private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
   private TrapezoidProfile.State lastSetpoint = new TrapezoidProfile.State();

   public Arm() {
      pivotMotor = new SparkMax(Constants.armId, MotorType.kBrushless);
      intakeMotorTop = new SparkMax(Constants.intakeMotorTopId, MotorType.kBrushless);
      intakeMotorBottom = new SparkMax(Constants.intakeMotorBottomId, MotorType.kBrushless);
      colorSensor = new TCS34725ColorSensor();
      colorSensor.init();
      pivotEncoder = new DutyCycleEncoder(0);
   }

   private final ArmFeedforward feedForward = new ArmFeedforward(kS, kG, kV);

   public Command rotate(Double speed) {
   
      return startEnd(() -> {
         pivotMotor.set(speed);
      }, () -> {
         pivotMotor.set(0);
      });
   }

   public Command intake(Double speed) {
      return startRun(() -> {
         endIntake = false;
      }, () -> {
         colorSum = color.getR() + color.getB() + color.getG();
         if (colorSum > Constants.colorSumThreshold) {
            isCoral = true;
         } else {
            isCoral = false;
         }

         if (!isCoral && prevIsCoral) {
            endIntake = true;
         }

         prevIsCoral = isCoral;

         if (endIntake) {
            intakeMotorTop.set(0);
            intakeMotorBottom.set(0);
         } else {
            intakeMotorTop.set(speed);
            intakeMotorBottom.set(speed);
         }
      }).finallyDo(() -> {
         intakeMotorTop.set(0);
         intakeMotorBottom.set(0);
      });
      

   }

   public boolean getEndIntake() {
      return endIntake;
   }

   public Command moveToPosition(double angle) {
      // controller.setGoal(height);
      //intake-0.88
      //outtake.97
      //L4 .99-> 0.98
      return startRun(() -> {
         startTime = Timer.getFPGATimestamp();
         isFinishedRotating = false;
         SmartDashboard.putBoolean("input detected", isFinishedRotating);

         System.out.println("arm start moving to position");
         // leftMotor.setVoltage(voltage);
         // rightMotor.setVoltage(-voltage);
      }, () -> {
         // lastSetpoint = setpoint;
         // setpoint = profile.calculate(Timer.getFPGATimestamp() - startTime, new TrapezoidProfile.State(0, 0),
         //       new TrapezoidProfile.State(height, 0));

         // // voltage = feedForward.calculate(velocity);
         // System.out.println("moving to outaking");
         // SmartDashboard.putNumber("target Velocity", setpoint.velocity);
         // // SmartDashboard.putNumber("measured Velocity", ArmEncoder.getVelocity());
         // voltage = feedForward.calculateWithVelocities(setpoint.position, lastSetpoint.velocity, setpoint.velocity);
         // pivotMotor.setVoltage(voltage);
         double error = pivotEncoder.get() - angle;
         SmartDashboard.putNumber("arm error", error);
         if(Math.abs(error) > 0.01){
            if(error < 0) {
               pivotMotor.set(Constants.pivotSpeed);
            } else {
               pivotMotor.set(-Constants.pivotSpeed);
            }
         } else {
            pivotMotor.set(0);
            isFinishedRotating = true;
         }
      })
      .until(() -> isFinishedRotating)
      .finallyDo(() -> {
         System.out.println("arm finished moving to position");

         voltage = 0;
         pivotMotor.setVoltage(0);
      });

   }

   // positive outtakes
   public Command outtake(double outtakeSpeed) {
      return startEnd(() -> {
         intakeMotorTop.set(outtakeSpeed);
         intakeMotorBottom.set(outtakeSpeed);
      }, () -> {
         intakeMotorTop.set(0);
         intakeMotorBottom.set(0);
      });
   }

   public double getPosition() {
      return pivotEncoder.get();
   }

   public void periodic() {
      color = colorSensor.readColors();
      SmartDashboard.putNumber("red", color.getR());
      SmartDashboard.putNumber("green", color.getG());
      SmartDashboard.putNumber("blue", color.getB());
      SmartDashboard.putNumber("sum", color.getR() + color.getB() + color.getG());
      SmartDashboard.putNumber("pivot pos", pivotEncoder.get());
      SmartDashboard.putBoolean("isFinishedRotating", isFinishedRotating);
      if (pivotEncoder.get() == 1) {
         System.out.println("arm pivot encoder no worky");
      }
   }
}
