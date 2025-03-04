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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
//    private SparkMax ArmMotor;
   private RelativeEncoder ArmEncoder;
   private double voltage = 0;
   private SparkMax intakeMotortop;
   private SparkMax intakeMotorbottom;
   private double kS = 0;
   private double kG = 0;
   private double kV = 0;
   private double startTime;
   private static double kMaxVelocity = (32* Math.PI);
   private static double kMaxAcceleration = (16 * Math.PI);
//    private TCS34725ColorSensor colorSensor;
//    private TCSColor color = colorSensor.readColors();
    private double colorSum = 0;
    private final double blackSum = 0;// find this number
   private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocity,
         kMaxAcceleration);
   private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
   private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
   private TrapezoidProfile.State lastSetpoint = new TrapezoidProfile.State();

   private double setToVoltage = 0;
   public Arm() {
    //   ArmMotor = new SparkMax(Constants.armId, MotorType.kBrushless);
      intakeMotortop = new SparkMax(Constants.intakeMotortopId, MotorType.kBrushless);
      intakeMotorbottom = new SparkMax(Constants.intakeMotorbottomId, MotorType.kBrushless);
    //   colorSensor = new TCS34725ColorSensor();
    //   ArmEncoder = ArmMotor.getEncoder();
   }

   private final ArmFeedforward feedForward = new ArmFeedforward(kS, kG, kV);

//    public Command rotate(DoubleSupplier speedDoubleSupplier) {
   
//       return startEnd(() -> {
//          ArmMotor.set(speedDoubleSupplier.getAsDouble());
//       }, () -> {
//          ArmMotor.set(0);
//       });
//    }

   public Command intake(Double speedDouble) {
         return startEnd(() -> {
            intakeMotorbottom.set(speedDouble);
            intakeMotortop.set(speedDouble);
         }, () -> {                                                                             
         intakeMotorbottom.set(0);
         intakeMotortop.set(0);
      });

   }

//    public Command moveToPositron(double height) {
//       // controller.setGoal(height);
//       return startRun(() -> {
//          startTime = Timer.getFPGATimestamp();
//          // leftMotor.setVoltage(voltage);
//          // rightMotor.setVoltage(-voltage);
//       }, () -> {
//          lastSetpoint = setpoint;
//          setpoint = profile.calculate(Timer.getFPGATimestamp() - startTime, new TrapezoidProfile.State(0, 0),
//                new TrapezoidProfile.State(height, 0));

//          // voltage = feedForward.calculate(velocity);
//          System.out.println("moving to outaking epic style");
//          SmartDashboard.putNumber("target: louis vuiotton", setpoint.velocity);
//          SmartDashboard.putNumber("measured: louis vuiotton", ArmEncoder.getVelocity());
//          voltage = feedForward.calculateWithVelocities(lastSetpoint.position, lastSetpoint.velocity, setpoint.velocity);
//          ArmMotor.setVoltage(voltage);
//       }).finallyDo(() -> {
//          voltage = 0;
//          ArmMotor.setVoltage(0);
//       });

//    }
//    public Command setArmSpeed() {
//       return startEnd(() ->{
// ArmMotor.setVoltage(setToVoltage);
//       }, () -> {
// ArmMotor.setVoltage(setToVoltage);
//       }

//       );
//    }

//    public void changeArmSpeed(double delta) {
//       setToVoltage += delta;
//   }


//    public Command outtakeColor(double outtakeSpeed) {
//       return run(() -> {
//        if(Math.abs(colorSum - blackSum) < 100) 
//          intakeMotortop.set(outtakeSpeed);
//          intakeMotorbottom.set(-outtakeSpeed);
//       });
//    }

// @Override 
// public void periodic(){
//     colorSum = color.getR() + color.getG() + color.getB();
// }
}

