package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.TCS34725ColorSensor;
import frc.robot.TCS34725ColorSensor.TCSColor;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
   private SparkMax ArmMotor;
   private SparkMax intakeMotor1;
   private SparkMax intakeMotor2;
   private TCS34725ColorSensor colorSensor;
   private TCSColor color = colorSensor.readColors();
   public Arm(){
      ArmMotor = new SparkMax(Constants.armId, MotorType.kBrushless);
      intakeMotor1 = new SparkMax(Constants.intakeMotor1Id, MotorType.kBrushless);
      intakeMotor2 = new SparkMax(Constants.intakeMotor2Id, MotorType.kBrushless);
      colorSensor = new TCS34725ColorSensor();
   }
   public Command rotate(DoubleSupplier speedDoubleSupplier){
         return run(() -> { ArmMotor.set(speedDoubleSupplier.getAsDouble());});
   }
   public Command intake(Double speedDouble){
      if(!color.equals(new TCSColor(0, 0, 0, 0))){
         return run(() ->{intakeMotor1.set(speedDouble);});
      }
      return run(() ->{intakeMotor1.set(0);});
   }
   public Command outake(Double speeDouble, int behavior){
      //We Set the feedforward for the appropriate behavior that is assigned
      //
      return run(()->new InstantCommand());
   }

}
