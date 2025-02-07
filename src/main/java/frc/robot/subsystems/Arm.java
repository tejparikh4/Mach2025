package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
   private SparkMax ArmMotor;
   public Arm(){
    ArmMotor = new SparkMax(Constants.armId, MotorType.kBrushless);
   }
   public Command rotate(DoubleSupplier speedDoubleSupplier){
    return run(() -> { ArmMotor.set(speedDoubleSupplier.getAsDouble());});
   }
}
