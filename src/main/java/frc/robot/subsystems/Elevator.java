package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private SparkClosedLoopController leftController;
    private SparkClosedLoopController rightController;

    public Elevator() {
        leftMotor = new SparkMax(Constants.elevatorLeftId, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.elevatorRightId, MotorType.kBrushless);

        leftController = leftMotor.getClosedLoopController();
        rightController = rightMotor.getClosedLoopController();
    }

    public void moveToHeight(double height){
        leftController.setReference(height / Constants.elevatorConstant, SparkBase.ControlType.kPosition);
        rightController.setReference(height / Constants.elevatorConstant, SparkBase.ControlType.kPosition);
    }
    
}
