package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmNew extends SubsystemBase() {
    private SparkMax armMotor;

    public ArmNew() {
        armMotor = new SparkMax(19, MotorType.kBrushless);
    }

    public Command rotateMotor(double speed) {
        return startEnd(() -> {
            armMotor.set(speed);
        }, () ->{
            armMotor.set(0);
        })
    }
}

