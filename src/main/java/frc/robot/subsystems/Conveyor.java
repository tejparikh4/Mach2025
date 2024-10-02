package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ConveyorConstants.Mod4;

public class Conveyor extends SubsystemBase {
    private final CANSparkMax intakeMotorTop;
    private final CANSparkMax intakeMotorBottom;

    public Conveyor() {
        intakeMotorTop = new CANSparkMax(Mod4.intakeMotorIDTop, MotorType.kBrushless);
        intakeMotorBottom = new CANSparkMax(Mod4.intakeMotorIDBottom, MotorType.kBrushless);
    }
    public void groundIntake() {
        // TCSColor color = colorSensor.readColors();
        
        SmartDashboard.putString("Conveyor Status", "GROUNDINTAKE");
        // if (!color.equals(noteColor)){
        //     intakeMotor.set(1);
        //     placementMotor.set(0.4);
        // }
        intakeMotorTop.set(0.7);
        intakeMotorBottom.set(-0.7);
    }

    public void groundOuttake() {
        SmartDashboard.putString("Conveyor Status", "GROUNDOUTTAKE");
        intakeMotorTop.set(-0.5);
        intakeMotorBottom.set(0.5);
    }
    public void stop() {
        SmartDashboard.putString("Conveyor Status", "STOP");
        intakeMotorTop.set(0);
        intakeMotorBottom.set(0);
    }
}
