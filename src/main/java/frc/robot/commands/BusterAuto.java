package frc.robot.commands;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class BusterAuto extends SequentialCommandGroup {

    private RobotContainer robotContainer;
    private SendableChooser<String> chooserAuto;
    private double angle;


    public BusterAuto(RobotContainer robotContainer, SendableChooser<String> chooserAuto) {
        this.robotContainer = robotContainer;
        this.chooserAuto = chooserAuto;
        boolean isRed = DriverStation.getAlliance().get().equals(Alliance.Red);
        SmartDashboard.putBoolean("isRed", isRed);

        

        switch (chooserAuto.getSelected()) {
            case "nothing":
                break;
            case "leave":
                addCommands(new PathPlannerAuto("straightauto"));
                break;
            case "3 coral right":
                // PathPlannerPath path = PathPlannerPath.fromPathFile("1 coral");
                
                if (isRed) {
                    angle = 0;
                } else {
                    angle = Math.PI;
                }

                addCommands(
                    new InstantCommand(() -> robotContainer.drivebase.setGyroRadians(angle)),
                    new PathPlannerAuto("3 coral auto right"));
                break;
            case "3 coral left":
                if (isRed) {
                    angle = 0;
                } else {
                    angle = Math.PI;
                }
                addCommands(
                    new InstantCommand(() -> robotContainer.drivebase.setGyroRadians(angle)),
                    new PathPlannerAuto("3 coral auto left"));
                break;
        }
    }
}