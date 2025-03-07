// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;

import java.io.File;
import java.util.Dictionary;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandPS4Controller controller = new CommandPS4Controller(Constants.kDriverControllerPort);
  private final Joystick control = new Joystick(Constants.kDriverControllerPort);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;



  /* Subsystems */

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    String rioSerialNum = RobotController.getSerialNumber();
  
     // find serial number of kraken roborio and update this line
     
    //neo seriall num 317B6CA
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    elevator.setDefaultCommand(new InstantCommand(() -> elevator.runMotors(elevator.getkG()), elevator));
    // arm.setDefaultCommand(arm.outtake(0));
  }
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                              () -> -controller.getRawAxis(translationAxis),
                                                              () -> -controller.getRawAxis(strafeAxis))
                                                              .withControllerRotationAxis(() -> -controller.getRightX())
                                                              .deadband(0.1)
                                                              .scaleTranslation(1.2)
                                                              .allianceRelativeControl(true);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);


  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().
                                        withControllerHeadingAxis(controller::getRightX, controller::getRightY).
                                        headingWhile(true);
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  /*
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary]
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  

    controller.options().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));
    controller.L1().whileTrue(arm.rotate(0.5));
    controller.R1().whileTrue(arm.rotate(-0.5));
    controller.pov(0).whileTrue(elevator.setSpeed(1));
    controller.cross().whileTrue(elevator.setSpeed(-1));
    controller.circle().whileTrue(arm.outtake(-0.5));
    controller.square().whileTrue(arm.intake(0.5));

    // controller.cross().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    // controller.circle().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));

    // controller.square().whileTrue(elevator.sysIdDynamic(Direction.kForward));
    // controller.triangle().whileTrue(elevator.sysIdDynamic(Direction.kReverse));

    // controller.triangle().whileTrue(arm.intake(0.5));
    controller.triangle().whileTrue(elevator.moveToHeight(114));

    controller.share().onTrue(new InstantCommand(() -> elevator.zeroEncoders()));
    //
    // controller.pov(0).whileTrue(arm.outtake(0.5));
    // controller.pov(180).whileTrue(arm.outtake(-.5));

    controller.pov(0).onTrue(new InstantCommand(() -> elevator.changeSpeed(0.01)));
    controller.pov(180).onTrue(new InstantCommand(() -> elevator.changeSpeed(-0.01)));

  }

  public void teleopInit() {
      // drivebase.stopAutoCentering();
   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
  return drivebase.getAutonomousCommand("fuck");
  }
}
