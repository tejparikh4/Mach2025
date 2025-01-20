// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  private final Joystick driver = new Joystick(0); 
  private final Joystick driver2 = new Joystick(1);
  private final CommandPS4Controller drive = new CommandPS4Controller(0);
  
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final POVButton dPad_Right = new POVButton(driver2, 90, 0);
  private final POVButton dPad_Top = new POVButton(driver2, 0, 0);
  private final POVButton dPad_Left = new POVButton(driver2, 270, 0);
  private final POVButton dPad_Down = new POVButton(driver2, 180);
  private final JoystickButton aButton = new JoystickButton(driver2, XboxController.Button.kA.value);
  private final JoystickButton leftBumper = new JoystickButton(driver2, XboxController.Button.kLeftBumper.value);
  private final JoystickButton limeLightDriveButton = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton rightTrigger = new JoystickButton(driver2, 3);
  private final JoystickButton hangarmUpButton = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton hangarmDownButton = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton x2Button = new JoystickButton(driver2, XboxController.Button.kX.value);

  /* Driver Buttons */



  private final JoystickButton slowSpeed = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final JoystickButton turbo = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  private final POVButton hangarmLeftDown = new POVButton(driver, 270, 0);
  private final POVButton hangarmRightDown = new POVButton(driver, +90, 0);

  /* Subsystems */

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();


  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    String rioSerialNum = RobotController.getSerialNumber();
  
     // find serial number of kraken roborio and update this line
     
    //neo seriall num 317B6CA
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    
  }
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                              () -> drive.getRawAxis(translationAxis),
                                                              () -> drive.getRawAxis(strafeAxis))
                                                              .withControllerRotationAxis(drive::getRightX)
                                                              .deadband(0.1)
                                                              .scaleTranslation(1.2)
                                                              .allianceRelativeControl(true);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);


  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().
                                        withControllerHeadingAxis(drive::getRightX, drive::getRightY).
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    aButton.whileTrue(c_GroundIntake);
    leftBumper.whileTrue(c_GroundOuttake);
  }

  public void teleopInit() { }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
