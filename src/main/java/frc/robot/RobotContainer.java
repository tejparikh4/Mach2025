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
import java.util.function.DoubleSupplier;

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

  private final CommandPS4Controller controller = new CommandPS4Controller(Constants.kDriverControllerPort);
  private final Joystick control = new Joystick(Constants.kDriverControllerPort);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton x = new JoystickButton(control,  XboxController.Button.kX.value);
  private final JoystickButton y = new JoystickButton(control,  XboxController.Button.kY.value);
  private final JoystickButton a = new JoystickButton(control,  XboxController.Button.kA.value);
  private final JoystickButton b = new JoystickButton(control,  XboxController.Button.kB.value);

  private final SendableChooser<String> chooserJoystick;


  /* Subsystems */

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    chooserJoystick = new SendableChooser<String>();
    chooserJoystick.setDefaultOption("PLaystation", "Playstation"); 
    chooserJoystick.addOption("Logitech", "logitech");
    chooserJoystick.addOption("Xbox", "Xbox");
    

  
     // find serial number of kraken roborio and update this line
     
    //neo seriall num 317B6CA
    // Configure the trigger bindings
     configureBindings();
     
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    
  }
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                              () -> -control.getRawAxis(translationAxis),
                                                              () -> -control.getRawAxis(strafeAxis))
                                                              .withControllerRotationAxis(() -> -control.getRawAxis(rotationAxis))
                                                              .deadband(0.1)
                                                              .scaleTranslation(1.2)
                                                              .allianceRelativeControl(true);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);


  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().
                                        withControllerHeadingAxis(getRightX(), getRightY()).
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
    // controller.`triangle().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));
    // controller.L2().onTrue(arm.rotate(() -> controller.getL2Axis()));
    // controller.R2().onTrue(arm.rotate(() -> -controller.getR2Axis()));
    // controller.cross().whileTrue(elevator.setSpeed(0.3));
    // controller.circle().whileTrue(elevator.setSpeed(-0.3));
    x.whileTrue(elevator.setSpeed(1));
    y.whileTrue(elevator.setSpeed(-1));
    a.whileTrue(arm.intake(0.5));
    b.whileTrue(arm.intake(-0.5));
//change code to use playstation and not command playstation. then make motor value variables for code
//sorry this comment is so bad i had like 10 secs - mark twain
  }

  public void teleopInit() {
      // drivebase.stopAutoCentering();
   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand()
  // {
  //   // An example command will be run in autonomous
  // return drivebase.getAutonomousCommand("fuck");
  // }

  public DoubleSupplier getRightX() {
    return () -> {return control.getRawAxis(rotationAxis);};
  }

  public DoubleSupplier getRightY() {
    return () -> {return control.getRawAxis(XboxController.Axis.kRightY.value);};
  }
}
