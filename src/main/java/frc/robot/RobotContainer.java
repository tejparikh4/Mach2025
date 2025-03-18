// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.BusterAuto;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandPS4Controller controller = new CommandPS4Controller(Constants.kDriverControllerPort);
  private final CommandXboxController controller2 = new CommandXboxController(Constants.kSecondaryControllerPort);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // private final JoystickButton aButton = new JoystickButton(control, XboxController.Button.kA.value);
  // private final JoystickButton bButton = new JoystickButton(control, XboxController.Button.kB.value);
  // private final JoystickButton xButton = new JoystickButton(control, XboxController.Button.kX.value);
  // private final JoystickButton yButton = new JoystickButton(control, XboxController.Button.kY.value);
  // private final POVButton dpadUp = new POVButton(control, 0);
  // private final POVButton dpadLeft = new POVButton(control, 270);
  // private final POVButton dpadRight = new POVButton(control, 90);
  // private final POVButton dpadDown = new POVButton(control, 180);

  // private final JoystickButton startButton = new JoystickButton(control, XboxController.Button.kStart.value);
  // private final JoystickButton backButton = new JoystickButton(control, XboxController.Button.kBack.value);



  /* Subsystems */

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator();
  private final Camera camera = new Camera(drivebase);


  private SendableChooser<String> chooserAuto;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    String rioSerialNum = RobotController.getSerialNumber();
  
     // find serial number of kraken roborio and update this line
     
    //neo seriall num 317B6CA
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    // elevator.setDefaultCommand(new InstantCommand(() -> elevator.runMotors(elevator.getkG()), elevator));
    // arm.setDefaultCommand(arm.outtake(0));

    NamedCommands.registerCommand("L2",
      arm.moveToPosition(Constants.transitionRotation).andThen(
      elevator.moveToHeight(Constants.L2Height)).andThen(
      arm.moveToPosition(Constants.L2Rotation)).andThen(
        arm.outtake(.25))
    );

    chooserAuto = new SendableChooser<String>();
    chooserAuto.setDefaultOption("nothing", "nothing");
    chooserAuto.addOption("leave", "leave");
    chooserAuto.addOption("1 coral (IMPOSSIBLE)", "1 coral");
    SmartDashboard.putData(chooserAuto);
    

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
                                        withControllerHeadingAxis(getRightX(), getRightY()).
                                        headingWhile(true);
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);



  SwerveInputStream driveAngularVelocitySlow = driveAngularVelocity.copy()
                                                .scaleTranslation(0.2);

  Command driveFieldOrientedAngularVelocitySlow = drivebase.driveFieldOriented(driveAngularVelocitySlow);


  SwerveInputStream driveIntakeRight = driveAngularVelocity.copy()
                                        .withControllerHeadingAxis(() -> -0.47381472041, () -> -1.0)
                                        .headingWhile(true);

  Command driveFieldOrientedFacingIntakeRight = drivebase.driveFieldOriented(driveIntakeRight);

  SwerveInputStream driveIntakeLeft = driveAngularVelocity.copy()
                                        .withControllerHeadingAxis(() -> 0.47381472041, () -> -1.0)
                                        .headingWhile(true);

  Command driveFieldOrientedFacingIntakeLeft = drivebase.driveFieldOriented(driveIntakeLeft);
  
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
  
    controller.R1().whileTrue(driveFieldOrientedAngularVelocitySlow);
    controller.options().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));

    controller2.back().onTrue(new InstantCommand(()-> elevator.zeroEncoders()));
    controller2.leftBumper().whileTrue(arm.rotate(0.25));
    //controller.L2().whileTrue(arm.moveToPosition(0.28));
    controller2.rightBumper().whileTrue(arm.rotate(-0.25));

    controller2.rightTrigger().whileTrue(arm.outtake(1));


    // controller.cross().whileTrue(elevator.setSpeed(-1));
    // controller.circle().whileTrue(arm.outtake(-0.5));
    // controller.square().whileTrue(arm.intake(0.5));

    controller2.y().whileTrue(elevator.setSpeed(3));
    controller2.a().whileTrue(elevator.setSpeed(-2.5));
    controller2.x().whileTrue(arm.intake(0.5));
    controller2.pov(0).whileTrue(
      arm.moveToPosition(Constants.transitionRotation).andThen(
      elevator.moveToHeight(Constants.L4Height)).andThen(
      arm.moveToPosition(Constants.L4Rotation))
    );
    controller2.pov(270).whileTrue(
      arm.moveToPosition(Constants.transitionRotation).andThen(
      elevator.moveToHeight(Constants.L3Height)).andThen(
      arm.moveToPosition(Constants.L3Rotation))
    );
    controller2.pov(90).whileTrue(
      arm.moveToPosition(Constants.transitionRotation).andThen(
      elevator.moveToHeight(Constants.L2Height)).andThen(
      arm.moveToPosition(Constants.L2Rotation))
      );
    controller2.pov(180).whileTrue(
      arm.moveToPosition(Constants.transitionRotation).andThen(
      elevator.moveToHeight(Constants.L1Height)).andThen(
      arm.moveToPosition(Constants.L1Rotation))
    );
    
    controller2.b().whileTrue(
      arm.moveToPosition(Constants.transitionRotation).andThen(
      elevator.moveToHeight(Constants.intakeHeight)).andThen(
      arm.moveToPosition(Constants.intakeRotation))
    );


    // controller.cross().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    // controller.circle().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));


    // controller.square().whileTrue(elevator.sysIdDynamic(Direction.kForward));
    // controller.triangle().whileTrue(elevator.sysIdDynamic(Direction.kReverse));

    // controller.triangle().whileTrue(arm.intake(0.5));
    // controller.triangle().whileTrue(elevator.moveToHeight(114));

    // controller.share().onTrue(new InstantCommand(() -> elevator.zeroEncoders()));
    // //
    // // controller.pov(0).whileTrue(arm.outtake(0.5));
    // // controller.pov(180).whileTrue(arm.outtake(-.5));

    // controller.pov(0).onTrue(new InstantCommand(() -> elevator.changeSpeed(0.01)));
    // controller.pov(180).onTrue(new InstantCommand(() -> elevator.changeSpeed(-0.01)));

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
    // return drivebase.getAutonomousCommand("straightauto");
    return new BusterAuto(this, chooserAuto);
  }

  public DoubleSupplier getRightX() {
    return () -> {return controller2.getRawAxis(rotationAxis);};
  }

  public DoubleSupplier getRightY() {
    return () -> {return controller2.getRawAxis(XboxController.Axis.kRightY.value);};
  }
}
