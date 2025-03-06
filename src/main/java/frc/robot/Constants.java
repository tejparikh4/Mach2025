// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final float maxSpeed = 1f;
  public static final int armId = 17;

  public static final int elevatorLeftId = 13; //not the right one yet
  public static final int elevatorRightId = 14;
  public static final double kElevatorGearRatio = 20;
  public static final double kElevatorGearDiameter = 1.757; // inches
  public static double elevatorConstant = 1 / (kElevatorGearDiameter * Math.PI) * kElevatorGearRatio;
  

  public static final int kDriverControllerPort = 0;
  public static int intakeMotorTopId=15;
  public static int intakeMotorBottomId=16;

  
}
