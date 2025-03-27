// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Arrays;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final float maxSpeed = 4f;
  public static final int armId = 17;

  public static final int elevatorLeftId = 13; //not the right one yet
  public static final int elevatorRightId = 14;
  public static final double kElevatorGearRatio = 20;
  public static final double kElevatorGearDiameter = 1.757; // inches
  public static double elevatorConstant = 1 / (kElevatorGearDiameter * Math.PI) * kElevatorGearRatio;
  public static final double kElevatorMaxVelocity  = 90;
  public static final double kElevatorMaxAcceleration = 80;
  

  public static final int kDriverControllerPort = 0;
  public static final int kSecondaryControllerPort = 1;
  public static int intakeMotorTopId=15;
  public static int intakeMotorBottomId=16;

  public static double intakeHeight = 0.5;
  public static double intakeRotation = 0.52;

  public static double L1Height = 0.5;
  public static double L1Rotation = 0.59;

  public static double L2Height = 26;
  public static double L2Rotation = 0.59;

  public static double L3Height = 57;
  public static double L3Rotation = L2Rotation;

  public static double outakeRotation = 0.63; 

  public static double L4Height = 114;
  public static double L4Rotation = 0.55;

  public static double transitionRotation = 0.61;

  public static double pivotSpeed = 0.45;

  public static double colorSumThreshold = 900;

  public static double closestFiducialIgnoreThreshold = 4;



}
