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
  public static final int armId = 0;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ConveyorConstants {
    /* Conveyor module - Module 4 */
    public static final class Mod4 {
      public static final int intakeMotorChannel = 1;
      public static final int intakeMotorIDTop = 13;
      public static final int intakeMotorIDBottom = 19;
      public static final int intakeEncoderPort = 15;
      public static final int placementMotorID = 14;
      // these ID's are left and right from the POV of the shooter
      public static final int flywheelMotorIDLeft = 15;
      public static final int flywheelMotorIDRight = 16;


      //  public static final boolean kEncoderReversed = false;
      // public static final int kEncoderCPR = 42;
      // public static final double kEnocderDistancePerPulse = 1.0 / (double) kEncoderCPR;

    }
  }


}
