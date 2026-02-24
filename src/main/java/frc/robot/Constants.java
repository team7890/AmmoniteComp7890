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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorIDs {
    public static final int iIntake = 13;
    public static final int iHopperExtender = 14;
    public static final int iIndexerLeader = 15;
    public static final int iIndexerFollower = 21;
    public static final int iFeederLeader = 16;
    public static final int iFeederFollower = 17;
    public static final int iHood = 18;
    public static final int iShooterLeader = 19;
    public static final int iShooterFollower = 20;
    //public static final int iClimbR = ;
    //public static final int iClimbL = ;
  }

  public static class MotorSpeeds{
    public static final double dShooterSpeed = 0.75;
    public static final double dShooterRPM = 2950;
    public static final double dFeederSpeed = 1.0;
    public static final double dIndexerSpeed = 1.0;
    public static final double dIntakeSpeed = 0.5;
  }
  public static class MotorPositions{
   // public static final Pos dshooterHood = 

  }
  
}